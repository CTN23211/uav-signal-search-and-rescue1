#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
import json
from collections import deque

import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import String, Bool

try:
    from openvino.runtime import Core
except Exception:
    from openvino import Core


class OpenVINOYoloPersonDetector:
    def __init__(self):
        rospy.init_node("openvino_yolo_person_detector", anonymous=False)

        self.image_topic = rospy.get_param("~image_topic", "/d435/rgb_from_jetson")
        self.model_xml = rospy.get_param(
            "~model_xml",
            "/opt/models/yolov8n_openvino_model/yolov8n.xml"
        )
        self.device = rospy.get_param("~device", "CPU")

        self.conf_threshold = float(rospy.get_param("~conf_threshold", 0.35))
        self.iou_threshold = float(rospy.get_param("~iou_threshold", 0.45))
        self.min_area_ratio = float(rospy.get_param("~min_area_ratio", 0.002))
        self.max_area_ratio = float(rospy.get_param("~max_area_ratio", 0.90))
        self.process_fps = float(rospy.get_param("~process_fps", 8.0))

        self.suspect_fps_threshold = float(rospy.get_param("~suspect_fps_threshold", 3.0))
        self.confirm_stable_sec = float(rospy.get_param("~confirm_stable_sec", 2.0))
        self.lost_timeout_sec = float(rospy.get_param("~lost_timeout_sec", 2.0))

        self.vis_topic = rospy.get_param("~vis_topic", "/dk/person_detection/vis")
        self.event_topic = rospy.get_param("~event_topic", "/dk_target_event")
        self.detected_topic = rospy.get_param("~detected_topic", "/dk/person_detected")

        self.bridge = CvBridge()
        self.last_process_time = 0.0
        self.frame_seq = 0

        self.det_time_window = deque()
        self.conf_window = deque()
        self.area_window = deque()
        self.last_valid_det_time = 0.0
        self.suspect_start_time = None
        self.confirmed = False
        self.last_event_state = "NO_TARGET"

        rospy.loginfo("Loading OpenVINO YOLO model: %s", self.model_xml)

        core = Core()
        model = core.read_model(self.model_xml)
        self.compiled_model = core.compile_model(model, self.device)
        self.input_layer = self.compiled_model.input(0)
        self.output_layer = self.compiled_model.output(0)

        shape = list(self.input_layer.shape)
        rospy.loginfo("Model input shape: %s", str(shape))

        if len(shape) != 4:
            raise RuntimeError("Unsupported input shape: %s" % str(shape))

        if shape[1] == 3:
            self.input_layout = "NCHW"
            self.input_h = int(shape[2])
            self.input_w = int(shape[3])
        elif shape[3] == 3:
            self.input_layout = "NHWC"
            self.input_h = int(shape[1])
            self.input_w = int(shape[2])
        else:
            raise RuntimeError("Unsupported input layout: %s" % str(shape))

        self.vis_pub = rospy.Publisher(self.vis_topic, Image, queue_size=1)
        self.event_pub = rospy.Publisher(self.event_topic, String, queue_size=10)
        self.detected_pub = rospy.Publisher(self.detected_topic, Bool, queue_size=10)

        rospy.Subscriber(
            self.image_topic,
            Image,
            self.image_callback,
            queue_size=1,
            buff_size=2 ** 24
        )

        rospy.loginfo("OpenVINO YOLO person detector started.")
        rospy.loginfo("Subscribe image: %s", self.image_topic)
        rospy.loginfo("Publish vis: %s", self.vis_topic)
        rospy.loginfo("Publish event: %s", self.event_topic)

    def letterbox(self, image):
        h, w = image.shape[:2]
        scale = min(float(self.input_w) / w, float(self.input_h) / h)

        nw = int(round(w * scale))
        nh = int(round(h * scale))

        resized = cv2.resize(image, (nw, nh), interpolation=cv2.INTER_LINEAR)

        pad_w = self.input_w - nw
        pad_h = self.input_h - nh
        pad_left = pad_w // 2
        pad_top = pad_h // 2

        padded = cv2.copyMakeBorder(
            resized,
            pad_top,
            pad_h - pad_top,
            pad_left,
            pad_w - pad_left,
            cv2.BORDER_CONSTANT,
            value=(114, 114, 114)
        )

        return padded, scale, pad_left, pad_top

    def preprocess(self, frame_bgr):
        img, scale, pad_left, pad_top = self.letterbox(frame_bgr)

        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        img = img.astype(np.float32) / 255.0

        if self.input_layout == "NCHW":
            img = img.transpose(2, 0, 1)
            blob = np.expand_dims(img, axis=0)
        else:
            blob = np.expand_dims(img, axis=0)

        return blob, scale, pad_left, pad_top

    def postprocess(self, output, frame_w, frame_h, scale, pad_left, pad_top):
        preds = np.array(output)
        preds = np.squeeze(preds)

        if preds.ndim != 2:
            rospy.logwarn_throttle(2.0, "Unsupported YOLO output shape: %s", str(preds.shape))
            return []

        # YOLOv8 常见输出为 [84, 8400]，需要转置为 [8400, 84]
        if preds.shape[0] in [84, 85] and preds.shape[1] > preds.shape[0]:
            preds = preds.T

        boxes = []
        scores = []

        for det in preds:
            if len(det) < 6:
                continue

            # 情况1：YOLOv8 输出 x,y,w,h + 80类分数
            if len(det) >= 84:
                cls_scores = det[4:]
                cls_id = int(np.argmax(cls_scores))
                conf = float(cls_scores[cls_id])
                x, y, w, h = map(float, det[:4])

                if cls_id != 0:
                    continue

                x1 = x - w / 2.0
                y1 = y - h / 2.0
                x2 = x + w / 2.0
                y2 = y + h / 2.0

            # 情况2：带 NMS 的输出 x1,y1,x2,y2,score,class
            else:
                x1, y1, x2, y2, conf, cls_id = det[:6]
                cls_id = int(cls_id)

                if cls_id != 0:
                    continue

            if conf < self.conf_threshold:
                continue

            x1 = (float(x1) - pad_left) / scale
            y1 = (float(y1) - pad_top) / scale
            x2 = (float(x2) - pad_left) / scale
            y2 = (float(y2) - pad_top) / scale

            x1 = int(max(0, min(frame_w - 1, x1)))
            y1 = int(max(0, min(frame_h - 1, y1)))
            x2 = int(max(0, min(frame_w - 1, x2)))
            y2 = int(max(0, min(frame_h - 1, y2)))

            bw = max(1, x2 - x1)
            bh = max(1, y2 - y1)
            area_ratio = float(bw * bh) / float(frame_w * frame_h)

            if area_ratio < self.min_area_ratio or area_ratio > self.max_area_ratio:
                continue

            boxes.append([x1, y1, bw, bh])
            scores.append(float(conf))

        if not boxes:
            return []

        indices = cv2.dnn.NMSBoxes(
            boxes,
            scores,
            self.conf_threshold,
            self.iou_threshold
        )

        detections = []

        if len(indices) > 0:
            for idx in np.array(indices).reshape(-1):
                x1, y1, bw, bh = boxes[idx]
                x2 = x1 + bw
                y2 = y1 + bh
                conf = scores[idx]

                cx = x1 + bw * 0.5
                cy = y1 + bh * 0.5

                detections.append({
                    "label": "person",
                    "conf": conf,
                    "bbox": [int(x1), int(y1), int(x2), int(y2)],
                    "area_ratio": float(bw * bh) / float(frame_w * frame_h),
                    "dx_ratio": (cx - frame_w * 0.5) / (frame_w * 0.5),
                    "dy_ratio": (cy - frame_h * 0.5) / (frame_h * 0.5)
                })

        detections.sort(key=lambda x: x["conf"], reverse=True)
        return detections

    def cleanup_windows(self, now):
        while self.det_time_window and now - self.det_time_window[0] > 1.0:
            self.det_time_window.popleft()

        while self.conf_window and now - self.conf_window[0][0] > 1.0:
            self.conf_window.popleft()

        while self.area_window and now - self.area_window[0][0] > 1.0:
            self.area_window.popleft()

    def compute_window_stats(self, now):
        self.cleanup_windows(now)

        det_fps = float(len(self.det_time_window))

        if self.conf_window:
            conf_avg = float(np.mean([x[1] for x in self.conf_window]))
            conf_max = float(np.max([x[1] for x in self.conf_window]))
        else:
            conf_avg = 0.0
            conf_max = 0.0

        if self.area_window:
            area_avg = float(np.mean([x[1] for x in self.area_window]))
        else:
            area_avg = 0.0

        return det_fps, conf_avg, conf_max, area_avg

    def publish_event(self, event, best_det, det_fps, conf_avg, conf_max, area_avg):
        msg = {
            "event": event,
            "target_id": "person_001",
            "stamp": round(time.time(), 3),
            "frame_seq": int(self.frame_seq),
            "det_fps": round(det_fps, 3),
            "conf_avg": round(conf_avg, 4),
            "conf_max": round(conf_max, 4),
            "area_avg": round(area_avg, 5),
            "source": "openvino_yolo_person_detector"
        }

        if best_det is not None:
            msg.update({
                "conf": round(float(best_det["conf"]), 4),
                "bbox": [int(v) for v in best_det["bbox"]],
                "dx_ratio": round(float(best_det["dx_ratio"]), 4),
                "dy_ratio": round(float(best_det["dy_ratio"]), 4),
                "area_ratio": round(float(best_det["area_ratio"]), 5)
            })

        self.event_pub.publish(String(data=json.dumps(msg, ensure_ascii=False)))

        rospy.loginfo(
            "[yolo_person] event=%s det_fps=%.1f conf_avg=%.2f area_avg=%.4f",
            event,
            det_fps,
            conf_avg,
            area_avg
        )

    def update_target_state(self, detections):
        now = time.time()
        best_det = detections[0] if detections else None

        if best_det is not None:
            self.det_time_window.append(now)
            self.conf_window.append((now, best_det["conf"]))
            self.area_window.append((now, best_det["area_ratio"]))
            self.last_valid_det_time = now

        det_fps, conf_avg, conf_max, area_avg = self.compute_window_stats(now)

        has_suspect = (
            det_fps >= self.suspect_fps_threshold and
            conf_avg >= self.conf_threshold and
            area_avg >= self.min_area_ratio
        )

        event_to_publish = None

        if has_suspect:
            if self.suspect_start_time is None:
                self.suspect_start_time = now

            stable_sec = now - self.suspect_start_time

            if not self.confirmed and stable_sec >= self.confirm_stable_sec:
                self.confirmed = True
                self.last_event_state = "TARGET_CONFIRM"
                event_to_publish = "TARGET_CONFIRM"
            else:
                if self.last_event_state not in ["TARGET_SUSPECTED", "TARGET_CONFIRM"]:
                    self.last_event_state = "TARGET_SUSPECTED"
                    event_to_publish = "TARGET_SUSPECTED"

        else:
            if self.last_valid_det_time > 0 and now - self.last_valid_det_time > self.lost_timeout_sec:
                if self.last_event_state not in ["NO_TARGET", "TARGET_LOST"]:
                    self.last_event_state = "TARGET_LOST"
                    event_to_publish = "TARGET_LOST"

                self.suspect_start_time = None
                self.confirmed = False
                self.last_event_state = "NO_TARGET"

        if event_to_publish is not None:
            self.publish_event(
                event_to_publish,
                best_det,
                det_fps,
                conf_avg,
                conf_max,
                area_avg
            )

        self.detected_pub.publish(Bool(data=(best_det is not None)))

        return self.last_event_state, det_fps, conf_avg

    def draw(self, frame, detections, state, det_fps, conf_avg):
        vis = frame.copy()

        for det in detections:
            x1, y1, x2, y2 = det["bbox"]
            conf = det["conf"]
            area = det["area_ratio"]

            cv2.rectangle(vis, (x1, y1), (x2, y2), (0, 255, 255), 2)
            cv2.putText(
                vis,
                "person %.2f area %.3f" % (conf, area),
                (x1, max(20, y1 - 8)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (0, 255, 255),
                2
            )

        cv2.putText(
            vis,
            "state=%s det_fps=%.1f conf_avg=%.2f" % (state, det_fps, conf_avg),
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.75,
            (0, 255, 0),
            2
        )

        return vis

    def image_callback(self, msg):
        now = time.time()

        if now - self.last_process_time < 1.0 / max(self.process_fps, 1.0):
            return

        self.last_process_time = now
        self.frame_seq += 1

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            rospy.logwarn("cv_bridge error: %s", str(e))
            return

        h, w = frame.shape[:2]

        try:
            blob, scale, pad_left, pad_top = self.preprocess(frame)
            result = self.compiled_model([blob])[self.output_layer]

            detections = self.postprocess(
                result,
                w,
                h,
                scale,
                pad_left,
                pad_top
            )

            state, det_fps, conf_avg = self.update_target_state(detections)

            vis = self.draw(frame, detections, state, det_fps, conf_avg)
            vis_msg = self.bridge.cv2_to_imgmsg(vis, encoding="bgr8")
            vis_msg.header = msg.header
            self.vis_pub.publish(vis_msg)

            rospy.loginfo_throttle(
                2.0,
                "YOLO person detector running. det=%d state=%s conf_avg=%.2f",
                len(detections),
                state,
                conf_avg
            )

        except Exception as e:
            rospy.logwarn("OpenVINO YOLO inference error: %s", str(e))


def main():
    OpenVINOYoloPersonDetector()
    rospy.spin()


if __name__ == "__main__":
    main()