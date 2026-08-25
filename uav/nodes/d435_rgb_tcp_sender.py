#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import socket
import struct
import time
import threading

import cv2
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image


class D435RgbTcpSender:
    """
    Jetson side node:
    1. Subscribe to the D435 RGB image topic.
    2. Convert ROS Image to OpenCV BGR.
    3. JPEG-compress the latest frame.
    4. Stream frames to the DK2500 over TCP.

    Network addresses and ports are deployment parameters. The defaults are
    examples from one field setup and should be overridden at launch time.
    """

    def __init__(self):
        rospy.init_node("d435_rgb_tcp_sender", anonymous=True)

        self.dk_ip = rospy.get_param("~dk_ip", "10.42.0.1")
        self.dk_port = rospy.get_param("~dk_port", 9100)

        self.image_topic = rospy.get_param(
            "~image_topic",
            "/camera/color/image_raw"
        )

        self.jpeg_quality = rospy.get_param("~jpeg_quality", 60)
        self.send_fps = rospy.get_param("~send_fps", 10.0)

        # 0 disables resizing. 640 or 424 are practical field values.
        self.resize_width = rospy.get_param("~resize_width", 640)

        self.bridge = CvBridge()
        self.sock = None
        self.seq = 0

        self.latest_frame = None
        self.latest_stamp = None
        self.frame_lock = threading.Lock()

        rospy.Subscriber(
            self.image_topic,
            Image,
            self.image_callback,
            queue_size=1,
            buff_size=2**24
        )

        rospy.loginfo("D435 RGB TCP Sender started.")
        rospy.loginfo("Image topic: %s", self.image_topic)
        rospy.loginfo("DK2500 address: %s:%d", self.dk_ip, self.dk_port)

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

            if self.resize_width and self.resize_width > 0:
                h, w = frame.shape[:2]
                if w != self.resize_width:
                    scale = self.resize_width / float(w)
                    new_h = int(h * scale)
                    frame = cv2.resize(frame, (self.resize_width, new_h))

            with self.frame_lock:
                self.latest_frame = frame
                self.latest_stamp = msg.header.stamp.to_sec()

        except Exception as e:
            rospy.logwarn("Image callback error: %s", str(e))

    def connect_to_dk2500(self):
        while not rospy.is_shutdown():
            try:
                rospy.loginfo("Connecting to DK2500 %s:%d ...", self.dk_ip, self.dk_port)

                sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                sock.settimeout(5.0)
                sock.connect((self.dk_ip, self.dk_port))
                sock.settimeout(None)

                self.sock = sock
                rospy.loginfo("Connected to DK2500.")
                return True

            except Exception as e:
                rospy.logwarn("Connect failed: %s. Retry in 1s.", str(e))
                time.sleep(1.0)

        return False

    def close_socket(self):
        if self.sock is not None:
            try:
                self.sock.close()
            except Exception:
                pass
            self.sock = None

    def send_loop(self):
        rate = rospy.Rate(self.send_fps)

        while not rospy.is_shutdown():
            if self.sock is None:
                ok = self.connect_to_dk2500()
                if not ok:
                    break

            with self.frame_lock:
                if self.latest_frame is None:
                    rate.sleep()
                    continue

                frame = self.latest_frame.copy()
                stamp = self.latest_stamp if self.latest_stamp is not None else time.time()

            try:
                encode_param = [
                    int(cv2.IMWRITE_JPEG_QUALITY),
                    int(self.jpeg_quality)
                ]

                ok, jpg = cv2.imencode(".jpg", frame, encode_param)
                if not ok:
                    rospy.logwarn("JPEG encode failed.")
                    rate.sleep()
                    continue

                jpg_bytes = jpg.tobytes()
                h, w = frame.shape[:2]
                payload_len = len(jpg_bytes)

                # Packet:
                # magic[4] = RGBD
                # seq:uint32, stamp:double, width:uint16, height:uint16,
                # payload_len:uint32, payload:JPEG bytes
                magic = b"RGBD"
                header = struct.pack(
                    "!I d H H I",
                    self.seq,
                    stamp,
                    w,
                    h,
                    payload_len
                )

                packet = magic + header + jpg_bytes
                self.sock.sendall(packet)

                rospy.loginfo_throttle(
                    2.0,
                    "RGB sent: seq=%d, size=%.2f KB, resolution=%dx%d",
                    self.seq,
                    payload_len / 1024.0,
                    w,
                    h
                )

                self.seq += 1

            except Exception as e:
                rospy.logwarn("Send failed: %s. Reconnecting...", str(e))
                self.close_socket()
                time.sleep(0.5)

            rate.sleep()


if __name__ == "__main__":
    node = D435RgbTcpSender()
    node.send_loop()
