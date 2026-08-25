#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import json
import socket
import struct
import time
import threading

import numpy as np
import rospy
import sensor_msgs.point_cloud2 as pc2
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String

try:
    import serial
except Exception:
    serial = None


class RawLivoxToPCL2Sender:
    def __init__(self):
        rospy.init_node("raw_livox_to_pcl2_sender", anonymous=True)

        self.dk_ip = rospy.get_param("~dk2500_ip", "10.42.0.1")
        self.dk_port = int(rospy.get_param("~dk2500_port", 9000))

        self.cloud_topic = rospy.get_param("~cloud_topic", "/cloud_registered")
        self.odom_topic = rospy.get_param("~odom_topic", "/Odometry")
        self.lora_topic = rospy.get_param("~lora_topic", "/lora_raw")

        self.enable_cloud = bool(rospy.get_param("~enable_cloud", True))
        self.enable_odom = bool(rospy.get_param("~enable_odom", True))
        self.enable_lora = bool(rospy.get_param("~enable_lora", True))

        self.lora_source = rospy.get_param("~lora_source", "serial")  # serial or topic
        self.lora_serial_port = rospy.get_param(
            "~lora_serial_port",
            "/dev/ttyUSB0"
        )
        self.lora_baudrate = int(rospy.get_param("~lora_baudrate", 115200))
        self.lora_timeout = float(rospy.get_param("~lora_timeout", 0.2))
        self.lora_reconnect_delay = float(rospy.get_param("~lora_reconnect_delay", 1.0))

        self.max_points = int(rospy.get_param("~max_points", 20000))
        self.send_every_n = int(rospy.get_param("~send_every_n", 1))
        self.odom_send_every_n = int(rospy.get_param("~odom_send_every_n", 1))

        self.reconnect_delay = float(rospy.get_param("~reconnect_delay", 2.0))
        self.socket_timeout = float(rospy.get_param("~socket_timeout", 5.0))

        self.sock = None
        self.sock_lock = threading.Lock()
        self.frame_count = 0
        self.odom_count = 0
        self.sent_count = 0
        self.lora_serial = None
        self.stop_event = threading.Event()

        rospy.loginfo("Raw Livox/PCL2 TCP Sender started.")
        rospy.loginfo("DK2500 address: %s:%d", self.dk_ip, self.dk_port)
        rospy.loginfo("Cloud: enable=%s topic=%s", self.enable_cloud, self.cloud_topic)
        rospy.loginfo("Odom : enable=%s topic=%s", self.enable_odom, self.odom_topic)
        rospy.loginfo(
            "LoRa : enable=%s source=%s topic=%s serial=%s baud=%d",
            self.enable_lora,
            self.lora_source,
            self.lora_topic,
            self.lora_serial_port,
            self.lora_baudrate
        )
        rospy.loginfo("max_points=%d, send_every_n=%d", self.max_points, self.send_every_n)

        self.sub_cloud = None
        self.sub_odom = None
        self.sub_lora = None
        self.lora_thread = None

        if self.enable_cloud:
            self.sub_cloud = rospy.Subscriber(
                self.cloud_topic,
                PointCloud2,
                self.cloud_callback,
                queue_size=1,
                buff_size=8 * 1024 * 1024
            )

        if self.enable_odom:
            self.sub_odom = rospy.Subscriber(
                self.odom_topic,
                Odometry,
                self.odom_callback,
                queue_size=20
            )

        if self.enable_lora:
            if self.lora_source == "topic":
                self.sub_lora = rospy.Subscriber(
                    self.lora_topic,
                    String,
                    self.lora_topic_callback,
                    queue_size=20
                )
            else:
                if serial is None:
                    rospy.logwarn("pyserial is not available. LoRa serial sender disabled.")
                else:
                    self.lora_thread = threading.Thread(target=self.lora_serial_loop)
                    self.lora_thread.daemon = True
                    self.lora_thread.start()

    def connect(self):
        with self.sock_lock:
            if self.sock is not None:
                return True

            try:
                rospy.loginfo("Connecting to DK2500 %s:%d ...", self.dk_ip, self.dk_port)
                sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                sock.settimeout(self.socket_timeout)
                sock.connect((self.dk_ip, self.dk_port))
                sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                self.sock = sock
                rospy.loginfo("Connected to DK2500.")
                return True
            except Exception as e:
                rospy.logwarn("Connect failed: %s", str(e))
                self.sock = None
                return False

    def close_socket(self):
        with self.sock_lock:
            if self.sock is not None:
                try:
                    self.sock.close()
                except Exception:
                    pass
                self.sock = None

    def pack_payload(self, payload):
        return struct.pack("!I", len(payload)) + payload

    def send_packet(self, packet):
        if self.sock is None:
            if not self.connect():
                time.sleep(self.reconnect_delay)
                return False

        try:
            with self.sock_lock:
                self.sock.sendall(packet)
            return True
        except Exception as e:
            rospy.logwarn("Send failed: %s", str(e))
            self.close_socket()
            return False

    def send_json(self, obj):
        payload = json.dumps(obj, separators=(",", ":"), ensure_ascii=False).encode("utf-8")
        return self.send_packet(self.pack_payload(payload))

    def get_field_names(self, msg):
        return [f.name for f in msg.fields]

    def cloud_to_numpy(self, msg):
        field_names = self.get_field_names(msg)
        has_intensity = "intensity" in field_names

        if has_intensity:
            read_fields = ("x", "y", "z", "intensity")
        else:
            read_fields = ("x", "y", "z")

        points = []
        count = 0

        for p in pc2.read_points(msg, field_names=read_fields, skip_nans=True):
            if has_intensity:
                x, y, z, intensity = p
            else:
                x, y, z = p
                intensity = 0.0

            points.append((x, y, z, intensity))
            count += 1

            if self.max_points > 0 and count >= self.max_points:
                break

        if not points:
            return None

        return np.asarray(points, dtype=np.float32)

    def build_cloud_packet(self, msg, points_np):
        """
        TCP packet format:

        uint32 packet_len
        char[4] magic = PCL2
        uint32 seq
        double stamp
        uint32 point_count
        uint32 payload_len
        payload: float32 x,y,z,intensity repeated
        """
        payload = points_np.tobytes()
        point_count = points_np.shape[0]

        stamp = msg.header.stamp.to_sec()
        seq = self.sent_count

        header = struct.pack(
            "!4sIdII",
            b"PCL2",
            seq,
            stamp,
            point_count,
            len(payload)
        )

        body = header + payload
        packet = self.pack_payload(body)
        return packet, point_count, len(payload)

    def odom_to_json(self, msg):
        stamp = msg.header.stamp.to_sec()
        pose = msg.pose.pose
        twist = msg.twist.twist

        return {
            "type": "odom",
            "frame_id": msg.header.frame_id or "map",
            "child_frame_id": msg.child_frame_id or "base_link",
            "stamp": stamp,
            "position": {
                "x": pose.position.x,
                "y": pose.position.y,
                "z": pose.position.z,
            },
            "orientation": {
                "x": pose.orientation.x,
                "y": pose.orientation.y,
                "z": pose.orientation.z,
                "w": pose.orientation.w,
            },
            "velocity": {
                "x": twist.linear.x,
                "y": twist.linear.y,
                "z": twist.linear.z,
            },
            "angular_velocity": {
                "x": twist.angular.x,
                "y": twist.angular.y,
                "z": twist.angular.z,
            },
        }

    def lora_to_json(self, text):
        return {
            "type": "lora",
            "stamp": time.time(),
            "data": text,
        }

    def cloud_callback(self, msg):
        self.frame_count += 1

        if self.send_every_n > 1 and self.frame_count % self.send_every_n != 0:
            return

        points_np = self.cloud_to_numpy(msg)
        if points_np is None:
            rospy.logwarn_throttle(2.0, "Empty point cloud, skip.")
            return

        packet, point_count, payload_len = self.build_cloud_packet(msg, points_np)

        if self.send_packet(packet):
            rospy.loginfo_throttle(
                1.0,
                "PointCloud sent: seq=%d, points=%d, payload=%.2f KB",
                self.sent_count,
                point_count,
                payload_len / 1024.0
            )
            self.sent_count += 1

    def odom_callback(self, msg):
        self.odom_count += 1

        if self.odom_send_every_n > 1 and self.odom_count % self.odom_send_every_n != 0:
            return

        if self.send_json(self.odom_to_json(msg)):
            rospy.loginfo_throttle(
                1.0,
                "Odom sent: frame=%s child=%s x=%.2f y=%.2f z=%.2f",
                msg.header.frame_id,
                msg.child_frame_id,
                msg.pose.pose.position.x,
                msg.pose.pose.position.y,
                msg.pose.pose.position.z
            )

    def lora_topic_callback(self, msg):
        text = msg.data.strip()
        if not text:
            return

        if self.send_json(self.lora_to_json(text)):
            rospy.loginfo_throttle(1.0, "LoRa sent(topic): %s", text[:120])

    def open_lora_serial(self):
        if serial is None:
            return False

        try:
            self.lora_serial = serial.Serial(
                port=self.lora_serial_port,
                baudrate=self.lora_baudrate,
                timeout=self.lora_timeout
            )
            rospy.loginfo("LoRa serial opened: %s", self.lora_serial_port)
            return True
        except Exception as e:
            rospy.logwarn("Open LoRa serial failed: %s", str(e))
            self.lora_serial = None
            return False

    def close_lora_serial(self):
        if self.lora_serial is not None:
            try:
                self.lora_serial.close()
            except Exception:
                pass
            self.lora_serial = None

    def lora_serial_loop(self):
        while not rospy.is_shutdown() and not self.stop_event.is_set():
            if self.lora_serial is None or not self.lora_serial.is_open:
                if not self.open_lora_serial():
                    time.sleep(self.lora_reconnect_delay)
                    continue

            try:
                line = self.lora_serial.readline()
                if not line:
                    continue

                text = line.decode("utf-8", errors="ignore").strip()
                if not text:
                    continue

                if self.send_json(self.lora_to_json(text)):
                    rospy.loginfo_throttle(1.0, "LoRa sent(serial): %s", text[:120])

            except Exception as e:
                rospy.logwarn("LoRa serial read/send failed: %s", str(e))
                self.close_lora_serial()
                time.sleep(self.lora_reconnect_delay)

    def shutdown(self):
        self.stop_event.set()
        self.close_lora_serial()
        self.close_socket()


def main():
    sender = RawLivoxToPCL2Sender()
    rospy.on_shutdown(sender.shutdown)
    rospy.spin()
    sender.shutdown()


if __name__ == "__main__":
    main()

