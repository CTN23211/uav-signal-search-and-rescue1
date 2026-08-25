#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import socket
import struct
import time
import threading
import json
import re
import math

import rospy
import numpy as np

from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2 as pc2

try:
    import serial
except ImportError:
    serial = None


class JetsonMinimalSender:
    def __init__(self):
        self.dk_ip = rospy.get_param("~dk2500_ip", "10.42.0.1")

        self.cloud_port = rospy.get_param("~cloud_port", 9000)
        self.lora_port = rospy.get_param("~lora_port", 9001)
        self.odom_port = rospy.get_param("~odom_port", 9002)

        self.cloud_topic = rospy.get_param("~cloud_topic", "/cloud_registered")
        self.odom_topic = rospy.get_param("~odom_topic", "/mavros/local_position/odom")

        self.lora_serial_port = rospy.get_param("~lora_serial_port", "/dev/ttyACM0")
        self.lora_serial_baud = rospy.get_param("~lora_serial_baud", 115200)

        self.max_points = rospy.get_param("~max_points", 10000)
        self.send_every_n = rospy.get_param("~send_every_n", 1)
        self.odom_send_hz = rospy.get_param("~odom_send_hz", 10.0)

        self.cloud_sock = None
        self.lora_sock = None
        self.odom_sock = None
        self.lora_ser = None

        self.frame_count = 0

        self.pose_lock = threading.Lock()
        self.has_odom = False
        self.odom_data = {
            "type": "odom",
            "stamp": time.time(),
            "frame_id": "map",
            "x": 0.0,
            "y": 0.0,
            "z": 0.0,
            "yaw": 0.0,
            "vx": 0.0,
            "vy": 0.0,
            "vz": 0.0
        }

        self.lora_pattern = re.compile(
            r"LORA_RX,"
            r"(?:COUNT=(?P<count>\d+),)?"
            r"RSSI=(?P<rssi>-?\d+),"
            r"SNR=(?P<snr>-?\d+\.?\d*),"
            r"SIZE=(?P<size>\d+),"
            r"MSG=(?P<msg>.*)"
        )

        self.connect_all()

        self.sub_cloud = rospy.Subscriber(
            self.cloud_topic,
            PointCloud2,
            self.cloud_callback,
            queue_size=1
        )

        self.sub_odom = rospy.Subscriber(
            self.odom_topic,
            Odometry,
            self.odom_callback,
            queue_size=20
        )

        self.lora_thread = threading.Thread(target=self.lora_loop, daemon=True)
        self.lora_thread.start()

        self.odom_thread = threading.Thread(target=self.odom_loop, daemon=True)
        self.odom_thread.start()

        rospy.loginfo("Jetson minimal sender started.")
        rospy.loginfo("DK2500 IP: %s", self.dk_ip)
        rospy.loginfo("Cloud topic: %s -> %d", self.cloud_topic, self.cloud_port)
        rospy.loginfo("LoRa serial: %s -> %d", self.lora_serial_port, self.lora_port)
        rospy.loginfo("Odom topic: %s -> %d", self.odom_topic, self.odom_port)

    def connect_tcp(self, port, name):
        while not rospy.is_shutdown():
            try:
                sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                sock.connect((self.dk_ip, port))
                rospy.loginfo("%s TCP connected: %s:%d", name, self.dk_ip, port)
                return sock
            except Exception as e:
                rospy.logwarn("%s TCP connect failed: %s", name, str(e))
                time.sleep(1.0)

    def connect_all(self):
        self.cloud_sock = self.connect_tcp(self.cloud_port, "Cloud")
        self.lora_sock = self.connect_tcp(self.lora_port, "LoRa")
        self.odom_sock = self.connect_tcp(self.odom_port, "Odom")

    def reconnect_cloud(self):
        try:
            self.cloud_sock.close()
        except Exception:
            pass
        self.cloud_sock = self.connect_tcp(self.cloud_port, "Cloud")

    def reconnect_lora(self):
        try:
            self.lora_sock.close()
        except Exception:
            pass
        self.lora_sock = self.connect_tcp(self.lora_port, "LoRa")

    def reconnect_odom(self):
        try:
            self.odom_sock.close()
        except Exception:
            pass
        self.odom_sock = self.connect_tcp(self.odom_port, "Odom")

    def quat_to_yaw(self, q):
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def odom_callback(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        v = msg.twist.twist.linear

        with self.pose_lock:
            self.odom_data = {
                "type": "odom",
                "stamp": msg.header.stamp.to_sec() if msg.header.stamp else time.time(),
                "frame_id": msg.header.frame_id if msg.header.frame_id else "map",
                "x": p.x,
                "y": p.y,
                "z": p.z,
                "yaw": self.quat_to_yaw(q),
                "vx": v.x,
                "vy": v.y,
                "vz": v.z
            }
            self.has_odom = True

    def get_odom_copy(self):
        with self.pose_lock:
            return self.has_odom, dict(self.odom_data)

    def send_json_line(self, sock, data):
        line = json.dumps(data, ensure_ascii=False) + "\n"
        sock.sendall(line.encode("utf-8"))

    def odom_loop(self):
        rate = rospy.Rate(self.odom_send_hz)

        while not rospy.is_shutdown():
            has_odom, data = self.get_odom_copy()

            if has_odom:
                try:
                    self.send_json_line(self.odom_sock, data)
                except Exception as e:
                    rospy.logwarn("Send odom failed: %s", str(e))
                    self.reconnect_odom()

            rate.sleep()

    def open_lora_serial(self):
        if serial is None:
            rospy.logerr("pyserial not installed. Run: pip3 install pyserial")
            return False

        while not rospy.is_shutdown():
            try:
                self.lora_ser = serial.Serial(
                    self.lora_serial_port,
                    self.lora_serial_baud,
                    timeout=1,
                    dsrdtr=False,
                    rtscts=False
                )
                self.lora_ser.setDTR(False)
                self.lora_ser.setRTS(False)
                time.sleep(2.0)

                rospy.loginfo(
                    "LoRa serial opened: %s, baud=%d",
                    self.lora_serial_port,
                    self.lora_serial_baud
                )
                return True

            except Exception as e:
                rospy.logwarn("Open LoRa serial failed: %s", str(e))
                time.sleep(1.0)

        return False

    def parse_lora_line(self, line):
        match = self.lora_pattern.search(line)
        if not match:
            return None

        data = {
            "type": "lora",
            "stamp": time.time(),
            "rssi": int(match.group("rssi")),
            "snr": float(match.group("snr")),
            "size": int(match.group("size")),
            "msg": match.group("msg")
        }

        count = match.group("count")
        if count is not None:
            data["count"] = int(count)

        return data

    def lora_loop(self):
        if not self.open_lora_serial():
            return

        while not rospy.is_shutdown():
            try:
                raw = self.lora_ser.readline()
                if not raw:
                    continue

                line = raw.decode("utf-8", errors="ignore").strip()
                if not line:
                    continue

                rospy.loginfo("LoRa RAW: %s", line)

                lora_data = self.parse_lora_line(line)
                if lora_data is None:
                    continue

                has_odom, odom = self.get_odom_copy()

                if has_odom:
                    lora_data["x"] = odom["x"]
                    lora_data["y"] = odom["y"]
                    lora_data["z"] = odom["z"]
                    lora_data["yaw"] = odom["yaw"]
                    lora_data["frame_id"] = odom["frame_id"]
                else:
                    lora_data["x"] = None
                    lora_data["y"] = None
                    lora_data["z"] = None
                    lora_data["yaw"] = None
                    lora_data["frame_id"] = "unknown"

                self.send_json_line(self.lora_sock, lora_data)

                rospy.loginfo(
                    "LoRa sent: RSSI=%d, SNR=%.2f, pos=(%s, %s, %s)",
                    lora_data["rssi"],
                    lora_data["snr"],
                    str(lora_data["x"]),
                    str(lora_data["y"]),
                    str(lora_data["z"])
                )

            except Exception as e:
                rospy.logwarn("LoRa loop error: %s", str(e))
                self.reconnect_lora()
                time.sleep(0.5)

    def cloud_callback(self, msg):
        self.frame_count += 1

        if self.frame_count % self.send_every_n != 0:
            return

        points = []

        for i, p in enumerate(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)):
            points.append(p)
            if len(points) >= self.max_points:
                break

        if not points:
            return

        points = np.asarray(points, dtype=np.float32).reshape(-1, 3)
        finite_mask = np.isfinite(points).all(axis=1)
        points = points[finite_mask]

        if points.shape[0] == 0:
            return

        self.send_cloud(points)

    def send_cloud(self, points):
        points = np.ascontiguousarray(points, dtype="<f4")
        n_points = points.shape[0]

        payload = struct.pack("!I", n_points) + points.tobytes()
        packet = struct.pack("!I", len(payload)) + payload

        try:
            self.cloud_sock.sendall(packet)

            rospy.loginfo_throttle(
                2.0,
                "Cloud sent: %d points, %.2f KB",
                n_points,
                len(packet) / 1024.0
            )

        except Exception as e:
            rospy.logwarn("Send cloud failed: %s", str(e))
            self.reconnect_cloud()


if __name__ == "__main__":
    rospy.init_node("jetson_minimal_heatmap_sender", anonymous=False)
    node = JetsonMinimalSender()
    rospy.spin()