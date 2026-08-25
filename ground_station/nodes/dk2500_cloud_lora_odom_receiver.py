#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import socket
import struct
import time
import json
import threading
import math

import rospy
import numpy as np

from std_msgs.msg import Header, String
from sensor_msgs.msg import PointCloud2, PointField
from nav_msgs.msg import Odometry


class DK2500CloudLoraOdomReceiver:
    def __init__(self):
        self.host = rospy.get_param("~host", "0.0.0.0")

        self.cloud_port = rospy.get_param("~cloud_port", 9000)
        self.lora_port = rospy.get_param("~lora_port", 9001)
        self.odom_port = rospy.get_param("~odom_port", 9002)

        self.cloud_topic = rospy.get_param("~cloud_topic", "/cloud_from_jetson")
        self.lora_topic = rospy.get_param("~lora_topic", "/lora_from_jetson")
        self.odom_topic = rospy.get_param("~odom_topic", "/odom_from_jetson")

        self.frame_id = rospy.get_param("~frame_id", "map")
        self.child_frame_id = rospy.get_param("~child_frame_id", "base_link")

        self.max_payload_size = rospy.get_param(
            "~max_payload_size",
            50 * 1024 * 1024
        )

        self.cloud_pub = rospy.Publisher(
            self.cloud_topic,
            PointCloud2,
            queue_size=1
        )

        self.lora_pub = rospy.Publisher(
            self.lora_topic,
            String,
            queue_size=20
        )

        self.odom_pub = rospy.Publisher(
            self.odom_topic,
            Odometry,
            queue_size=20
        )

        self.cloud_thread = threading.Thread(
            target=self.cloud_server_loop,
            daemon=True
        )

        self.lora_thread = threading.Thread(
            target=self.lora_server_loop,
            daemon=True
        )

        self.odom_thread = threading.Thread(
            target=self.odom_server_loop,
            daemon=True
        )

    def start(self):
        rospy.loginfo("DK2500 receiver started.")
        rospy.loginfo("PointCloud listening: %s:%d -> %s", self.host, self.cloud_port, self.cloud_topic)
        rospy.loginfo("LoRa listening      : %s:%d -> %s", self.host, self.lora_port, self.lora_topic)
        rospy.loginfo("Odom listening      : %s:%d -> %s", self.host, self.odom_port, self.odom_topic)
        rospy.loginfo("Frame ID            : %s", self.frame_id)

        self.cloud_thread.start()
        self.lora_thread.start()
        self.odom_thread.start()

        while not rospy.is_shutdown():
            time.sleep(0.5)

    def create_server_socket(self, port):
        server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server.bind((self.host, port))
        server.listen(1)
        server.settimeout(1.0)
        return server

    def recvall(self, conn, n):
        data = bytearray()

        while len(data) < n and not rospy.is_shutdown():
            try:
                packet = conn.recv(n - len(data))
            except socket.timeout:
                return None
            except Exception:
                return None

            if not packet:
                return None

            data.extend(packet)

        return bytes(data)

    def numpy_xyz_to_pointcloud2(self, points):
        points = np.asarray(points, dtype=np.float32).reshape(-1, 3)
        points = np.ascontiguousarray(points, dtype=np.float32)

        msg = PointCloud2()
        msg.header = Header()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.frame_id

        msg.height = 1
        msg.width = points.shape[0]

        msg.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]

        msg.is_bigendian = False
        msg.point_step = 12
        msg.row_step = msg.point_step * points.shape[0]
        msg.is_dense = True
        msg.data = points.tobytes()

        return msg

    def cloud_server_loop(self):
        server = self.create_server_socket(self.cloud_port)
        rospy.loginfo("PointCloud server ready on port %d", self.cloud_port)

        while not rospy.is_shutdown():
            conn = None
            try:
                conn, addr = server.accept()
                conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                conn.settimeout(5.0)

                rospy.loginfo("Jetson pointcloud connected: %s", str(addr))
                self.handle_cloud_client(conn, addr)

            except socket.timeout:
                continue
            except Exception as e:
                rospy.logwarn("PointCloud server error: %s", str(e))
            finally:
                if conn is not None:
                    try:
                        conn.close()
                    except Exception:
                        pass

        server.close()

    def handle_cloud_client(self, conn, addr):
        frame_count = 0
        last_stat_time = time.time()
        last_frame_count = 0

        while not rospy.is_shutdown():
            header = self.recvall(conn, 4)
            if header is None:
                rospy.logwarn("PointCloud connection lost: %s", str(addr))
                break

            payload_len = struct.unpack("!I", header)[0]

            if payload_len <= 4 or payload_len > self.max_payload_size:
                rospy.logwarn("Invalid pointcloud payload_len: %d", payload_len)
                break

            payload = self.recvall(conn, payload_len)
            if payload is None:
                rospy.logwarn("PointCloud payload lost: %s", str(addr))
                break

            n_points = struct.unpack("!I", payload[:4])[0]
            raw_points = payload[4:]
            expected_bytes = n_points * 3 * 4

            if len(raw_points) != expected_bytes:
                rospy.logwarn(
                    "Bad pointcloud frame: n=%d, expected=%d, received=%d",
                    n_points,
                    expected_bytes,
                    len(raw_points)
                )
                continue

            if n_points == 0:
                continue

            points = np.frombuffer(raw_points, dtype="<f4").reshape(n_points, 3)

            cloud_msg = self.numpy_xyz_to_pointcloud2(points)
            self.cloud_pub.publish(cloud_msg)

            frame_count += 1

            now = time.time()
            if now - last_stat_time >= 2.0:
                fps = (frame_count - last_frame_count) / (now - last_stat_time)
                rospy.loginfo(
                    "Received cloud: %d points, FPS: %.2f, topic: %s",
                    n_points,
                    fps,
                    self.cloud_topic
                )
                last_stat_time = now
                last_frame_count = frame_count

    def lora_server_loop(self):
        server = self.create_server_socket(self.lora_port)
        rospy.loginfo("LoRa server ready on port %d", self.lora_port)

        while not rospy.is_shutdown():
            conn = None
            try:
                conn, addr = server.accept()
                conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                conn.settimeout(5.0)

                rospy.loginfo("Jetson LoRa connected: %s", str(addr))
                self.handle_lora_client(conn, addr)

            except socket.timeout:
                continue
            except Exception as e:
                rospy.logwarn("LoRa server error: %s", str(e))
            finally:
                if conn is not None:
                    try:
                        conn.close()
                    except Exception:
                        pass

        server.close()

    def handle_lora_client(self, conn, addr):
        buffer = ""

        while not rospy.is_shutdown():
            try:
                data = conn.recv(2048)
            except socket.timeout:
                continue
            except Exception:
                rospy.logwarn("LoRa connection error: %s", str(addr))
                break

            if not data:
                rospy.logwarn("LoRa connection lost: %s", str(addr))
                break

            buffer += data.decode("utf-8", errors="ignore")

            while "\n" in buffer:
                line, buffer = buffer.split("\n", 1)
                line = line.strip()

                if not line:
                    continue

                self.publish_lora_line(line)

    def publish_lora_line(self, line):
        out = String()
        out.data = line
        self.lora_pub.publish(out)

        try:
            data = json.loads(line)

            rospy.loginfo(
                "LoRa RX: RSSI=%s dBm, SNR=%s dB, pos=(%s,%s,%s), MSG=%s",
                str(data.get("rssi", "")),
                str(data.get("snr", "")),
                str(data.get("x", "")),
                str(data.get("y", "")),
                str(data.get("z", "")),
                str(data.get("msg", ""))
            )

        except Exception:
            rospy.loginfo("LoRa RAW: %s", line)

    def odom_server_loop(self):
        server = self.create_server_socket(self.odom_port)
        rospy.loginfo("Odom server ready on port %d", self.odom_port)

        while not rospy.is_shutdown():
            conn = None
            try:
                conn, addr = server.accept()
                conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                conn.settimeout(5.0)

                rospy.loginfo("Jetson odom connected: %s", str(addr))
                self.handle_odom_client(conn, addr)

            except socket.timeout:
                continue
            except Exception as e:
                rospy.logwarn("Odom server error: %s", str(e))
            finally:
                if conn is not None:
                    try:
                        conn.close()
                    except Exception:
                        pass

        server.close()

    def handle_odom_client(self, conn, addr):
        buffer = ""
        recv_count = 0
        last_stat_time = time.time()
        last_recv_count = 0

        while not rospy.is_shutdown():
            try:
                data = conn.recv(2048)
            except socket.timeout:
                continue
            except Exception:
                rospy.logwarn("Odom connection error: %s", str(addr))
                break

            if not data:
                rospy.logwarn("Odom connection lost: %s", str(addr))
                break

            buffer += data.decode("utf-8", errors="ignore")

            while "\n" in buffer:
                line, buffer = buffer.split("\n", 1)
                line = line.strip()

                if not line:
                    continue

                ok = self.publish_odom_line(line)
                if ok:
                    recv_count += 1

            now = time.time()
            if now - last_stat_time >= 2.0:
                hz = (recv_count - last_recv_count) / (now - last_stat_time)
                rospy.loginfo(
                    "Received odom: %.2f Hz, topic: %s",
                    hz,
                    self.odom_topic
                )
                last_stat_time = now
                last_recv_count = recv_count

    def yaw_to_quaternion(self, yaw):
        qz = math.sin(yaw * 0.5)
        qw = math.cos(yaw * 0.5)
        return qz, qw

    def publish_odom_line(self, line):
        try:
            data = json.loads(line)

            msg = Odometry()

            stamp = data.get("stamp", None)
            if stamp is not None:
                try:
                    msg.header.stamp = rospy.Time.from_sec(float(stamp))
                except Exception:
                    msg.header.stamp = rospy.Time.now()
            else:
                msg.header.stamp = rospy.Time.now()

            msg.header.frame_id = data.get("frame_id", self.frame_id)
            msg.child_frame_id = data.get("child_frame_id", self.child_frame_id)

            msg.pose.pose.position.x = float(data.get("x", 0.0))
            msg.pose.pose.position.y = float(data.get("y", 0.0))
            msg.pose.pose.position.z = float(data.get("z", 0.0))

            yaw = float(data.get("yaw", 0.0))
            qz, qw = self.yaw_to_quaternion(yaw)

            msg.pose.pose.orientation.x = 0.0
            msg.pose.pose.orientation.y = 0.0
            msg.pose.pose.orientation.z = qz
            msg.pose.pose.orientation.w = qw

            msg.twist.twist.linear.x = float(data.get("vx", 0.0))
            msg.twist.twist.linear.y = float(data.get("vy", 0.0))
            msg.twist.twist.linear.z = float(data.get("vz", 0.0))

            self.odom_pub.publish(msg)

            rospy.loginfo_throttle(
                2.0,
                "Odom RX: x=%.2f y=%.2f z=%.2f yaw=%.2f",
                msg.pose.pose.position.x,
                msg.pose.pose.position.y,
                msg.pose.pose.position.z,
                yaw
            )

            return True

        except Exception as e:
            rospy.logwarn("Bad odom json: %s, line=%s", str(e), line)
            return False


if __name__ == "__main__":
    rospy.init_node("dk2500_cloud_lora_odom_receiver", anonymous=False)

    receiver = DK2500CloudLoraOdomReceiver()
    receiver.start()