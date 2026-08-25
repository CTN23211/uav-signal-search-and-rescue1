#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
uav_mission_supervisor.py

Jetson-side mission supervisor for DK2500 route/command integration.

Functions:
  1. Read fox_waypoints.txt.
  2. Use the first fox waypoint as HOME.
  3. Receive DK2500 JSON command packets / route packets from ROS topics or TCP.
  4. Decide when to switch mission states.
  5. Keep EGO Planner unchanged: only publish PoseStamped goals to EGO,
     or stop publishing new goals to keep the UAV hovering.
  6. Do not perform auto-landing. Landing is left to RC / PX4 / fox_controller.
"""

import os
import re
import json
import math
import time
import traceback
import socket
import threading
import queue
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Tuple

import rospy
from std_msgs.msg import String, Bool
from geometry_msgs.msg import PoseStamped, Quaternion
from nav_msgs.msg import Odometry

try:
    from mavros_msgs.msg import State as MavrosState
    HAS_MAVROS_STATE = True
except Exception:
    MavrosState = None
    HAS_MAVROS_STATE = False


@dataclass
class MissionGoal:
    seq: int
    x: float
    y: float
    z: float
    yaw: float = 0.0
    hold: float = 2.0
    name: str = ""
    source: str = ""

    def to_dict(self) -> Dict[str, Any]:
        return {
            "seq": self.seq,
            "x": round(float(self.x), 4),
            "y": round(float(self.y), 4),
            "z": round(float(self.z), 4),
            "yaw": round(float(self.yaw), 4),
            "hold": round(float(self.hold), 3),
            "name": self.name,
            "source": self.source,
        }


@dataclass
class MissionRoute:
    route_id: str
    route_type: str
    frame_id: str
    goals: List[MissionGoal] = field(default_factory=list)
    preempt: bool = False
    mode: str = ""
    priority: int = 50
    raw: Dict[str, Any] = field(default_factory=dict)

    def to_brief(self) -> Dict[str, Any]:
        return {
            "route_id": self.route_id,
            "route_type": self.route_type,
            "frame_id": self.frame_id,
            "goal_count": len(self.goals),
            "preempt": self.preempt,
            "mode": self.mode,
            "priority": self.priority,
        }


def normalize_yaw(yaw: float) -> float:
    """Accept yaw in radians. If it looks like degrees, convert to radians."""
    yaw = float(yaw)
    if abs(yaw) > 2.0 * math.pi + 0.1:
        yaw = math.radians(yaw)
    while yaw > math.pi:
        yaw -= 2.0 * math.pi
    while yaw < -math.pi:
        yaw += 2.0 * math.pi
    return yaw


def yaw_to_quat(yaw: float) -> Quaternion:
    yaw = normalize_yaw(yaw)
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


def quat_to_yaw(q: Quaternion) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def now_sec() -> float:
    try:
        return rospy.Time.now().to_sec()
    except Exception:
        return time.time()


def extract_floats_from_line(line: str) -> List[float]:
    """
    Robust parser for fox_waypoints.txt.
    Supports:
      1.0 2.0 1.5
      1.0, 2.0, 1.5, 0.0
      wp1: x=1.0 y=2.0 z=1.5 yaw=90 hold=2
    """
    line = line.split("#", 1)[0].strip()
    if not line:
        return []
    nums = re.findall(r"[-+]?(?:\d+\.\d*|\.\d+|\d+)(?:[eE][-+]?\d+)?", line)
    return [float(x) for x in nums]


class UAVMissionSupervisor:
    BOOT_LOAD_FILE = "BOOT_LOAD_FILE"
    FILE_ROUTE = "FILE_ROUTE"
    WAIT_DK_ROUTE = "WAIT_DK_ROUTE"
    EXEC_DK_MAIN = "EXEC_DK_MAIN"
    GOAL_HOVER_SAMPLE = "GOAL_HOVER_SAMPLE"
    EXEC_LOCAL_SEARCH = "EXEC_LOCAL_SEARCH"
    TARGET_CONFIRM = "TARGET_CONFIRM"
    RETURN_HOME = "RETURN_HOME"
    HOLD = "HOLD"
    HOLD_WAIT_RC_LAND = "HOLD_WAIT_RC_LAND"
    RC_OVERRIDE = "RC_OVERRIDE"

    EXEC_STATES = {FILE_ROUTE, EXEC_DK_MAIN, EXEC_LOCAL_SEARCH, RETURN_HOME}

    def __init__(self) -> None:
        rospy.init_node("uav_mission_supervisor", anonymous=False)

        self.frame_id = rospy.get_param("~frame_id", "map")
        self.output_frame_id = rospy.get_param("~output_frame_id", self.frame_id)

        self.waypoints_file = rospy.get_param(
            "~waypoints_file",
            "",
        )

        self.goal_topic = rospy.get_param("~goal_topic", "/move_base_simple/goal")
        self.odom_topic = rospy.get_param("~odom_topic", "/mavros/local_position/odom")
        self.dk_route_topic = rospy.get_param("~dk_route_topic", "/dk_route_cmd")
        self.dk_mission_cmd_topic = rospy.get_param("~dk_mission_cmd_topic", "/dk_mission_cmd")
        self.external_goal_reached_topic = rospy.get_param("~external_goal_reached_topic", "/uav_goal_reached")

        # Optional TCP server for receiving DK2500 JSON packets without ROS multi-machine.
        # Protocol: newline-delimited UTF-8 JSON. One line = one route/command packet.
        self.enable_tcp_server = bool(rospy.get_param("~enable_tcp_server", False))
        self.tcp_bind_host = str(rospy.get_param("~tcp_bind_host", "0.0.0.0"))
        self.tcp_port = int(rospy.get_param("~tcp_port", 9100))
        self.tcp_recv_buffer_size = int(rospy.get_param("~tcp_recv_buffer_size", 4096))
        self.tcp_max_line_bytes = int(rospy.get_param("~tcp_max_line_bytes", 262144))
        self.tcp_queue_size = int(rospy.get_param("~tcp_queue_size", 100))
        self.tcp_max_packets_per_tick = int(rospy.get_param("~tcp_max_packets_per_tick", 20))

        self.status_topic = rospy.get_param("~status_topic", "/uav_exec_status")
        self.route_ack_topic = rospy.get_param("~route_ack_topic", "/uav_route_ack")
        self.current_goal_topic = rospy.get_param("~current_goal_topic", "/uav_current_goal")

        self.auto_start_file_route = bool(rospy.get_param("~auto_start_file_route", True))
        self.auto_start_dk_route = bool(rospy.get_param("~auto_start_dk_route", True))
        self.allow_midflight_preempt = bool(rospy.get_param("~allow_midflight_preempt", False))

        self.default_hold_sec = float(rospy.get_param("~default_hold_sec", 2.0))
        self.local_finish_to_target_confirm = bool(rospy.get_param("~local_finish_to_target_confirm", True))

        self.safe_z_min = float(rospy.get_param("~safe_z_min", 0.2))
        self.safe_z_max = float(rospy.get_param("~safe_z_max", 5.0))
        self.route_frame_must_match = bool(rospy.get_param("~route_frame_must_match", False))

        self.goal_xy_tolerance = float(rospy.get_param("~goal_xy_tolerance", 0.35))
        self.goal_z_tolerance = float(rospy.get_param("~goal_z_tolerance", 0.35))
        self.goal_reached_stable_sec = float(rospy.get_param("~goal_reached_stable_sec", 0.35))
        self.min_goal_active_sec = float(rospy.get_param("~min_goal_active_sec", 0.8))

        self.return_altitude = float(rospy.get_param("~return_altitude", 2.0))
        self.home_use_return_altitude = bool(rospy.get_param("~home_use_return_altitude", True))
        self.return_climb_first = bool(rospy.get_param("~return_climb_first", True))

        self.hold_publish_current_goal = bool(rospy.get_param("~hold_publish_current_goal", False))
        self.hold_goal_republish_sec = float(rospy.get_param("~hold_goal_republish_sec", 0.0))

        self.status_pub_rate = float(rospy.get_param("~status_pub_rate", 2.0))
        self.control_rate = float(rospy.get_param("~control_rate", 10.0))

        self.enable_rc_override_detection = bool(rospy.get_param("~enable_rc_override_detection", True))
        self.expected_flight_mode = str(rospy.get_param("~expected_flight_mode", "OFFBOARD"))
        self.detect_disarmed_as_rc_override = bool(rospy.get_param("~detect_disarmed_as_rc_override", False))

        self.goal_pub = rospy.Publisher(self.goal_topic, PoseStamped, queue_size=5)
        self.status_pub = rospy.Publisher(self.status_topic, String, queue_size=10)
        self.ack_pub = rospy.Publisher(self.route_ack_topic, String, queue_size=10)
        self.current_goal_pub = rospy.Publisher(self.current_goal_topic, String, queue_size=10)

        rospy.Subscriber(self.odom_topic, Odometry, self.odom_callback, queue_size=20)
        rospy.Subscriber(self.dk_route_topic, String, self.dk_route_callback, queue_size=20)
        rospy.Subscriber(self.dk_mission_cmd_topic, String, self.dk_mission_cmd_callback, queue_size=20)
        rospy.Subscriber(self.external_goal_reached_topic, Bool, self.external_goal_reached_callback, queue_size=10)

        if HAS_MAVROS_STATE:
            rospy.Subscriber("/mavros/state", MavrosState, self.mavros_state_callback, queue_size=10)

        self.state = self.BOOT_LOAD_FILE
        self.previous_exec_state = ""
        self.hold_resume_state = ""

        self.file_route: Optional[MissionRoute] = None
        self.active_route: Optional[MissionRoute] = None
        self.pending_route: Optional[MissionRoute] = None
        self.pending_main_route: Optional[MissionRoute] = None

        self.current_idx = 0
        self.current_goal: Optional[MissionGoal] = None
        self.current_goal_sent = False
        self.current_goal_sent_time = 0.0

        self.hover_start_time = 0.0
        self.hover_goal: Optional[MissionGoal] = None

        self.current_pose: Optional[Tuple[float, float, float, float]] = None
        self.last_goal_distance_ok_since: Optional[float] = None

        self.home_goal: Optional[MissionGoal] = None
        self.home_locked = False

        self.last_status_time = 0.0
        self.last_hold_goal_pub_time = 0.0
        self.last_event = "node_started"

        self.mavros_mode = ""
        self.mavros_armed = False

        self.tcp_packet_queue = queue.Queue(maxsize=max(self.tcp_queue_size, 1))
        self.tcp_server_socket = None
        self.tcp_server_thread = None
        self.tcp_running = False
        self.tcp_clients_total = 0
        self.tcp_packets_received = 0
        self.tcp_packets_dropped = 0
        self.tcp_last_peer = ""
        self.tcp_last_error = ""

        if self.enable_tcp_server:
            self.start_tcp_server()

        self.load_file_route_and_home()

        if self.file_route and self.file_route.goals:
            self.active_route = self.file_route
            self.current_idx = 0
            if self.auto_start_file_route:
                self.transition_to(self.FILE_ROUTE, "auto_start_file_route")
            else:
                self.transition_to(self.WAIT_DK_ROUTE, "file_loaded_wait_dk_or_start")
        else:
            self.transition_to(self.WAIT_DK_ROUTE, "no_file_route_wait_dk")

        rospy.Timer(rospy.Duration(1.0 / max(self.control_rate, 1.0)), self.timer_callback)
        rospy.loginfo("[mission_supervisor] started. state=%s, goal_topic=%s, file=%s",
                      self.state, self.goal_topic, self.waypoints_file)

    def start_tcp_server(self) -> None:
        """Start a small TCP server for DK2500 route/command packets."""
        if self.tcp_running:
            return

        self.tcp_running = True
        self.tcp_server_thread = threading.Thread(
            target=self.tcp_server_loop,
            name="uav_mission_supervisor_tcp_server",
            daemon=True,
        )
        self.tcp_server_thread.start()
        rospy.on_shutdown(self.stop_tcp_server)
        rospy.loginfo(
            "[mission_supervisor] TCP server enabled: %s:%d",
            self.tcp_bind_host,
            self.tcp_port,
        )

    def stop_tcp_server(self) -> None:
        self.tcp_running = False
        try:
            if self.tcp_server_socket is not None:
                self.tcp_server_socket.close()
        except Exception:
            pass
        self.tcp_server_socket = None

    def tcp_server_loop(self) -> None:
        try:
            srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            srv.bind((self.tcp_bind_host, self.tcp_port))
            srv.listen(5)
            srv.settimeout(0.5)
            self.tcp_server_socket = srv
        except Exception as e:
            self.tcp_last_error = f"tcp_bind_failed: {e}"
            rospy.logerr("[mission_supervisor] TCP bind/listen failed: %s", str(e))
            self.tcp_running = False
            return

        while self.tcp_running and not rospy.is_shutdown():
            try:
                conn, addr = self.tcp_server_socket.accept()
            except socket.timeout:
                continue
            except OSError:
                break
            except Exception as e:
                self.tcp_last_error = f"tcp_accept_error: {e}"
                rospy.logwarn("[mission_supervisor] TCP accept error: %s", str(e))
                continue

            self.tcp_clients_total += 1
            peer = f"{addr[0]}:{addr[1]}"
            self.tcp_last_peer = peer
            rospy.loginfo("[mission_supervisor] TCP client connected: %s", peer)

            th = threading.Thread(
                target=self.tcp_client_loop,
                args=(conn, peer),
                name=f"uav_mission_supervisor_tcp_client_{peer}",
                daemon=True,
            )
            th.start()

    def tcp_client_loop(self, conn: socket.socket, peer: str) -> None:
        """
        Receive newline-delimited JSON.
        Also accepts one final JSON packet when the client closes without a trailing newline.
        """
        buffer = b""
        try:
            conn.settimeout(1.0)
            while self.tcp_running and not rospy.is_shutdown():
                try:
                    data = conn.recv(max(self.tcp_recv_buffer_size, 256))
                except socket.timeout:
                    continue

                if not data:
                    break

                buffer += data
                if len(buffer) > self.tcp_max_line_bytes:
                    self.tcp_packets_dropped += 1
                    self.tcp_last_error = f"tcp_packet_too_large_from_{peer}"
                    try:
                        conn.sendall(b"ERROR packet_too_large\n")
                    except Exception:
                        pass
                    buffer = b""
                    continue

                while b"\n" in buffer:
                    line, buffer = buffer.split(b"\n", 1)
                    self.enqueue_tcp_packet_bytes(line, peer, conn)

            if buffer.strip():
                self.enqueue_tcp_packet_bytes(buffer, peer, conn)

        except Exception as e:
            self.tcp_last_error = f"tcp_client_error_from_{peer}: {e}"
            rospy.logwarn("[mission_supervisor] TCP client error from %s: %s", peer, str(e))
        finally:
            try:
                conn.close()
            except Exception:
                pass
            rospy.loginfo("[mission_supervisor] TCP client disconnected: %s", peer)

    def enqueue_tcp_packet_bytes(self, raw: bytes, peer: str, conn: Optional[socket.socket] = None) -> bool:
        raw = raw.strip()
        if not raw:
            return False

        try:
            text = raw.decode("utf-8")
        except UnicodeDecodeError:
            self.tcp_packets_dropped += 1
            self.tcp_last_error = f"tcp_decode_error_from_{peer}"
            if conn is not None:
                try:
                    conn.sendall(b"ERROR invalid_utf8\n")
                except Exception:
                    pass
            return False

        try:
            self.tcp_packet_queue.put_nowait((text, peer))
        except queue.Full:
            self.tcp_packets_dropped += 1
            self.tcp_last_error = f"tcp_queue_full_from_{peer}"
            rospy.logwarn("[mission_supervisor] TCP queue full. Dropped packet from %s", peer)
            if conn is not None:
                try:
                    conn.sendall(b"ERROR queue_full\n")
                except Exception:
                    pass
            return False

        self.tcp_packets_received += 1
        if conn is not None:
            try:
                conn.sendall(b"OK queued\n")
            except Exception:
                pass
        return True

    def process_tcp_queue(self) -> None:
        if not self.enable_tcp_server:
            return

        max_count = max(int(self.tcp_max_packets_per_tick), 1)
        count = 0
        while count < max_count:
            try:
                text, peer = self.tcp_packet_queue.get_nowait()
            except queue.Empty:
                break

            source = f"tcp://{peer}"
            rospy.loginfo("[mission_supervisor] TCP packet from %s: %s", peer, text[:200])
            self.handle_json_packet(text, source_topic=source)
            count += 1

    def load_file_route_and_home(self) -> None:
        goals: List[MissionGoal] = []

        if not os.path.exists(self.waypoints_file):
            rospy.logwarn("[mission_supervisor] waypoints_file not found: %s", self.waypoints_file)
            self.file_route = None
            return

        try:
            with open(self.waypoints_file, "r", encoding="utf-8") as f:
                lines = f.readlines()

            seq = 1
            for raw in lines:
                nums = extract_floats_from_line(raw)
                if len(nums) < 3:
                    continue

                x, y, z = nums[0], nums[1], nums[2]
                yaw = normalize_yaw(nums[3]) if len(nums) >= 4 else 0.0
                hold = float(nums[4]) if len(nums) >= 5 else self.default_hold_sec

                if z < self.safe_z_min or z > self.safe_z_max:
                    rospy.logwarn(
                        "[mission_supervisor] skip file waypoint seq=%d, unsafe z=%.3f, line=%s",
                        seq, z, raw.strip()
                    )
                    continue

                goals.append(MissionGoal(
                    seq=seq, x=x, y=y, z=z, yaw=yaw, hold=hold,
                    name=f"file_{seq:02d}", source="fox_waypoints"
                ))
                seq += 1

            self.file_route = MissionRoute(
                route_id="fox_file_route",
                route_type="FILE_ROUTE",
                frame_id=self.frame_id,
                goals=goals,
                preempt=False,
                mode="FOX_WAYPOINTS",
                priority=10,
                raw={"source": self.waypoints_file},
            )

            if goals:
                first = goals[0]
                home_z = self.return_altitude if self.home_use_return_altitude else first.z
                self.home_goal = MissionGoal(
                    seq=0,
                    x=first.x,
                    y=first.y,
                    z=home_z,
                    yaw=first.yaw,
                    hold=self.default_hold_sec,
                    name="HOME_FROM_FIRST_FOX_WAYPOINT",
                    source="fox_waypoints_first_goal",
                )
                self.home_locked = True
                rospy.loginfo(
                    "[mission_supervisor] loaded %d file goals. HOME=(%.3f, %.3f, %.3f)",
                    len(goals), self.home_goal.x, self.home_goal.y, self.home_goal.z
                )
            else:
                rospy.logwarn("[mission_supervisor] no valid waypoints parsed from: %s", self.waypoints_file)

        except Exception as e:
            rospy.logerr("[mission_supervisor] failed to load waypoints file: %s", str(e))
            rospy.logerr(traceback.format_exc())
            self.file_route = None

    def odom_callback(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = quat_to_yaw(q)
        self.current_pose = (float(p.x), float(p.y), float(p.z), float(yaw))

    def mavros_state_callback(self, msg: Any) -> None:
        self.mavros_mode = str(getattr(msg, "mode", ""))
        self.mavros_armed = bool(getattr(msg, "armed", False))

        if not self.enable_rc_override_detection:
            return

        if self.state in [self.WAIT_DK_ROUTE, self.HOLD_WAIT_RC_LAND, self.RC_OVERRIDE, self.BOOT_LOAD_FILE]:
            return

        if self.expected_flight_mode and self.mavros_mode and self.mavros_mode != self.expected_flight_mode:
            self.enter_rc_override(f"mavros_mode_changed_to_{self.mavros_mode}")
            return

        if self.detect_disarmed_as_rc_override and not self.mavros_armed:
            self.enter_rc_override("mavros_disarmed")

    def external_goal_reached_callback(self, msg: Bool) -> None:
        if bool(msg.data):
            self.handle_goal_reached("external_goal_reached_topic")

    def dk_route_callback(self, msg: String) -> None:
        self.handle_json_packet(msg.data, source_topic=self.dk_route_topic)

    def dk_mission_cmd_callback(self, msg: String) -> None:
        self.handle_json_packet(msg.data, source_topic=self.dk_mission_cmd_topic)

    def handle_json_packet(self, text: str, source_topic: str = "") -> None:
        text = text.strip()
        if not text:
            return

        try:
            pkt = json.loads(text)
        except Exception as e:
            rospy.logwarn("[mission_supervisor] invalid JSON from %s: %s | %s", source_topic, str(e), text[:200])
            self.publish_ack("", False, "invalid_json", {"source_topic": source_topic})
            return

        if isinstance(pkt, list):
            pkt = {"cmd": "ROUTE_UPDATE", "route_type": "MAIN_ROUTE", "goals": pkt}

        if not isinstance(pkt, dict):
            self.publish_ack("", False, "packet_not_dict", {"source_topic": source_topic})
            return

        cmd = str(pkt.get("cmd", pkt.get("event", ""))).upper().strip()

        if "goals" in pkt and (cmd in ["", "ROUTE", "ROUTE_UPDATE", "MAIN_ROUTE", "LOCAL_SEARCH_ROUTE"]):
            route = self.parse_route_packet(pkt)
            if route is None:
                return
            self.on_new_route(route)
            return

        self.on_command_packet(pkt)

    def parse_route_packet(self, pkt: Dict[str, Any]) -> Optional[MissionRoute]:
        route_type = str(pkt.get("route_type", pkt.get("type", pkt.get("cmd", "MAIN_ROUTE")))).upper()

        if route_type in ["ROUTE_UPDATE", "ROUTE"]:
            route_type = str(pkt.get("mode", "MAIN_ROUTE")).upper()

        if route_type in ["MAIN", "DK_MAIN", "ORDERED_GOAL_ARRAY"]:
            route_type = "MAIN_ROUTE"
        elif route_type in ["LOCAL", "LOCAL_ROUTE", "LAWNMOWER", "REFINE_SEARCH"]:
            route_type = "LOCAL_SEARCH_ROUTE"
        elif route_type not in ["MAIN_ROUTE", "LOCAL_SEARCH_ROUTE", "BACKUP_ROUTE"]:
            route_type = "MAIN_ROUTE"

        route_id = str(pkt.get("route_id", f"{route_type.lower()}_{int(now_sec())}"))
        frame_id = str(pkt.get("frame_id", self.frame_id))
        preempt = bool(pkt.get("preempt", False))
        mode = str(pkt.get("mode", ""))
        priority = int(pkt.get("priority", 50))

        raw_goals = pkt.get("goals", [])
        if not isinstance(raw_goals, list):
            self.publish_ack(route_id, False, "goals_not_list", {"route_type": route_type})
            return None

        goals: List[MissionGoal] = []
        for i, g in enumerate(raw_goals):
            try:
                goal = self.parse_goal_item(g, i + 1, route_type)
            except Exception as e:
                self.publish_ack(route_id, False, f"bad_goal_{i+1}: {str(e)}", {"route_type": route_type})
                return None

            if goal.z < self.safe_z_min or goal.z > self.safe_z_max:
                self.publish_ack(route_id, False, "unsafe_goal_z", {
                    "seq": goal.seq,
                    "z": goal.z,
                    "safe_z_min": self.safe_z_min,
                    "safe_z_max": self.safe_z_max,
                })
                return None

            goals.append(goal)

        route = MissionRoute(
            route_id=route_id,
            route_type=route_type,
            frame_id=frame_id,
            goals=goals,
            preempt=preempt,
            mode=mode,
            priority=priority,
            raw=pkt,
        )

        if not self.validate_route(route):
            return None

        return route

    def parse_goal_item(self, g: Any, default_seq: int, route_type: str) -> MissionGoal:
        if isinstance(g, dict):
            seq = int(g.get("seq", default_seq))
            x = float(g["x"])
            y = float(g["y"])
            z = float(g["z"])
            yaw = normalize_yaw(float(g.get("yaw", 0.0)))
            hold = float(g.get("hold", g.get("hold_sec", self.default_hold_sec)))
            name = str(g.get("name", f"{route_type.lower()}_{seq:02d}"))
            return MissionGoal(seq=seq, x=x, y=y, z=z, yaw=yaw, hold=hold, name=name, source="dk2500")

        if isinstance(g, (list, tuple)):
            if len(g) < 3:
                raise ValueError("goal list must have at least x,y,z")
            seq = default_seq
            x, y, z = float(g[0]), float(g[1]), float(g[2])
            yaw = normalize_yaw(float(g[3])) if len(g) >= 4 else 0.0
            hold = float(g[4]) if len(g) >= 5 else self.default_hold_sec
            return MissionGoal(seq=seq, x=x, y=y, z=z, yaw=yaw, hold=hold,
                               name=f"{route_type.lower()}_{seq:02d}", source="dk2500")

        raise ValueError("goal item must be dict or list")

    def validate_route(self, route: MissionRoute) -> bool:
        if not route.goals:
            self.publish_ack(route.route_id, False, "empty_route", route.to_brief())
            return False

        if self.route_frame_must_match and route.frame_id != self.frame_id:
            self.publish_ack(route.route_id, False, "frame_mismatch", {
                "route_frame": route.frame_id,
                "expected_frame": self.frame_id,
            })
            return False

        return True

    def on_command_packet(self, pkt: Dict[str, Any]) -> None:
        cmd = str(pkt.get("cmd", pkt.get("event", ""))).upper().strip()

        if not cmd:
            self.publish_ack("", False, "missing_cmd", pkt)
            return

        if cmd in ["START_ROUTE", "START", "RESUME"]:
            self.on_start_or_resume(cmd, pkt)
            return

        if cmd in ["PAUSE", "HOLD"]:
            self.enter_hold(reason=cmd.lower())
            self.publish_ack("", True, f"{cmd.lower()}_accepted", {"state": self.state})
            return

        if cmd in ["CANCEL", "STOP_TASK", "MISSION_FINISHED"]:
            self.pending_route = None
            self.pending_main_route = None
            self.enter_hold_wait_rc_land(reason=cmd.lower())
            self.publish_ack("", True, f"{cmd.lower()}_accepted", {"state": self.state})
            return

        if cmd in ["CLEAR_ROUTE", "CLEAR_ROUTES"]:
            self.clear_dk_routes()
            self.publish_ack("", True, "routes_cleared", {"state": self.state})
            return

        if cmd in ["RETURN_HOME", "RTH", "GO_HOME"]:
            self.start_return_home(reason=str(pkt.get("reason", cmd.lower())))
            self.publish_ack("", True, "return_home_started", {"state": self.state})
            return

        if cmd in ["USE_FILE_ROUTE", "START_FILE_ROUTE"]:
            if self.file_route and self.file_route.goals:
                self.activate_route(self.file_route, "use_file_route_cmd")
                self.publish_ack(self.file_route.route_id, True, "file_route_activated", self.file_route.to_brief())
            else:
                self.publish_ack("", False, "no_file_route_available", {})
            return

        if cmd in ["TARGET_FOUND", "TARGET_CONFIRM", "CONFIRM_TARGET"]:
            # Keep the semantic state as TARGET_CONFIRM. Do not enter LANDING;
            # landing is handled by RC / PX4 / fox_controller.
            self.transition_to(self.TARGET_CONFIRM, "target_confirm_cmd")
            self.publish_hold_goal_if_needed(force=True)
            self.publish_ack("", True, "target_confirmed_hold", {"state": self.state})
            return

        if cmd in ["RC_OVERRIDE"]:
            self.enter_rc_override("cmd_rc_override")
            self.publish_ack("", True, "rc_override_set", {"state": self.state})
            return

        self.publish_ack("", False, f"unknown_cmd_{cmd}", pkt)

    def on_new_route(self, route: MissionRoute) -> None:
        rospy.loginfo("[mission_supervisor] new route: %s", route.to_brief())

        if self.state in [self.RETURN_HOME, self.TARGET_CONFIRM, self.RC_OVERRIDE]:
            self.publish_ack(route.route_id, False, f"state_not_accepting_route_{self.state}", route.to_brief())
            return

        if self.state == self.WAIT_DK_ROUTE:
            if self.auto_start_dk_route:
                self.activate_route(route, "new_route_in_wait")
                self.publish_ack(route.route_id, True, "activate_now", route.to_brief())
            else:
                self.active_route = route
                self.current_idx = 0
                self.current_goal_sent = False
                self.publish_ack(route.route_id, True, "stored_wait_start", route.to_brief())
            return

        if self.state in [self.HOLD, self.HOLD_WAIT_RC_LAND]:
            if bool(route.raw.get("start_immediately", self.auto_start_dk_route)):
                self.activate_route(route, "new_route_in_hold_start")
                self.publish_ack(route.route_id, True, "activate_now_from_hold", route.to_brief())
            else:
                self.active_route = route
                self.current_idx = 0
                self.current_goal_sent = False
                self.publish_ack(route.route_id, True, "stored_wait_start_or_resume", route.to_brief())
            return

        if self.state == self.GOAL_HOVER_SAMPLE:
            self.activate_route(route, "new_route_in_hover")
            self.publish_ack(route.route_id, True, "activate_now_in_hover", route.to_brief())
            return

        if self.state in [self.FILE_ROUTE, self.EXEC_DK_MAIN]:
            if route.preempt and self.allow_midflight_preempt:
                self.activate_route(route, "midflight_preempt_enabled")
                self.publish_ack(route.route_id, True, "midflight_preempt_activate_now", route.to_brief())
            else:
                self.pending_route = route
                self.publish_ack(route.route_id, True, "activate_after_current_goal", route.to_brief())
            return

        if self.state == self.EXEC_LOCAL_SEARCH:
            if route.route_type == "LOCAL_SEARCH_ROUTE":
                self.pending_route = route
                self.publish_ack(route.route_id, True, "activate_after_current_local_goal", route.to_brief())
            elif route.preempt and self.allow_midflight_preempt:
                self.activate_route(route, "preempt_local_search")
                self.publish_ack(route.route_id, True, "preempt_local_search_now", route.to_brief())
            else:
                self.pending_main_route = route
                self.publish_ack(route.route_id, True, "cached_main_route_after_local_search", route.to_brief())
            return

        self.pending_route = route
        self.publish_ack(route.route_id, True, f"stored_in_state_{self.state}", route.to_brief())

    def activate_route(self, route: MissionRoute, reason: str = "") -> None:
        self.active_route = route
        self.pending_route = None
        self.current_idx = 0
        self.current_goal = None
        self.current_goal_sent = False
        self.last_goal_distance_ok_since = None
        self.transition_to(self.route_to_exec_state(route), reason or f"activate_{route.route_id}")

    def route_to_exec_state(self, route: MissionRoute) -> str:
        if route.route_type == "LOCAL_SEARCH_ROUTE":
            return self.EXEC_LOCAL_SEARCH
        if route.route_type == "FILE_ROUTE":
            return self.FILE_ROUTE
        if route.route_type == "RETURN_HOME":
            return self.RETURN_HOME
        return self.EXEC_DK_MAIN

    def clear_dk_routes(self) -> None:
        self.pending_route = None
        self.pending_main_route = None
        if self.active_route and self.active_route.route_type != "FILE_ROUTE":
            self.active_route = None
            self.current_idx = 0
            self.current_goal = None
            self.current_goal_sent = False
            self.transition_to(self.WAIT_DK_ROUTE, "clear_dk_routes")

    def on_start_or_resume(self, cmd: str, pkt: Dict[str, Any]) -> None:
        if self.pending_route is not None:
            route = self.pending_route
            self.activate_route(route, f"{cmd.lower()}_pending_route")
            self.publish_ack(route.route_id, True, f"{cmd.lower()}_activated_pending", route.to_brief())
            return

        if self.active_route is not None:
            state = self.route_to_exec_state(self.active_route)
            self.transition_to(state, f"{cmd.lower()}_active_route")
            self.current_goal_sent = False
            self.publish_ack(self.active_route.route_id, True, f"{cmd.lower()}_active_route", self.active_route.to_brief())
            return

        if self.file_route and self.file_route.goals:
            self.activate_route(self.file_route, f"{cmd.lower()}_file_route")
            self.publish_ack(self.file_route.route_id, True, f"{cmd.lower()}_file_route", self.file_route.to_brief())
            return

        self.publish_ack("", False, f"{cmd.lower()}_failed_no_route", {"state": self.state})

    def enter_hold(self, reason: str = "hold") -> None:
        if self.state in self.EXEC_STATES:
            self.hold_resume_state = self.state
        self.transition_to(self.HOLD, reason)
        self.publish_hold_goal_if_needed(force=True)

    def enter_hold_wait_rc_land(self, reason: str = "wait_rc_land") -> None:
        self.transition_to(self.HOLD_WAIT_RC_LAND, reason)
        self.publish_hold_goal_if_needed(force=True)

    def enter_rc_override(self, reason: str = "rc_override") -> None:
        if self.state != self.RC_OVERRIDE:
            self.transition_to(self.RC_OVERRIDE, reason)
            self.current_goal_sent = False
            self.pending_route = None
            rospy.logwarn("[mission_supervisor] RC_OVERRIDE: %s", reason)

    def start_return_home(self, reason: str = "return_home") -> None:
        if self.home_goal is None:
            if self.current_pose is not None:
                x, y, z, yaw = self.current_pose
                self.home_goal = MissionGoal(
                    seq=0, x=x, y=y, z=self.return_altitude,
                    yaw=yaw, hold=self.default_hold_sec,
                    name="HOME_FROM_CURRENT_ODOM_FALLBACK",
                    source="current_odom_fallback",
                )
                self.home_locked = True
            else:
                self.publish_ack("", False, "return_home_failed_no_home_no_odom", {})
                return

        self.pending_route = None
        self.pending_main_route = None
        goals: List[MissionGoal] = []

        if self.return_climb_first and self.current_pose is not None:
            cx, cy, cz, cyaw = self.current_pose
            climb_z = max(float(cz), float(self.return_altitude))
            climb_z = min(max(climb_z, self.safe_z_min), self.safe_z_max)
            goals.append(MissionGoal(
                seq=1, x=cx, y=cy, z=climb_z, yaw=cyaw,
                hold=0.5, name="RETURN_CLIMB", source="return_home"
            ))

        hz = float(self.home_goal.z)
        if self.home_use_return_altitude:
            hz = float(self.return_altitude)
        hz = min(max(hz, self.safe_z_min), self.safe_z_max)

        goals.append(MissionGoal(
            seq=len(goals) + 1,
            x=self.home_goal.x,
            y=self.home_goal.y,
            z=hz,
            yaw=self.home_goal.yaw,
            hold=self.default_hold_sec,
            name="RETURN_HOME",
            source="return_home",
        ))

        route = MissionRoute(
            route_id=f"return_home_{int(now_sec())}",
            route_type="RETURN_HOME",
            frame_id=self.frame_id,
            goals=goals,
            preempt=True,
            mode="RETURN_HOME",
            priority=100,
            raw={"reason": reason},
        )

        self.active_route = route
        self.current_idx = 0
        self.current_goal = None
        self.current_goal_sent = False
        self.transition_to(self.RETURN_HOME, reason)

    def timer_callback(self, _event: Any) -> None:
        try:
            self.process_tcp_queue()
            self.run_state_machine_once()
            self.publish_status_periodic()
        except Exception as e:
            rospy.logerr("[mission_supervisor] timer error: %s", str(e))
            rospy.logerr(traceback.format_exc())

    def run_state_machine_once(self) -> None:
        if self.state in self.EXEC_STATES:
            if self.active_route is not None and not self.current_goal_sent:
                if 0 <= self.current_idx < len(self.active_route.goals):
                    self.publish_goal(self.active_route.goals[self.current_idx])
                else:
                    self.finish_active_route("idx_out_of_range")
            self.check_goal_reached_by_odom()
            return

        if self.state == self.GOAL_HOVER_SAMPLE:
            self.process_hover_sample_state()
            return

        if self.state in [self.HOLD, self.HOLD_WAIT_RC_LAND, self.TARGET_CONFIRM]:
            if self.hold_goal_republish_sec > 0.0:
                if now_sec() - self.last_hold_goal_pub_time >= self.hold_goal_republish_sec:
                    self.publish_hold_goal_if_needed(force=True)
            return

    def publish_goal(self, goal: MissionGoal) -> None:
        msg = PoseStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.output_frame_id
        msg.pose.position.x = float(goal.x)
        msg.pose.position.y = float(goal.y)
        msg.pose.position.z = float(goal.z)
        msg.pose.orientation = yaw_to_quat(goal.yaw)

        self.goal_pub.publish(msg)
        self.current_goal = goal
        self.current_goal_sent = True
        self.current_goal_sent_time = now_sec()
        self.last_goal_distance_ok_since = None
        self.last_event = f"publish_goal_{goal.name or goal.seq}"

        self.current_goal_pub.publish(String(data=json.dumps({
            "event": "current_goal",
            "state": self.state,
            "route_id": self.active_route.route_id if self.active_route else "",
            "route_type": self.active_route.route_type if self.active_route else "",
            "current_idx": self.current_idx,
            "goal": goal.to_dict(),
        }, ensure_ascii=False)))

        rospy.loginfo("[mission_supervisor] GO_TO %s route=%s idx=%d goal=%s",
                      self.state,
                      self.active_route.route_id if self.active_route else "",
                      self.current_idx,
                      goal.to_dict())

    def publish_hold_goal_if_needed(self, force: bool = False) -> None:
        if not self.hold_publish_current_goal:
            return

        t = now_sec()
        if not force and self.hold_goal_republish_sec > 0.0:
            if t - self.last_hold_goal_pub_time < self.hold_goal_republish_sec:
                return

        if self.current_pose is not None:
            x, y, z, yaw = self.current_pose
            goal = MissionGoal(seq=-1, x=x, y=y, z=z, yaw=yaw,
                               hold=self.default_hold_sec, name="HOLD_CURRENT", source="current_odom")
        elif self.current_goal is not None:
            goal = MissionGoal(seq=-1, x=self.current_goal.x, y=self.current_goal.y, z=self.current_goal.z,
                               yaw=self.current_goal.yaw, hold=self.default_hold_sec,
                               name="HOLD_LAST_GOAL", source="last_goal")
        else:
            return

        msg = PoseStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.output_frame_id
        msg.pose.position.x = goal.x
        msg.pose.position.y = goal.y
        msg.pose.position.z = goal.z
        msg.pose.orientation = yaw_to_quat(goal.yaw)
        self.goal_pub.publish(msg)
        self.last_hold_goal_pub_time = t
        self.last_event = f"publish_hold_goal_{goal.name}"

    def check_goal_reached_by_odom(self) -> None:
        if self.current_pose is None or self.current_goal is None:
            return

        if now_sec() - self.current_goal_sent_time < self.min_goal_active_sec:
            return

        cx, cy, cz, _ = self.current_pose
        g = self.current_goal
        dxy = math.sqrt((cx - g.x) ** 2 + (cy - g.y) ** 2)
        dz = abs(cz - g.z)

        if dxy <= self.goal_xy_tolerance and dz <= self.goal_z_tolerance:
            if self.last_goal_distance_ok_since is None:
                self.last_goal_distance_ok_since = now_sec()
            elif now_sec() - self.last_goal_distance_ok_since >= self.goal_reached_stable_sec:
                self.handle_goal_reached("odom_distance")
        else:
            self.last_goal_distance_ok_since = None

    def handle_goal_reached(self, source: str = "unknown") -> None:
        if self.state not in self.EXEC_STATES:
            return
        if not self.current_goal_sent:
            return

        rospy.loginfo("[mission_supervisor] goal reached. state=%s idx=%d source=%s",
                      self.state, self.current_idx, source)
        self.last_event = f"goal_reached_by_{source}"

        if self.state == self.RETURN_HOME:
            self.current_goal_sent = False
            self.current_goal = None
            self.last_goal_distance_ok_since = None
            self.current_idx += 1
            if self.active_route and self.current_idx < len(self.active_route.goals):
                self.last_event = "return_home_next_stage"
                return
            self.finish_active_route("return_home_completed")
            return

        self.previous_exec_state = self.state
        self.hover_start_time = now_sec()
        self.hover_goal = self.current_goal
        self.current_goal_sent = False
        self.current_goal = None
        self.last_goal_distance_ok_since = None
        self.transition_to(self.GOAL_HOVER_SAMPLE, f"goal_reached_{source}")

    def process_hover_sample_state(self) -> None:
        hold_sec = self.default_hold_sec
        if self.hover_goal is not None:
            hold_sec = max(0.0, float(self.hover_goal.hold))

        if now_sec() - self.hover_start_time < hold_sec:
            return

        if self.pending_route is not None:
            route = self.pending_route
            self.activate_route(route, "activate_pending_after_hover")
            return

        if self.active_route is not None:
            self.current_idx += 1
            if self.current_idx < len(self.active_route.goals):
                self.transition_to(self.route_to_exec_state(self.active_route), "hover_done_next_goal")
                self.current_goal_sent = False
                return

        self.finish_active_route("route_goals_completed")

    def finish_active_route(self, reason: str = "finished") -> None:
        route = self.active_route
        brief = route.to_brief() if route else {}
        rospy.loginfo("[mission_supervisor] route finished: %s reason=%s", brief, reason)

        self.current_goal = None
        self.current_goal_sent = False
        self.current_idx = 0
        self.last_goal_distance_ok_since = None

        if route and route.route_type == "RETURN_HOME":
            self.active_route = None
            self.enter_hold_wait_rc_land("home_reached_wait_rc_land")
            self.publish_ack(route.route_id, True, "home_reached_wait_rc_land", brief)
            return

        if route and route.route_type == "LOCAL_SEARCH_ROUTE":
            if self.local_finish_to_target_confirm:
                self.transition_to(self.TARGET_CONFIRM, "local_search_completed")
                self.publish_hold_goal_if_needed(force=True)
                self.publish_ack(route.route_id, True, "local_search_completed_target_confirm", brief)
            else:
                self.enter_hold_wait_rc_land("local_search_completed_wait_rc_land")
                self.publish_ack(route.route_id, True, "local_search_completed_wait_rc_land", brief)
            return

        if route:
            self.publish_ack(route.route_id, True, "route_finished_wait_rc_land", brief)

        self.active_route = None
        self.enter_hold_wait_rc_land("route_finished_wait_rc_land")

    def transition_to(self, new_state: str, reason: str = "") -> None:
        old = self.state
        self.state = new_state
        self.last_event = reason or f"{old}_to_{new_state}"
        rospy.loginfo("[mission_supervisor] STATE %s -> %s | %s", old, new_state, self.last_event)

    def publish_ack(self, route_id: str, accepted: bool, action_or_reason: str, extra: Dict[str, Any]) -> None:
        msg = {
            "event": "route_ack",
            "stamp": round(now_sec(), 3),
            "route_id": route_id,
            "accepted": bool(accepted),
            "action_or_reason": action_or_reason,
            "state": self.state,
            "extra": extra,
        }
        self.ack_pub.publish(String(data=json.dumps(msg, ensure_ascii=False)))

    def publish_status_periodic(self) -> None:
        if self.status_pub_rate <= 0.0:
            return
        t = now_sec()
        if t - self.last_status_time < 1.0 / self.status_pub_rate:
            return
        self.last_status_time = t
        self.status_pub.publish(String(data=json.dumps(self.build_status(), ensure_ascii=False)))

    def build_status(self) -> Dict[str, Any]:
        pose = None
        if self.current_pose is not None:
            x, y, z, yaw = self.current_pose
            pose = {"x": round(x, 4), "y": round(y, 4), "z": round(z, 4), "yaw": round(yaw, 4)}

        return {
            "event": "uav_exec_status",
            "stamp": round(now_sec(), 3),
            "state": self.state,
            "last_event": self.last_event,
            "active_route": self.active_route.to_brief() if self.active_route else None,
            "pending_route": self.pending_route.to_brief() if self.pending_route else None,
            "pending_main_route": self.pending_main_route.to_brief() if self.pending_main_route else None,
            "current_idx": self.current_idx,
            "current_goal_sent": self.current_goal_sent,
            "current_goal": self.current_goal.to_dict() if self.current_goal else None,
            "hover_goal": self.hover_goal.to_dict() if self.hover_goal else None,
            "home": self.home_goal.to_dict() if self.home_goal else None,
            "home_locked": self.home_locked,
            "current_pose": pose,
            "mavros": {
                "available": HAS_MAVROS_STATE,
                "mode": self.mavros_mode,
                "armed": self.mavros_armed,
            },
            "topics": {
                "goal_topic": self.goal_topic,
                "dk_route_topic": self.dk_route_topic,
                "dk_mission_cmd_topic": self.dk_mission_cmd_topic,
                "odom_topic": self.odom_topic,
            },
            "tcp": {
                "enabled": self.enable_tcp_server,
                "bind_host": self.tcp_bind_host,
                "port": self.tcp_port,
                "queue_size": self.tcp_packet_queue.qsize() if self.enable_tcp_server else 0,
                "clients_total": self.tcp_clients_total,
                "packets_received": self.tcp_packets_received,
                "packets_dropped": self.tcp_packets_dropped,
                "last_peer": self.tcp_last_peer,
                "last_error": self.tcp_last_error,
            }
        }


def main() -> None:
    UAVMissionSupervisor()
    rospy.spin()


if __name__ == "__main__":
    main()
