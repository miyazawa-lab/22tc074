#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32
import subprocess
import re
import os
import signal
import time

PING_TARGET = "192.168.123.21"
LIDAR_SERIAL_PORT = "/dev/ttyUSB0"


class SlamController(Node):
    def __init__(self):
        super().__init__("slam_controller")

        self.publisher_ = self.create_publisher(String, "slam_mode", 10)
        self.timer = self.create_timer(2.0, self.check_connection)
        self.last_mode = "local"  # default local

        # SLAM品質指標
        self.slam_score = 0.0
        self.create_subscription(Float32, "/slam_quality", self.slam_quality_cb, 10)

        # 動的閾値パラメータ
        self.base_comm_thresh = 100.0
        self.base_slam_thresh = 0.6
        self.alpha = 0.6
        self.beta = 0.4

        # プロセス管理
        self.lidar_proc = None
        self.tf_procs = []
        self.local_slam_proc = None

        # static TF クリア
        self.cleanup_old_static_transforms()
        self.start_lidar()
        self.start_tf_publishers()

        # シグナル
        signal.signal(signal.SIGINT, self._signal_handler)
        signal.signal(signal.SIGTERM, self._signal_handler)
        signal.signal(signal.SIGTSTP, self._signal_handler)

    def slam_quality_cb(self, msg):
        self.slam_score = msg.data

    # ---------------- TF cleanup ----------------
    def cleanup_old_static_transforms(self):
        self.get_logger().info("Cleaning up old static_transform_publisher processes...")
        try:
            proc = subprocess.run(
                ["pgrep", "-f", "static_transform_publisher"],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
            )
            pids = proc.stdout.strip().split()
            for pid in pids:
                if pid.isdigit():
                    os.kill(int(pid), signal.SIGTERM)
            time.sleep(0.5)
        except Exception as e:
            self.get_logger().warn(f"Error cleaning static_transform_publisher: {e}")

    # ---------------- Signal ----------------
    def _signal_handler(self, sig, frame):
        self.get_logger().info(f"Signal {sig} received, shutting down...")
        self._terminate_all_processes()
        rclpy.shutdown()

    def _terminate_all_processes(self):
        procs = [self.lidar_proc] + self.tf_procs + [self.local_slam_proc]
        for p in procs:
            if p:
                try:
                    os.killpg(os.getpgid(p.pid), signal.SIGTERM)
                    p.wait(timeout=5)
                except Exception:
                    pass
        self.lidar_proc = None
        self.tf_procs = []
        self.local_slam_proc = None

    # ---------------- LIDAR ----------------
    def start_lidar(self):
        if self.lidar_proc is None:
            cmd = [
                "ros2", "launch",
                "ldlidar_stl_ros2", "ld19.launch.py",
                f"serial_port:={LIDAR_SERIAL_PORT}"
            ]
            self.lidar_proc = subprocess.Popen(cmd, preexec_fn=os.setsid)
            self.get_logger().info("LD19 LiDAR started")

    # ---------------- TF publisher ----------------
    def start_tf_publishers(self):
        tf_cmds = [
            ["ros2", "run", "tf2_ros", "static_transform_publisher",
             "0", "0", "0", "0", "0", "0", "base_link", "base_laser"],
            ["ros2", "run", "tf2_ros", "static_transform_publisher",
             "0", "0", "0", "0", "0", "0", "base_link", "base_footprint"],
            ["ros2", "run", "tf2_ros", "static_transform_publisher",
             "0", "0", "0", "0", "0", "0", "odom", "base_link"],
            ["ros2", "run", "tf2_ros", "static_transform_publisher",
             "0", "0", "0", "0", "0", "0", "base_footprint", "laser"],
        ]
        for cmd in tf_cmds:
            proc = subprocess.Popen(cmd, preexec_fn=os.setsid)
            self.tf_procs.append(proc)
        self.get_logger().info("TF static publishers started")

    # ---------------- Local SLAM ----------------
    def start_local_slam(self):
        if self.local_slam_proc is None:
            cmd = ["ros2", "launch", "slam_toolbox", "online_async_launch.py"]
            self.local_slam_proc = subprocess.Popen(cmd, preexec_fn=os.setsid)
            self.get_logger().info("Local SLAM started")

    def stop_local_slam(self):
        if self.local_slam_proc:
            try:
                os.killpg(os.getpgid(self.local_slam_proc.pid), signal.SIGTERM)
                self.local_slam_proc.wait(timeout=8)
            except Exception:
                pass
            self.local_slam_proc = None
            self.get_logger().info("Local SLAM stopped")

    # ---------------- Mode Switch ----------------
    def switch_to_server(self):
        msg = String()
        msg.data = "server"
        self.publisher_.publish(msg)
        self.stop_local_slam()
        self.last_mode = "server"
        self.get_logger().info("Switched to SERVER SLAM")

    def switch_to_local(self):
        msg = String()
        msg.data = "local"
        self.publisher_.publish(msg)
        self.start_local_slam()
        self.last_mode = "local"
        self.get_logger().warn("Switched to LOCAL SLAM")

    # ---------------- Connectivity Check + Dynamic Thresholds ----------------
    def check_connection(self):

        # Ping
        try:
            proc = subprocess.run(
                ["ping", "-c", "3", PING_TARGET],
                stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=8
            )
            output = proc.stdout.decode(errors="ignore")
        except subprocess.TimeoutExpired:
            output = ""

        packet_loss = 100
        m_loss = re.search(r"(\d+)% packet loss", output)
        if m_loss:
            packet_loss = int(m_loss.group(1))

        avg_rtt = None
        m_rtt = re.search(r"=\s*([\d\.]+)/([\d\.]+)/", output)
        if m_rtt:
            avg_rtt = float(m_rtt.group(2))

        # ---------- 動的閾値 ----------
        dynamic_comm_thresh = self.base_comm_thresh * (1 + self.alpha * self.slam_score)
        dynamic_slam_thresh = self.base_slam_thresh * (1 + self.beta * (avg_rtt or 200) / 200)

        # ---------- 詳細ログ出力 ----------
        self.get_logger().info(
            "\n===== STATUS REPORT =====\n"
            f"Ping: loss={packet_loss}%  rtt={avg_rtt}\n"
            f"SLAM score: {self.slam_score}\n"
            f"Dynamic_comm_thresh={dynamic_comm_thresh:.2f}\n"
            f"Dynamic_slam_thresh={dynamic_slam_thresh:.2f}\n"
            f"Current mode: {self.last_mode.upper()}\n"
            "=========================="
        )

        # ---------- ヒステリシス制御 ----------
        if self.last_mode == "server":   # cloud → local
            cond_comm_bad = (avg_rtt is None or avg_rtt > dynamic_comm_thresh)
            cond_loss_bad = packet_loss > 30
            cond_slam_bad = self.slam_score > dynamic_slam_thresh

            if cond_comm_bad or cond_loss_bad or cond_slam_bad:
                self.switch_to_local()

        else:  # local → cloud
            cond_comm_good = (avg_rtt is not None and avg_rtt < dynamic_comm_thresh * 0.8)
            cond_loss_good = packet_loss < 10
            cond_slam_good = self.slam_score < dynamic_slam_thresh * 0.7

            if cond_comm_good and cond_loss_good and cond_slam_good:
                self.switch_to_server()

    def destroy_node(self):
        self.get_logger().info("Destroying node...")
        self._terminate_all_processes()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SlamController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("KeyboardInterrupt received")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()