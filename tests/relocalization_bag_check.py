#!/usr/bin/env python3
"""Observe real FAST-LIO replay and call both public service modes."""
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from tf2_msgs.msg import TFMessage
from fast_lio.srv import Relocalize

ROOT = Path('/home/zxz/workspace/localization_ws')
ARTIFACTS = Path(tempfile.mkdtemp(prefix='reloc-real-bag-check-'))
processes = []
files = []


def start(label, args):
    stream = (ARTIFACTS / f'{label}.log').open('w')
    files.append(stream)
    process = subprocess.Popen(args, stdout=stream, stderr=subprocess.STDOUT, start_new_session=True)
    processes.append(process)
    return process


class Observer(Node):
    def __init__(self):
        super().__init__('real_bag_reloc_observer', parameter_overrides=[
            rclpy.parameter.Parameter('use_sim_time', value=True)])
        self.odom = []
        self.hf_count = 0
        self.poses = []
        self.corrections = []
        self.reset_messages = 0
        self.subs = [
            self.create_subscription(Odometry, '/Odometry', self.on_odom, qos_profile_sensor_data),
            self.create_subscription(Odometry, '/OdometryHighFreq', self.on_hf, qos_profile_sensor_data),
            self.create_subscription(PoseStamped, '/reloc/pose', self.on_pose, 10),
            self.create_subscription(PoseStamped, '/reloc/cloud_align', self.on_reset, 10),
            self.create_subscription(TFMessage, '/tf', self.on_tf, 100)]
        self.client = self.create_client(Relocalize, '/reloc/relocalize')

    def on_odom(self, msg):
        self.odom.append(dict(t=msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
                              wall=time.monotonic(),
                              p=[msg.pose.pose.position.x, msg.pose.pose.position.y,
                                 msg.pose.pose.position.z]))

    def on_hf(self, msg):
        self.hf_count += 1

    def on_pose(self, msg):
        self.poses.append(msg)

    def on_reset(self, msg):
        self.reset_messages += 1

    def on_tf(self, msg):
        for tf in msg.transforms:
            if tf.header.frame_id == 'map' and tf.child_frame_id == 'odom':
                self.corrections.append(tf)

    def pump(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=.02)

    def call(self, mode):
        request = Relocalize.Request()
        request.mode = mode
        if mode == 0:
            assert self.poses, 'no successful pose for a manual seed'
            pose = self.poses[-1].pose
            request.x, request.y, request.z = pose.position.x, pose.position.y, pose.position.z
            q = pose.orientation
            request.yaw_deg = math.degrees(math.atan2(
                2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))
        before = len(self.odom)
        started = time.monotonic()
        future = self.client.call_async(request)
        while not future.done() and time.monotonic() - started < 13:
            self.pump(.02)
        assert future.done(), 'real bag service client timeout'
        response = future.result()
        row = dict(mode=mode, success=response.success, message=response.message,
                   frames=response.accumulated_frames, correspondences=response.correspondences,
                   rmse=response.rmse, overlap=response.overlap_ratio, elapsed=response.elapsed_sec,
                   odometry_messages_during_call=len(self.odom) - before)
        print(json.dumps(row), flush=True)
        return row


rclpy.init()
node = Observer()
results = {}
try:
    launch = start('launch', ['ros2', 'launch', 'fast_lio', 'reloc_mid360.launch.py',
                             'use_sim_time:=true', 'use_rviz:=false', 'start_map:=false'])
    end = time.monotonic() + 15
    while not node.client.service_is_ready() and time.monotonic() < end:
        node.pump(.1)
        assert launch.poll() is None, 'launch exited before service discovery'
    assert node.client.service_is_ready(), 'service unavailable'
    player = start('player', ['ros2', 'bag', 'play', '/home/zxz/rosbag_home/test1',
                             '--clock', '--disable-keyboard-controls'])
    end = time.monotonic() + 16
    while not node.poses and time.monotonic() < end:
        node.pump(.1)
    results['startup_pose_count'] = len(node.poses)
    print('Startup poses:', len(node.poses), 'odom:', len(node.odom), flush=True)
    results['odometry_service'] = node.call(1)
    if node.poses:
        results['manual_service'] = node.call(0)
    node.pump(.5)
    results['odometry_messages'] = len(node.odom)
    results['high_frequency_messages'] = node.hf_count
    results['reset_pose_messages'] = node.reset_messages
    results['map_to_odom_messages'] = len(node.corrections)
    if len(node.odom) > 1:
        positions = np.array([row['p'] for row in node.odom])
        times = np.array([row['t'] for row in node.odom])
        results['span_sec'] = float(times[-1] - times[0])
        results['max_adjacent_position_change_m'] = float(np.linalg.norm(np.diff(positions, axis=0), axis=1).max())
        results['max_odometry_gap_sec'] = float(np.diff(times).max())
        results['positions_finite'] = bool(np.isfinite(positions).all())
    assert node.reset_messages == 0
    assert len(node.odom) > 30 and node.hf_count > 100
    assert results['startup_pose_count'] >= 1
    assert results['odometry_service']['success'] and results['manual_service']['success']
    assert results['max_adjacent_position_change_m'] < .05
    assert results['max_odometry_gap_sec'] < .15
    results['checks_passed'] = True
finally:
    for process in reversed(processes):
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait()
    for stream in files:
        stream.close()
    results['frontend_reset_logs'] = 'Reloc received' in (ARTIFACTS / 'launch.log').read_text(errors='replace')
    (ARTIFACTS / 'results.json').write_text(json.dumps(results, indent=2))
    (ARTIFACTS / 'odom.json').write_text(json.dumps(node.odom))
    print(json.dumps(results, indent=2), flush=True)
    print('Artifacts:', ARTIFACTS, flush=True)
    node.destroy_node()
    rclpy.shutdown()
