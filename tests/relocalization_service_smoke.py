#!/usr/bin/env python3
"""External-node integration checks; runs an isolated server with a synthetic PCD."""
import json
import math
import os
from pathlib import Path
import subprocess
import tempfile
import time

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2, PointField
from tf2_msgs.msg import TFMessage
from std_msgs.msg import Empty
from fast_lio.srv import Relocalize

WORKSPACE = Path(os.environ.get('RELOC_WORKSPACE', '/home/zxz/workspace/localization_ws'))
ARTIFACTS = Path(tempfile.mkdtemp(prefix='reloc-service-check-'))


def transform(x, y, z, yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, -s, 0, x], [s, c, 0, y], [0, 0, 1, z], [0, 0, 0, 1]])


def matrix_from_pose(pose):
    q = pose.orientation
    yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y**2 + q.z**2))
    return transform(pose.position.x, pose.position.y, pose.position.z, yaw)


z = np.linspace(0.05, 1.25, 16)
a = np.linspace(-4, 4, 90)
POINTS = np.array(
    [(4., y, h) for y in a for h in z]
    + [(x, -3.5, h) for x in a for h in z]
    + [(1.3, y, h) for y in np.linspace(-1.5, -0.3, 20) for h in z]
    + [(x, y, .22) for x in np.linspace(-2, 2, 35) for y in np.linspace(-2, 2, 35)], dtype=np.float32)
TRUE_POSE = transform(0.4, -0.2, 0.1, math.radians(8))
ODOM_POSE = transform(1.0, 0.5, -0.05, math.radians(-5))
target = POINTS @ TRUE_POSE[:3, :3].T + TRUE_POSE[:3, 3]
MAP = ARTIFACTS / 'prior.pcd'
with MAP.open('w') as f:
    f.write(f'VERSION .7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n'
            f'WIDTH {len(target)}\nHEIGHT 1\nPOINTS {len(target)}\nDATA ascii\n')
    np.savetxt(f, target, fmt='%.7f')


class Probe(Node):
    def __init__(self):
        super().__init__('reloc_external_probe')
        self.cloud_pub = self.create_publisher(PointCloud2, '/cloud_registered_body', 10)
        self.odom_pub = self.create_publisher(Odometry, '/Odometry', 10)
        self.tf_pub = self.create_publisher(TFMessage, '/tf', 100)
        self.client = self.create_client(Relocalize, '/reloc/relocalize')
        self.corrections = []
        self.sub = self.create_subscription(TFMessage, '/tf', self.tf_callback, 100)
        self.last_publish = 0.0

    def tf_callback(self, msg):
        for tf in msg.transforms:
            if tf.header.frame_id == 'map' and tf.child_frame_id == 'odom':
                self.corrections.append(tf)

    def feed(self, points=POINTS, odom=True, tf=True, cloud=True):
        stamp = self.get_clock().now().to_msg()
        if odom:
            msg = Odometry()
            msg.header.stamp = stamp
            msg.header.frame_id = 'odom'
            msg.child_frame_id = 'base_link'
            msg.pose.pose.position.x = 1.0
            msg.pose.pose.position.y = 0.5
            msg.pose.pose.position.z = -0.05
            msg.pose.pose.orientation.z = math.sin(math.radians(-5) / 2)
            msg.pose.pose.orientation.w = math.cos(math.radians(-5) / 2)
            self.odom_pub.publish(msg)
            if tf:
                t = TransformStamped()
                t.header = msg.header
                t.child_frame_id = msg.child_frame_id
                t.transform.translation.x = msg.pose.pose.position.x
                t.transform.translation.y = msg.pose.pose.position.y
                t.transform.translation.z = msg.pose.pose.position.z
                t.transform.rotation = msg.pose.pose.orientation
                self.tf_pub.publish(TFMessage(transforms=[t]))
        if cloud:
            msg = PointCloud2()
            msg.header.stamp = stamp
            msg.header.frame_id = 'base_link'
            msg.height = 1
            msg.width = len(points)
            msg.fields = [PointField(name=name, offset=i * 4, datatype=PointField.FLOAT32, count=1)
                          for i, name in enumerate(('x', 'y', 'z'))]
            msg.point_step = 12
            msg.row_step = len(points) * 12
            msg.is_dense = True
            msg.data = np.asarray(points, dtype='<f4').tobytes()
            self.cloud_pub.publish(msg)

    def pump(self, seconds, feed=None):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            if feed is not None and time.monotonic() - self.last_publish >= 0.04:
                self.feed(**feed())
                self.last_publish = time.monotonic()
            rclpy.spin_once(self, timeout_sec=0.005)

    def wait(self, future, feed=None, timeout=12):
        deadline = time.monotonic() + timeout
        while not future.done() and time.monotonic() < deadline:
            self.pump(0.01, feed)
        assert future.done(), 'client did not receive bounded response'
        return future.result()

    def request(self, mode=0, **kwargs):
        req = Relocalize.Request()
        req.mode = mode
        req.x, req.y, req.z, req.yaw_deg = 0.4, -0.2, 0.1, 8.0
        for key, value in kwargs.items():
            setattr(req, key, value)
        return self.client.call_async(req)


class Server:
    def __init__(self, label, *extra):
        self.log_path = ARTIFACTS / f'{label}.log'
        self.log = self.log_path.open('w')
        self.process = subprocess.Popen([
            str(WORKSPACE / 'install/fast_lio/lib/fast_lio/icp_relocalizer'),
            '--ros-args', '--params-file',
            str(WORKSPACE / 'install/fast_lio/share/fast_lio/config/reloc/relocalization.yaml'),
            '-p', f'pcd_file:={MAP}', '-p', 'auto_relocalize_on_start:=false', *extra],
            stdout=self.log, stderr=subprocess.STDOUT)

    def ready(self, node):
        end = time.monotonic() + 8
        while time.monotonic() < end:
            node.pump(0.05)
            assert self.process.poll() is None, self.log_path.read_text()
            if node.client.service_is_ready():
                node.pump(0.3)
                return
        raise AssertionError('server not discoverable: ' + self.log_path.read_text())

    def close(self):
        self.process.terminate()
        try:
            self.process.wait(timeout=15)
        except subprocess.TimeoutExpired:
            self.process.kill()
            self.process.wait()
        self.log.close()


def summary(label, result):
    row = dict(case=label, success=result.success, message=result.message,
               frames=result.accumulated_frames, correspondences=result.correspondences,
               rmse=result.rmse, elapsed=result.elapsed_sec)
    RESULTS.append(row)
    print(json.dumps(row), flush=True)


RESULTS = []
rclpy.init()
node = Probe()
server = None
try:
    server = Server('normal')
    server.ready(node)
    result = node.wait(node.request(mode=1))
    assert not result.success and 'odometry' in result.message
    summary('missing_odometry', result)
    result = node.wait(node.request(mode=9))
    assert not result.success and 'mode' in result.message
    summary('invalid_mode', result)
    result = node.wait(node.request(x=float('nan')))
    assert not result.success and 'finite' in result.message
    summary('invalid_manual', result)
    node.pump(0.4, lambda: dict(cloud=False))
    result = node.wait(node.request(), lambda: {})
    assert result.success and result.accumulated_frames == 10
    matched = matrix_from_pose(result.pose.pose)
    assert np.linalg.norm(matched[:3, 3] - TRUE_POSE[:3, 3]) < .04
    assert np.linalg.norm(matched[:3, :3] - TRUE_POSE[:3, :3]) < .03
    correction = transform(result.map_to_odom.transform.translation.x,
                           result.map_to_odom.transform.translation.y,
                           result.map_to_odom.transform.translation.z,
                           2 * math.atan2(result.map_to_odom.transform.rotation.z,
                                          result.map_to_odom.transform.rotation.w))
    assert np.linalg.norm(correction @ ODOM_POSE - matched) < 1e-4
    summary('manual_success', result)
    result = node.wait(node.request(mode=1), lambda: {})
    assert result.success and result.accumulated_frames == 10
    summary('odometry_repeat_nonidentity_correction', result)
    cpp_client = os.environ.get('RELOC_CPP_CLIENT')
    if cpp_client:
        for mode in ('0', '1'):
            process = subprocess.Popen([cpp_client, mode], stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
            deadline = time.monotonic() + 12
            while process.poll() is None and time.monotonic() < deadline:
                node.pump(.02, lambda: {})
            assert process.poll() is not None, 'C++ client timeout'
            output = process.communicate()[0]
            assert process.returncode == 0, output
            print('C++ mode', mode, output.strip(), flush=True)
    old = correction.copy()
    future = node.request()
    node.pump(.1, lambda: dict(cloud=False))
    busy = node.wait(node.request(mode=1), lambda: dict(cloud=False))
    assert not busy.success and 'busy' in busy.message
    summary('concurrent_busy', busy)
    result = node.wait(future, lambda: {})
    assert result.success
    # Existing ten sparse frames are retained; a second full batch supplies new voxels.
    count = [0]
    def extension_feed():
        count[0] += 1
        return dict(points=POINTS[:120] if count[0] <= 11 else POINTS)
    result = node.wait(node.request(), extension_feed)
    assert result.success and result.accumulated_frames == 20
    summary('extension_success', result)
    result = node.wait(node.request(), lambda: dict(points=POINTS[:120]))
    assert not result.success and result.accumulated_frames == 20
    summary('insufficient_after_extension', result)
    result = node.wait(node.request(x=100.0), lambda: {})
    assert not result.success and 'coarse' in result.message
    summary('registration_rejected', result)
    result = node.wait(node.request(), lambda: dict(tf=False))
    assert not result.success and 'reference scan' in result.message
    summary('reference_tf_missing', result)
    node.pump(.7)
    result = node.wait(node.request(mode=1))
    assert not result.success and 'stale' in result.message
    summary('stale_odometry', result)
    # Rejections must not replace the previously accepted correction.
    node.pump(.1)
    latest = node.corrections[-1].transform
    latest_matrix = transform(latest.translation.x, latest.translation.y, latest.translation.z,
                              2 * math.atan2(latest.rotation.z, latest.rotation.w))
    assert np.linalg.norm(latest_matrix - old) < .03
    server.close()
    server = None
    node.pump(.5)
    server = Server('late_result', '-p', 'service_timeout_sec:=0.3',
                    '-p', 'tf_wait_sec:=1.0', '-p', 'accumulate_frames:=1')
    server.ready(node)
    result = node.wait(node.request(), lambda: dict(tf=False))
    assert not result.success and 'timeout' in result.message
    summary('worker_timeout', result)
    result = node.wait(node.request(), lambda: dict(tf=False, cloud=False))
    assert not result.success and 'busy' in result.message
    summary('busy_until_late_worker_exits', result)
    node.pump(1.1, lambda: dict(tf=False, cloud=False))
    assert 'Discarding late registration result' in server.log_path.read_text()
    assert 'ACCEPT:' not in server.log_path.read_text()
    result = node.wait(node.request(), lambda: {})
    assert result.success
    summary('recovery_after_late_result_discard', result)
    server.close()
    server = None
    node.pump(.5)
    server = Server('startup', '-p', 'auto_relocalize_on_start:=true',
                    '-p', 'startup_delay:=0.15', '-p', 'cooldown:=0.1')
    server.ready(node)
    deadline = time.monotonic() + 4
    while 'ACCEPT:' not in server.log_path.read_text() and time.monotonic() < deadline:
        node.pump(.02, lambda: {})
    assert 'ACCEPT:' in server.log_path.read_text(), server.log_path.read_text()
    print('Startup automatic registration: ACCEPT', flush=True)
    count = server.log_path.read_text().count('ACCEPT:')
    node.pump(.4, lambda: {})
    assert server.log_path.read_text().count('ACCEPT:') == count
    legacy = node.create_publisher(Empty, '/reloc/request', 10)
    node.pump(.2, lambda: {})
    legacy.publish(Empty())
    deadline = time.monotonic() + 4
    while server.log_path.read_text().count('ACCEPT:') == count and time.monotonic() < deadline:
        node.pump(.02, lambda: {})
    assert server.log_path.read_text().count('ACCEPT:') == count + 1
    print('Legacy /reloc/request: ACCEPT', flush=True)
    result = node.wait(node.request(mode=1), lambda: {})
    assert result.success
    summary('service_after_startup_and_legacy', result)
    node.destroy_publisher(legacy)
    server.close()
    server = None
    node.pump(.5)
    server = Server('paused_clock_timeout', '-p', 'use_sim_time:=true',
                    '-p', 'service_timeout_sec:=0.6')
    server.ready(node)
    result = node.wait(node.request(), timeout=3)
    assert not result.success and 'timeout' in result.message and .5 < result.elapsed_sec < 1.5
    summary('wall_timeout_with_paused_clock', result)
    assert 'ACCEPT:' not in server.log_path.read_text()
finally:
    if server:
        server.close()
    node.destroy_node()
    rclpy.shutdown()
    (ARTIFACTS / 'results.json').write_text(json.dumps(RESULTS, indent=2))
    print('Artifacts:', ARTIFACTS, flush=True)
