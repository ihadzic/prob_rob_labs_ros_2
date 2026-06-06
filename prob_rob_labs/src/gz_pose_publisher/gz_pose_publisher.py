import subprocess
import threading

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped


class GzPosePublisher(Node):

    def __init__(self):
        super().__init__('gz_pose_publisher')

        self.frame_name = self.declare_parameter(
            'frame_name', 'waffle_pi/base_footprint').value
        self.gz_topic = self.declare_parameter(
            'gz_topic', '/world/default/pose/info').value
        self.output_topic = self.declare_parameter(
            'output_topic', '/tb3/ground_truth/pose').value
        self.reference_frame = self.declare_parameter(
            'reference_frame', 'world').value

        self.pub_pose = self.create_publisher(PoseStamped, self.output_topic, 10)
        self.gz_process = None
        self.reader_thread = None
        self.running = True

        self.get_logger().info(
            f'Subscribing to Gazebo topic {self.gz_topic}, filtering frame '
            f'{self.frame_name}, publishing PoseStamped on {self.output_topic} '
            f'with frame {self.reference_frame}')

        self._start_gz_subscription()

    def _start_gz_subscription(self):
        try:
            self.gz_process = subprocess.Popen(
                ['gz', 'topic', '-e', '-t', self.gz_topic],
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                bufsize=1,
            )
        except FileNotFoundError:
            self.get_logger().error('Failed to start gz CLI. Is Gazebo sourced?')
            return

        self.reader_thread = threading.Thread(target=self._read_gz_output, daemon=True)
        self.reader_thread.start()

    def _matches_frame(self, child_frame_id: str) -> bool:
        if child_frame_id == self.frame_name:
            return True
        if child_frame_id.replace('::', '/') == self.frame_name:
            return True
        if child_frame_id.replace('/', '::') == self.frame_name:
            return True
        if child_frame_id.endswith('::' + self.frame_name):
            return True
        if child_frame_id.endswith('/' + self.frame_name):
            return True
        return False

    def _read_gz_output(self):
        message_stamp = {'sec': 0, 'nsec': 0}
        current_pose = None
        section = None
        in_header = False
        in_stamp = False

        if self.gz_process is None or self.gz_process.stdout is None:
            return

        for raw_line in self.gz_process.stdout:
            if not self.running:
                break

            line = raw_line.strip()
            if not line:
                continue

            if line == 'pose {':
                current_pose = {
                    'name': '',
                    'position': {'x': 0.0, 'y': 0.0, 'z': 0.0},
                    'orientation': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0},
                    'stamp': {'sec': message_stamp['sec'], 'nsec': message_stamp['nsec']},
                }
                section = None
                continue

            if line == 'header {':
                in_header = True
                in_stamp = False
                continue

            if in_header:
                if line == 'stamp {':
                    in_stamp = True
                    continue
                if line == '}':
                    if in_stamp:
                        in_stamp = False
                    else:
                        in_header = False
                    continue
                if in_stamp and ':' in line:
                    key, value = line.split(':', 1)
                    key = key.strip()
                    value = value.strip()
                    if key in ('sec', 'nsec'):
                        try:
                            message_stamp[key] = int(value)
                        except ValueError:
                            pass
                    continue

            if current_pose is None:
                continue

            if line == 'position {':
                section = 'position'
                continue

            if line == 'orientation {':
                section = 'orientation'
                continue

            if line == '}':
                if section is not None:
                    section = None
                else:
                    self._publish_if_match(current_pose)
                    current_pose = None
                continue

            if line.startswith('name:'):
                current_pose['name'] = line.split(':', 1)[1].strip().strip('"')
                continue

            if ':' in line and section in ('position', 'orientation'):
                key, value = line.split(':', 1)
                key = key.strip()
                value = value.strip()
                if key in current_pose[section]:
                    try:
                        current_pose[section][key] = float(value)
                    except ValueError:
                        pass

    def _publish_if_match(self, pose_data):
        if not self._matches_frame(pose_data['name']):
            return

        pose = PoseStamped()
        pose.header.stamp.sec = pose_data['stamp']['sec']
        pose.header.stamp.nanosec = pose_data['stamp']['nsec']
        pose.header.frame_id = self.reference_frame
        pose.pose.position.x = pose_data['position']['x']
        pose.pose.position.y = pose_data['position']['y']
        pose.pose.position.z = pose_data['position']['z']
        pose.pose.orientation.x = pose_data['orientation']['x']
        pose.pose.orientation.y = pose_data['orientation']['y']
        pose.pose.orientation.z = pose_data['orientation']['z']
        pose.pose.orientation.w = pose_data['orientation']['w']
        self.pub_pose.publish(pose)

    def destroy_node(self):
        self.running = False
        if self.gz_process is not None:
            self.gz_process.terminate()
        return super().destroy_node()


def main():
    rclpy.init()
    node = GzPosePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
