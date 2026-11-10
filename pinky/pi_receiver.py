import os
import signal
import subprocess
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class PiReceiverNode(Node):
    def __init__(self):
        super().__init__('pi_receiver_node')
        self.lcd_process = None
        self.bringup_process = None
        self.led_server_process = None
        self.nav_process = None
        self.cam_process = None

        # LCD 경로 설정
        self.lcd_dir = '/home/pinky/pinky_lcd/example'
        self.lcd_script = 'catpopo.py'

        # 카메라 스트림 경로 설정
        self.cam_dir = '/home/pinky/camera_stream'
        self.cam_script = 'app.py'

        # 맵 경로 설정
        #self.map_path = '/home/pinky/pinky_map1.yaml' #class room map
        self.map_path = '/home/pinky/police_map.yaml'

        # 토픽 구독자 등록
        self.lcd_sub = self.create_subscription(String, '/lcd_cmd', self.lcd_callback, 10)
        self.bringup_sub = self.create_subscription(String, '/bringup_cmd', self.bringup_callback, 10)
        self.nav_sub = self.create_subscription(String, '/nav_cmd', self.nav_callback, 10)
        self.cam_sub = self.create_subscription(String, '/cam_cmd', self.cam_callback, 10)

        self.get_logger().info('라즈베리파이 통합 수신기(LCD + Bringup + LED + Nav2 + Camera) 대기 중...')

    # --- 카메라 제어 로직 (~/camera_stream/app.py) ---
    def cam_callback(self, msg):
        if msg.data == 'on':
            self.start_cam()
        elif msg.data == 'off':
            self.stop_cam()

    def start_cam(self):
        if self.cam_process is None or self.cam_process.poll() is not None:
            try:
                self.cam_process = subprocess.Popen(
                    ['python3', self.cam_script],
                    cwd=self.cam_dir,
                    preexec_fn=os.setsid
                )
                self.get_logger().info('camera_stream/app.py 실행됨!')
            except Exception as e:
                self.get_logger().error(f'카메라 실행 실패: {e}')

    def stop_cam(self):
        if self.cam_process is not None and self.cam_process.poll() is None:
            try:
                os.killpg(os.getpgid(self.cam_process.pid), signal.SIGINT)
                self.cam_process = None
                self.get_logger().info('camera_stream/app.py 종료됨!')
            except Exception as e:
                self.get_logger().error(f'카메라 종료 실패: {e}')

    # --- LCD 제어 로직 ---
    def lcd_callback(self, msg):
        if msg.data == 'on':
            self.start_lcd()
        elif msg.data == 'off':
            self.stop_lcd()

    def start_lcd(self):
        if self.lcd_process is None or self.lcd_process.poll() is not None:
            try:
                self.lcd_process = subprocess.Popen(
                    ['python3', self.lcd_script],
                    cwd=self.lcd_dir,
                    preexec_fn=os.setsid
                )
                self.get_logger().info('catpopo.py LCD 실행됨!')
            except Exception as e:
                self.get_logger().error(f'LCD 실행 실패: {e}')

    def stop_lcd(self):
        if self.lcd_process is not None and self.lcd_process.poll() is None:
            try:
                os.killpg(os.getpgid(self.lcd_process.pid), signal.SIGTERM)
                self.lcd_process = None
                self.get_logger().info('catpopo.py LCD 종료됨!')
            except Exception as e:
                self.get_logger().error(f'LCD 종료 실패: {e}')

    # --- Bringup + LED Server 제어 로직 ---
    def bringup_callback(self, msg):
        if msg.data == 'on':
            self.start_bringup()
        elif msg.data == 'off':
            self.stop_bringup()

    def start_bringup(self):
        if self.bringup_process is None or self.bringup_process.poll() is not None:
            try:
                cmd_bringup = 'ros2 launch pinky_bringup bringup_robot.launch.xml'
                self.bringup_process = subprocess.Popen(
                    cmd_bringup,
                    shell=True,
                    executable='/bin/bash',
                    preexec_fn=os.setsid
                )
                self.get_logger().info('pinky_bringup 실행됨!')
            except Exception as e:
                self.get_logger().error(f'Bringup 실행 실패: {e}')

        if self.led_server_process is None or self.led_server_process.poll() is not None:
            try:
                cmd_led = 'ros2 run pinky_led led_server'
                self.led_server_process = subprocess.Popen(
                    cmd_led,
                    shell=True,
                    executable='/bin/bash',
                    preexec_fn=os.setsid
                )
                self.get_logger().info('pinky_led led_server 실행됨!')
            except Exception as e:
                self.get_logger().error(f'LED Server 실행 실패: {e}')

    def stop_bringup(self):
        if self.bringup_process is not None and self.bringup_process.poll() is None:
            try:
                os.killpg(os.getpgid(self.bringup_process.pid), signal.SIGINT)
                self.bringup_process = None
                self.get_logger().info('pinky_bringup 종료됨!')
            except Exception as e:
                self.get_logger().error(f'Bringup 종료 실패: {e}')

        if self.led_server_process is not None and self.led_server_process.poll() is None:
            try:
                os.killpg(os.getpgid(self.led_server_process.pid), signal.SIGINT)
                self.led_server_process = None
                self.get_logger().info('pinky_led led_server 종료됨!')
            except Exception as e:
                self.get_logger().error(f'LED Server 종료 실패: {e}')

    # --- Navigation 제어 로직 ---
    def nav_callback(self, msg):
        if msg.data == 'on':
            self.start_nav()
        elif msg.data == 'off':
            self.stop_nav()

    def start_nav(self):
        if self.nav_process is None or self.nav_process.poll() is not None:
            try:
                cmd_nav = f'ros2 launch pinky_navigation bringup_launch.xml map:={self.map_path}'
                self.nav_process = subprocess.Popen(
                    cmd_nav,
                    shell=True,
                    executable='/bin/bash',
                    preexec_fn=os.setsid
                )
                self.get_logger().info(f'Navigation 실행됨! (맵: {self.map_path})')
            except Exception as e:
                self.get_logger().error(f'Navigation 실행 실패: {e}')

    def stop_nav(self):
        if self.nav_process is not None and self.nav_process.poll() is None:
            try:
                os.killpg(os.getpgid(self.nav_process.pid), signal.SIGINT)
                self.nav_process = None
                self.get_logger().info('Navigation 종료됨!')
            except Exception as e:
                self.get_logger().error(f'Navigation 종료 실패: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = PiReceiverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_lcd()
        node.stop_cam()
        node.stop_nav()
        node.stop_bringup()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
