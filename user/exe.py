import os
import sys
import time
import signal
import subprocess
from datetime import datetime
from enum import Enum

import cv2
import requests
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import String
from pinky_interfaces.srv import SetLed

from PyQt6.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QHBoxLayout,
    QPushButton, QLabel, QTextEdit
)
from PyQt6.QtCore import QTimer, Qt, pyqtSignal, QObject, QThread
from PyQt6.QtGui import QFont, QTextCursor, QImage, QPixmap


class PatrolState(Enum):
    IDLE = 1
    PATROL = 2
    ALERT = 3


class GuiEmitter(QObject):
    log_signal = pyqtSignal(str)
    image_signal = pyqtSignal(QImage)


class VideoStreamThread(QThread):
    """라즈베리파이 HTTP MJPEG 스트림 수신 및 다중 규격 ArUco 마커 인식 스레드"""
    frame_received = pyqtSignal(QImage)
    log_message = pyqtSignal(str)

    def __init__(self, url):
        super().__init__()
        self.url = url
        self.running = False
        self.last_detected_time = {}

        # 자주 쓰이는 모든 ArUco 딕셔너리 등록
        self.dict_list = [
            ("4x4", cv2.aruco.DICT_4X4_250),
            ("5x5", cv2.aruco.DICT_5X5_250),
            ("6x6", cv2.aruco.DICT_6X6_250),
            ("ORIG", cv2.aruco.DICT_ARUCO_ORIGINAL),
        ]
        
        self.detectors = []
        self.use_new_api = hasattr(cv2.aruco, 'ArucoDetector')
        
        for name, dict_id in self.dict_list:
            aruco_dict = cv2.aruco.getPredefinedDictionary(dict_id)
            if self.use_new_api:
                params = cv2.aruco.DetectorParameters()
                detector = cv2.aruco.ArucoDetector(aruco_dict, params)
                self.detectors.append((name, detector))
            else:
                params = cv2.aruco.DetectorParameters_create()
                self.detectors.append((name, (aruco_dict, params)))

    def detect_aruco(self, frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        now = time.time()

        for dict_name, detector_obj in self.detectors:
            if self.use_new_api:
                corners, ids, _ = detector_obj.detectMarkers(gray)
            else:
                aruco_dict, params = detector_obj
                corners, ids, _ = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=params)

            if ids is not None and len(ids) > 0:
                cv2.aruco.drawDetectedMarkers(frame, corners, ids)

                for i, marker_id in enumerate(ids.flatten()):
                    # corners[i]의 형태는 (1, 4, 2)이므로 좌상단 좌표 추출
                    pts = corners[i].reshape((4, 2))
                    top_left = (int(pts[0][0]), int(pts[0][1]))

                    cv2.putText(
                        frame,
                        f"[{dict_name}] ID: {marker_id}",
                        (top_left[0], max(top_left[1] - 10, 15)),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.6,
                        (0, 255, 0),
                        2
                    )

                    key = f"{dict_name}_{marker_id}"
                    if key not in self.last_detected_time or (now - self.last_detected_time[key]) > 2.0:
                        self.last_detected_time[key] = now
                        self.log_message.emit(f"🎯 ArUco 인식됨 -> 규격: {dict_name}, ID: {marker_id}")

        return frame


    def run(self):
        self.running = True
        time.sleep(1.5)
        bytes_data = b''

        while self.running:
            try:
                stream = requests.get(self.url, stream=True, timeout=3.0)
                for chunk in stream.iter_content(chunk_size=1024):
                    if not self.running:
                        break
                    bytes_data += chunk
                    start = bytes_data.find(b'\xff\xd8')
                    end = bytes_data.find(b'\xff\xd9')
                    if start != -1 and end != -1:
                        jpg = bytes_data[start:end + 2]
                        bytes_data = bytes_data[end + 2:]
                        frame = cv2.imdecode(np.frombuffer(jpg, dtype=np.uint8), cv2.IMREAD_COLOR)
                        if frame is not None:
                            frame = self.detect_aruco(frame)

                            rgb_image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                            h, w, ch = rgb_image.shape
                            qt_image = QImage(
                                rgb_image.data, w, h, ch * w, QImage.Format.Format_RGB888
                            ).copy()
                            self.frame_received.emit(qt_image)
            except Exception:
                if self.running:
                    time.sleep(1.0)

    def stop(self):
        self.running = False
        self.wait(1000)


class PatrolTaskManager(Node):
    def __init__(self, emitter):
        super().__init__('patrol_task_manager')
        self.emitter = emitter
        self.state = PatrolState.IDLE
        self.toggle = False
        self.goal_handle = None
        self.nav_view_process = None

        self.waypoints = [
            (0.12027, 0.48958),
            (0.46586, 0.17803),
            (1.01143, 0.56825),
            (1.02187, 1.44805),
            (0.93204, 2.32778),
            (1.17705, 2.42820),
            (2.42820, 2.42820),
            (0.90894, 3.36347),

            #(0.16512, 0.12211), #starting point,


        ]
        self.current_wp_idx = 0

        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.led_client = self.create_client(SetLed, '/set_led')

        # 라즈베리파이 원격 제어용 퍼블리셔
        self.lcd_pub = self.create_publisher(String, '/lcd_cmd', 10)
        self.bringup_pub = self.create_publisher(String, '/bringup_cmd', 10)
        self.nav_pub = self.create_publisher(String, '/nav_cmd', 10)
        self.cam_pub = self.create_publisher(String, '/cam_cmd', 10)

        self.timer = self.create_timer(0.25, self.control_loop)
        self.emit_log('시스템 초기화 완료 (현재 상태: IDLE)')

    def emit_log(self, message):
        now = datetime.now().strftime('%H:%M:%S.%f')[:-3]
        formatted_msg = f'[{now}] {message}'
        self.get_logger().info(message)
        self.emitter.log_signal.emit(formatted_msg)

    def start_camera(self):
        msg = String()
        msg.data = 'on'
        self.cam_pub.publish(msg)
        self.emit_log('📷 라즈베리파이 카메라 실행 명령 전송 (/cam_cmd: on)')

    def stop_camera(self):
        msg = String()
        msg.data = 'off'
        self.cam_pub.publish(msg)
        self.emit_log('📴 라즈베리파이 카메라 종료 명령 전송 (/cam_cmd: off)')

    def start_nav_view(self):
        if self.nav_view_process is None or self.nav_view_process.poll() is not None:
            try:
                # 시스템 Qt5 플러그인 경로를 우선 적용하여 cv2(venv)와의 Qt 충돌 방지
                env = os.environ.copy()
                env['QT_QPA_PLATFORM_PLUGIN_PATH'] = '/usr/lib/x86_64-linux-gnu/qt5/plugins'

                cmd = 'ros2 launch pinky_navigation nav2_view.launch.xml'
                self.nav_view_process = subprocess.Popen(
                    cmd,
                    shell=True,
                    executable='/bin/bash',
                    env=env,
                    preexec_fn=os.setsid
                )
                self.emit_log('🖥️ PC Nav2 View (RViz2) 실행됨')
            except Exception as e:
                self.emit_log(f'⚠️ Nav2 View 실행 실패: {e}')

    def stop_nav_view(self):
        if self.nav_view_process is not None and self.nav_view_process.poll() is None:
            try:
                os.killpg(os.getpgid(self.nav_view_process.pid), signal.SIGINT)
                self.nav_view_process = None
                self.emit_log('❎ PC Nav2 View (RViz2) 종료됨')
            except Exception as e:
                self.emit_log(f'⚠️ Nav2 View 종료 실패: {e}')

    def start_navigation(self):
        msg = String()
        msg.data = 'on'
        self.nav_pub.publish(msg)
        self.emit_log('🗺️ 라즈베리파이 Navigation 실행 명령 전송 (/nav_cmd: on)')

    def stop_navigation(self):
        msg = String()
        msg.data = 'off'
        self.nav_pub.publish(msg)
        self.emit_log('🛑 라즈베리파이 Navigation 종료 명령 전송 (/nav_cmd: off)')

    def start_bringup(self):
        msg = String()
        msg.data = 'on'
        self.bringup_pub.publish(msg)
        self.emit_log('🚀 라즈베리파이 Bringup + LED Server 실행 명령 전송')

    def stop_bringup(self):
        msg = String()
        msg.data = 'off'
        self.bringup_pub.publish(msg)
        self.emit_log('🔌 라즈베리파이 Bringup + LED Server 종료 명령 전송')

    def start_lcd_script(self):
        msg = String()
        msg.data = 'on'
        self.lcd_pub.publish(msg)
        self.emit_log('라즈베리파이 LCD 실행 명령 전송 (/lcd_cmd: on)')

    def stop_lcd_script(self):
        msg = String()
        msg.data = 'off'
        self.lcd_pub.publish(msg)
        self.emit_log('라즈베리파이 LCD 종료 명령 전송 (/lcd_cmd: off)')

    def set_led(self, pixels, r, g, b):
        if not self.led_client.service_is_ready():
            return
        req = SetLed.Request()
        req.command = 'set_pixel'
        req.pixels = pixels
        req.r, req.g, req.b = r, g, b
        self.led_client.call_async(req)

    def handle_event(self, event):
        all_pixels = [0, 1, 2, 3, 4, 5, 6, 7]

        if event == 'start' and self.state == PatrolState.IDLE:
            self.state = PatrolState.PATROL
            self.set_led(all_pixels, 0, 0, 255)
            self.emit_log('▶ 순찰 모드 시작 (PATROL)')
            self.send_next_waypoint()

        elif event == 'intruder_detected' and self.state != PatrolState.ALERT:
            self.state = PatrolState.ALERT
            self.emit_log('🚨 비상 상황 발생! 경광등 점멸 및 LCD 가동 (ALERT)')
            if self.goal_handle is not None:
                self.goal_handle.cancel_goal_async()
                self.emit_log('진행 중인 주행 목표(Nav2) 취소 요청됨')
            self.start_lcd_script()

        elif event == 'clear' and self.state == PatrolState.ALERT:
            self.state = PatrolState.PATROL
            self.stop_lcd_script()
            self.set_led(all_pixels, 0, 0, 255)
            self.emit_log('✅ 비상 상황 해제. 순찰을 재개합니다.')
            self.send_next_waypoint()

        elif event == 'stop':
            self.state = PatrolState.IDLE
            self.stop_lcd_script()
            if self.goal_handle is not None:
                self.goal_handle.cancel_goal_async()
            self.set_led(all_pixels, 0, 255, 0)
            self.emit_log('⏹ 순찰 정지. 대기 상태로 전환 (IDLE)')

    def send_next_waypoint(self):
        if self.state != PatrolState.PATROL:
            return
        if not self.nav_client.wait_for_server(timeout_sec=1.0):
            self.emit_log('⚠️ Nav2 서버 미연결 상태 (이동 명령 건너뜀)')
            return

        x, y = self.waypoints[self.current_wp_idx]
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = float(x)
        goal_msg.pose.pose.position.y = float(y)
        goal_msg.pose.pose.orientation.w = 1.0

        self.emit_log(f'웨이포인트 [{self.current_wp_idx}] ({x}, {y})로 이동 시작')
        send_goal_future = self.nav_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        self.goal_handle = future.result()
        if self.goal_handle.accepted:
            result_future = self.goal_handle.get_result_async()
            result_future.add_done_callback(self.get_result_callback)
        else:
            self.emit_log('⚠️ 웨이포인트 이동 요청이 거부되었습니다.')

    def get_result_callback(self, future):
        if self.state == PatrolState.PATROL:
            self.emit_log(f'웨이포인트 [{self.current_wp_idx}] 도착 완료!')
            self.current_wp_idx = (self.current_wp_idx + 1) % len(self.waypoints)
            self.send_next_waypoint()

    def control_loop(self):
        if self.state == PatrolState.ALERT:
            left_pixels = [0, 1, 2, 3]
            right_pixels = [4, 5, 6, 7]

            if self.toggle:
                self.set_led(left_pixels, 255, 0, 0)
                self.set_led(right_pixels, 0, 0, 255)
            else:
                self.set_led(left_pixels, 0, 0, 255)
                self.set_led(right_pixels, 255, 0, 0)
            self.toggle = not self.toggle


class PatrolDashboard(QWidget):
    def __init__(self, task_manager, emitter):
        super().__init__()
        self.task_manager = task_manager
        self.video_thread = None
        self.stream_url = 'http://192.168.14.80:5000/video_feed'
        self.init_ui()

        emitter.log_signal.connect(self.append_log)

        self.ros_timer = QTimer(self)
        self.ros_timer.timeout.connect(self.spin_ros)
        self.ros_timer.start(30)

    def spin_ros(self):
        rclpy.spin_once(self.task_manager, timeout_sec=0)

    def keyPressEvent(self, event):
        if event.key() == Qt.Key.Key_Escape:
            self.close()
        else:
            super().keyPressEvent(event)

    def init_ui(self):
        self.setWindowTitle('순찰 로봇 통합 제어기 + ArUco 인식 (ESC: 종료)')
        self.resize(940, 720)

        main_layout = QHBoxLayout()
        left_panel = QVBoxLayout()
        left_panel.setSpacing(10)

        # 1. 상단 상태 표시 라벨
        self.status_label = QLabel('현재 상태: 대기 중 (IDLE)')
        self.status_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.status_label.setFont(QFont('Arial', 14, QFont.Weight.Bold))
        self.status_label.setStyleSheet(
            "background-color: #2c3e50; color: white; padding: 15px; border-radius: 8px;"
        )
        left_panel.addWidget(self.status_label)

        # 2. 로봇 Bringup + LED Server 원격 제어 버튼
        bringup_layout = QHBoxLayout()
        btn_bringup_on = QPushButton('🚀 로봇 Bringup 켜기')
        btn_bringup_on.setStyleSheet("background-color: #8e44ad; color: white; font-size: 13px; padding: 10px; font-weight: bold;")
        btn_bringup_on.clicked.connect(self.task_manager.start_bringup)
        bringup_layout.addWidget(btn_bringup_on)

        btn_bringup_off = QPushButton('🔌 로봇 Bringup 끄기')
        btn_bringup_off.setStyleSheet("background-color: #34495e; color: white; font-size: 13px; padding: 10px; font-weight: bold;")
        btn_bringup_off.clicked.connect(self.task_manager.stop_bringup)
        bringup_layout.addWidget(btn_bringup_off)
        left_panel.addLayout(bringup_layout)

        # 3. 라즈베리파이 Navigation (Nav2) 원격 제어 버튼
        nav_layout = QHBoxLayout()
        btn_nav_on = QPushButton('🗺️ Navigation 켜기')
        btn_nav_on.setStyleSheet("background-color: #16a085; color: white; font-size: 13px; padding: 10px; font-weight: bold;")
        btn_nav_on.clicked.connect(self.task_manager.start_navigation)
        nav_layout.addWidget(btn_nav_on)

        btn_nav_off = QPushButton('🛑 Navigation 끄기')
        btn_nav_off.setStyleSheet("background-color: #2c3e50; color: white; font-size: 13px; padding: 10px; font-weight: bold;")
        btn_nav_off.clicked.connect(self.task_manager.stop_navigation)
        nav_layout.addWidget(btn_nav_off)
        left_panel.addLayout(nav_layout)

        # 4. PC Nav2 View (RViz2) 로컬 제어 버튼
        view_layout = QHBoxLayout()
        btn_view_on = QPushButton('🖥️ Nav2 View 켜기')
        btn_view_on.setStyleSheet("background-color: #2980b9; color: white; font-size: 13px; padding: 10px; font-weight: bold;")
        btn_view_on.clicked.connect(self.task_manager.start_nav_view)
        view_layout.addWidget(btn_view_on)

        btn_view_off = QPushButton('❎ Nav2 View 끄기')
        btn_view_off.setStyleSheet("background-color: #34495e; color: white; font-size: 13px; padding: 10px; font-weight: bold;")
        btn_view_off.clicked.connect(self.task_manager.stop_nav_view)
        view_layout.addWidget(btn_view_off)
        left_panel.addLayout(view_layout)

        # 5. 라즈베리파이 카메라 원격 제어 버튼
        cam_layout = QHBoxLayout()
        btn_cam_on = QPushButton('📷 카메라 + ArUco 켜기')
        btn_cam_on.setStyleSheet("background-color: #d35400; color: white; font-size: 13px; padding: 10px; font-weight: bold;")
        btn_cam_on.clicked.connect(self.on_cam_on_click)
        cam_layout.addWidget(btn_cam_on)

        btn_cam_off = QPushButton('📴 카메라 끄기')
        btn_cam_off.setStyleSheet("background-color: #34495e; color: white; font-size: 13px; padding: 10px; font-weight: bold;")
        btn_cam_off.clicked.connect(self.on_cam_off_click)
        cam_layout.addWidget(btn_cam_off)
        left_panel.addLayout(cam_layout)

        # 6. 순찰 및 경광등 제어 버튼
        btn_start = QPushButton('▶ 순찰 시작 (START)')
        btn_start.setStyleSheet("background-color: #27ae60; color: white; font-size: 14px; padding: 11px; font-weight: bold;")
        btn_start.clicked.connect(lambda: self.on_button_click('start', '순찰 중 (PATROL)', '#2980b9'))
        left_panel.addWidget(btn_start)

        btn_alert = QPushButton('🚨 비상 상황 / 경광등 + LCD (ALERT)')
        btn_alert.setStyleSheet("background-color: #e74c3c; color: white; font-size: 14px; padding: 11px; font-weight: bold;")
        btn_alert.clicked.connect(lambda: self.on_button_click('intruder_detected', '🚨 비상 경광등 + LCD 가동', '#c0392b'))
        left_panel.addWidget(btn_alert)

        btn_clear = QPushButton('✅ 상황 해제 / 순찰 재개 (CLEAR)')
        btn_clear.setStyleSheet("background-color: #f39c12; color: white; font-size: 14px; padding: 11px; font-weight: bold;")
        btn_clear.clicked.connect(lambda: self.on_button_click('clear', '순찰 재개 (PATROL)', '#2980b9'))
        left_panel.addWidget(btn_clear)

        btn_stop = QPushButton('⏹ 순찰 정지 (STOP)')
        btn_stop.setStyleSheet("background-color: #7f8c8d; color: white; font-size: 14px; padding: 11px; font-weight: bold;")
        btn_stop.clicked.connect(lambda: self.on_button_click('stop', '대기 중 (IDLE)', '#2c3e50'))
        left_panel.addWidget(btn_stop)

        # 7. 실시간 로그 영역
        log_header_layout = QHBoxLayout()
        log_title = QLabel('📋 실시간 이벤트 로그')
        log_title.setFont(QFont('Arial', 11, QFont.Weight.Bold))
        log_header_layout.addWidget(log_title)

        btn_clear_log = QPushButton('로그 지우기')
        btn_clear_log.setFixedWidth(90)
        btn_clear_log.clicked.connect(self.clear_log)
        log_header_layout.addWidget(btn_clear_log)
        left_panel.addLayout(log_header_layout)

        self.log_box = QTextEdit()
        self.log_box.setReadOnly(True)
        self.log_box.setFont(QFont('Consolas', 10))
        self.log_box.setStyleSheet("background-color: #1e1e1e; color: #00ff66; padding: 8px; border-radius: 6px;")
        left_panel.addWidget(self.log_box)

        # 우측 패널: 실시간 카메라 영상 + ArUco 인식
        right_panel = QVBoxLayout()
        cam_title = QLabel('📹 실시간 로봇 카메라 & ArUco 마커 인식')
        cam_title.setAlignment(Qt.AlignmentFlag.AlignCenter)
        cam_title.setFont(QFont('Arial', 12, QFont.Weight.Bold))
        right_panel.addWidget(cam_title)

        self.camera_label = QLabel('카메라 대기 중...\n[📷 카메라 + ArUco 켜기] 버튼을 눌러주세요.')
        self.camera_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.camera_label.setFixedSize(480, 360)
        self.camera_label.setStyleSheet("background-color: #111111; color: #aaaaaa; border: 2px solid #333333; border-radius: 8px;")
        right_panel.addWidget(self.camera_label)
        right_panel.addStretch()

        main_layout.addLayout(left_panel, 1)
        main_layout.addLayout(right_panel, 1)
        self.setLayout(main_layout)

    def on_cam_on_click(self):
        self.task_manager.start_camera()
        self.camera_label.setText('카메라 연결 중...')
        if self.video_thread is None or not self.video_thread.isRunning():
            self.video_thread = VideoStreamThread(self.stream_url)
            self.video_thread.frame_received.connect(self.update_camera_feed)
            self.video_thread.log_message.connect(self.task_manager.emit_log)
            self.video_thread.start()

    def on_cam_off_click(self):
        if self.video_thread is not None and self.video_thread.isRunning():
            self.video_thread.stop()
            self.video_thread = None
        self.task_manager.stop_camera()
        self.camera_label.clear()
        self.camera_label.setText('카메라 꺼짐')

    def update_camera_feed(self, qt_image):
        pixmap = QPixmap.fromImage(qt_image).scaled(
            self.camera_label.width(),
            self.camera_label.height(),
            Qt.AspectRatioMode.KeepAspectRatio,
            Qt.TransformationMode.SmoothTransformation
        )
        self.camera_label.setPixmap(pixmap)

    def append_log(self, text):
        self.log_box.append(text)
        self.log_box.moveCursor(QTextCursor.MoveOperation.End)

    def clear_log(self):
        self.log_box.clear()

    def on_button_click(self, command, status_text, color_hex):
        self.task_manager.handle_event(command)
        self.status_label.setText(f'현재 상태: {status_text}')
        self.status_label.setStyleSheet(f"background-color: {color_hex}; color: white; padding: 15px; border-radius: 8px;")

    def closeEvent(self, event):
        if self.video_thread is not None and self.video_thread.isRunning():
            self.video_thread.stop()
        super().closeEvent(event)


def main():
    rclpy.init()
    emitter = GuiEmitter()
    task_manager = PatrolTaskManager(emitter)

    app = QApplication(sys.argv)
    dashboard = PatrolDashboard(task_manager, emitter)
    dashboard.show()

    dashboard.append_log(f"[{datetime.now().strftime('%H:%M:%S.%f')[:-3]}] 대시보드 UI 로드 완료 (ESC: 종료)")

    try:
        sys.exit(app.exec())
    finally:
        task_manager.stop_lcd_script()
        task_manager.stop_camera()
        task_manager.stop_nav_view()
        task_manager.stop_navigation()
        task_manager.stop_bringup()
        task_manager.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
