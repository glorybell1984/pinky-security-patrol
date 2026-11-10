import os
import sys
import signal
import subprocess
from datetime import datetime
from enum import Enum

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
from PyQt6.QtCore import QTimer, Qt, pyqtSignal, QObject
from PyQt6.QtGui import QFont, QTextCursor

class PatrolState(Enum):
    IDLE = 1
    PATROL = 2
    ALERT = 3

class LogEmitter(QObject):
    log_signal = pyqtSignal(str)

class PatrolTaskManager(Node):
    def __init__(self, log_emitter):
        super().__init__('patrol_task_manager')
        self.log_emitter = log_emitter
        self.state = PatrolState.IDLE
        self.toggle = False
        self.goal_handle = None
        self.view_process = None

        self.waypoints = [
            (1.658, -0.010),
            (3.7301, 0.0358),
            (5.8468, 0.1043),
            (5.2444, 0.89458),
            (5.5354, -0.7979),
            (4.6991, 0.14799),
            (6.0696, -0.1627),

            #(-0.0241, -0.0673) # 시작포인트
        ]
        self.current_wp_idx = 0

        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.led_client = self.create_client(SetLed, '/set_led')
        
        self.lcd_pub = self.create_publisher(String, '/lcd_cmd', 10)
        self.bringup_pub = self.create_publisher(String, '/bringup_cmd', 10)
        self.nav_pub = self.create_publisher(String, '/nav_cmd', 10)

        self.timer = self.create_timer(0.25, self.control_loop)
        self.emit_log('시스템 초기화 완료 (현재 상태: IDLE)')

    def emit_log(self, message):
        now = datetime.now().strftime('%H:%M:%S.%f')[:-3]
        formatted_msg = f'[{now}] {message}'
        self.get_logger().info(message)
        self.log_emitter.log_signal.emit(formatted_msg)

    # --- PC 로컬 Nav2 View (RViz2) 제어 ---
    def start_nav_view(self):
        if self.view_process is None or self.view_process.poll() is not None:
            try:
                cmd = 'ros2 launch pinky_navigation nav2_view.launch.xml'
                self.view_process = subprocess.Popen(
                    cmd,
                    shell=True,
                    executable='/bin/bash',
                    preexec_fn=os.setsid
                )
                self.emit_log('🖥️ PC Nav2 View (RViz2) 실행됨!')
            except Exception as e:
                self.emit_log(f'⚠️ Nav2 View 실행 실패: {e}')

    def stop_nav_view(self):
        if self.view_process is not None and self.view_process.poll() is None:
            try:
                os.killpg(os.getpgid(self.view_process.pid), signal.SIGINT)
                self.view_process = None
                self.emit_log('🖥️ PC Nav2 View (RViz2) 종료됨!')
            except Exception as e:
                self.emit_log(f'⚠️ Nav2 View 종료 실패: {e}')

    # --- 라즈베리파이 Navigation(Nav2) 원격 제어 ---
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

    # --- 라즈베리파이 Bringup + LED Server 원격 제어 ---
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

    # --- 라즈베리파이 LCD 원격 제어 ---
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
    def __init__(self, task_manager, log_emitter):
        super().__init__()
        self.task_manager = task_manager
        self.init_ui()

        log_emitter.log_signal.connect(self.append_log)

        self.ros_timer = QTimer(self)
        self.ros_timer.timeout.connect(self.spin_ros)
        self.ros_timer.start(50)

    def spin_ros(self):
        rclpy.spin_once(self.task_manager, timeout_sec=0)

    def keyPressEvent(self, event):
        if event.key() == Qt.Key.Key_Escape:
            self.close()
        else:
            super().keyPressEvent(event)

    def init_ui(self):
        self.setWindowTitle('순찰 로봇 통합 제어기 (ESC: 종료)')
        self.resize(450, 790)

        layout = QVBoxLayout()
        layout.setSpacing(10)

        # 1. 상단 상태 표시 라벨
        self.status_label = QLabel('현재 상태: 대기 중 (IDLE)')
        self.status_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.status_label.setFont(QFont('Arial', 14, QFont.Weight.Bold))
        self.status_label.setStyleSheet(
            "background-color: #2c3e50; color: white; padding: 15px; border-radius: 8px;"
        )
        layout.addWidget(self.status_label)

        # 2. 라즈베리파이 Bringup + LED Server 원격 제어 버튼
        bringup_layout = QHBoxLayout()

        btn_bringup_on = QPushButton('🚀 로봇 Bringup 켜기')
        btn_bringup_on.setStyleSheet(
            "background-color: #8e44ad; color: white; font-size: 13px; padding: 10px; font-weight: bold;"
        )
        btn_bringup_on.clicked.connect(self.task_manager.start_bringup)
        bringup_layout.addWidget(btn_bringup_on)

        btn_bringup_off = QPushButton('🔌 로봇 Bringup 끄기')
        btn_bringup_off.setStyleSheet(
            "background-color: #34495e; color: white; font-size: 13px; padding: 10px; font-weight: bold;"
        )
        btn_bringup_off.clicked.connect(self.task_manager.stop_bringup)
        bringup_layout.addWidget(btn_bringup_off)

        layout.addLayout(bringup_layout)

        # 3. 라즈베리파이 Navigation (Nav2) 원격 제어 버튼
        nav_layout = QHBoxLayout()

        btn_nav_on = QPushButton('🗺️ Navigation 켜기')
        btn_nav_on.setStyleSheet(
            "background-color: #16a085; color: white; font-size: 13px; padding: 10px; font-weight: bold;"
        )
        btn_nav_on.clicked.connect(self.task_manager.start_navigation)
        nav_layout.addWidget(btn_nav_on)

        btn_nav_off = QPushButton('🛑 Navigation 끄기')
        btn_nav_off.setStyleSheet(
            "background-color: #2c3e50; color: white; font-size: 13px; padding: 10px; font-weight: bold;"
        )
        btn_nav_off.clicked.connect(self.task_manager.stop_navigation)
        nav_layout.addWidget(btn_nav_off)

        layout.addLayout(nav_layout)

        # 4. PC 로컬 Nav2 View (RViz2) 제어 버튼
        view_layout = QHBoxLayout()

        btn_view_on = QPushButton('🖥️ Nav2 View 켜기')
        btn_view_on.setStyleSheet(
            "background-color: #2980b9; color: white; font-size: 13px; padding: 10px; font-weight: bold;"
        )
        btn_view_on.clicked.connect(self.task_manager.start_nav_view)
        view_layout.addWidget(btn_view_on)

        btn_view_off = QPushButton('❎ Nav2 View 끄기')
        btn_view_off.setStyleSheet(
            "background-color: #5d6d7e; color: white; font-size: 13px; padding: 10px; font-weight: bold;"
        )
        btn_view_off.clicked.connect(self.task_manager.stop_nav_view)
        view_layout.addWidget(btn_view_off)

        layout.addLayout(view_layout)

        # 5. 순찰 및 경광등 제어 버튼
        btn_start = QPushButton('▶ 순찰 시작 (START)')
        btn_start.setStyleSheet(
            "background-color: #27ae60; color: white; font-size: 14px; padding: 11px; font-weight: bold;"
        )
        btn_start.clicked.connect(
            lambda: self.on_button_click('start', '순찰 중 (PATROL)', '#2980b9')
        )
        layout.addWidget(btn_start)

        btn_alert = QPushButton('🚨 비상 상황 / 경광등 + LCD (ALERT)')
        btn_alert.setStyleSheet(
            "background-color: #e74c3c; color: white; font-size: 14px; padding: 11px; font-weight: bold;"
        )
        btn_alert.clicked.connect(
            lambda: self.on_button_click('intruder_detected', '🚨 비상 경광등 + LCD 가동', '#c0392b')
        )
        layout.addWidget(btn_alert)

        btn_clear = QPushButton('✅ 상황 해제 / 순찰 재개 (CLEAR)')
        btn_clear.setStyleSheet(
            "background-color: #f39c12; color: white; font-size: 14px; padding: 11px; font-weight: bold;"
        )
        btn_clear.clicked.connect(
            lambda: self.on_button_click('clear', '순찰 재개 (PATROL)', '#2980b9')
        )
        layout.addWidget(btn_clear)

        btn_stop = QPushButton('⏹ 순찰 정지 (STOP)')
        btn_stop.setStyleSheet(
            "background-color: #7f8c8d; color: white; font-size: 14px; padding: 11px; font-weight: bold;"
        )
        btn_stop.clicked.connect(
            lambda: self.on_button_click('stop', '대기 중 (IDLE)', '#2c3e50')
        )
        layout.addWidget(btn_stop)

        # 6. 실시간 로그 영역
        log_header_layout = QHBoxLayout()
        log_title = QLabel('📋 실시간 이벤트 로그')
        log_title.setFont(QFont('Arial', 11, QFont.Weight.Bold))
        log_header_layout.addWidget(log_title)

        btn_clear_log = QPushButton('로그 지우기')
        btn_clear_log.setFixedWidth(90)
        btn_clear_log.clicked.connect(self.clear_log)
        log_header_layout.addWidget(btn_clear_log)
        layout.addLayout(log_header_layout)

        self.log_box = QTextEdit()
        self.log_box.setReadOnly(True)
        self.log_box.setFont(QFont('Consolas', 10))
        self.log_box.setStyleSheet(
            "background-color: #1e1e1e; color: #00ff66; padding: 8px; border-radius: 6px;"
        )
        layout.addWidget(self.log_box)
        self.setLayout(layout)

    def append_log(self, text):
        self.log_box.append(text)
        self.log_box.moveCursor(QTextCursor.MoveOperation.End)

    def clear_log(self):
        self.log_box.clear()

    def on_button_click(self, command, status_text, color_hex):
        self.task_manager.handle_event(command)
        self.status_label.setText(f'현재 상태: {status_text}')
        self.status_label.setStyleSheet(
            f"background-color: {color_hex}; color: white; padding: 15px; border-radius: 8px;"
        )

def main():
    rclpy.init()
    log_emitter = LogEmitter()
    task_manager = PatrolTaskManager(log_emitter)

    app = QApplication(sys.argv)
    dashboard = PatrolDashboard(task_manager, log_emitter)
    dashboard.show()

    dashboard.append_log(f"[{datetime.now().strftime('%H:%M:%S.%f')[:-3]}] 대시보드 UI 로드 완료 (ESC: 종료)")

    try:
        sys.exit(app.exec())
    finally:
        task_manager.stop_lcd_script()
        task_manager.stop_nav_view()
        task_manager.stop_navigation()
        task_manager.stop_bringup()
        task_manager.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
