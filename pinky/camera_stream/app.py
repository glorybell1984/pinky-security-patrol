from picamera2 import Picamera2
import cv2
from flask import Flask, Response, render_template

app = Flask(__name__)

# Picamera2 초기화 - 전역 변수로 선언
picam2 = None

def init_camera():
    global picam2
    if picam2 is None:
        picam2 = Picamera2()
        picam2.configure(picam2.create_preview_configuration(main={"format": 'RGB888', "size": (320, 240)}))
        picam2.start()

def gen_frames():
    while True:
        frame = picam2.capture_array()
        frame = cv2.rotate(frame, cv2.ROTATE_180)
        ret, buffer = cv2.imencode('.jpg', frame)
        frame = buffer.tobytes()
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')

@app.route('/')
def index():
    return render_template('index.html')

@app.route('/video_feed')
def video_feed():
    return Response(gen_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

if __name__ == '__main__':
    try:
        init_camera()  # 카메라 초기화
        app.run(host='0.0.0.0', port=5000, debug=False)  # debug 모드 비활성화
    finally:
        if picam2:
            picam2.close()
