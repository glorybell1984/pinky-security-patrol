"""Pinky Pro camera -> MJPEG over HTTP (Flask). Runs ON THE ROBOT. No ROS needed.

Over Wi-Fi, one HTTP stream is smoother than ROS image topics: a ROS image is
split into many UDP packets and one lost packet loses the whole frame, while
HTTP (TCP) just slows down. This follows the Pinky Pro teacher's advice: Flask,
lower resolution, lower frame rate.

    scp camera_server.py pinky@<robot-ip>:~/
    ssh pinky@<robot-ip>
    pip install flask            # if missing (python3 -c "import flask")
    python3 ~/camera_server.py                          # 640x480, 10 fps, turned 180
    python3 ~/camera_server.py --width 320 --height 240 --fps 15 --quality 60

Endpoints (port 5000):
    /video_feed     MJPEG stream (perception on the laptop and the dashboard read this)
    /snapshot.jpg   latest frame
    /stats          JSON: real fps, frame size in KB, number of viewers
    /               simple page with the stream

One capture thread reads the camera at the set fps and encodes each frame once;
every viewer gets the same latest frame. A slow viewer skips frames instead of
falling behind, so the delay stays short.
"""
import argparse
import json
import threading
import time

import cv2

ROTATIONS = {90: cv2.ROTATE_90_CLOCKWISE, 180: cv2.ROTATE_180,
             270: cv2.ROTATE_90_COUNTERCLOCKWISE}


# ---------------------------------------------------------------- camera sources
class PinkylibSource:
    name = 'pinkylib'

    def __init__(self, width, height):
        from pinkylib import Camera
        self.cam = Camera()
        self.cam.start()

    def read(self):
        return self.cam.get_frame()

    def close(self):
        self.cam.close()


class Picamera2Source:
    name = 'picamera2'

    def __init__(self, width, height):
        from picamera2 import Picamera2
        self.cam = Picamera2()
        # "RGB888" in picamera2 is stored B, G, R in memory: already OpenCV order.
        self.cam.configure(self.cam.create_preview_configuration(
            main={'format': 'RGB888', 'size': (width, height)}))
        self.cam.start()

    def read(self):
        return self.cam.capture_array()

    def close(self):
        self.cam.stop()
        self.cam.close()


class OpenCVSource:
    name = 'opencv'

    def __init__(self, width, height, device=0):
        self.cap = cv2.VideoCapture(device)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        if not self.cap.isOpened():
            raise RuntimeError(f'cannot open video device {device}')
        # On a Pi with the camera busy, /dev/video0 can open but never deliver a
        # frame (seen on the Pinky 2026-11-11): fail now instead of "15 Hz" of nothing.
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline:
            ok, frame = self.cap.read()
            if ok and frame is not None:
                return
            time.sleep(0.05)
        self.cap.release()
        raise RuntimeError(f'video device {device} opens but gives no frames')

    def read(self):
        ok, frame = self.cap.read()
        return frame if ok else None

    def close(self):
        self.cap.release()


class TestSource:
    """Synthetic frames with ArUco markers, for testing without a camera."""
    name = 'test'

    def __init__(self, width, height, ids=(7,), dictionary='DICT_4X4_50'):
        import numpy as np
        self.np = np
        self.w, self.h = width, height
        d = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, dictionary))
        side = height // 3
        gen = getattr(cv2.aruco, 'generateImageMarker', None) or cv2.aruco.drawMarker
        self.markers = [gen(d, i, side) for i in ids]
        self.side = side
        self.t0 = time.monotonic()

    def read(self):
        np = self.np
        img = np.full((self.h, self.w), 200, np.uint8)
        t = time.monotonic() - self.t0
        for k, m in enumerate(self.markers):
            # slide left and right so the stream visibly moves
            x = int((self.w - self.side) / 2 + (self.w / 4) * np.sin(t * 0.8 + k))
            y = (self.h - self.side) // 2
            pad = self.side // 8
            img[y - pad:y + self.side + pad, x - pad:x + self.side + pad] = 255
            img[y:y + self.side, x:x + self.side] = m
        img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
        cv2.putText(img, f'test {t:6.1f}s', (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                    (0, 0, 255), 2)
        time.sleep(0.005)
        return img

    def close(self):
        pass


def open_source(backend, width, height, device=0, test_ids=(7,)):
    """backend: auto | pinkylib | picamera2 | opencv | test"""
    order = ['pinkylib', 'picamera2', 'opencv'] if backend == 'auto' else [backend]
    errors = []
    for name in order:
        try:
            if name == 'pinkylib':
                return PinkylibSource(width, height)
            if name == 'picamera2':
                return Picamera2Source(width, height)
            if name == 'opencv':
                return OpenCVSource(width, height, device)
            if name == 'test':
                return TestSource(width, height, test_ids)
        except Exception as exc:  # library missing or camera busy
            errors.append(f'{name}: {exc}')
    raise RuntimeError('No camera backend worked. Is another program (Jupyter, '
                       'another camera program) using the camera?\n  ' + '\n  '.join(errors))


def prepare(frame, width, height, rotate):
    """Camera frame -> BGR at the requested size, turned."""
    if frame is None:
        return None
    if frame.ndim == 3 and frame.shape[2] == 4:          # XRGB8888 / BGRA
        frame = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)
    if frame.shape[1] != width or frame.shape[0] != height:
        frame = cv2.resize(frame, (width, height), interpolation=cv2.INTER_AREA)
    if rotate in ROTATIONS:
        frame = cv2.rotate(frame, ROTATIONS[rotate])
    return frame


# ---------------------------------------------------------------- capture loop
class FrameHub:
    """Latest JPEG, shared by all viewers."""

    def __init__(self, source, width, height, fps, quality, rotate, swap_rb):
        self.source = source
        self.size = (width, height)
        self.period = 1.0 / max(fps, 0.5)
        self.encode_args = [int(cv2.IMWRITE_JPEG_QUALITY), int(quality)]
        self.rotate = rotate
        self.swap_rb = swap_rb
        self.cond = threading.Condition()
        self.jpeg = None
        self.stamp = 0.0
        self.seq = 0
        self.viewers = 0
        self.running = True
        self.rate = 0.0
        self.errors = 0

    def run(self):
        next_t = time.monotonic()
        count, t_rate = 0, time.monotonic()
        while self.running:
            try:
                frame = prepare(self.source.read(), *self.size, self.rotate)
            except Exception as exc:
                self.errors += 1
                if self.errors % 30 == 1:
                    print(f'[camera_server] read failed: {exc}', flush=True)
                time.sleep(0.1)
                continue
            if frame is None:
                continue
            if self.swap_rb:
                frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            ok, buf = cv2.imencode('.jpg', frame, self.encode_args)
            if ok:
                with self.cond:
                    self.jpeg = buf.tobytes()
                    self.stamp = time.time()
                    self.seq += 1
                    self.cond.notify_all()
                count += 1
            now = time.monotonic()
            if now - t_rate >= 5.0:
                self.rate = count / (now - t_rate)
                count, t_rate = 0, now
            next_t += self.period
            delay = next_t - time.monotonic()
            if delay > 0:
                time.sleep(delay)
            else:
                next_t = time.monotonic()          # camera slower than the set fps

    def frames(self):
        """Generator for one viewer: waits for each new frame, skips old ones."""
        last = -1
        with self.cond:
            self.viewers += 1
        try:
            while self.running:
                with self.cond:
                    self.cond.wait_for(lambda: self.seq != last or not self.running, 2.0)
                    if self.seq == last:
                        continue
                    jpeg, stamp, last = self.jpeg, self.stamp, self.seq
                yield (b'--frame\r\nContent-Type: image/jpeg\r\n'
                       b'X-Timestamp: ' + f'{stamp:.3f}'.encode() + b'\r\n'
                       b'Content-Length: ' + str(len(jpeg)).encode() + b'\r\n\r\n'
                       + jpeg + b'\r\n')
        finally:
            with self.cond:
                self.viewers -= 1


def make_app(hub):
    from flask import Flask, Response

    app = Flask(__name__)

    @app.route('/video_feed')
    def video_feed():
        resp = Response(hub.frames(), mimetype='multipart/x-mixed-replace; boundary=frame')
        resp.headers['Cache-Control'] = 'no-cache, no-store'
        resp.headers['Access-Control-Allow-Origin'] = '*'
        return resp

    @app.route('/snapshot.jpg')
    def snapshot():
        with hub.cond:
            jpeg = hub.jpeg
        if jpeg is None:
            return Response('no frame yet', status=503)
        return Response(jpeg, mimetype='image/jpeg',
                        headers={'Access-Control-Allow-Origin': '*'})

    @app.route('/stats')
    def stats():
        with hub.cond:
            kb = len(hub.jpeg) / 1024 if hub.jpeg else 0
        data = {'fps': round(hub.rate, 1), 'width': hub.size[0], 'height': hub.size[1],
                'frame_kb': round(kb, 1), 'viewers': hub.viewers, 'backend': hub.source.name,
                'target_fps': round(1 / hub.period, 1)}
        return Response(json.dumps(data), mimetype='application/json',
                        headers={'Access-Control-Allow-Origin': '*'})

    @app.route('/')
    def index():
        return ('<html><body style="background:#111;color:#ccc;font-family:sans-serif">'
                '<p>Pinky camera &middot; <a href="/stats" style="color:#8af">stats</a></p>'
                '<img src="/video_feed" style="max-width:100%"></body></html>')

    return app


def main():
    ap = argparse.ArgumentParser(description='Pinky camera MJPEG server')
    ap.add_argument('--backend', default='auto', help='auto | pinkylib | picamera2 | opencv | test')
    ap.add_argument('--device', type=int, default=0, help='opencv device number')
    ap.add_argument('--width', type=int, default=640)
    ap.add_argument('--height', type=int, default=480)
    ap.add_argument('--fps', type=float, default=10.0)
    ap.add_argument('--quality', type=int, default=70, help='JPEG quality 1-100')
    ap.add_argument('--rotate', type=int, default=0, help='0, 90, 180, 270 (upright with pinkylib on our Pinky Pro)')
    ap.add_argument('--swap-rb', action='store_true', help='if red and blue look swapped')
    ap.add_argument('--port', type=int, default=5000)
    ap.add_argument('--test-ids', type=int, nargs='+', default=[7],
                    help='marker IDs drawn by the test backend')
    args = ap.parse_args()

    source = open_source(args.backend, args.width, args.height, args.device, args.test_ids)
    hub = FrameHub(source, args.width, args.height, args.fps, args.quality, args.rotate,
                   args.swap_rb)
    threading.Thread(target=hub.run, daemon=True).start()
    print(f'[camera_server] {source.name} {args.width}x{args.height} @ {args.fps:.0f} fps, '
          f'quality {args.quality} -> http://0.0.0.0:{args.port}/video_feed', flush=True)
    try:
        make_app(hub).run(host='0.0.0.0', port=args.port, threaded=True, debug=False)
    finally:
        hub.running = False
        source.close()


if __name__ == '__main__':
    main()
