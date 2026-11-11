"""Read an MJPEG HTTP stream (camera_server.py, or any Flask MJPEG server) in a thread.

Keeps only the newest frame, so processing never falls behind the stream.
Works with streams that send Content-Length per part (camera_server.py) and
with plain ones that don't (JPEG start/end markers are used then).
"""
import threading
import time
import urllib.request

SOI, EOI = b'\xff\xd8', b'\xff\xd9'


class MjpegReader:

    def __init__(self, url, logger=None, timeout=5.0):
        self.url = url
        self.log = logger
        self.timeout = timeout
        self.lock = threading.Lock()
        self.jpeg = None
        self.arrival = 0.0           # time.monotonic() when the frame arrived
        self.robot_stamp = None      # X-Timestamp from camera_server (robot clock)
        self.seq = 0
        self.connected = False
        self.running = True
        self.received = 0
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def latest(self, after_seq=-1):
        """(seq, jpeg bytes, arrival time, robot stamp) or None if nothing newer."""
        with self.lock:
            if self.jpeg is None or self.seq == after_seq:
                return None
            return self.seq, self.jpeg, self.arrival, self.robot_stamp

    def stop(self):
        self.running = False

    def _store(self, jpeg, stamp):
        with self.lock:
            self.jpeg = jpeg
            self.arrival = time.monotonic()
            self.robot_stamp = stamp
            self.seq += 1
        self.received += 1

    def _say(self, level, text):
        if self.log is not None:
            getattr(self.log, level)(text)

    def _run(self):
        while self.running:
            try:
                with urllib.request.urlopen(self.url, timeout=self.timeout) as resp:
                    if not self.connected:
                        self._say('info', f'Camera stream connected: {self.url}')
                    self.connected = True
                    self._read_parts(resp)
            except Exception as exc:
                if self.connected:
                    self._say('warn', f'Camera stream lost ({exc}); reconnecting')
                elif self.running:
                    self._say('warn', f'Camera stream not reachable: {self.url} ({exc})')
                self.connected = False
            if self.running:
                time.sleep(1.0)

    def _read_parts(self, resp):
        buf = b''
        while self.running:
            # part headers end with a blank line
            while b'\r\n\r\n' not in buf:
                chunk = resp.read1(65536) if hasattr(resp, 'read1') else resp.read(4096)
                if not chunk:
                    return
                buf += chunk
                if len(buf) > 4_000_000:
                    buf = buf[-100_000:]
            head, _, buf = buf.partition(b'\r\n\r\n')
            headers = {}
            for line in head.split(b'\r\n'):
                if b':' in line:
                    k, _, v = line.partition(b':')
                    headers[k.strip().lower()] = v.strip()
            stamp = None
            if b'x-timestamp' in headers:
                try:
                    stamp = float(headers[b'x-timestamp'])
                except ValueError:
                    pass
            length = headers.get(b'content-length')
            if length is not None:
                n = int(length)
                while len(buf) < n:
                    chunk = resp.read1(65536) if hasattr(resp, 'read1') else resp.read(n - len(buf))
                    if not chunk:
                        return
                    buf += chunk
                jpeg, buf = buf[:n], buf[n:]
            else:
                # no length: read until the JPEG end marker
                while True:
                    s = buf.find(SOI)
                    e = buf.find(EOI, s + 2) if s >= 0 else -1
                    if s >= 0 and e >= 0:
                        jpeg, buf = buf[s:e + 2], buf[e + 2:]
                        break
                    chunk = resp.read1(65536) if hasattr(resp, 'read1') else resp.read(4096)
                    if not chunk:
                        return
                    buf += chunk
            if jpeg.startswith(SOI):
                self._store(jpeg, stamp)
