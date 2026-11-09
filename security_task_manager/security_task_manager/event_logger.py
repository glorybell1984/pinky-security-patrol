"""CSV event log: every detection, state change and decision."""
import csv
import os
from datetime import datetime

HEADER = ['time', 'source', 'marker_id', 'event', 'state', 'warning_level',
          'x_map', 'y_map', 'robot_x', 'robot_y', 'patrol_pass', 'message']


class EventLogger:

    def __init__(self, log_dir):
        log_dir = os.path.expanduser(log_dir)
        os.makedirs(log_dir, exist_ok=True)
        stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.path = os.path.join(log_dir, f'events_{stamp}.csv')
        self._file = open(self.path, 'w', newline='')
        self._writer = csv.writer(self._file)
        self._writer.writerow(HEADER)
        self._file.flush()

    def log(self, **row):
        row.setdefault('time', datetime.now().isoformat(timespec='seconds'))
        self._writer.writerow([row.get(k, '') for k in HEADER])
        self._file.flush()

    def close(self):
        self._file.close()
