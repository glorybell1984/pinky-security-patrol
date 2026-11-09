"""CSV event log: every detection, state change and decision."""
import csv
import os
from datetime import datetime

HEADER = ['time', 'marker_id', 'event', 'state', 'warning_level',
          'x_map', 'y_map', 'robot_x', 'robot_y', 'patrol_pass']


class EventLogger:

    def __init__(self, path):
        self.path = path
        new_file = not os.path.exists(path)
        self._file = open(path, 'a', newline='')
        self._writer = csv.writer(self._file)
        if new_file:
            self._writer.writerow(HEADER)

    def log(self, **row):
        row.setdefault('time', datetime.now().isoformat(timespec='seconds'))
        self._writer.writerow([row.get(k, '') for k in HEADER])
        self._file.flush()

    def close(self):
        self._file.close()
