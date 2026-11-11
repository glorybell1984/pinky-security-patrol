"""Narrator: patrol events -> one short sentence for the operator, by section.

Subscribes:
    /fleet/events       std_msgs/String (JSON rows from the task manager)
Publishes:
    /fleet/narration    std_msgs/String (JSON: time, text, source <provider>|template,
                        event, marker_id, section, level); keeps the last 10
                        for dashboards that connect later
    /fleet/zones        std_msgs/String (JSON list of sections, latched) for the dashboard

An LLM provider (llm_provider: claude | none, see narrator.py) writes the
sentence when its package, credentials and the network are there; otherwise a
template does. Narration runs in a worker thread, so a
slow or missing LLM never delays the patrol.
"""
import json
import queue
import threading
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String

from .narrator import Narrator, facts_for, make_provider
from .zones import ZoneMap


class NarratorNode(Node):

    def __init__(self):
        super().__init__('narrator')
        p = self.declare_parameter
        p('llm_provider', 'claude')           # claude | none (templates only)
        p('llm_model', '')                    # '' = the provider's default model
        p('llm_effort', 'low')                # one sentence: low effort is enough
        p('llm_timeout', 12.0)                # s per request, then the template
        p('language', 'English')
        p('zone_names', ['A', 'B', 'C'])
        p('zone_descriptions', [''])
        p('zone_polygons', [0.0])             # flat x, y list, zone_corners points per zone
        p('zone_corners', 4)
        g = lambda name: self.get_parameter(name).value  # noqa: E731

        names = list(g('zone_names'))
        polys = [float(v) for v in g('zone_polygons')]
        self.zones = ZoneMap.from_flat(names, list(g('zone_descriptions')), polys,
                                       int(g('zone_corners')))
        log = self.get_logger().warn
        provider = make_provider(g('llm_provider'), model=g('llm_model'),
                                 effort=g('llm_effort'), timeout=float(g('llm_timeout')), log=log)
        self.narrator = Narrator(provider, language=g('language'), log=log)

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        history = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub = self.create_publisher(String, '/fleet/narration', history)
        self.zones_pub = self.create_publisher(String, '/fleet/zones', latched)
        self.zones_pub.publish(String(data=self.zones.to_json()))
        self.create_subscription(String, '/fleet/events', self.on_event, 50)

        self.jobs = queue.Queue(maxsize=20)
        threading.Thread(target=self.worker, daemon=True).start()
        who = (f'LLM {self.narrator.provider.describe()}' if self.narrator.llm_enabled
               else f'templates: {self.narrator.llm_off_reason}')
        self.get_logger().info(f'Narrator ready: {len(self.zones.zones)} sections '
                               f'({", ".join(z.name for z in self.zones.zones)}), {who}')

    def on_event(self, msg):
        try:
            row = json.loads(msg.data)
        except ValueError:
            return
        facts = facts_for(row, self.zones)
        if facts is None:
            return
        try:
            self.jobs.put_nowait((row, facts))
        except queue.Full:                    # never block the executor; drop the oldest
            try:
                self.jobs.get_nowait()
            except queue.Empty:
                pass
            self.jobs.put_nowait((row, facts))

    def worker(self):
        while rclpy.ok():
            try:
                row, facts = self.jobs.get(timeout=0.5)
            except queue.Empty:
                continue
            text, source = self.narrator.narrate(facts)
            out = {'time': datetime.now().isoformat(timespec='seconds'), 'text': text,
                   'source': source, 'event': facts['event'],
                   'marker_id': row.get('marker_id', ''), 'section': facts['section'],
                   'level': facts['warning_level']}
            self.pub.publish(String(data=json.dumps(out, ensure_ascii=False)))
            self.get_logger().info(f'[{source}] {text}')


def main(args=None):
    rclpy.init(args=args)
    node = NarratorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
