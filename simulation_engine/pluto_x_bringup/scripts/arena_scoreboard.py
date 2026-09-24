#!/usr/bin/env python3
"""arena_scoreboard: live score and final result of an arena run.

Terminal:   every arena event as it happens, a status line every second,
            and a result table when the time limit ends the run.
RViz:       /arena/viz (visualization_msgs/MarkerArray, frame 'world'):
            score / time-left text above the field, the final result, and
            the vehicle's flight path (from /sim/pluto/odometry, development
            ground truth; judges' view, not a participant interface).
"""

import rclpy
import yaml
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Float64, Int32, String
from visualization_msgs.msg import Marker, MarkerArray

TEXT_POSITION = (0.0, 0.0, 3.0)
TRAIL_PERIOD_S = 0.1
TRAIL_MAX_POINTS = 3000


def colour(r, g, b, a=1.0):
    from std_msgs.msg import ColorRGBA
    return ColorRGBA(r=float(r), g=float(g), b=float(b), a=float(a))


class Scoreboard(Node):

    def __init__(self):
        super().__init__('arena_scoreboard')
        self.score = 0
        self.remaining = None
        self.result = None
        self.pops = {}  # balloon name -> (time, event text)
        self.trail = []
        self.last_trail_t = -1.0
        self.pub = self.create_publisher(MarkerArray, '/arena/viz', 10)
        self.create_subscription(Int32, '/arena/score', self.on_score, 10)
        self.create_subscription(Float64, '/arena/time_remaining', self.on_remaining, 10)
        self.create_subscription(String, '/arena/events', self.on_event, 50)
        self.create_subscription(String, '/arena/result', self.on_result, 10)
        self.create_subscription(Odometry, '/sim/pluto/odometry', self.on_odom, 50)
        self.create_timer(1.0, self.status_line, clock=rclpy.clock.Clock())  # wall clock
        self.create_timer(0.2, self.publish_markers, clock=rclpy.clock.Clock())

    # -- inputs ------------------------------------------------------------

    def on_score(self, msg):
        self.score = msg.data

    def on_remaining(self, msg):
        self.remaining = msg.data

    def on_event(self, msg):
        text = msg.data
        self.get_logger().info(f'[arena] {text}')
        if ' POP ' in text:
            try:
                t = float(text.split('t=')[1].split(' s')[0])
                name = text.split('(')[1].split(',')[0]
                self.pops[name] = t
            except (IndexError, ValueError):
                pass

    def on_result(self, msg):
        if self.result is not None:
            return
        self.result = yaml.safe_load(msg.data)
        self.print_result()
        self.publish_markers()

    def on_odom(self, msg):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.result is not None or t - self.last_trail_t < TRAIL_PERIOD_S:
            return
        self.last_trail_t = t
        p = msg.pose.pose.position
        self.trail.append(Point(x=p.x, y=p.y, z=p.z))
        del self.trail[:-TRAIL_MAX_POINTS]

    # -- outputs -----------------------------------------------------------

    def status_line(self):
        if self.result is None and self.remaining is not None:
            self.get_logger().info(f'score {self.score:4d} | time left {self.remaining:5.1f} s')

    def print_result(self):
        r = self.result
        lines = ['',
                 '=' * 64,
                 '  AVATIC ARENA - RESULT',
                 '=' * 64,
                 f"  time limit {r['time_limit_s']:.1f} s (clock from "
                 f"{'arming' if r['clock_start'] == 'armed' else 'simulation start'}),"
                 f" ended at sim time {r['ended_at_sim_time_s']:.3f} s",
                 '',
                 f"  {'balloon':<20}{'colour':<9}{'points':>7}   {'popped':<7}{'at':>9}",
                 f"  {'-' * 58}"]
        for b in r['balloons']:
            at = self.pops.get(b['name'])
            lines.append(f"  {b['name']:<20}{b['color']:<9}{b['points']:>7}   "
                         f"{'YES' if b['popped'] else '-':<7}"
                         f"{(f'{at:.2f} s') if at is not None else '':>9}")
        lines += [f"  {'-' * 58}",
                  f"  SCORE  {r['score']} / {r['max_score']}    "
                  f"({r['popped']} of {len(r['balloons'])} balloons)",
                  '=' * 64, '']
        self.get_logger().info('\n'.join(lines))

    def publish_markers(self):
        markers = MarkerArray()
        text = Marker()
        text.header.frame_id = 'world'
        text.header.stamp = self.get_clock().now().to_msg()
        text.ns, text.id = 'scoreboard', 0
        text.type, text.action = Marker.TEXT_VIEW_FACING, Marker.ADD
        text.pose.position.x, text.pose.position.y, text.pose.position.z = TEXT_POSITION
        text.pose.orientation.w = 1.0
        text.scale.z = 0.35
        if self.result is None:
            left = f'{self.remaining:.1f} s left' if self.remaining is not None else 'waiting'
            text.text = f'SCORE {self.score}   |   {left}'
            text.color = colour(1, 1, 1)
        else:
            r = self.result
            text.text = (f"TIME UP - FINAL SCORE {r['score']} / {r['max_score']}"
                         f"  ({r['popped']} balloons)")
            text.color = colour(1.0, 0.85, 0.1)
        markers.markers.append(text)
        if len(self.trail) >= 2:
            path = Marker()
            path.header = text.header
            path.ns, path.id = 'flight_path', 1
            path.type, path.action = Marker.LINE_STRIP, Marker.ADD
            path.pose.orientation.w = 1.0
            path.scale.x = 0.03
            path.color = colour(1.0, 0.3, 0.1, 0.9)
            path.points = list(self.trail)
            markers.markers.append(path)
        self.pub.publish(markers)


def main():
    rclpy.init()
    node = Scoreboard()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass  # Ctrl-C / launch shutdown: the result was already printed
    finally:
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
