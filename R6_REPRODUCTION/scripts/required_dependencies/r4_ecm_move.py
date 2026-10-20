#!/usr/bin/env python3
"""R4 F-2: guarded relative ECM joint move (yaw/pitch in deg, insertion in mm).

Refuses if the requested total offset from the recorded start pose exceeds limits
(|yaw|,|pitch| <= 15 deg, insertion 0..+60 mm) or a single step > 10 mm / 3 deg.
'--home' returns to the recorded start pose. Every move is appended to a JSONL log.

  python3 r4_ecm_move.py --record-start      # once, before any move
  python3 r4_ecm_move.py --dins 8            # insert 8 mm (towards the board)
  python3 r4_ecm_move.py --dyaw 3            # yaw +3 deg
  python3 r4_ecm_move.py --home              # back to the recorded start pose
"""
import os, sys
if 'AMENT_PREFIX_PATH' not in os.environ:  # re-exec inside the ROS 2 + dVRK environment
    import shlex
    os.execvp('bash', ['bash', '-c', 'source /opt/ros/jazzy/setup.bash && source ~/ros2_ws/install/setup.bash && exec python3 '
                       + ' '.join(shlex.quote(x) for x in [os.path.abspath(__file__)] + sys.argv[1:])])
import argparse, json, math, time, rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from crtk_msgs.msg import OperatingState
START = os.path.expanduser('~/suturing-policy-sim2real/records/logs/R4_20260928/F2_board/ecm_start_pose.json')
LOG = os.path.expanduser('~/suturing-policy-sim2real/records/logs/R4_20260928/F2_board/ecm_moves.jsonl')


class N(Node):
    def __init__(s):
        super().__init__('r4_ecm_move'); s.js = None; s.reached = None; s.state = None
        s.create_subscription(JointState, '/ECM/measured_js', lambda m: setattr(s, 'js', m), 10)
        s.sp = None; s.create_subscription(JointState, '/ECM/setpoint_js', lambda m: setattr(s, 'sp', m), 10)
        s.create_subscription(Bool, '/ECM/goal_reached', lambda m: setattr(s, 'reached', m.data), 10)
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        s.create_subscription(OperatingState, '/ECM/operating_state', s._on_state, latched)

    def _on_state(s, m):  # latched history can arrive out of order: keep the newest stamp only
        st = lambda x: x.header.stamp.sec + x.header.stamp.nanosec * 1e-9
        if s.state is None or st(m) >= st(s.state):
            s.state = m
        s.pub = s.create_publisher(JointState, '/ECM/move_jp', 10)

    def spin_until(s, cond, t):
        t0 = time.time()
        while time.time() - t0 < t and not cond():
            rclpy.spin_once(s, timeout_sec=0.05)
        return cond()


ap = argparse.ArgumentParser()
ap.add_argument('--dyaw', type=float, default=0); ap.add_argument('--dpitch', type=float, default=0); ap.add_argument('--dins', type=float, default=0)
ap.add_argument('--home', action='store_true'); ap.add_argument('--record-start', action='store_true'); ap.add_argument('--note', default='')
ap.add_argument('--state-dir', help='dir holding ecm_start_pose.json / ecm_moves.jsonl (default: day-1 F2_board); day 2 uses records/logs/R4_20260928/day2')
ap.add_argument('--min-ins', type=float, default=-0.5, help='lowest allowed insertion offset from start, mm (negative = retract)')
a = ap.parse_args()
if a.state_dir:
    START, LOG = os.path.join(os.path.expanduser(a.state_dir), 'ecm_start_pose.json'), os.path.join(os.path.expanduser(a.state_dir), 'ecm_moves.jsonl')
assert -40.0 <= a.min_ins <= -0.5, 'min-ins must be within [-40, -0.5] mm'
rclpy.init(); n = N()
assert n.spin_until(lambda: n.js is not None and n.state is not None, 5), 'no ECM state'
q = list(n.js.position)
if a.record_start:
    assert not os.path.exists(START), 'start pose already recorded'
    json.dump({'names': list(n.js.name), 'position': q, 'time': time.time()}, open(START, 'w'), indent=1)
    print('recorded start', q); raise SystemExit
q0 = json.load(open(START))['position']
n.spin_until(lambda: False, 0.5)  # drain the latched history before judging the state
assert n.state.state == 'ENABLED' and n.state.is_homed and not n.state.is_busy, f'ECM not ready: {n.state}'
# relative moves are applied to the SETPOINT (not measured) so tracking error does not accumulate (roll crept 1.8 deg before this fix)
assert n.spin_until(lambda: n.sp is not None, 5), 'no ECM setpoint_js'
q = list(n.sp.position)
goal = list(q0) if a.home else [q[0] + math.radians(a.dyaw), q[1] + math.radians(a.dpitch), q[2] + a.dins / 1000.0, q[3]]
if not a.home:
    step = [abs(math.degrees(goal[0] - q[0])), abs(math.degrees(goal[1] - q[1])), abs(goal[2] - q[2]) * 1000]
    tot = [math.degrees(goal[0] - q0[0]), math.degrees(goal[1] - q0[1]), (goal[2] - q0[2]) * 1000]
    assert step[0] <= 3.0 + 1e-6 and step[1] <= 3.0 + 1e-6 and step[2] <= 10.0 + 1e-6, f'step too large {step}'
    assert abs(tot[0]) <= 15 and abs(tot[1]) <= 15 and a.min_ins <= tot[2] <= 60, f'total offset out of limits {tot}'
m = JointState(); m.name = list(n.js.name); m.position = [float(x) for x in goal]
assert n.spin_until(lambda: n.pub.get_subscription_count() > 0, 5), 'no subscriber on /ECM/move_jp'
n.spin_until(lambda: False, 0.3)  # let discovery settle so the first message is not dropped
n.reached = None; n.pub.publish(m)
n.spin_until(lambda: n.reached is not None, 30)
n.spin_until(lambda: False, 0.5); qf = list(n.js.position)
rec = {'time': time.time(), 'from': q, 'goal': goal, 'final': qf, 'goal_reached': n.reached, 'home': a.home, 'note': a.note,
       'offset_from_start_deg_deg_mm': [math.degrees(qf[0] - q0[0]), math.degrees(qf[1] - q0[1]), (qf[2] - q0[2]) * 1000]}
open(LOG, 'a').write(json.dumps(rec) + '\n')
print('goal_reached', n.reached, 'offset from start (yaw deg, pitch deg, ins mm):', [round(x, 3) for x in rec['offset_from_start_deg_deg_mm']])
