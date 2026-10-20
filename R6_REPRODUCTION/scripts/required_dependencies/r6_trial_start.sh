#!/bin/bash
# R6: start one attempt: trial dir, start.json (time + ECM js), 2 fresh snapshots with the arm at calib base, FS on the laptop for each.
set -e
TR=$1; mkdir -p "$TR"; H=$(dirname "$0")
source /opt/ros/jazzy/setup.bash; export ROS_AUTOMATIC_DISCOVERY_RANGE=SYSTEM_DEFAULT FASTDDS_BUILTIN_TRANSPORTS=UDPv4
python3 - "$TR" <<'PY'
import sys, json, time, rclpy
from sensor_msgs.msg import JointState
rclpy.init(); n = rclpy.create_node('r6_start'); L = {}
n.create_subscription(JointState, '/ECM/measured_js', lambda m: L.setdefault('e', list(m.position)[:4]), 10)
t = time.time()
while 'e' not in L and time.time() - t < 5: rclpy.spin_once(n, timeout_sec=0.1)
json.dump({'t': time.time(), 'date': time.strftime('%Y-%m-%dT%H:%M:%S%z'), 'ecm_js': L['e']}, open(sys.argv[1] + '/start.json', 'w'))
import os; os._exit(0)
PY
if [ "$2" == "--nofs" ]; then python3 "$H/r6_snap.py" "$TR/snap0" -k 6 > /dev/null; else
for k in 0 1; do python3 "$H/r6_snap.py" "$TR/snap$k" -k 6 > /dev/null; "$H/r6_fs_remote.sh" "$TR/snap$k" "$(basename $TR)_s$k"; done; fi
cat "$TR/start.json"
