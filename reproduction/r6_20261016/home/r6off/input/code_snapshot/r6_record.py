#!/usr/bin/env python3
"""R6: record the stereo camera view to mp4 (physical left = topic right, plus topic left), ~15 fps, with a timestamp overlay.
Stop with SIGINT/SIGTERM; files: video/<tag>_right.mp4, _left.mp4, _stamps.csv"""
import os, sys
if 'AMENT_PREFIX_PATH' not in os.environ:
    import shlex
    os.execvp('bash', ['bash', '-c', 'source /opt/ros/jazzy/setup.bash && export ROS_AUTOMATIC_DISCOVERY_RANGE=SYSTEM_DEFAULT FASTDDS_BUILTIN_TRANSPORTS=UDPv4 && exec python3 ' + ' '.join(shlex.quote(x) for x in [os.path.abspath(__file__)] + sys.argv[1:])])
import signal, time, cv2, rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
D = os.path.expanduser('~/suturing-policy-sim2real/records/logs/R6_20261016/video'); tag = sys.argv[1] if len(sys.argv) > 1 else time.strftime('%H%M%S')
rclpy.init(); n = rclpy.create_node('r6_record'); br = CvBridge(); W = {}; st = open(f'{D}/{tag}_stamps.csv', 'w'); stop = [False]
def cb(m, c):
    img = br.imgmsg_to_cv2(m, 'bgr8'); ts = m.header.stamp.sec + 1e-9 * m.header.stamp.nanosec
    if c not in W: W[c] = cv2.VideoWriter(f'{D}/{tag}_{c}.mp4', cv2.VideoWriter_fourcc(*'mp4v'), 15, (img.shape[1], img.shape[0]))
    cv2.putText(img, time.strftime('%H:%M:%S', time.localtime(ts)) + '.%02d' % int((ts % 1) * 100), (20, 40), 0, 1.2, (255, 255, 255), 2)
    W[c].write(img); st.write(f'{c},{ts:.3f}\n')
for c in ('left', 'right'): n.create_subscription(Image, f'/jhu_daVinci/{c}/image_rect', lambda m, c=c: cb(m, c), qos_profile_sensor_data)
signal.signal(signal.SIGINT, lambda *a: stop.__setitem__(0, True)); signal.signal(signal.SIGTERM, lambda *a: stop.__setitem__(0, True))
while not stop[0]: rclpy.spin_once(n, timeout_sec=0.1)
for w in W.values(): w.release()
st.close(); os._exit(0)
