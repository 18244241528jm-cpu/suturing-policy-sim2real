#!/usr/bin/env python3
"""
dVRK Policy Adapter — 将 Image IL 模型输出桥接到 dVRK CRTK 接口

用法：
  source /opt/ros/humble/setup.bash
  python3 dvrk_policy_adapter.py --arm PSM1 --camera_topic /your/camera/topic

前置：dVRK 已 Power On、Home；相机 topic 已发布。
"""

import sys
import os
import argparse
import numpy as np
import torch
import torch.nn as nn
import torchvision.transforms as transforms
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import Image as RosImage, JointState
from std_msgs.msg import Empty

# 路径设置
_script_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(_script_dir, '..', 'RL'))
# 本地 r3m 包（与 Task_evaluation_R3M 相同）
_r3m_dir = os.path.join(_script_dir, 'r3m')
if os.path.isdir(_r3m_dir):
    sys.path.insert(0, _r3m_dir)
# ral（AMBF ros_abstraction_layer）用于 AMBF 相机订阅，QoS 与 Task_evaluation_R3M 一致
_ambf_client_py = os.path.normpath(os.path.join(_script_dir, '..', '..', 'ambf-ambf-3.0', 'ros_modules', 'ambf_client', 'python'))
if os.path.isdir(_ambf_client_py):
    sys.path.insert(0, _ambf_client_py)
from r3m import load_r3m

# 与 Task_evaluation_R3M 相同的 step_size（Approach 任务）
TRANS_STEP = 1.0e-3
ANGLE_STEP = np.deg2rad(3)
JAW_STEP = 0.3
STEP_SIZE = np.array([TRANS_STEP, TRANS_STEP, TRANS_STEP, ANGLE_STEP, ANGLE_STEP, ANGLE_STEP, JAW_STEP], dtype=np.float32)

# dVRK jaw：0=闭合，弧度；Policy：0=开，1=闭。线性映射。
JAW_MAX_RAD = 0.8  # 实测或查器械文档，Large Needle Driver 约 0.8


def quat_to_rpy(qx, qy, qz, qw):
    """四元数转 RPY (rad)"""
    from scipy.spatial.transform import Rotation as R
    r = R.from_quat([qx, qy, qz, qw])
    return r.as_euler('xyz')  # roll, pitch, yaw


def rpy_to_quat(roll, pitch, yaw):
    """RPY (rad) 转四元数 [x,y,z,w]"""
    from scipy.spatial.transform import Rotation as R
    r = R.from_euler('xyz', [roll, pitch, yaw])
    return r.as_quat()  # [x,y,z,w]


class DVRKPolicyAdapter(Node):
    def __init__(self, arm="PSM1", camera_topic="/dvrk/ECM/camera/image", model_path=None, control_hz=50, dry_run=False, mock_proprio=None, dry_run_steps=0, use_ral_camera=False):
        super().__init__("dvrk_policy_adapter")
        self.arm = arm
        self.camera_topic = camera_topic
        self.control_hz = control_hz
        self.dry_run = dry_run
        self.dry_run_steps = dry_run_steps  # >0 时跑完指定步数后退出
        self.use_ral_camera = use_ral_camera  # True 时用 ral 订阅相机（与 Task_evaluation_R3M 相同，兼容 AMBF）
        self.bridge = CvBridge()

        # 状态缓存
        self._measured_cp = None
        self._jaw_measured = None
        self._measured_cp_frame_id = None
        self._image = None
        self._image_received = False
        # dry_run 时用合成 proprio
        self._mock_proprio = np.array([0.0, 0.0, 0.10, 0.0, 0.0, 0.0, 0.5], dtype=np.float32) if mock_proprio is None else np.array(mock_proprio, dtype=np.float32)

        # 模型
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        self.model, self.r3m_model = self._load_model(model_path)
        self.transform = transforms.Compose([
            transforms.ToPILImage(),
            transforms.Resize((256, 256)),
            transforms.CenterCrop(224),
            transforms.ToTensor(),
            transforms.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225])
        ])

        # QoS：dVRK 用 BEST_EFFORT
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=1)

        if not dry_run:
            self.create_subscription(TransformStamped, f"/{arm}/measured_cp", self._measured_cp_cb, qos)
            self.create_subscription(JointState, f"/{arm}/jaw/measured_js", self._jaw_cb, qos)

        # 相机订阅：use_ral_camera 时用 ral（与 Task_evaluation_R3M 相同，兼容 AMBF）；否则用 rclpy
        self._ral_instance = None
        self._ral_thread = None
        if use_ral_camera:
            try:
                from ros_abstraction_layer import ral
                import threading
                self._ral_instance = ral("dvrk_adapter_camera")
                import time
                time.sleep(0.5)  # ral 初始化
                self._ral_instance.subscriber(camera_topic, RosImage, self._image_cb)
                self._ral_thread = threading.Thread(target=self._ral_instance.spin, daemon=True)
                self._ral_thread.start()
                self.get_logger().info(f"相机订阅使用 ral: {camera_topic}")
            except ImportError as e:
                self.get_logger().error(f"无法加载 ral，回退到 rclpy: {e}")
                use_ral_camera = False
        if not use_ral_camera:
            qos_cam = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE, history=HistoryPolicy.KEEP_LAST, depth=10)
            self.create_subscription(RosImage, camera_topic, self._image_cb, qos_cam)

        # 发布（dry_run 时也创建，但可不 publish）
        self._servo_cp_pub = self.create_publisher(TransformStamped, f"/{arm}/servo_cp", 1)
        self._jaw_pub = self.create_publisher(JointState, f"/{arm}/jaw/servo_jp", 1)
        self._hold_pub = self.create_publisher(Empty, f"/{arm}/hold", 1)

        self._step_count = 0
        self._control_timer = None  # 用于 dry_run 达到步数时取消
        mode = "DRY_RUN (no dVRK)" if dry_run else "LIVE"
        self.get_logger().info(f"Adapter: arm={arm}, camera={camera_topic}, control_hz={control_hz}, mode={mode}")

    def _load_model(self, model_path):
        if model_path is None:
            model_path = os.path.join(_script_dir, '..', '..', '..', 'base_env_imageIL', 'model_final.pth')
        if not os.path.exists(model_path):
            raise FileNotFoundError(f"Model not found: {model_path}")

        r3m_model = load_r3m("resnet50")
        r3m_model.eval()
        r3m_model.to(self.device)

        class BehaviorCloningModel(nn.Module):
            def __init__(self, r3m):
                super().__init__()
                self.r3m = r3m
                self.regressor = nn.Sequential(
                    nn.BatchNorm1d(2048 + 7),
                    nn.Linear(2048 + 7, 256),
                    nn.ReLU(),
                    nn.Linear(256, 7),
                    nn.Tanh()
                )

            def forward(self, x, proprio):
                with torch.no_grad():
                    v = self.r3m(x)
                return self.regressor(torch.cat((v, proprio), dim=1))

        model = BehaviorCloningModel(r3m_model).to(self.device)
        model.load_state_dict(torch.load(model_path, map_location=self.device))
        model.eval()
        return model, r3m_model

    def _measured_cp_cb(self, msg):
        self._measured_cp_frame_id = msg.header.frame_id
        t = msg.transform.translation
        r = msg.transform.rotation
        rpy = quat_to_rpy(r.x, r.y, r.z, r.w)
        self._measured_cp = np.array([t.x, t.y, t.z, rpy[0], rpy[1], rpy[2]], dtype=np.float32)

    def _jaw_cb(self, msg):
        if len(msg.position) > 0:
            self._jaw_measured = float(msg.position[0])

    def _image_cb(self, msg, *args, **kwargs):
        """ral 可能传入额外参数 camera_id 等"""
        try:
            self._image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            if not self._image_received:
                self._image_received = True
                self.get_logger().info(f"已收到相机图像: {self.camera_topic}")
        except Exception as e:
            self.get_logger().warn(f"Image conversion failed: {e}")

    def _get_proprio(self):
        """从 dVRK 状态构造 policy 输入 [x,y,z,rx,ry,rz,jaw]"""
        if self.dry_run:
            return self._mock_proprio.copy()
        if self._measured_cp is None or self._jaw_measured is None:
            return None
        jaw_01 = 1.0 - (self._jaw_measured / JAW_MAX_RAD)  # dVRK rad(0=闭) -> policy 0-1(1=闭)
        jaw_01 = np.clip(jaw_01, 0.0, 1.0)
        return np.append(self._measured_cp, jaw_01).astype(np.float32)

    def _predict_action(self, image_np, proprio):
        image = self.transform(image_np).unsqueeze(0).to(self.device)
        proprio_t = torch.tensor(proprio, dtype=torch.float32).unsqueeze(0).to(self.device)
        with torch.no_grad():
            action = self.model(image, proprio_t)
        return action.cpu().numpy().squeeze()

    def _publish_servo(self, goal_vector):
        """goal_vector: [x,y,z,roll,pitch,yaw,jaw] — 米、弧度、0-1"""
        # servo_cp
        msg = TransformStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._measured_cp_frame_id or "PSM1_psm_base_link"
        msg.transform.translation.x = float(goal_vector[0])
        msg.transform.translation.y = float(goal_vector[1])
        msg.transform.translation.z = float(goal_vector[2])
        q = rpy_to_quat(goal_vector[3], goal_vector[4], goal_vector[5])
        msg.transform.rotation.x = q[0]
        msg.transform.rotation.y = q[1]
        msg.transform.rotation.z = q[2]
        msg.transform.rotation.w = q[3]
        self._servo_cp_pub.publish(msg)

        # jaw: policy 0-1 -> dVRK rad
        jaw_rad = JAW_MAX_RAD * (1.0 - np.clip(goal_vector[6], 0.0, 1.0))
        jaw_msg = JointState()
        jaw_msg.header.stamp = msg.header.stamp
        jaw_msg.position = [jaw_rad]
        self._jaw_pub.publish(jaw_msg)

    def _get_image(self):
        """获取当前图像；dry_run 且无相机时用合成图"""
        if self._image_received and self._image is not None:
            return self._image
        if self.dry_run:
            # 首次用黑图时打印一次，便于排查
            if self._step_count == 0:
                self.get_logger().warn(
                    f"[dry_run] 未收到相机 {self.camera_topic}，使用黑图。"
                    " 请确认：1) AMBF 已带 GUI 启动  2) ros2 topic list | grep Image 能看到该 topic"
                )
            return np.zeros((480, 640, 3), dtype=np.uint8)  # 合成黑图，模型可跑
        return None

    def _control_timer_cb(self):
        proprio = self._get_proprio()
        if proprio is None:
            return
        img = self._get_image()
        if img is None:
            return
        action = self._predict_action(img, proprio)
        goal = proprio + STEP_SIZE * action
        if self.dry_run:
            self._step_count += 1
            if self._step_count % 50 == 1:
                self.get_logger().info(f"[dry_run] step={self._step_count} proprio={proprio[:3]} action={action[:3]} goal={goal[:3]}")
            if self.dry_run_steps > 0 and self._step_count >= self.dry_run_steps:
                self.get_logger().info(f"[dry_run] 完成 {self._step_count} 步，退出")
                if self._control_timer is not None:
                    self._control_timer.cancel()
                # 用节点内 timer 触发 shutdown（同线程），避免 threading.Timer 的 RuntimeError
                self.create_timer(0.05, self._dry_run_exit_cb)
                return  # 不 publish
        self._publish_servo(goal)

    def _dry_run_exit_cb(self):
        """dry_run 达到步数后由 timer 调用，在主 executor 中 shutdown"""
        if getattr(self, '_exit_requested', False):
            return
        self._exit_requested = True
        try:
            import rclpy
            rclpy.shutdown()
        except RuntimeError:
            pass

    def run(self):
        import rclpy
        from rclpy.executors import SingleThreadedExecutor
        executor = SingleThreadedExecutor()
        executor.add_node(self)

        # 启动时等待相机首帧（最多 5 秒），需 spin 才能处理回调
        self.get_logger().info("等待相机首帧...")
        for _ in range(100):
            executor.spin_once(timeout_sec=0.05)
            if self._image_received:
                self.get_logger().info("相机就绪")
                break
        else:
            self.get_logger().warn("5 秒内未收到相机，继续运行（可能用黑图）")

        self._control_timer = self.create_timer(1.0 / self.control_hz, self._control_timer_cb)
        executor.spin()


def main():
    import rclpy
    rclpy.init()
    parser = argparse.ArgumentParser()
    parser.add_argument("--arm", default="PSM1", help="dVRK arm namespace")
    parser.add_argument("--camera_topic", default="/dvrk/ECM/camera/image", help="Camera image topic")
    parser.add_argument("--model_path", default=None, help="Path to model_final.pth")
    parser.add_argument("--control_hz", type=int, default=50, help="Control loop frequency")
    parser.add_argument("--dry_run", action="store_true", help="No dVRK: use synthetic proprio + camera topic; 可配合 AMBF 相机验证")
    parser.add_argument("--dry_run_steps", type=int, default=0, help="dry_run 时跑完 N 步后退出，0=不限制")
    parser.add_argument("--use_ral_camera", action="store_true", help="用 ral 订阅相机（与 Task_evaluation_R3M 相同，AMBF ImageData 推荐）")
    parser.add_argument("--mock_proprio", type=str, default=None, help="dry_run 时合成 proprio，格式: x,y,z,rx,ry,rz,jaw")
    args = parser.parse_args()

    mock_proprio = None
    if args.mock_proprio:
        mock_proprio = [float(x) for x in args.mock_proprio.split(",")]

    node = DVRKPolicyAdapter(arm=args.arm, camera_topic=args.camera_topic, model_path=args.model_path, control_hz=args.control_hz, dry_run=args.dry_run, mock_proprio=mock_proprio, dry_run_steps=args.dry_run_steps, use_ral_camera=args.use_ral_camera)
    try:
        node.run()
    except (KeyboardInterrupt, Exception):
        pass  # dry_run 退出时 Timer 会 shutdown，可能触发异常
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except RuntimeError:
            pass  # 可能已被 Timer 调用过


if __name__ == "__main__":
    main()
