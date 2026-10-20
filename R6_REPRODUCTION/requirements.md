# 需要准备什么

## 1. 复跑随包样例

只需要 Python 3、numpy、opencv-python、scipy。无需 GPU、ROS、机器人、FP 或 RL。

包内已带左右图、人工点选、保存的 FS 视差，以及 r2_arc.py、r5_grasp_v3.py 两个代码依赖。运行前按协议设置 PYTHONPATH，无需修改 import。

历史精简数据保存为 `fs_disp.npz`，键是 `disp`，单位是全分辨率像素；原 r6_perceive.py 读取 `fs_disp.npy`。协议只在临时工作副本中展开视差。

本次用于样例验证的环境：Python 3.14.2、numpy 2.3.5、opencv-python 4.13.0.92、scipy 1.16.3。这是验证记录，不是要求覆盖 3090 的现有环境。

## 2. 在 Adnan 3090 上重新计算 FS

| 必需项 | 当前状态 |
|---|---|
| FoundationStereo 源码及可用 CUDA/PyTorch 环境 | 到实验室后核对 R5 已有安装 |
| `pretrained_models/23-51-11/model_best_bp2.pth` | 到工作站核对文件及哈希；未放进此目录 |
| 与权重配对的 `cfg.yaml` | 必须和权重在同一目录 |
| imageio、omegaconf 及 FoundationStereo 自身依赖 | 使用原项目环境；不要仅按 r6_fs_infer 的 import 安装 |
| DINOv2 Torch Hub 源码、timm EdgeNeXt 预训练缓存 | 首次启动可能联网获取；离线使用须提前备齐 |

本包包含 `scripts/required_dependencies/r6_fs_infer.py`。设置 FoundationStereo 的 PYTHONPATH 后直接调用，输入/输出目录通过参数传入。

`r6_fs_remote.sh` 是原笔记本的 SSH/WSL 入口，带旧机器路径；此次只作为原始依赖保留，**不要用它连接 3090**。

## 3. 真机运行

- 实验室 dVRK、PSM1 ProGrasp 400093、ECM 双目相机及现场操作者。
- 机器人端 ROS2 Jazzy 和已构建的 `~/ros2_ws`：dvrk_robot、dvrk_video、dvrk_config_jhu。
- Python ROS 模块：rclpy、sensor_msgs、geometry_msgs、std_msgs、crtk_msgs、cv_bridge、message_filters；另需 numpy、opencv、scipy。
- 当前摆放的标定、beta 文件、新鲜图像、clicks.json 和 template_clicks.json。历史标定仅为格式示例。

原 r6_grasp.py 固定读取：

```text
~/suturing-policy-sim2real/records/logs/R6_20261016/
├── calib/manifest.json
├── calib/calibration_r6.json
└── beta/beta_sweep.json
```

不改代码的前提下，要在这个位置提供**当前有效文件**。若已有历史文件，先由现场操作者核对，不能用示例自动覆盖。脚本中的 R2 TCP 常量也绑定原器械。

**仍缺的验证：** 3090 的 R5 环境是否可直接运行这版 FS、当前装置标定是否有效、现场 ROS 通信是否正常。只有到实验室后才能确认。
