# SurgicAI ROS 2 真机 Runtime

这个包是**独立真机 runtime**：把 dVRK、相机、外部 DA/mask/FoundationPose 和
安全 Reach 控制之间的数据固定成 `/suturing/*` ROS 2 topic。它不导入 AMBF 环境。

## 能做什么

- 把已知 JHU dVRK topics 转成稳定的 `/suturing/*` 接口；
- 启动前只读检查 topic 名、消息类型、消息到达和 `frame_id`；
- 操作者触发一组双目初始化快照，并拒绝跨帧 RGB/depth/mask；
- 自动导出锁存 RGB，并支持本机多边形或外部 PNG 的人工首帧 mask；
- 验收外部 DA、mask 和 FoundationPose 全候选输出；
- 用 phantom 平面方向筛选平放针候选，并要求人工确认；
- 同时生成 camera-frame 与 PSM-base 的冻结 Approach goal；
- 持续报告缺失输入和 freshness；
- 以 preview-only 或显式四重解锁的低速模式执行一次 PSM Reach。

R2 已包含经过 P5a 验收的 DA 推理节点，但默认关闭，必须配置 3.8 GiB checkpoint 和
Depth-Anything-V2 源码路径。人工首帧 mask 已有工具；自动 mask 与 FoundationPose GPU
backend 仍是外部程序。
它们共同使用以下边界：

```text
/suturing/external/depth              sensor_msgs/msg/Image, 32FC1, meters
/suturing/external/needle_mask        sensor_msgs/msg/Image, mono8/8UC1
/suturing/external/needle_candidates  std_msgs/msg/String, suturing.fp_candidates.v1 JSON
```

DA 节点的配置示例：

```yaml
metric_da_depth:
  ros__parameters:
    enabled: true
    checkpoint_path: /home/user/suturing-policy-sim2real/models/da/best.pth
    depth_anything_repo: /home/user/surgicai_external/Depth-Anything-V2
    device: cuda
```

它启动时核完整 SHA256，严格要求 ViT-L/518×518/FP32，并把预测发布到
`/suturing/external/depth`；后面的 adapter 仍会独立复查 source header。

三者必须复制锁存 RGB 的原始 stamp/frame/shape。runtime 自己产生
`/suturing/needle/pose_gated`；不能把 raw FP top-1 直接 remap 到该 topic。

## 人工首帧 mask（第一次真机推荐）

capture 后，mask 节点自动创建：

```text
~/surgicai_operator_masks/stamp_<source_stamp>/source_rgb.png
~/surgicai_operator_masks/stamp_<source_stamp>/source.json
```

本机操作可在 YAML 设 `gui_enabled: true`：左键沿针轮廓加点、右键撤销，按 `p` 发布。
若使用外部工具/GPT，只发送 `source_rgb.png`，把结果按原分辨率保存为同一 session 下的
`needle_mask.png`；白色只能是针，黑色是背景。然后执行：

```bash
ros2 service call /suturing/operator_mask/publish_file std_srvs/srv/Trigger '{}'
ros2 topic echo /suturing/operator_mask/status --qos-durability transient_local --once
```

必须打开同目录的 `mask_overlay.png` 人工确认。脚本能验证分辨率、面积、二值化和 source
header，不能证明人工/GPT 选中的像素语义正确。

## 一键只读诊断包

在 read-only runtime 已启动后执行：

```bash
cd "$HOME/suturing-policy-sim2real"
bash scripts/run_real_diagnostics.sh \
  "$HOME/surgicai_diagnostics/$(date +%Y%m%d_%H%M%S)" 20
```

它安全触发一次 snapshot，收集 topic graph、消息数/类型/frame/stamp、K/D/R/P、RGB、
DA depth NPY/预览、mask、PSM pose/twist/jaw、TF、FP/runtime/execution status 和 GPU 信息，
并行抓取 PSM1/ECM 的 `operating_state/error/warning/goal_reached` 单条 raw 消息，
并发布 **0 条机器人运动命令**。发回分析时优先发送：

```text
SHARE_THIS_FIRST.md
SUMMARY.json
```

摘要按 R0–R9 标出第一个未解决阶段，后续阶段标成 `BLOCKED_BY_EARLIER_STAGE`，避免把
上游缺 mask 错诊为 FoundationPose 故障。

诊断器能保存这些安全状态，不等于 executor 已把它们接成运动互锁。当前 executor 尚未
订阅 dVRK `operating_state/error/warning/goal_reached`；完成该合同并在真机监督验证前，
不要启用真实输出。

## 1. 构建

```bash
source /opt/ros/humble/setup.bash
cd "$HOME/suturing-policy-sim2real/ros2_ws"
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

## 2. 先检查真实 topics（只读，5 秒）

```bash
ros2 run suturing_runtime topic_preflight
```

成功必须是 `"passed": true`。它不发布 robot command。你给出的 topic list 只证明
名字存在；这里还验证类型和收到的消息。dVRK 2.2+ 使用 `PoseStamped`，旧版本/旧
项目草案可能是 `TransformStamped`；本包支持两者，但必须把 preflight 实测结果写入
`raw_psm_pose_type/raw_servo_type`，不要猜版本。

## 3. 先跑完整 mock

```bash
ros2 launch suturing_runtime mock_read_only.launch.py
```

另一个终端触发首帧并人工放行 mock overlay：

```bash
ros2 service call /suturing/initialization/capture std_srvs/srv/Trigger '{}'
ros2 topic echo /suturing/needle/gate_status --qos-durability transient_local --once
ros2 service call /suturing/needle/confirm_pending std_srvs/srv/SetBool '{data: true}'
ros2 service call /suturing/runtime/check std_srvs/srv/Trigger '{}'
```

预期 capture 成功、gate 为 `WAITING_OPERATOR`、确认后 runtime 为
`READY_READ_ONLY` 且 `command_enabled=false`。mock 候选是假数据，只验安装和接口。

## 4. 真机只读启动

先启动 dVRK system 和 ECM video；确认机器人已 home，但不启用本包运动输出：

```bash
ros2 launch suturing_runtime real_read_only.launch.py \
  config:="$HOME/suturing-policy-sim2real/ros2_ws/src/suturing_runtime/config/jhu_real.yaml"
```

配置文件中的 `camera_frame`、`required_needle_frame`、phantom plane 和 hand-eye TF
默认留空/关闭，因此第一次真实启动应停在明确的 `D12-E*`，不能直接 READY。

另一个终端按段检查：

```bash
source "$HOME/suturing-policy-sim2real/ros2_ws/install/setup.bash"
ros2 topic echo /suturing/psm1/measured_pose --once
ros2 service call /suturing/initialization/capture std_srvs/srv/Trigger '{}'
ros2 topic echo /suturing/perception_input/status --qos-durability transient_local --once
ros2 topic echo /suturing/fp_input/ready --qos-durability transient_local --once
ros2 topic echo /suturing/fp_candidate/status --qos-durability transient_local --once
ros2 topic echo /suturing/needle/gate_status --qos-durability transient_local --once
ros2 topic echo /suturing/needle/pose_gated --once
ros2 topic echo /suturing/approach/goal_camera --qos-durability transient_local --once
ros2 topic echo /suturing/approach/goal --once
ros2 service call /suturing/runtime/check std_srvs/srv/Trigger '{}'
```

只读模式即使调用 arm 也返回 `D10-E50-OUTPUT_LOCKED`。

## 5. 真机低速单次 Reach（必须现场人工监督）

开始前实测并修改 `jhu_real.yaml` 的 `target_frame/command_frame`、workspace、最大
位移/旋转、PSM arm、`servo_cp` 和 camera→PSM-base hand-eye TF。

只有操作者在急停旁并确认 overlay 后：

```bash
ros2 launch suturing_runtime real_guarded.launch.py \
  config:="$HOME/suturing-policy-sim2real/ros2_ws/src/suturing_runtime/config/jhu_real.yaml" \
  operator_acknowledgement:=I_HAVE_OPERATOR_AND_ESTOP
```

它仍不会自动运动。另一个终端依次执行：

```bash
ros2 service call /suturing/runtime/check std_srvs/srv/Trigger '{}'
ros2 service call /suturing/execution/arm std_srvs/srv/SetBool '{data: true}'
ros2 topic echo /suturing/execution/preview --once
ros2 service call /suturing/execution/execute_once std_srvs/srv/Trigger '{}'
```

随时停止并自动 disarm：

```bash
ros2 service call /suturing/execution/stop std_srvs/srv/Trigger '{}'
```

## 6. 默认 topic

| 作用 | 现场 raw topic | 标准 topic |
|---|---|---|
| 左图 | `/jhu_daVinci/left/image_rect` | `/suturing/camera/left/image` |
| 右图 | `/jhu_daVinci/right/image_rect` | `/suturing/camera/right/image` |
| PSM pose | `/PSM1/measured_cp` | `/suturing/psm1/measured_pose` |
| PSM velocity | `/PSM1/measured_cv` | `/suturing/psm1/measured_twist` |
| jaw | `/PSM1/jaw/measured_js` | `/suturing/psm1/jaw/measured_js` |
| PSM command | `/PSM1/servo_cp` | 只有 guarded executor 可发布 |

完整逐文件自然语言讲义见公开仓库
`docs/zh/真机主代码逐段讲义.md`；接口真值以
`config/jhu_real.yaml` 和各节点 `D10/D12-E*` 错误码为准。

## 7. 当前边界

- `[测量]` topic 名来自 2026-08-13 现场完整 topic list；消息类型仍要现场确认。
- `[假设]` workspace 是占位安全盒，不是真机测量值；未确认前不得启用输出。
- `[测量]` runtime 只做 D2 Reach；尚未接 RL，不发 jaw close，不声称 physical grasp。
- `[已证伪]` raw FP top-1、semantic mask、GT depth 或 scripted attachment 都不能
  冒充真机输入。
