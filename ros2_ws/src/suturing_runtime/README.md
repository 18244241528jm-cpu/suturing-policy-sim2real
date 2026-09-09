# SurgicAI ROS 2 真机 Runtime

这个包是**独立真机 runtime**：把 dVRK、相机、外部 DA/mask/FoundationPose 和
安全 Reach 控制之间的数据固定成 `/suturing/*` ROS 2 topic。它不导入 AMBF 环境。

## 能做什么

- 把已知 JHU dVRK topics 转成稳定的 `/suturing/*` 接口；
- 启动前只读检查 topic 名、消息类型、消息到达和 `frame_id`；
- 操作者触发一组双目初始化快照，并拒绝跨帧 RGB/depth/mask；
- 自动导出锁存 RGB，并支持本机多边形或外部 PNG 的人工首帧 mask；
- 验收外部 DA、mask 和 FoundationPose 全候选输出；
- 按配置在 raw FP、平放方向、OBJ-origin支撑高度、planar mask+plane+CAD 之间做可切换针候选筛选，并要求人工确认；
- 把 PSM 位姿明确切成纯视觉、纯 FK 或带显式误差模型的候选融合；
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

针与 PSM 的所有模式、消息 schema、融合公式和当前必须实测但尚未填入的 hand-eye / mesh-control-point / covariance 项，见仓库根目录 [`docs/REAL_PERCEPTION_MODE_SWITCHES.md`](../../../docs/REAL_PERCEPTION_MODE_SWITCHES.md)。默认针为 `fp_only`、PSM 为 `disabled`，方便逐项 A/B；切换模式不需要改 Python 源码。

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

## 5.1 旧 Approach policy 真机桥接（实验性，先 shadow）

`approach_policy_node` 接收冻结的 `/suturing/approach/goal`、PSM measured pose 和
measured jaw，重建旧 checkpoint 的 GoalEnv observation，然后输出一个 7-D action。
其中 pose action 被换算成下一步 `/suturing/policy/next_goal`，再交给
`guarded_pose_executor` 做 workspace、freshness、限速和显式 arm 检查。jaw 目前只发布
`/suturing/policy/jaw_preview`，**不会控制真机 jaw**；在 jaw topic/type、限位和夹针检测
完成现场验收前不要把它直接 remap 到 dVRK command topic。

旧契约不是“每帧 measured pose 直接喂模型”：它以 measured feedback 初始化一次，随后
按 `command_state += action * step_size` 保持 command-integrated state；每一步仍等待 measured
PSM 收敛，防止命令跑在机器人前面。observation 固定为：

```text
observation  = concat(current7, desired7, desired7-current7), shape (21,)
achieved_goal/current xyz: m -> cm；desired xyz: m -> cm
RPY: rad，保留旧 roll<=0 分支；jaw: 旧 checkpoint 标量
action scale = [1.5 mm, 1.5 mm, 1.5 mm, 3 deg, 3 deg, 3 deg, 0.05]
```

### 安装模型推理依赖

在运行 ROS 2 节点的**同一个 Python 3** 环境安装（训练版本优先）：

```bash
python3 -m pip install --user \
  'torch>=2.0' 'gymnasium==0.29.1' 'stable-baselines3==2.2.1'
```

修改源码或第一次 clone 后构建：

```bash
cd "$HOME/suturing-policy-sim2real/ros2_ws"
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select suturing_runtime
source install/setup.bash
ros2 pkg executables suturing_runtime | grep approach_policy_node
```

模型必须保留为 Stable-Baselines3 外层 `.zip`，不要只取出里面的 `policy.pth`。自定义
`TD3_BC` 源码也必须存在；`surgicai_rl_path` 应指向包含
`RL_algo/td3_BC.py` 的目录，而不是仓库根目录。

旧 Approach 环境和当前 `approach_goal_matrix` 使用的是 **PSM2 gripper 几何**。如果真机
实际运动臂是 PSM2，必须在 `jhu_real.yaml` 同步修改至少：

```yaml
dvrk_topic_adapter.ros__parameters.raw_psm_pose: /PSM2/measured_cp
dvrk_topic_adapter.ros__parameters.raw_psm_twist: /PSM2/measured_cv
dvrk_topic_adapter.ros__parameters.raw_psm_jaw: /PSM2/jaw/measured_js
dvrk_topic_adapter.ros__parameters.fallback_psm_frame: PSM2_psm_base_link
approach_goal_builder.ros__parameters.target_frame: PSM2_psm_base_link
approach_policy_node.ros__parameters.command_frame: PSM2_psm_base_link
guarded_pose_executor.ros__parameters.command_frame: PSM2_psm_base_link
guarded_pose_executor.ros__parameters.raw_servo_topic: /PSM2/servo_cp
```

还要让 camera→PSM-base hand-eye TF 的 target frame 与此一致。内部标准 topic 仍可叫
`/suturing/psm1/*`，但容易混淆，现场记录中必须写清 raw arm。若确实要控制 PSM1，不能
直接假设左右臂 gripper orientation 相同：先改/验收 PSM1 goal geometry，再做 shadow，
否则不要 arm。

建议把模型放在 Git 仓库外，例如：

```bash
mkdir -p "$HOME/surgicai_models/approach"
cp /path/from/usb/approach_poseclose.zip \
  "$HOME/surgicai_models/approach/approach_poseclose.zip"
sha256sum "$HOME/surgicai_models/approach/approach_poseclose.zip"
```

### 第一次只做 shadow inference

先运行第 4 节的 `real_read_only.launch.py`，确认 FP goal、measured pose/jaw 和 frame 后，
单独启动 policy 节点。以下模式允许 `step_once` 产生 preview，但不会发布 executor 使用的
`/suturing/policy/next_goal`：

```bash
ros2 run suturing_runtime approach_policy_node --ros-args \
  --params-file "$HOME/suturing-policy-sim2real/ros2_ws/src/suturing_runtime/config/jhu_real.yaml" \
  -p enabled:=true \
  -p model_path:="$HOME/surgicai_models/approach/approach_poseclose.zip" \
  -p model_sha256:="PUT_SHA256_HERE" \
  -p surgicai_rl_path:="$HOME/suturing-policy-sim2real/src/SurgicAI/RL"

ros2 service call /suturing/policy/step_once std_srvs/srv/Trigger '{}'
ros2 topic echo /suturing/policy/status --qos-durability transient_local --once
ros2 topic echo /suturing/policy/action --once
ros2 topic echo /suturing/policy/next_goal_preview --once
ros2 topic echo /suturing/policy/jaw_preview --once
```

如果 status 是 `MODEL_ERROR`，先检查：路径指向外层 ZIP、SHA256、Python 能否 import
`torch/stable_baselines3/gymnasium`，以及 `surgicai_rl_path/RL_algo/td3_BC.py` 是否存在。

### 有操作者和急停时启动 policy pose 闭环

只有完成 shadow overlay、hand-eye、PSM1 frame、workspace 和 1.5 mm 单步方向验收后：

```bash
ros2 launch suturing_runtime real_policy_guarded.launch.py \
  config:="$HOME/suturing-policy-sim2real/ros2_ws/src/suturing_runtime/config/jhu_real.yaml" \
  model_path:="$HOME/surgicai_models/approach/approach_poseclose.zip" \
  model_sha256:="PUT_SHA256_HERE" \
  surgicai_rl_path:="$HOME/suturing-policy-sim2real/src/SurgicAI/RL" \
  operator_acknowledgement:=I_HAVE_OPERATOR_AND_ESTOP
```

另一个终端依次解锁。先 arm policy，让它发布第一步；再 arm executor 并只调用一次
`execute_once`。之后 policy 等 measured pose 收敛再给下一步，executor 连续跟随，policy
发布 complete 后 executor 才完成并 disarm：

```bash
ros2 service call /suturing/runtime/check std_srvs/srv/Trigger '{}'
ros2 service call /suturing/policy/arm std_srvs/srv/SetBool '{data: true}'
ros2 topic echo /suturing/policy/next_goal --once
ros2 topic echo /suturing/execution/preview --once
ros2 service call /suturing/execution/arm std_srvs/srv/SetBool '{data: true}'
ros2 service call /suturing/execution/execute_once std_srvs/srv/Trigger '{}'
```

同时观察：

```bash
ros2 topic echo /suturing/policy/status --qos-durability transient_local
ros2 topic echo /suturing/execution/status
```

停止时两个节点都停，executor stop 会撤销真机输出：

```bash
ros2 service call /suturing/policy/stop std_srvs/srv/Trigger '{}'
ros2 service call /suturing/execution/stop std_srvs/srv/Trigger '{}'
```

这个节点只解决 Stage1 Approach pose policy 的推理桥接，并不让旧模型自动适配真机域。
第一次部署至少要记录 FP/goal、21-D observation、7-D action、next goal、measured tracking
error 和 stop/fault；模型训练范围以外的针位姿应由上游 gate 拒绝。
默认还会拒绝初始 goal 超过 `10 mm/30 deg`、jaw 不在 `[0,1]`、或 goal/PSM 超出
workspace；这些只是最外层安全界限，不等于模型已经在整个范围内训练成功。

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
## 8. 真机运行前的离线分段门

不要用本 ROS package 的完整 launch 来判断 DA 或 FoundationPose 为什么失败。先回到仓库根目录，按
`scripts/run_real_perception_stage.py` 的 `inspect -> da -> mask -> bundle` 顺序检查，再用
`scripts/run_fp_stage_docker.sh` 单独 register。只有离线 bundle 的 RGB、深度、mask、K 与尺寸合同都通过后，
才进入本 package 的 ROS stamp/frame 对账与只读诊断。这样 DA 模型失败、mask 画错、raw/rect K 混用和 ROS
异步不会被混成同一个“主程序没反应”。
