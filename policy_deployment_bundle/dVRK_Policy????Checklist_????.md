# 第一次 Policy 真机最小闭环 Checklist / First Policy Minimal Closed-Loop Checklist

> **用途 / Purpose**：用于第一次把 `policy -> ROS/CRTK -> PSM` 接到真机上。  
> Use this for the first real-robot `policy -> ROS/CRTK -> PSM` deployment.
>
> **今天目标 / Goal**：不是完成任务，而是完成**最小闭环**。  
> The goal is not task success, but a **minimal closed loop**:
> 1. 能收到相机图像 / Receive camera images  
> 2. 能收到 `measured_cp` / Receive `measured_cp`  
> 3. 能安全发出小幅 `servo_cp` 和 jaw 指令 / Safely send small `servo_cp` and jaw commands  
> 4. 能记录错误与失败模式 / Log failures and error modes
>
> **权威来源 / Source of truth**：以官方 dVRK 文档和现场 `ros2 topic ...` 输出为准。  
> Use official dVRK docs and live `ros2 topic ...` output as the source of truth.

---

## 0. 官方文档先读 / Read These Official Pages First

- [ ] 真机启动：`usage/real`  
  https://dvrk.readthedocs.io/main/pages/usage/real.html
- [ ] PSM / ECM / jaw / `servo_cp` API：`development/api/arms`  
  https://dvrk.readthedocs.io/main/pages/development/api/arms.html
- [ ] 坐标系：`development/frames`  
  https://dvrk.readthedocs.io/main/pages/development/frames.html
- [ ] ROS bridge：`development/components/fromROS`  
  https://dvrk.readthedocs.io/main/pages/development/components/fromROS.html
- [ ] 视频 / ECM ROS：`video/software/ros`  
  https://dvrk.readthedocs.io/main/pages/video/software/ros.html

**只从官方文档确认的事实 / Officially confirmed facts**
- `servo_cp` 是笛卡尔位置伺服目标，适合单步、小幅、可达的目标。
- jaw 走 `/<arm>/jaw/...` 相关接口。
- ECM / 视频走单独的视频 ROS 管线，topic 名称取决于现场 rig / gscam 配置。

**官方没替你决定的 / Must be measured on site**
- `camera_topic`
- `measured_cp.header.frame_id`
- jaw 全开 / 全闭弧度范围
- 实际 `arm`（`PSM1` 或 `PSM2`）
- 现场 ROS 发行版与 system JSON

---

## 1. 上机前准备 / Before Leaving

- [ ] 带门禁卡、电脑、电源、这份 checklist
- [ ] 准备好要查看的项目文件：
  - `project34/SurgicAI-SurgicAI-DR-RAL/Image_IL/dvrk_policy_adapter.py`
  - `project34/SurgicAI-SurgicAI-DR-RAL/Image_IL/README_dvrk_adapter.md`
- [ ] 明确今天使用的臂：`PSM1 / PSM2 = __________`
- [ ] 明确模型路径：`model_path = __________`
- [ ] 明确今天使用的模型版本：`IL / RL / 其他 = __________`
- [ ] 若你们计划和 Ed 一起做，先约定分工：
  - Ed：`dvrk_system`、`Power On`、`Home`、机器人安全
  - 你：topic 核对、adapter 参数、日志记录

### 1.1 上机前最好先问清 / Ask Before the Session If Possible

> 这些信息不是明天“必须先拿到”才能开始，但越早拿到，第一次部署越有意义。

- [ ] 模型对应的任务与版本：`这个 checkpoint 具体对应哪一个任务/训练配置？`
- [ ] 观测定义：`image + proprio` 的精确定义是否与当前 adapter 一致
- [ ] 动作语义：`7D action` 是不是当前 proprio 上的增量
- [ ] 训练时相机视角 / 相机摆位信息
- [ ] 训练时任务初始布置：needle、phantom、PSM 初始关系
- [ ] 若有：训练时使用的 frame / preprocessing / crop 假设

**建议优先向 Edward 追问 / Good follow-up questions for Edward**
- [ ] 是否能提供训练时仿真场景的截图或简图
- [ ] 是否能提供相机位置/朝向的近似信息
- [ ] 是否能提供初始 task setup 的近似摆放方式
- [ ] 当前给的模型更推荐先测 IL 还是 RL

---

## 2. 真机基础状态确认 / Real-Robot Baseline First

> 若这一步失败，今天不要进入 policy。

- [ ] 控制器已上电
- [ ] `qladisp` 能识别板卡
- [ ] system JSON 已确认：`__________`
- [ ] `dvrk_system` 启动成功，GUI 无报错
- [ ] `Power On` 完成
- [ ] `Home` 完成
- [ ] 机械臂稳定，无异常振动/异响
- [ ] 已约定异常处理：你停脚本，Ed 处理机器人

官方阅读位置：
- 真机启动与 `qladisp`、`Power On`、`Home`：  
  https://dvrk.readthedocs.io/main/pages/usage/real.html

---

## 3. 先做“接口与输入链”核实 / Verify Interface and Input Chain First

### 3.1 记录当前环境

```bash
source ~/ros2_ws/install/setup.bash
echo $ROS_DISTRO
ros2 topic list
```

- [ ] ROS 发行版：`__________`
- [ ] 实际 arm：`__________`
- [ ] system JSON：`__________`

### 3.2 核对 PSM / jaw topic

```bash
ros2 topic list | grep PSM
ros2 topic info /<arm>/measured_cp -v
ros2 topic info /<arm>/servo_cp -v
ros2 topic info /<arm>/jaw/measured_js -v
ros2 topic info /<arm>/jaw/servo_jp -v
```

- [ ] `/<arm>/measured_cp` 存在
- [ ] `/<arm>/servo_cp` 存在
- [ ] `/<arm>/jaw/measured_js` 存在
- [ ] `/<arm>/jaw/servo_jp` 存在

### 3.3 记录 `frame_id`

```bash
ros2 topic echo /<arm>/measured_cp --once
```

- [ ] `frame_id = __________`

### 3.4 找相机 topic

```bash
ros2 topic list | grep -Ei "image|camera|left|right"
ros2 topic echo <camera_topic> --once
```

- [ ] `camera_topic = __________`
- [ ] 能收到图像消息

如果现场相机 topic 不清楚，去读官方视频 ROS 文档：
- https://dvrk.readthedocs.io/main/pages/video/software/ros.html

### 3.5 记录 jaw 范围

```bash
ros2 topic echo /<arm>/jaw/measured_js --once
```

- [ ] 当前 jaw 弧度已记录
- [ ] 若可安全操作，记录全开 / 全闭大致范围：
  - `jaw_open_rad = __________`
  - `jaw_closed_rad = __________`

---

## 4. Adapter 参数核对 / Adapter Parameter Check

> 这一步对照项目实现，不把项目 README 当官方事实来源。

项目内参考文件：
- `project34/SurgicAI-SurgicAI-DR-RAL/Image_IL/dvrk_policy_adapter.py`
- `project34/SurgicAI-SurgicAI-DR-RAL/Image_IL/README_dvrk_adapter.md`

上机前必须明确：
- [ ] `--arm = __________`
- [ ] `--camera_topic = __________`
- [ ] `--model_path = __________`
- [ ] 当前模型版本：`IL / RL / 其他 = __________`
- [ ] `frame_id = __________`
- [ ] `JAW_MAX_RAD` 是否与当前器械一致
- [ ] 当前模型是不是需要 `image + proprio` 而不是 image-only
- [ ] 当前模型输出是不是按 `goal = proprio + step_size * action` 解释

**特别提醒 / Important warning**
- [ ] 不要把 `README_dvrk_adapter.md` 里的“`dry_run` = no dVRK”直接当真
- [ ] 先看脚本实际行为，再决定是否可以安全用 `--dry_run`

你需要知道的脚本行为：

```202:223:project34/SurgicAI-SurgicAI-DR-RAL/Image_IL/dvrk_policy_adapter.py
    def _publish_servo(self, goal_vector):
        """goal_vector: [x,y,z,roll,pitch,yaw,jaw] — 米、弧度、0-1"""
        # servo_cp
        msg = TransformStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._measured_cp_frame_id or "PSM1_psm_base_link"
        ...
        self._servo_cp_pub.publish(msg)

        # jaw: policy 0-1 -> dVRK rad
        jaw_rad = JAW_MAX_RAD * (1.0 - np.clip(goal_vector[6], 0.0, 1.0))
        jaw_msg = JointState()
        jaw_msg.header.stamp = msg.header.stamp
        jaw_msg.position = [jaw_rad]
        self._jaw_pub.publish(jaw_msg)
```

```239:259:project34/SurgicAI-SurgicAI-DR-RAL/Image_IL/dvrk_policy_adapter.py
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
            ...
                return  # 不 publish
        self._publish_servo(goal)
```

含义：
- `dry_run` 只在达到 `dry_run_steps` 时提前 `return`
- 平时它仍会进入 `_publish_servo(goal)`
- 所以若真机在同一 ROS 域内监听这些 topic，`dry_run` 也可能有风险

### 4.1 不要默认成立的假设 / Assumptions You Should Not Make Automatically

- [ ] 不要默认“有图像输入”就等于 policy 能正确理解场景
- [ ] 不要默认“读到了 `measured_cp`”就等于 proprio 语义与训练一致
- [ ] 不要默认“模型会动”就等于动作方向对任务有意义
- [ ] 不要默认真机相机可以任意摆放

你明天真正需要确认的是：
- [ ] 当前真机观测是否仍在训练分布附近
- [ ] 当前相机视角是否与仿真训练时大致一致
- [ ] 当前初始姿态 / task setup 是否离训练假设太远

---

## 5. 低风险验证 / Low-Risk Validation

### 5.1 不接真机控制栈时，验证模型和图像

- [ ] 确认模型文件真实存在
- [ ] 记录模型版本：`__________`
- [ ] 确认相机 topic 能收到图
- [ ] 若要跑 `dry_run`，先确认：
  - 当前没有真实 arm 在监听同名 `servo_cp` / `jaw/servo_jp`
  - 或你们已隔离 ROS 域

参考命令：

```bash
python3 dvrk_policy_adapter.py \
  --dry_run --dry_run_steps 20 \
  --arm <arm> \
  --camera_topic <camera_topic>
```

- [ ] 日志能显示收到图像
- [ ] 模型加载成功
- [ ] `goal` 计算与输出正常

### 5.2 只读验证真机状态流

> 如果你们担心 `dry_run` 风险，优先做“只读 topic 验证”，不要急着运行 adapter。

- [ ] `measured_cp` 能持续收到
- [ ] `jaw/measured_js` 能持续收到
- [ ] 相机图像能持续收到
- [ ] 三者时间上大体稳定

### 5.3 明天最值得优先完成的“最大可达成目标” / Best Realistic Goal for Tomorrow

> 如果你们明天只有一到两个 session，这里就是最现实、最有价值的目标。

- [ ] 把真实 setup 的 `camera_topic`、`frame_id`、jaw 范围、实际 arm 记录完整
- [ ] 确认模型能在现场环境里成功加载
- [ ] 确认 `image + measured_cp + jaw -> action` 这条链在现场能跑通
- [ ] 在安全前提下做一次**极小幅度** live test，观察动作是否基本合理
- [ ] 把失败模式分成：`interface` / `timing` / `geometry` / `perception`

如果这五项完成，明天就已经很有价值，足以支持 checkpoint。

---

## 6. 第一次真机最小闭环 / First Live Minimal Closed Loop

> 前提：前面所有空白项都已填上。

建议策略：
- [ ] 首跑只允许小幅动作
- [ ] 降低控制频率开始，例如先从较低频率试
- [ ] 人盯着 GUI、机器人姿态、声音、振动
- [ ] 手随时放在 `Ctrl+C` 和停止流程上

参考命令模板：

```bash
python3 dvrk_policy_adapter.py \
  --arm <arm> \
  --camera_topic <camera_topic> \
  --model_path <model_path> \
  --control_hz 10
```

现场观察项：
- [ ] 机器人有响应
- [ ] 运动幅度可控
- [ ] 运动方向基本合理
- [ ] jaw 开闭方向正确
- [ ] 没有异常抖动、异响、明显延迟

### 6.1 明天大概率做不了的事 / What Is Probably Not Realistic Tomorrow

> 这些不是“不重要”，而是通常需要更多准备、协调或额外信息。

- [ ] 不要把“第一次 live test”当成“明天就能完成自动缝合”
- [ ] 不要默认你们明天就能完成完整环境对齐（camera / needle / phantom / initial pose）
- [ ] 不要默认单次上机就能判断模型是否真正具备 Sim2Real 能力
- [ ] 不要默认 RL / IL 两个版本都能在明天完整比较
- [ ] 不要默认若结果不好就一定是 adapter 的问题，也可能是 scene geometry 与训练假设不一致

**通常还需要进一步协调 / Often requires follow-up coordination**
- [ ] 向 Edward 追问训练时场景信息或截图
- [ ] 与导师确认第一次 meaningful test 需要多接近仿真摆位
- [ ] 若要做更可靠的 image-based deployment，可能需要进一步相机/场景对齐
- [ ] 若要做严格验证，可能需要额外 rosbag / 截图 / 视频记录方案

官方提醒：
- `servo_cp` 适合单步、小幅、可达目标；大位移不该直接靠它硬推  
  https://dvrk.readthedocs.io/main/pages/development/api/arms.html
- 真实 dVRK 运行时要持续监控噪声、振动、负载和 IO 频率  
  https://dvrk.readthedocs.io/main/pages/usage/real.html

---

## 7. 失败模式记录 / Failure Logging

> 这一步和“跑成功”同样重要；checkpoint 可直接用。

至少记录这些字段：
- [ ] 日期 / 机器 / arm
- [ ] system JSON
- [ ] ROS 版本
- [ ] `camera_topic`
- [ ] `frame_id`
- [ ] `jaw_open_rad / jaw_closed_rad`
- [ ] `control_hz`
- [ ] `model_path`
- [ ] 成功或失败现象
- [ ] 截图 / 终端日志是否保存

把问题按四类记录：
- [ ] `perception`
- [ ] `frame / units / interface mismatch`
- [ ] `timing / control`
- [ ] `hardware / calibration`
- [ ] `geometry / scene mismatch`

可直接抄下面模板：

```text
Date:
Machine:
Arm:
System JSON:
ROS Distro:
Camera Topic:
frame_id:
Jaw Range:
Control Hz:
Model Path:

Observed Behavior:

Failure Category:
- perception / frame-interface / timing-control / hardware-calibration / geometry-scene

Evidence Saved:
- screenshot:
- terminal log:
- rosbag:
```

---

## 8. 收尾 / Wrap-Up

- [ ] 停止 adapter
- [ ] 确认机器人回到安全状态
- [ ] 若由 Ed 操作，完成 `Power Off`
- [ ] 回去后把结果写入：
  - `project34/records/logs/Project34_Dev_Log.md`

---

## 一页速记 / One-Page Summary

1. 先按官方文档确认真机基础状态，不要一上来跑 policy。
2. 先把 `arm`、`camera_topic`、`frame_id`、jaw 范围、ROS 版本记清楚。
3. `dvrk_policy_adapter.py` 是项目实现，不是官方保证；默认值一律现场核对。
4. `dry_run` 不能想当然当成“绝对安全”。
5. 第一次部署的价值在于建立最小闭环并记录失败模式。
6. 真正难点不只是 adapter 能不能跑，而是当前物理场景是否仍符合训练时的观测与几何假设。
