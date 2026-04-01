# dVRK Policy Adapter

将 Image IL (R3M) 模型输出桥接到真实 dVRK 的 CRTK 接口。

## 逻辑

1. **订阅**：`/<arm>/measured_cp`、`/<arm>/jaw/measured_js`、相机 topic
2. **构造 proprio**：`[x,y,z,roll,pitch,yaw,jaw]`（米、弧度、jaw 0–1）
3. **模型推理**：`action = model(image, proprio)`，action 为 [-1,1]
4. **增量**：`goal = proprio + step_size * action`
5. **发布**：`/<arm>/servo_cp`、`/<arm>/jaw/servo_jp`

## 前置条件

- dVRK 已 Power On、Home
- 相机 topic 已发布（内窥镜或外接相机）
- ROS2 Humble
- 模型：`base_env_imageIL/model_final.pth`（与 project34 同级）

## 运行

```bash
source /opt/ros/humble/setup.bash
cd SurgicAI-SurgicAI-DR-RAL/Image_IL

# 默认：PSM1，相机 topic 需根据实际修改
python3 dvrk_policy_adapter.py --arm PSM1 --camera_topic /your/camera/topic

# 指定模型路径
python3 dvrk_policy_adapter.py --camera_topic /dvrk/ECM/camera/image --model_path /path/to/model_final.pth
```

## 不上机时的验证（dry_run）

`dry_run` 的设计目标是先验证 Adapter 逻辑和模型推理，但它**不是天然等价于“绝对不会发控制”**。请先确认当前 ROS 域内**没有真实 dVRK arm 在监听同名 `servo_cp` / `jaw/servo_jp`**，或先做 ROS 隔离，再使用下列命令。

```bash
# 方式 1：纯合成数据（无 ROS 相机），跑 100 步后自动退出
python3 dvrk_policy_adapter.py --dry_run --dry_run_steps 100

# 方式 2：配合 AMBF 仿真相机（先启动 AMBF+SRC）
python3 dvrk_policy_adapter.py --dry_run --camera_topic /ambf/env/cameras/cameraL/ImageData

# 方式 3：自定义合成 proprio
python3 dvrk_policy_adapter.py --dry_run --mock_proprio "0.0,0.0,0.12,0,0,0,0.5"
```

dry_run 会：用合成 proprio、有相机则用相机图否则用黑图；跑模型、算 goal、打印日志。它可用于验证模型加载、转换、闭环逻辑是否正常，但在共享 ROS 域里仍应先确认 topic 监听方，避免误把它当成“零风险真机测试”。

## 参数

| 参数 | 默认 | 说明 |
|------|------|------|
| `--arm` | PSM1 | dVRK 臂命名空间 |
| `--camera_topic` | /dvrk/ECM/camera/image | 相机图像 topic（**需向 Ed 确认**） |
| `--model_path` | ../base_env_imageIL/model_final.pth | 模型路径 |
| `--control_hz` | 50 | 控制频率 |

## 需实测/确认的项

1. **相机 topic**：dVRK 内窥镜或外接相机的实际 topic 名称
2. **frame_id**：`servo_cp` 的 `header.frame_id`（脚本中为占位，需 `ros2 topic echo /PSM1/measured_cp` 实测）
3. **JAW_MAX_RAD**：夹爪全开时的弧度（脚本默认 0.8，Large Needle Driver 约 0.8）

## 安全

- 首次上机建议与 Ed 同行
- 可先不接 policy，仅订阅 measured_cp 验证数据流
- 异常时发布 `/<arm>/hold` 保持当前位置
- 首次 live 前，先核实官方 dVRK 文档中的真机启动、`servo_cp` 语义和视频 ROS 管线：
  - https://dvrk.readthedocs.io/main/pages/usage/real.html
  - https://dvrk.readthedocs.io/main/pages/development/api/arms.html
  - https://dvrk.readthedocs.io/main/pages/video/software/ros.html
