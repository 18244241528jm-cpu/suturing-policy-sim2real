# SurgicAI ROS 2 真机 Runtime

这个包把 dVRK、相机、DA/mask/FP 和控制器之间的数据固定成 `/suturing/*` ROS 2
topic。主代码不再直接区分 AMBF 和真机。

## 能做什么

- 把已知 JHU dVRK topics 转成稳定的 `/suturing/*` 接口；
- 启动前只读检查 topic 名、消息类型、消息到达和 `frame_id`；
- 从已经通过物理门的 camera-frame needle pose，通过 TF 生成冻结 Approach goal；
- 持续报告缺失输入和 freshness；
- 以 preview-only 或显式四重解锁的低速模式执行一次 PSM Reach。

它不自动生成真实 needle mask，也不在本节点内运行 DA/FP。对应上游节点必须发布：

```text
/suturing/depth/metric       sensor_msgs/msg/Image, 32FC1, meters
/suturing/needle/mask        sensor_msgs/msg/Image, mono8
/suturing/needle/pose_gated  geometry_msgs/msg/PoseWithCovarianceStamped
```

`pose_gated` 必须已通过首帧物理一致性门，并保持 FoundationPose needle mesh frame
合同；不能把 raw FP top-1 直接 remap 到该 topic。

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

## 3. 先跑 mock

```bash
ros2 launch suturing_runtime mock_read_only.launch.py
ros2 topic echo /suturing/runtime/status --once
```

预期 `READY_READ_ONLY`、`command_enabled=false`。

## 4. 真机只读启动

先启动 dVRK system 和 ECM video；确认机器人已 home，但不启用本包运动输出：

```bash
ros2 launch suturing_runtime real_read_only.launch.py \
  config:="$HOME/suturing-policy-sim2real/ros2_ws/src/suturing_runtime/config/jhu_real.yaml"
```

另一个终端检查：

```bash
source "$HOME/suturing-policy-sim2real/ros2_ws/install/setup.bash"
ros2 topic echo /suturing/psm1/measured_pose --once
ros2 topic echo /suturing/needle/pose_gated --once
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

## 7. 当前边界

- `[测量]` topic 名来自 2026-08-13 现场完整 topic list；消息类型仍要现场确认。
- `[假设]` workspace 是占位安全盒，不是真机测量值；未确认前不得启用输出。
- `[测量]` runtime 只做 Reach，不发 jaw close，不声称 physical grasp。
- `[已证伪]` raw FP top-1、semantic mask、GT depth 或 scripted attachment 都不能
  冒充真机输入。
