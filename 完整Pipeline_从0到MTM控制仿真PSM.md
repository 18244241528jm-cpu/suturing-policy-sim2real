# 完整 Pipeline：你需要做的全部事情（按顺序）

> **来源**：3/6 组会 Ed 要求 + 3/4 组会  
> **目标**：在 ics-dvrk-11 上完成 teleoperation、ROS bag、Python 控制 PSM、reservation  
> **原则**：本文件包含全部操作，无需跳转其他文件

---

## 执行顺序总览

| 步 | 事项 | 完成 |
|----|------|------|
| 0 | 前置：账号、门禁、路径 | ☐ |
| 1 | 环境搭建：Phase 0–3（系统依赖、ros2_ws、AMBF、SRC） | ☐ |
| 2 | 跑 teleoperation：动 MTM，确认仿真 PSM 跟随 | ☐ |
| 3 | ROS bag 录制 + 识别 topic（camera、PSM、frame_id） | ☐ |
| 4 | Python 直接控制 PSM：轨迹（圆、方、正弦）+ 三种控制方式 + re-enable | ☐ |
| 5 | 熟悉 reservation 流程 | ☐ |
| 6 | 参考 Anton 的 dVRK README，熟悉 Python libraries | ☐ |
| — | Project Plan 汇报 3/10；发 GitHub ID 给 Adnan | ☐ |

---

# Step 0：前置

## 0.1 账号与门禁

| 事项 | 负责人 | 操作 |
|------|--------|------|
| dvrk 电脑账号 | Anton | 发邮件请求（见 `records/emails/Project34_Email_Draft_Anton_Michelle.md`），CC Adnan、Dr. Liu |
| Roboterium 门禁 | Michelle | 同上 |

## 0.2 路径说明

- `~` = `/home/<你的用户名>/`
- MTM JSON（构建后）：`~/ros2_ws/install/dvrk_config_jhu/share/jhu-daVinci/system-PSM1.json`
- 若路径不同：`find ~/ros2_ws/install -name "system-PSM1.json"`

---

# Step 1：环境搭建

## 1.1 检查并安装系统依赖

```bash
# 检查 Ubuntu
lsb_release -a   # 需 22.04 或 20.04/24.04

# 检查 sudo
sudo -v   # 无则请 lab 管理员加

# 检查 ROS 2 Humble
source /opt/ros/humble/setup.bash 2>/dev/null && ros2 --version
# 无则安装：
sudo apt update
sudo apt install -y ros-humble-desktop python-is-python3 python3-vcstool python3-colcon-common-extensions python3-pykdl

# 检查并安装 dVRK 依赖（Ubuntu 22.04）
sudo apt install -y libraw1394-dev libncurses5-dev qtcreator swig sox espeak cmake-curses-gui cmake-qt-gui git subversion libcppunit-dev libqt5xmlpatterns5-dev libbluetooth-dev libhidapi-dev python3-pyudev
sudo apt install -y ros-humble-joint-state-publisher ros-humble-joint-state-publisher-gui ros-humble-xacro

# 检查并安装 AMBF 依赖
sudo apt install -y build-essential cmake libasound2-dev libgl1-mesa-dev libglx0-dev libegl1-mesa-dev xorg-dev libusb-1.0-0-dev libyaml-cpp-dev libboost-all-dev freeglut3-dev ros-humble-cv-bridge ros-humble-image-transport

# 检查硬件权限
groups   # 需有 plugdev、dialout
# 无则：sudo usermod -aG plugdev $USER && sudo usermod -aG dialout $USER，然后注销重登
```

## 1.2 ros2_ws（dVRK）

```bash
source /opt/ros/humble/setup.bash
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
vcs import --input https://raw.githubusercontent.com/jhu-saw/vcs/main/ros2-dvrk-main.vcs --recursive
# 若 access denied：加 --retry 10 --workers 1

cd ~/ros2_ws
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --symlink-install
source ~/ros2_ws/install/setup.bash

# 验证
ros2 pkg list | grep dvrk_robot
find ~/ros2_ws/install -name "system-PSM1.json"
```

## 1.3 AMBF + ambf_ros_ws

```bash
cd ~
git clone https://github.com/WPI-AIM/ambf.git
# 若用 ambf-ambf-3.0：git clone ... ambf-ambf-3.0，向 Ed 确认版本

AMBF_ROOT=~/ambf   # 或 ~/ambf-ambf-3.0
cd $AMBF_ROOT/core && mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release && make -j$(nproc)

mkdir -p ~/ambf_ros_ws/src && cd ~/ambf_ros_ws/src
ln -sf "$AMBF_ROOT/ros_modules/ambf_msgs" .
ln -sf "$AMBF_ROOT/ros_modules/ambf_server" .
ln -sf "$AMBF_ROOT/ros_modules/ros_comm_plugin" .
ln -sf "$AMBF_ROOT/ros_modules/tf_function" .

cd ~/ambf_ros_ws
source /opt/ros/humble/setup.bash
colcon build --cmake-args -DAMBF_DIR="$AMBF_ROOT/core/build"
source ~/ambf_ros_ws/install/setup.bash
```

## 1.4 SRC

```bash
cd ~
git clone https://github.com/surgical-robotics-ai/surgical_robotics_challenge.git
# 若无 run_env_SIMPLE_LND_420006.sh，向 Ed 要 fork

cd ~/surgical_robotics_challenge/scripts
pip3 install -e .   # 无 sudo 用 pip3 install --user -e .

# 若 teleop 报错缺 ambf_client
cd ~/ambf_ros_ws/src && ln -sf "$AMBF_ROOT/ros_modules/ambf_client" .
cd ~/ambf_ros_ws && colcon build --packages-select ambf_client
```

---

# Step 2：跑 teleoperation（动 MTM，仿真 PSM 跟随）

**开 3 个终端，全部在 dvrk11 上。**

## 终端 1：AMBF 仿真

```bash
source /opt/ros/humble/setup.bash
source ~/ambf_ros_ws/install/setup.bash
AMBF_ROOT=~/ambf   # 或 ~/ambf-ambf-3.0
SRC_ROOT=~/surgical_robotics_challenge

export AMBF_PLUGINS_PATH="$AMBF_ROOT/core/build/ambf_plugins:$HOME/ambf_ros_ws/install/ros_comm_plugin/lib"
export PATH="$AMBF_ROOT/core/build/bin:$PATH"
export LD_LIBRARY_PATH="$AMBF_ROOT/core/build/lib:$LD_LIBRARY_PATH"

cd $SRC_ROOT
LIBGL_ALWAYS_SOFTWARE=1 ./run_env_SIMPLE_LND_420006.sh
```

**预期**：AMBF 窗口弹出，显示仿真 PSM。

## 终端 2：dVRK（MTM）

```bash
source ~/ros2_ws/install/setup.bash
ros2 run dvrk_robot dvrk_system -j ~/ros2_ws/install/dvrk_config_jhu/share/jhu-daVinci/system-PSM1.json
```

**预期**：dVRK GUI 出现，MTM 可动。

## 终端 3：Teleoperation

```bash
source /opt/ros/humble/setup.bash
source ~/ambf_ros_ws/install/setup.bash
cd ~/surgical_robotics_challenge/scripts/surgical_robotics_challenge/teleoperation
./mtm_psm_pair_teleop_420006.sh
```

**预期**：动物理 MTM，仿真 PSM 跟着动。✓ Step 2 完成

---

# Step 3：ROS bag 录制 + 识别 topic

**目标**：搞清 kinematic state、视频、数据存储格式（Ed 要求 1）；弃用 CSV，统一用 rosbag（Ed 要求 4）。

## 3.1 识别 topic 名称

在 Step 2 的 3 终端运行时，新开终端 4：

```bash
source ~/ros2_ws/install/setup.bash
source /opt/ros/humble/setup.bash

# 列出所有 topic
ros2 topic list

# 按关键词过滤
ros2 topic list | grep -Ei "camera|image|PSM|MTM|measured"

# 查看消息类型
ros2 topic info /PSM1/measured_cp -v
ros2 topic info /MTML/measured_cp -v

# 查看 frame_id
ros2 topic echo /PSM1/measured_cp --once
```

**记录到文档**：camera topic、PSM topic、MTM topic、frame_id、消息类型。

## 3.2 录制 rosbag

```bash
# 录制所有 topic
ros2 bag record -a -o ~/rosbag_teleop_$(date +%Y%m%d_%H%M%S)

# 或只录关键 topic（按 3.1 识别的名称替换）
ros2 bag record /MTML/measured_cp /MTMR/measured_cp /PSM1/measured_cp /PSM1/measured_jp \
  /ambf/env/cameras/cameraL/ImageData /ambf/env/cameras/cameraR/ImageData \
  -o ~/rosbag_teleop_$(date +%Y%m%d_%H%M%S)
```

按 Ctrl+C 停止录制。

## 3.3 查看 bag 信息

```bash
ros2 bag info ~/rosbag_teleop_<时间戳>
```

**IL 数据**：MTM 驱动 PSM 时，用 rosbag 录制命令流；理解 rosbag 与 labeling 流程，制定数据标注计划。✓ Step 3 完成

---

# Step 4：Python 直接控制 PSM

**目标**：不用 GUI，用 Python 程序化控制；获取 PSM handle，发 move 指令；实现轨迹（圆、方、正弦）；熟悉 Position、Velocity、Effort；学会 re-enable（Ed 要求 2、3）。

## 4.1 前置

- 真机：dVRK 已启动（`dvrk_system`），Power On、Home 完成
- 安装：`sudo apt install python3-pykdl`

## 4.2 基本代码（真机 dvrk_python）

创建 `~/psm_control_demo.py`：

```python
#!/usr/bin/env python3
import crtk
import dvrk
import numpy
import math

ral = crtk.ral('dvrk_python_node')
p = dvrk.psm(ral, 'PSM1')
ral.check_connections()
ral.spin()

p.enable()
p.home()

# 读取状态
jp, ts = p.measured_jp()   # 关节位置
cp, ts = p.measured_cp()   # 笛卡尔位姿

# 笛卡尔画圆
cp, _ = p.measured_cp()
center = cp.p
r = 0.02   # 半径 2cm
for i in range(36):
    theta = 2 * math.pi * i / 36
    goal = cp
    goal.p[0] = center[0] + r * math.cos(theta)
    goal.p[1] = center[1] + r * math.sin(theta)
    p.move_cp(goal).wait()

# 笛卡尔画方（在 xy 平面）
cp, _ = p.measured_cp()
for dx, dy in [(0.02,0), (0,0.02), (-0.02,0), (0,-0.02)]:
    goal = cp
    goal.p[0] += dx
    goal.p[1] += dy
    p.move_cp(goal).wait()
```

运行：

```bash
source ~/ros2_ws/install/setup.bash
python3 ~/psm_control_demo.py
```

## 4.3 三种控制方式

| 方式 | CRTK 接口 | 用途 |
|------|-----------|------|
| **Position** | `servo_cp`/`move_cp`、`servo_jp`/`move_jp` | 发目标位姿/关节角 |
| **Velocity** | `servo_cv`、`servo_jv` | 发速度指令 |
| **Effort** | `servo_cf`、`servo_jf` | 发力/力矩 |

- `servo_*`：高频闭环（如 100Hz policy），目标需单步可达
- `move_*`：大范围运动，内部做轨迹规划

## 4.4 re-enable（joint 超限或 controller 关闭后）

```python
p.enable()   # 重新使能
p.home()     # 回零
```

GUI：dVRK Arm 标签页点 Enable / Home。

## 4.5 Ed 要求：2 分钟内「让机器人画个圆」

- 提前写好脚本（圆、方、正弦等）
- 上机：`source ~/ros2_ws/install/setup.bash` → `python3 your_script.py`
- 录制 endoscope 视频：`ros2 bag record` 录相机 topic，与 PSM 运动同步

✓ Step 4 完成

---

# Step 5：熟悉 reservation 流程

- **联系**：Anton（若被预订但无人用，可联系）
- **方式**：向 Ed/Anton 确认预约系统（网页、邮件、表格）
- **记录**：预约成功后记下时间、station（如 dvrk11）

✓ Step 5 完成

---

# Step 6：参考 Anton 的 dVRK README

- **文档**：Anton 编写的 dVRK README，视为参考标准
- **Python libraries**：PSM/MTM 控制约 5–6 行代码
- **Video**：录制 ROS 视频
- **Ed 会发**：Python 控制示例链接

收到后按示例练习。✓ Step 6 完成

---

# 路径速查

| 变量 | 值 |
|------|-----|
| AMBF_ROOT | ~/ambf 或 ~/ambf-ambf-3.0 |
| SRC_ROOT | ~/surgical_robotics_challenge |
| MTM JSON | ~/ros2_ws/install/dvrk_config_jhu/share/jhu-daVinci/system-PSM1.json |

---

# 故障排查

| 现象 | 处理 |
|------|------|
| qladisp 未找到 | `source ~/ros2_ws/install/setup.bash` |
| qladisp 权限错误 | `sudo usermod -aG plugdev $USER` 和 `sudo usermod -aG dialout $USER`，注销重登；仍不行联系 Ed/Anton |
| teleop 找不到 MTM topic | 确认终端 2 已启动 dvrk_system |
| 仿真 PSM 不动 | 确认 3 终端同一机器、同一 ROS 网络 |
| ModuleNotFoundError: surgical_robotics_challenge | `cd ~/surgical_robotics_challenge/scripts && pip3 install -e .` |
| 找不到 system-PSM1.json | `find ~/ros2_ws/install -name "system*.json"` 用输出路径 |
| AMBF segfault | `LIBGL_ALWAYS_SOFTWARE=1` 或 `-g 0` |
| sudo 无权限 | 请 lab 管理员加 |

---

*整理日期：2026-03-10*
