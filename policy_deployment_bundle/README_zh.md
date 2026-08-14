# Image IL (R3M) 评估运行说明

使用 **SurgicAI-SurgicAI-DR-RAL** 版本的 `Task_evaluation_R3M.py` 重跑 base_env_imageIL 模型。

## 修改内容

- **path_to_add**：改为本仓库 `RL` 目录（Approach_env 等）
- **model_path**：改为 `base_env_imageIL/model_final.pth`（与 project34 同级）
- **ros_abstraction_layer**：自动添加 AMBF `ambf_client/python` 到 Python 路径（支持 ROS2）
- **results_dir**：改为 `Image_IL/Results`
- **max_episode_steps**：500

## 前置条件

1. **WSL2 + Ubuntu 22.04 + ROS2 Humble**
2. **AMBF 仿真器已启动**，并加载 SRC 场景（发布 `/ambf/env/cameras/cameraL/ImageData`）
3. **base_env_imageIL** 与 project34 同级：`cis2/base_env_imageIL/model_final.pth`
4. **ambf-ambf-3.0** 在 project34 内：`project34/ambf-ambf-3.0`

## 运行步骤

### 1. 启动 AMBF 仿真

```bash
cd /mnt/c/Users/30518/OneDrive\ -\ Johns\ Hopkins/Desktop/cis2/project34/ambf-ambf-3.0
source /opt/ros/humble/setup.bash
source ~/ambf_ros_ws/install/setup.bash
export AMBF_PLUGINS_PATH="$(pwd)/core/build/ambf_plugins:$HOME/ambf_ros_ws/install/ros_comm_plugin/lib"
LIBGL_ALWAYS_SOFTWARE=1 ./run_ambf.sh   # 或按需加载 SRC launch 文件
```

### 2. 运行 IL 评估

```bash
cd /mnt/c/Users/30518/OneDrive\ -\ Johns\ Hopkins/Desktop/cis2/project34/SurgicAI-SurgicAI-DR-RAL/Image_IL
chmod +x run_il_eval.sh
./run_il_eval.sh
```

或直接：

```bash
cd .../SurgicAI-SurgicAI-DR-RAL/Image_IL
source /opt/ros/humble/setup.bash
source ~/ambf_ros_ws/install/setup.bash
python3 Task_evaluation_R3M.py --task_name Approach --view_name front --trans_error 0.5 --angle_error 15
```

### 3. 依赖

- `torch`, `torchvision`, `gymnasium`, `stable_baselines3`, `r3m`, `cv_bridge`, `sensor_msgs`
- ROS2 的 `cv_bridge`：`sudo apt install ros-humble-cv-bridge`

## 结果

- 控制台输出：成功率、平均轨迹长度、平均步数
- 结果文件：`Image_IL/Results/Approach_front_drTrue_results.txt`
