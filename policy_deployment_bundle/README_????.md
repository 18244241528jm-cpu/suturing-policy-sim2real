# Image IL (R3M) è¯„ä¼°è¿è¡Œè¯´æ˜Ž

ä½¿ç”¨ **SurgicAI-SurgicAI-DR-RAL** ç‰ˆæœ¬çš„ `Task_evaluation_R3M.py` é‡è·‘ base_env_imageIL æ¨¡åž‹ã€‚

## ä¿®æ”¹å†…å®¹

- **path_to_add**ï¼šæ”¹ä¸ºæœ¬ä»“åº“ `RL` ç›®å½•ï¼ˆApproach_env ç­‰ï¼‰
- **model_path**ï¼šæ”¹ä¸º `base_env_imageIL/model_final.pth`ï¼ˆä¸Ž project34 åŒçº§ï¼‰
- **ros_abstraction_layer**ï¼šè‡ªåŠ¨æ·»åŠ  AMBF `ambf_client/python` åˆ° Python è·¯å¾„ï¼ˆæ”¯æŒ ROS2ï¼‰
- **results_dir**ï¼šæ”¹ä¸º `Image_IL/Results`
- **max_episode_steps**ï¼š500

## å‰ç½®æ¡ä»¶

1. **WSL2 + Ubuntu 22.04 + ROS2 Humble**
2. **AMBF ä»¿çœŸå™¨å·²å¯åŠ¨**ï¼Œå¹¶åŠ è½½ SRC åœºæ™¯ï¼ˆå‘å¸ƒ `/ambf/env/cameras/cameraL/ImageData`ï¼‰
3. **base_env_imageIL** ä¸Ž project34 åŒçº§ï¼š`cis2/base_env_imageIL/model_final.pth`
4. **ambf-ambf-3.0** åœ¨ project34 å†…ï¼š`project34/ambf-ambf-3.0`

## è¿è¡Œæ­¥éª¤

### 1. å¯åŠ¨ AMBF ä»¿çœŸ

```bash
cd /mnt/c/Users/30518/OneDrive\ -\ Johns\ Hopkins/Desktop/cis2/project34/ambf-ambf-3.0
source /opt/ros/humble/setup.bash
source ~/ambf_ros_ws/install/setup.bash
export AMBF_PLUGINS_PATH="$(pwd)/core/build/ambf_plugins:$HOME/ambf_ros_ws/install/ros_comm_plugin/lib"
LIBGL_ALWAYS_SOFTWARE=1 ./run_ambf.sh   # æˆ–æŒ‰éœ€åŠ è½½ SRC launch æ–‡ä»¶
```

### 2. è¿è¡Œ IL è¯„ä¼°

```bash
cd /mnt/c/Users/30518/OneDrive\ -\ Johns\ Hopkins/Desktop/cis2/project34/SurgicAI-SurgicAI-DR-RAL/Image_IL
chmod +x run_il_eval.sh
./run_il_eval.sh
```

æˆ–ç›´æŽ¥ï¼š

```bash
cd .../SurgicAI-SurgicAI-DR-RAL/Image_IL
source /opt/ros/humble/setup.bash
source ~/ambf_ros_ws/install/setup.bash
python3 Task_evaluation_R3M.py --task_name Approach --view_name front --trans_error 0.5 --angle_error 15
```

### 3. ä¾èµ–

- `torch`, `torchvision`, `gymnasium`, `stable_baselines3`, `r3m`, `cv_bridge`, `sensor_msgs`
- ROS2 çš„ `cv_bridge`ï¼š`sudo apt install ros-humble-cv-bridge`

## ç»“æžœ

- æŽ§åˆ¶å°è¾“å‡ºï¼šæˆåŠŸçŽ‡ã€å¹³å‡è½¨è¿¹é•¿åº¦ã€å¹³å‡æ­¥æ•°
- ç»“æžœæ–‡ä»¶ï¼š`Image_IL/Results/Approach_front_drTrue_results.txt`
