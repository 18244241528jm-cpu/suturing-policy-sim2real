# dVRK Policy Adapter

å°† Image IL (R3M) æ¨¡åž‹è¾“å‡ºæ¡¥æŽ¥åˆ°çœŸå®ž dVRK çš„ CRTK æŽ¥å£ã€‚

## é€»è¾‘

1. **è®¢é˜…**ï¼š`/<arm>/measured_cp`ã€`/<arm>/jaw/measured_js`ã€ç›¸æœº topic
2. **æž„é€  proprio**ï¼š`[x,y,z,roll,pitch,yaw,jaw]`ï¼ˆç±³ã€å¼§åº¦ã€jaw 0â€“1ï¼‰
3. **æ¨¡åž‹æŽ¨ç†**ï¼š`action = model(image, proprio)`ï¼Œaction ä¸º [-1,1]
4. **å¢žé‡**ï¼š`goal = proprio + step_size * action`
5. **å‘å¸ƒ**ï¼š`/<arm>/servo_cp`ã€`/<arm>/jaw/servo_jp`

## å‰ç½®æ¡ä»¶

- dVRK å·² Power Onã€Home
- ç›¸æœº topic å·²å‘å¸ƒï¼ˆå†…çª¥é•œæˆ–å¤–æŽ¥ç›¸æœºï¼‰
- ROS2 Humble
- æ¨¡åž‹ï¼š`base_env_imageIL/model_final.pth`ï¼ˆä¸Ž project34 åŒçº§ï¼‰

## è¿è¡Œ

```bash
source /opt/ros/humble/setup.bash
cd SurgicAI-SurgicAI-DR-RAL/Image_IL

# é»˜è®¤ï¼šPSM1ï¼Œç›¸æœº topic éœ€æ ¹æ®å®žé™…ä¿®æ”¹
python3 dvrk_policy_adapter.py --arm PSM1 --camera_topic /your/camera/topic

# æŒ‡å®šæ¨¡åž‹è·¯å¾„
python3 dvrk_policy_adapter.py --camera_topic /dvrk/ECM/camera/image --model_path /path/to/model_final.pth
```

## ä¸ä¸Šæœºæ—¶çš„éªŒè¯ï¼ˆdry_runï¼‰

`dry_run` çš„è®¾è®¡ç›®æ ‡æ˜¯å…ˆéªŒè¯ Adapter é€»è¾‘å’Œæ¨¡åž‹æŽ¨ç†ï¼Œä½†å®ƒ**ä¸æ˜¯å¤©ç„¶ç­‰ä»·äºŽâ€œç»å¯¹ä¸ä¼šå‘æŽ§åˆ¶â€**ã€‚è¯·å…ˆç¡®è®¤å½“å‰ ROS åŸŸå†…**æ²¡æœ‰çœŸå®ž dVRK arm åœ¨ç›‘å¬åŒå `servo_cp` / `jaw/servo_jp`**ï¼Œæˆ–å…ˆåš ROS éš”ç¦»ï¼Œå†ä½¿ç”¨ä¸‹åˆ—å‘½ä»¤ã€‚

```bash
# æ–¹å¼ 1ï¼šçº¯åˆæˆæ•°æ®ï¼ˆæ—  ROS ç›¸æœºï¼‰ï¼Œè·‘ 100 æ­¥åŽè‡ªåŠ¨é€€å‡º
python3 dvrk_policy_adapter.py --dry_run --dry_run_steps 100

# æ–¹å¼ 2ï¼šé…åˆ AMBF ä»¿çœŸç›¸æœºï¼ˆå…ˆå¯åŠ¨ AMBF+SRCï¼‰
python3 dvrk_policy_adapter.py --dry_run --camera_topic /ambf/env/cameras/cameraL/ImageData

# æ–¹å¼ 3ï¼šè‡ªå®šä¹‰åˆæˆ proprio
python3 dvrk_policy_adapter.py --dry_run --mock_proprio "0.0,0.0,0.12,0,0,0,0.5"
```

dry_run ä¼šï¼šç”¨åˆæˆ proprioã€æœ‰ç›¸æœºåˆ™ç”¨ç›¸æœºå›¾å¦åˆ™ç”¨é»‘å›¾ï¼›è·‘æ¨¡åž‹ã€ç®— goalã€æ‰“å°æ—¥å¿—ã€‚å®ƒå¯ç”¨äºŽéªŒè¯æ¨¡åž‹åŠ è½½ã€è½¬æ¢ã€é—­çŽ¯é€»è¾‘æ˜¯å¦æ­£å¸¸ï¼Œä½†åœ¨å…±äº« ROS åŸŸé‡Œä»åº”å…ˆç¡®è®¤ topic ç›‘å¬æ–¹ï¼Œé¿å…è¯¯æŠŠå®ƒå½“æˆâ€œé›¶é£Žé™©çœŸæœºæµ‹è¯•â€ã€‚

## å‚æ•°

| å‚æ•° | é»˜è®¤ | è¯´æ˜Ž |
|------|------|------|
| `--arm` | PSM1 | dVRK è‡‚å‘½åç©ºé—´ |
| `--camera_topic` | /dvrk/ECM/camera/image | ç›¸æœºå›¾åƒ topicï¼ˆ**éœ€å‘ Ed ç¡®è®¤**ï¼‰ |
| `--model_path` | ../base_env_imageIL/model_final.pth | æ¨¡åž‹è·¯å¾„ |
| `--control_hz` | 50 | æŽ§åˆ¶é¢‘çŽ‡ |

## éœ€å®žæµ‹/ç¡®è®¤çš„é¡¹

1. **ç›¸æœº topic**ï¼šdVRK å†…çª¥é•œæˆ–å¤–æŽ¥ç›¸æœºçš„å®žé™… topic åç§°
2. **frame_id**ï¼š`servo_cp` çš„ `header.frame_id`ï¼ˆè„šæœ¬ä¸­ä¸ºå ä½ï¼Œéœ€ `ros2 topic echo /PSM1/measured_cp` å®žæµ‹ï¼‰
3. **JAW_MAX_RAD**ï¼šå¤¹çˆªå…¨å¼€æ—¶çš„å¼§åº¦ï¼ˆè„šæœ¬é»˜è®¤ 0.8ï¼ŒLarge Needle Driver çº¦ 0.8ï¼‰

## å®‰å…¨

- é¦–æ¬¡ä¸Šæœºå»ºè®®ä¸Ž Ed åŒè¡Œ
- å¯å…ˆä¸æŽ¥ policyï¼Œä»…è®¢é˜… measured_cp éªŒè¯æ•°æ®æµ
- å¼‚å¸¸æ—¶å‘å¸ƒ `/<arm>/hold` ä¿æŒå½“å‰ä½ç½®
- é¦–æ¬¡ live å‰ï¼Œå…ˆæ ¸å®žå®˜æ–¹ dVRK æ–‡æ¡£ä¸­çš„çœŸæœºå¯åŠ¨ã€`servo_cp` è¯­ä¹‰å’Œè§†é¢‘ ROS ç®¡çº¿ï¼š
  - https://dvrk.readthedocs.io/main/pages/usage/real.html
  - https://dvrk.readthedocs.io/main/pages/development/api/arms.html
  - https://dvrk.readthedocs.io/main/pages/video/software/ros.html
