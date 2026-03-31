# ç¬¬ä¸€æ¬¡ Policy çœŸæœºæœ€å°é—­çŽ¯ Checklist / First Policy Minimal Closed-Loop Checklist

> **ç”¨é€” / Purpose**ï¼šç”¨äºŽç¬¬ä¸€æ¬¡æŠŠ `policy -> ROS/CRTK -> PSM` æŽ¥åˆ°çœŸæœºä¸Šã€‚  
> Use this for the first real-robot `policy -> ROS/CRTK -> PSM` deployment.
>
> **ä»Šå¤©ç›®æ ‡ / Goal**ï¼šä¸æ˜¯å®Œæˆä»»åŠ¡ï¼Œè€Œæ˜¯å®Œæˆ**æœ€å°é—­çŽ¯**ã€‚  
> The goal is not task success, but a **minimal closed loop**:
> 1. èƒ½æ”¶åˆ°ç›¸æœºå›¾åƒ / Receive camera images  
> 2. èƒ½æ”¶åˆ° `measured_cp` / Receive `measured_cp`  
> 3. èƒ½å®‰å…¨å‘å‡ºå°å¹… `servo_cp` å’Œ jaw æŒ‡ä»¤ / Safely send small `servo_cp` and jaw commands  
> 4. èƒ½è®°å½•é”™è¯¯ä¸Žå¤±è´¥æ¨¡å¼ / Log failures and error modes
>
> **æƒå¨æ¥æº / Source of truth**ï¼šä»¥å®˜æ–¹ dVRK æ–‡æ¡£å’ŒçŽ°åœº `ros2 topic ...` è¾“å‡ºä¸ºå‡†ã€‚  
> Use official dVRK docs and live `ros2 topic ...` output as the source of truth.

---

## 0. å®˜æ–¹æ–‡æ¡£å…ˆè¯» / Read These Official Pages First

- [ ] çœŸæœºå¯åŠ¨ï¼š`usage/real`  
  https://dvrk.readthedocs.io/main/pages/usage/real.html
- [ ] PSM / ECM / jaw / `servo_cp` APIï¼š`development/api/arms`  
  https://dvrk.readthedocs.io/main/pages/development/api/arms.html
- [ ] åæ ‡ç³»ï¼š`development/frames`  
  https://dvrk.readthedocs.io/main/pages/development/frames.html
- [ ] ROS bridgeï¼š`development/components/fromROS`  
  https://dvrk.readthedocs.io/main/pages/development/components/fromROS.html
- [ ] è§†é¢‘ / ECM ROSï¼š`video/software/ros`  
  https://dvrk.readthedocs.io/main/pages/video/software/ros.html

**åªä»Žå®˜æ–¹æ–‡æ¡£ç¡®è®¤çš„äº‹å®ž / Officially confirmed facts**
- `servo_cp` æ˜¯ç¬›å¡å°”ä½ç½®ä¼ºæœç›®æ ‡ï¼Œé€‚åˆå•æ­¥ã€å°å¹…ã€å¯è¾¾çš„ç›®æ ‡ã€‚
- jaw èµ° `/<arm>/jaw/...` ç›¸å…³æŽ¥å£ã€‚
- ECM / è§†é¢‘èµ°å•ç‹¬çš„è§†é¢‘ ROS ç®¡çº¿ï¼Œtopic åç§°å–å†³äºŽçŽ°åœº rig / gscam é…ç½®ã€‚

**å®˜æ–¹æ²¡æ›¿ä½ å†³å®šçš„ / Must be measured on site**
- `camera_topic`
- `measured_cp.header.frame_id`
- jaw å…¨å¼€ / å…¨é—­å¼§åº¦èŒƒå›´
- å®žé™… `arm`ï¼ˆ`PSM1` æˆ– `PSM2`ï¼‰
- çŽ°åœº ROS å‘è¡Œç‰ˆä¸Ž system JSON

---

## 1. ä¸Šæœºå‰å‡†å¤‡ / Before Leaving

- [ ] å¸¦é—¨ç¦å¡ã€ç”µè„‘ã€ç”µæºã€è¿™ä»½ checklist
- [ ] å‡†å¤‡å¥½è¦æŸ¥çœ‹çš„é¡¹ç›®æ–‡ä»¶ï¼š
  - `project34/SurgicAI-SurgicAI-DR-RAL/Image_IL/dvrk_policy_adapter.py`
  - `project34/SurgicAI-SurgicAI-DR-RAL/Image_IL/README_dvrk_adapter.md`
- [ ] æ˜Žç¡®ä»Šå¤©ä½¿ç”¨çš„è‡‚ï¼š`PSM1 / PSM2 = __________`
- [ ] æ˜Žç¡®æ¨¡åž‹è·¯å¾„ï¼š`model_path = __________`
- [ ] è‹¥ä½ ä»¬è®¡åˆ’å’Œ Ed ä¸€èµ·åšï¼Œå…ˆçº¦å®šåˆ†å·¥ï¼š
  - Edï¼š`dvrk_system`ã€`Power On`ã€`Home`ã€æœºå™¨äººå®‰å…¨
  - ä½ ï¼štopic æ ¸å¯¹ã€adapter å‚æ•°ã€æ—¥å¿—è®°å½•

---

## 2. çœŸæœºåŸºç¡€çŠ¶æ€ç¡®è®¤ / Real-Robot Baseline First

> è‹¥è¿™ä¸€æ­¥å¤±è´¥ï¼Œä»Šå¤©ä¸è¦è¿›å…¥ policyã€‚

- [ ] æŽ§åˆ¶å™¨å·²ä¸Šç”µ
- [ ] `qladisp` èƒ½è¯†åˆ«æ¿å¡
- [ ] system JSON å·²ç¡®è®¤ï¼š`__________`
- [ ] `dvrk_system` å¯åŠ¨æˆåŠŸï¼ŒGUI æ— æŠ¥é”™
- [ ] `Power On` å®Œæˆ
- [ ] `Home` å®Œæˆ
- [ ] æœºæ¢°è‡‚ç¨³å®šï¼Œæ— å¼‚å¸¸æŒ¯åŠ¨/å¼‚å“
- [ ] å·²çº¦å®šå¼‚å¸¸å¤„ç†ï¼šä½ åœè„šæœ¬ï¼ŒEd å¤„ç†æœºå™¨äºº

å®˜æ–¹é˜…è¯»ä½ç½®ï¼š
- çœŸæœºå¯åŠ¨ä¸Ž `qladisp`ã€`Power On`ã€`Home`ï¼š  
  https://dvrk.readthedocs.io/main/pages/usage/real.html

---

## 3. å…ˆåšâ€œæŽ¥å£ä¸Žè¾“å…¥é“¾â€æ ¸å®ž / Verify Interface and Input Chain First

### 3.1 è®°å½•å½“å‰çŽ¯å¢ƒ

```bash
source ~/ros2_ws/install/setup.bash
echo $ROS_DISTRO
ros2 topic list
```

- [ ] ROS å‘è¡Œç‰ˆï¼š`__________`
- [ ] å®žé™… armï¼š`__________`
- [ ] system JSONï¼š`__________`

### 3.2 æ ¸å¯¹ PSM / jaw topic

```bash
ros2 topic list | grep PSM
ros2 topic info /<arm>/measured_cp -v
ros2 topic info /<arm>/servo_cp -v
ros2 topic info /<arm>/jaw/measured_js -v
ros2 topic info /<arm>/jaw/servo_jp -v
```

- [ ] `/<arm>/measured_cp` å­˜åœ¨
- [ ] `/<arm>/servo_cp` å­˜åœ¨
- [ ] `/<arm>/jaw/measured_js` å­˜åœ¨
- [ ] `/<arm>/jaw/servo_jp` å­˜åœ¨

### 3.3 è®°å½• `frame_id`

```bash
ros2 topic echo /<arm>/measured_cp --once
```

- [ ] `frame_id = __________`

### 3.4 æ‰¾ç›¸æœº topic

```bash
ros2 topic list | grep -Ei "image|camera|left|right"
ros2 topic echo <camera_topic> --once
```

- [ ] `camera_topic = __________`
- [ ] èƒ½æ”¶åˆ°å›¾åƒæ¶ˆæ¯

å¦‚æžœçŽ°åœºç›¸æœº topic ä¸æ¸…æ¥šï¼ŒåŽ»è¯»å®˜æ–¹è§†é¢‘ ROS æ–‡æ¡£ï¼š
- https://dvrk.readthedocs.io/main/pages/video/software/ros.html

### 3.5 è®°å½• jaw èŒƒå›´

```bash
ros2 topic echo /<arm>/jaw/measured_js --once
```

- [ ] å½“å‰ jaw å¼§åº¦å·²è®°å½•
- [ ] è‹¥å¯å®‰å…¨æ“ä½œï¼Œè®°å½•å…¨å¼€ / å…¨é—­å¤§è‡´èŒƒå›´ï¼š
  - `jaw_open_rad = __________`
  - `jaw_closed_rad = __________`

---

## 4. Adapter å‚æ•°æ ¸å¯¹ / Adapter Parameter Check

> è¿™ä¸€æ­¥å¯¹ç…§é¡¹ç›®å®žçŽ°ï¼Œä¸æŠŠé¡¹ç›® README å½“å®˜æ–¹äº‹å®žæ¥æºã€‚

é¡¹ç›®å†…å‚è€ƒæ–‡ä»¶ï¼š
- `project34/SurgicAI-SurgicAI-DR-RAL/Image_IL/dvrk_policy_adapter.py`
- `project34/SurgicAI-SurgicAI-DR-RAL/Image_IL/README_dvrk_adapter.md`

ä¸Šæœºå‰å¿…é¡»æ˜Žç¡®ï¼š
- [ ] `--arm = __________`
- [ ] `--camera_topic = __________`
- [ ] `--model_path = __________`
- [ ] `frame_id = __________`
- [ ] `JAW_MAX_RAD` æ˜¯å¦ä¸Žå½“å‰å™¨æ¢°ä¸€è‡´

**ç‰¹åˆ«æé†’ / Important warning**
- [ ] ä¸è¦æŠŠ `README_dvrk_adapter.md` é‡Œçš„â€œ`dry_run` = no dVRKâ€ç›´æŽ¥å½“çœŸ
- [ ] å…ˆçœ‹è„šæœ¬å®žé™…è¡Œä¸ºï¼Œå†å†³å®šæ˜¯å¦å¯ä»¥å®‰å…¨ç”¨ `--dry_run`

ä½ éœ€è¦çŸ¥é“çš„è„šæœ¬è¡Œä¸ºï¼š

```202:223:project34/SurgicAI-SurgicAI-DR-RAL/Image_IL/dvrk_policy_adapter.py
    def _publish_servo(self, goal_vector):
        """goal_vector: [x,y,z,roll,pitch,yaw,jaw] â€” ç±³ã€å¼§åº¦ã€0-1"""
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
                return  # ä¸ publish
        self._publish_servo(goal)
```

å«ä¹‰ï¼š
- `dry_run` åªåœ¨è¾¾åˆ° `dry_run_steps` æ—¶æå‰ `return`
- å¹³æ—¶å®ƒä»ä¼šè¿›å…¥ `_publish_servo(goal)`
- æ‰€ä»¥è‹¥çœŸæœºåœ¨åŒä¸€ ROS åŸŸå†…ç›‘å¬è¿™äº› topicï¼Œ`dry_run` ä¹Ÿå¯èƒ½æœ‰é£Žé™©

---

## 5. ä½Žé£Žé™©éªŒè¯ / Low-Risk Validation

### 5.1 ä¸æŽ¥çœŸæœºæŽ§åˆ¶æ ˆæ—¶ï¼ŒéªŒè¯æ¨¡åž‹å’Œå›¾åƒ

- [ ] ç¡®è®¤æ¨¡åž‹æ–‡ä»¶çœŸå®žå­˜åœ¨
- [ ] ç¡®è®¤ç›¸æœº topic èƒ½æ”¶åˆ°å›¾
- [ ] è‹¥è¦è·‘ `dry_run`ï¼Œå…ˆç¡®è®¤ï¼š
  - å½“å‰æ²¡æœ‰çœŸå®ž arm åœ¨ç›‘å¬åŒå `servo_cp` / `jaw/servo_jp`
  - æˆ–ä½ ä»¬å·²éš”ç¦» ROS åŸŸ

å‚è€ƒå‘½ä»¤ï¼š

```bash
python3 dvrk_policy_adapter.py \
  --dry_run --dry_run_steps 20 \
  --arm <arm> \
  --camera_topic <camera_topic>
```

- [ ] æ—¥å¿—èƒ½æ˜¾ç¤ºæ”¶åˆ°å›¾åƒ
- [ ] æ¨¡åž‹åŠ è½½æˆåŠŸ
- [ ] `goal` è®¡ç®—ä¸Žè¾“å‡ºæ­£å¸¸

### 5.2 åªè¯»éªŒè¯çœŸæœºçŠ¶æ€æµ

> å¦‚æžœä½ ä»¬æ‹…å¿ƒ `dry_run` é£Žé™©ï¼Œä¼˜å…ˆåšâ€œåªè¯» topic éªŒè¯â€ï¼Œä¸è¦æ€¥ç€è¿è¡Œ adapterã€‚

- [ ] `measured_cp` èƒ½æŒç»­æ”¶åˆ°
- [ ] `jaw/measured_js` èƒ½æŒç»­æ”¶åˆ°
- [ ] ç›¸æœºå›¾åƒèƒ½æŒç»­æ”¶åˆ°
- [ ] ä¸‰è€…æ—¶é—´ä¸Šå¤§ä½“ç¨³å®š

---

## 6. ç¬¬ä¸€æ¬¡çœŸæœºæœ€å°é—­çŽ¯ / First Live Minimal Closed Loop

> å‰æï¼šå‰é¢æ‰€æœ‰ç©ºç™½é¡¹éƒ½å·²å¡«ä¸Šã€‚

å»ºè®®ç­–ç•¥ï¼š
- [ ] é¦–è·‘åªå…è®¸å°å¹…åŠ¨ä½œ
- [ ] é™ä½ŽæŽ§åˆ¶é¢‘çŽ‡å¼€å§‹ï¼Œä¾‹å¦‚å…ˆä»Žè¾ƒä½Žé¢‘çŽ‡è¯•
- [ ] äººç›¯ç€ GUIã€æœºå™¨äººå§¿æ€ã€å£°éŸ³ã€æŒ¯åŠ¨
- [ ] æ‰‹éšæ—¶æ”¾åœ¨ `Ctrl+C` å’Œåœæ­¢æµç¨‹ä¸Š

å‚è€ƒå‘½ä»¤æ¨¡æ¿ï¼š

```bash
python3 dvrk_policy_adapter.py \
  --arm <arm> \
  --camera_topic <camera_topic> \
  --model_path <model_path> \
  --control_hz 10
```

çŽ°åœºè§‚å¯Ÿé¡¹ï¼š
- [ ] æœºå™¨äººæœ‰å“åº”
- [ ] è¿åŠ¨å¹…åº¦å¯æŽ§
- [ ] è¿åŠ¨æ–¹å‘åŸºæœ¬åˆç†
- [ ] jaw å¼€é—­æ–¹å‘æ­£ç¡®
- [ ] æ²¡æœ‰å¼‚å¸¸æŠ–åŠ¨ã€å¼‚å“ã€æ˜Žæ˜¾å»¶è¿Ÿ

å®˜æ–¹æé†’ï¼š
- `servo_cp` é€‚åˆå•æ­¥ã€å°å¹…ã€å¯è¾¾ç›®æ ‡ï¼›å¤§ä½ç§»ä¸è¯¥ç›´æŽ¥é å®ƒç¡¬æŽ¨  
  https://dvrk.readthedocs.io/main/pages/development/api/arms.html
- çœŸå®ž dVRK è¿è¡Œæ—¶è¦æŒç»­ç›‘æŽ§å™ªå£°ã€æŒ¯åŠ¨ã€è´Ÿè½½å’Œ IO é¢‘çŽ‡  
  https://dvrk.readthedocs.io/main/pages/usage/real.html

---

## 7. å¤±è´¥æ¨¡å¼è®°å½• / Failure Logging

> è¿™ä¸€æ­¥å’Œâ€œè·‘æˆåŠŸâ€åŒæ ·é‡è¦ï¼›checkpoint å¯ç›´æŽ¥ç”¨ã€‚

è‡³å°‘è®°å½•è¿™äº›å­—æ®µï¼š
- [ ] æ—¥æœŸ / æœºå™¨ / arm
- [ ] system JSON
- [ ] ROS ç‰ˆæœ¬
- [ ] `camera_topic`
- [ ] `frame_id`
- [ ] `jaw_open_rad / jaw_closed_rad`
- [ ] `control_hz`
- [ ] `model_path`
- [ ] æˆåŠŸæˆ–å¤±è´¥çŽ°è±¡
- [ ] æˆªå›¾ / ç»ˆç«¯æ—¥å¿—æ˜¯å¦ä¿å­˜

æŠŠé—®é¢˜æŒ‰å››ç±»è®°å½•ï¼š
- [ ] `perception`
- [ ] `frame / units / interface mismatch`
- [ ] `timing / control`
- [ ] `hardware / calibration`

å¯ç›´æŽ¥æŠ„ä¸‹é¢æ¨¡æ¿ï¼š

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
- perception / frame-interface / timing-control / hardware-calibration

Evidence Saved:
- screenshot:
- terminal log:
- rosbag:
```

---

## 8. æ”¶å°¾ / Wrap-Up

- [ ] åœæ­¢ adapter
- [ ] ç¡®è®¤æœºå™¨äººå›žåˆ°å®‰å…¨çŠ¶æ€
- [ ] è‹¥ç”± Ed æ“ä½œï¼Œå®Œæˆ `Power Off`
- [ ] å›žåŽ»åŽæŠŠç»“æžœå†™å…¥ï¼š
  - `project34/records/logs/Project34_Dev_Log.md`

---

## ä¸€é¡µé€Ÿè®° / One-Page Summary

1. å…ˆæŒ‰å®˜æ–¹æ–‡æ¡£ç¡®è®¤çœŸæœºåŸºç¡€çŠ¶æ€ï¼Œä¸è¦ä¸€ä¸Šæ¥è·‘ policyã€‚
2. å…ˆæŠŠ `arm`ã€`camera_topic`ã€`frame_id`ã€jaw èŒƒå›´ã€ROS ç‰ˆæœ¬è®°æ¸…æ¥šã€‚
3. `dvrk_policy_adapter.py` æ˜¯é¡¹ç›®å®žçŽ°ï¼Œä¸æ˜¯å®˜æ–¹ä¿è¯ï¼›é»˜è®¤å€¼ä¸€å¾‹çŽ°åœºæ ¸å¯¹ã€‚
4. `dry_run` ä¸èƒ½æƒ³å½“ç„¶å½“æˆâ€œç»å¯¹å®‰å…¨â€ã€‚
5. ç¬¬ä¸€æ¬¡éƒ¨ç½²çš„ä»·å€¼åœ¨äºŽå»ºç«‹æœ€å°é—­çŽ¯å¹¶è®°å½•å¤±è´¥æ¨¡å¼ã€‚
