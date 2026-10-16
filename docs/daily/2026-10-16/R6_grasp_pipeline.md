# R6 真机自主抓针：完整流程（2026-10-16 当前版本）

机器人：dVRK `lcsr-dvrk-15`，PSM1 = **ProGrasp 400093**，ECM 双目相机（1300×1024 校正图，fx 1860.72，基线 5.45 mm）
感知主机：Jiaming 笔记本（WSL Ubuntu-22.04，RTX 5080），通过反向隧道 `localhost:2222` 连接；连接方法见 `~/jiaming_laptop_connection.md`
代码：`~/suturing-policy-sim2real/scripts/r6_realrobot/`；数据：`records/logs/R6_20261016/`

已经成功两次：debug7（调试），Q1_C（正式计数）。

---

## 0. 每次开机

```bash
# dvrk_system（窗口可能落在 HDMI-0，用 xwininfo 查位置）
source /opt/ros/jazzy/setup.bash && source ~/ros2_ws/install/setup.bash
export ROS_AUTOMATIC_DISCOVERY_RANGE=SYSTEM_DEFAULT
ros2 run dvrk_robot dvrk_system -j ~/ros2_ws/install/dvrk_config_jhu/share/jhu-daVinci/system-SUJ-ECM-PSM1-PSM2.json
# 相机（用脚本文件启动，避免 pgrep -f 匹配到自己的命令）
ros2 launch dvrk_video decklink_stereo_1280x1024.launch.py stereo_rig_name:=jhu_daVinci stereo_proc:=True
```

- 在 console 上 Home，确认日志里出现 `PROGRASP_FORCEPS:400093`。
- **ROS 坑（今天出过两次）**：脚本退出时 rclpy 的 spin 线程还在跑，会触发 `std::terminate`（core dump），之后 dvrk_system 和相机都会和新节点失联。现在的修法：
  - 所有 R6 脚本都设 `ROS_AUTOMATIC_DISCOVERY_RANGE=SYSTEM_DEFAULT FASTDDS_BUILTIN_TRANSPORTS=UDPv4`；
  - 用 `os._exit` 和 `sys.excepthook` 干净退出；
  - 万一又失联：用 SIGINT 停掉 dvrk_system，重新启动，重新 Home，然后做标定检查。
- 笔记本那边：开好反向隧道 `ssh -N -R 2222:localhost:22 xsun97@10.162.34.171`。

## 1. 标定（一次性；ECM 和 SUJ 动了就要重做或检查）

| 步骤 | 脚本 | 说明 |
|---|---|---|
| 取景 | `r4_realrobot/r4_ecm_move.py --state-dir R6/ecm` | 让夹爪和针都在画面里（今天 ECM pitch −2°） |
| 垫面和尖端高度 | `r6_snap.py` + `r6_fs_remote.sh` | 一帧 FS 得到垫面平面和尖端高度 |
| manifest | `r6_calib.py --phase manifest` | 固定种子 20261016，12 个点：x、y 各 ±8 mm，高度只往上 0 到 +8 mm，3 个点留作测试 |
| 探测 | `--phase probe` | 3 次 2 mm 移动，得到粗略雅可比 |
| 网格 | `--phase grid` | 夹爪模板 NCC 跟踪 + 两个视角各自的模板 → 视差 → 3D |
| 拟合 | `--phase fit` | 仿射 cp(ECM 系) → cam，门槛：测试误差 ≤ 3 mm |
| 重启后检查 | `--phase check2` | 新拍夹爪模板，在起点、g04、g08 检查；今天 0.89 / 1.84 / 0.44 mm |

结果文件：`calib/calibration_r6.json`（测试误差 0.35 / 1.58 / 2.93 mm；奇异值 1.24 / 1.12 / 0.83，和 R4 形状相同）

## 2. 每次尝试（条件 C = 夹爪伸直 + roll 对齐 + 沿轴下降到碰到 phantom）

```bash
T=records/logs/R6_20261016/trials/<id>
scripts/r6_realrobot/r6_trial_start.sh $T            # 机器人停在起点：start.json + 2 帧 + 笔记本 FS（约 60 s）
#   同一个摆放的第二个条件：r6_trial_start.sh $T --nofs
#   然后 r6_reuse_perception.py --trial $T --from_trial <第一次>（NCC 补丁检查针移动 ≤ 1 mm）
# 在 snap0/right_rect.png 上点针的折线（从尾端/缝线端到尖端）-> $T/clicks.json
python3 scripts/r6_realrobot/r6_perceive.py --trial $T --snaps $T/snap0 $T/snap1 --clicks $T/clicks.json
G="python3 scripts/r6_realrobot/r6_grasp.py --trial $T"
$G --phase approach --cond C   # 参考姿态，连续运动到 p* 上方 25 mm（> 20 mm 的空中段用 servo，≤ 4.5 mm/s）
$G --phase wrist               # 绕尖端（R2 TCP）：pitch 和 yaw 归零 -> roll，让开合轴 y 和切线成 ≥ 80°；A 条件跳过这一步
$G --phase open                # 张开 0.6 rad，拍模板帧
# 在 open25/ 左右图上点：夹爪特征（腕部高光）+ 两片夹爪尖的中点 -> $T/template_clicks.json
$G --phase template
$G --phase hover               # 移到 p* 沿夹爪轴上方 10 mm；最多 3 次双目修正（5 帧，或 10 帧中位数）
$G --phase descend             # 沿轴 8 × 1 mm；保存 preclose_pose.json
$G --phase touch --mm 6        # 每步 0.5 mm，插入关节力 > 0.15 N（或跟踪误差 > 0.3 mm）判定接触；闭合前检查针是否移动
$G --phase close_lift          # 10 步约 3 s 慢慢闭合 -> 沿轴抬起 10 mm -> 保持 2 s
$G --phase putdown && $G --phase release
```

### 感知（`r6_perceive.py`）
- 针的掩膜：在人工点选的折线两侧 5 px 带内，取比周围垫面暗 18 灰度以上的像素。
- **深度不直接用 FS 在针丝上的值**：FS 会把细针丝的深度“压平”，两端看起来高出垫面 2–4 mm。改成：用折线外侧环带（10–25 px）里粉色垫面像素的 FS 点拟合二次曲面，针像素的射线和曲面求交，再沿法向抬 0.5 mm。
- `r2_arc.fit` 圆弧拟合，抓取点在 s = 0.5（从缝线端算起），同时得到切线和法向。
- 检查项：R 在 6–13 mm、弧 100–240°、RMS ≤ 1.6 mm、平整度（曲面法向在尾、中、尖三处差 ≤ 15°）、两帧 p* 差 ≤ 1 mm、切线和法向差 ≤ 10°。
- 归档：`xfer/<trial_id>/`（左右图、状态、FS 视差、perception.json），供事后离线跑 FP。

### 关键几何事实（今天才弄清楚的）
- ProGrasp：tool z 沿夹爪长轴，**开合方向是 tool y**（原以为是 x，debug6 因此压在针上）。
- 今天的摆放下，夹爪伸直时 β ≈ 85°，夹爪几乎竖直。
- R2 的 TCP = (−2.70, 1.08, 23.81) mm；pitch 每转 4°、roll 每转 6°，补偿平移都 ≤ 3 mm。

### 安全
- 每步平移 ≤ 3 mm（离垫面 20 mm 以内）；腕部转动只在离垫面 25 mm 处进行，并且绕尖端转。
- 没有新鲜、有效的 perception 就不运动；ECM 被动过就中止；换摆放前 ECM 退回 20 mm，摆好后再插回。
- 闭合前针移动超过 1 mm，或者无法验证，就中止。

## 3. 记录
- `STATUS.md`（时间一律用 `date` 取）、`pre_registration.json` 加修订 1–4（全部只读，有 sha256）。
- 每次尝试：`trial_state.json`、`trial_log.jsonl`、各阶段的帧（`pre_close*`、`closed`、`lifted_2s`）、`preclose_pose.json`（尖端 6D 位姿和针坐标系，ECM 系和相机系各一份）。

## 4. 耗时（Q1_C）
FS 感知约 60 s；机器人各阶段加起来约 1–2 分钟；剩下的主要是人工点选（针 1 次，夹爪 1 次）和确认。

## 5. 待改进
- 点选还是人工的：针的自动分割会把缝线当成针，夹爪抓取点的深度误差约 ±1–2 mm。
- 下降的深度靠力判定接触来补；ψ③ 的定义要和仿真核对；FP 今天离线跑。
