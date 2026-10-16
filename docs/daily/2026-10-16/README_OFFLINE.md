# R6 2026-10-16 离线包

给 FS、FP 和 policy 离线复跑用。全部数据都在这个目录里，坐标系约定见 `calibration_summary.json`。

| 内容 | 位置 |
|---|---|
| 逐次结果表 | `trials_summary.csv` / `.json` |
| 每次尝试 | `trials/<id>/`：snap0/snap1（左右校正图 + `state.json` + FS 视差），`hover_*`（悬停每次修正的 5–10 帧，`R??/L??.png`），`pre_close*`（闭合前 10 帧），`closed`、`lifted_2s`，`preclose_pose.json`（尖端 6D 位姿和针坐标系，ECM 系和相机系），`perception.json`（抓取点、切线、法向、圆弧拟合），`clicks.json`（人工点的针折线，作为参考答案），`template_clicks.json`（人工点的夹爪尖），`trial_state.json`、`trial_log.jsonl` |
| 用于离线 FP 的精简副本 | `xfer/<id>/` |
| 相机 | `camera/camera_info_topic_{left,right}.yaml`（P1/P2、R、D、K），`camera/calibration_files/` |
| 标定 | `calibration_summary.json`（仿射 M、TCP、训练点和测试点、重启后检查），`calib/`（原始数据） |
| 机器人状态 | 每个 `snap0/state.json`（T4 起包含 SUJ 和 local_cp），`robot_state_1352/state.json`（SUJ 快照，用于补之前的尝试），每个 `start.json`（ECM js） |
| policy | `policy_offline/policy_offline_inputs.json`（checkpoint 路径和 sha256、观测和动作的约定、每次尝试的起点和目标，以及还缺什么），以及换算代码副本 |
| 预注册 | `pre_registration.json` 加修订 1–4（都有 .sha256） |
| 日志和报告 | `STATUS.md`、`SurgicAI_R6_realrobot_20261016.md`、`task/`、`dvrk_logs/` |
| 代码 | `code_snapshot/` |
| 视频（单独的包） | `video/T3b_*`、`T4_*`（全分辨率，左右两路，15 fps，带时间戳） |

左右的约定：文件名里的 left/right 是 **topic** 名；物理左目 = topic right。FS 的输入是 left = 物理左（topic right）。
