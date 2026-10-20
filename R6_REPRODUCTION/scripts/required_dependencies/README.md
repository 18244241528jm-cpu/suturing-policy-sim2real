# 这些文件为什么需要

| 文件 | 用途 |
|---|---|
| r2_arc.py | r6_perceive 的圆弧拟合 |
| r5_grasp_v3.py | 感知和抓取共用的相机参数、几何及模板函数 |
| r6_snap.py | 采集双目图像和机器人状态 |
| r6_calib.py | 当前装置的标定 |
| r6_beta.py | beta 测量，包含机器人运动 |
| r4_ecm_move.py | ECM 取景运动 |
| r6_fs_infer.py | 在已配置好的 FoundationStereo 环境运行推理 |
| r6_trial_start.sh | 建立尝试记录；`--nofs` 只记录起点和 snap0，不调用旧远程 FS |
| r6_fs_remote.sh | 旧笔记本入口，保留原件；3090 不使用 |
| r6_reuse_perception.py | 同摆放复用感知及移动检查；带历史绝对路径假设 |

这些都是未修改的原文件。主脚本运行前，把本目录加入 PYTHONPATH，具体命令见协议。
