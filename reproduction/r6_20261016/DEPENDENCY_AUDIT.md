# R6 依赖审计（2026-10-19）

## 文件是否必须

| 任务 | 必需文件/环境 | 状态 |
|---|---|---|
| 感知回放 | 左右校正图、fs_disp.npz、clicks/perception、相机 YAML、r2_arc、r5_grasp_v3、numpy/opencv/scipy | 代码已集中；数据在独立 ZIP，下载地址待补 |
| policy CPU 推演 | snap 状态、trial_state、preclose_pose、calibration_summary、obs/contract/frames/policy、R6 checkpoint、torch/SB3/gymnasium | 模块已复制；checkpoint 在仓库 models/rl；数据仍需 ZIP |
| FP 输入准备 | 上述图像/视差/点选、exp0 结果、哈希正确的 M0 mesh | prep_inputs 已复制；M0 来自数据包 fp_out/meshes |
| FP 指标重算 | M0/M2 mesh、ref.json、M0/M2 candidates | analyze_fp 已复制；候选在独立 ZIP |
| FS 重新推理 | FoundationStereo 源码、23-51-11 权重及 cfg.yaml、DINOv2 Torch Hub 和 timm EdgeNeXt 初始化缓存、CUDA/PyTorch | 笔记本 GPU 补充包已有；3090 上是否齐全待现场核对 |
| FP 重新配准 | FoundationPose、refine/score 权重、occlusion_eval、run_fp_p7_psm_tracker → p9a_goal_geometry、nvdiffrast/pytorch3d/warp/mycpp | 3090 的 R5 环境待现场核对；不能直接套用 5080 镜像 |
| 真机抓针 | ROS2 Jazzy、rclpy、cv_bridge、message_filters、标准 ROS 消息、crtk_msgs、dvrk_robot/video/config、ros2_ws、当前标定 | 不属于本次 GitHub 代码包，须实验室提供 |
| 人工输入 | 针尾到尖的折线、左右夹爪模板点击 | 历史 JSON 在数据包；新摆放仍须产生 |

视频、__pycache__、重复 xfer、全部训练示范和仿真器并非上述离线分析的运行依赖。保留预注册、修订、日志和原结果是为了审计，不代表它们全都参与计算。

## 已确认的缺口

1. 真机 r6_perceive.py 读取 fs_disp.npy；精简输入是 fs_disp.npz（键 disp、float16）。直接调用会失败。exp0 已适配该存储格式；不修改真机代码。
2. code_snapshot 平铺目录不满足真机的 r5_realrobot 导入。已额外原样复制成 scripts/r2_realrobot、r4_realrobot、r5_realrobot、r6_realrobot。
3. r6_fs_remote.sh 是旧笔记本传输入口：SSH、用户名、localhost 端口、WSL/venv 及 Windows 路径均绑定旧机器。它不是新的 3090 入口。工作站连接凭据不写入仓库。
4. exp0/1/3/4 固定 ~/r6off；exp1 权重固定 ~/ref/daily-notes-ro/models/rl；prep_inputs 固定 ~/surgical_robotics_challenge/ADF/Phantoms/3D_MED/high_res/Needle.OBJ。REPRODUCE.md 用独立 HOME 满足路径。
5. run_exp2_gpu.sh 固定 /home/jiaming、/project、/workspace、WSL GPU、监控日志及 foundationpose:blackwell；3090 应先检查 R5 环境，不能把旧命令当成已验证配置。
6. FS 初始化还有 timm.create_model(pretrained=True) 和 torch.hub.load 的隐藏依赖。仅带主权重不足以保证首次离线启动。
7. policy_offline/run_approach.py 所需的 loop.py、ros_node.py、staging.py、jaw.py 不在快照。exp1 是离线重建循环，没有完整残差伺服或真实执行误差。
8. SLIM_NOTE.md 才描述实际精简输入：xfer 删除、部分帧删除、部分 JPEG 有损、视差 float16。不应按完整版 README 假定所有帧存在。
9. M2 线径暂定，尺寸来自同批人工参考；14 次尝试只有 13 个独立摆放。FP 结果是与人工参考的一致性，不能宣称独立真值精度。
10. 校外不能连接实验室工作站。当前未读取其安装环境、未安装软件、未运行 GPU；待回实验室补充版本、模型哈希和 smoke 结果。

## 待完成的交付项

- 提供可访问的数据 ZIP 下载地址及访问方式（当前仅有文件名和哈希）。
- 在实验室核对 Adnan 3090 的 R5 FS/FP 环境、权重与源码版本。
- 在独立目录运行 FS/FP smoke，记录实际命令、版本、输入/输出哈希。
- 再把草稿提升为已验证的 3090 复现版本。
