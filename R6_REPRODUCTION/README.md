# R6 抓针复现

给祥瑞：先用随包样例复跑感知，再到实验室做真机。**从 [REPRODUCTION_PROTOCOL.md](REPRODUCTION_PROTOCOL.md) 开始。**

这版 R6 用 FoundationStereo、人工点针和夹爪模板、几何控制完成抓针；FP 和 RL policy 没有参与当天的在线抓针，不是本包的运行依赖。

```text
R6_REPRODUCTION/
├── README.md
├── requirements.md
├── scripts/
│   ├── r6_perceive.py
│   ├── r6_grasp.py
│   └── required_dependencies/
├── configs/
│   └── calibration_example/
├── examples/
│   ├── successful_trial/       # Q1_C，成功
│   └── failed_trial/           # T3b_C，失败
├── SAFETY_CHECKLIST.md
└── REPRODUCTION_PROTOCOL.md
```

| 要做什么 | 看哪里 |
|---|---|
| 检查环境和缺失依赖 | [requirements.md](requirements.md) |
| 了解输入和结果 | 两个样例目录中的 README |
| 理解标定文件 | [calibration_example/README.md](configs/calibration_example/README.md) |
| 复跑样例或现场执行 | [REPRODUCTION_PROTOCOL.md](REPRODUCTION_PROTOCOL.md) |
| 真机前逐项检查 | [SAFETY_CHECKLIST.md](SAFETY_CHECKLIST.md) |

代码、图像、记录和标定均从已有 R6 材料逐字节复制；只新增说明。复制来源和 SHA256 在 [COPY_MANIFEST.csv](scripts/required_dependencies/COPY_MANIFEST.csv)。

样例图像和视差已放在此目录，无需先下载全量数据包。历史记录中的路径保留原样，按协议使用实际工作副本路径。

**当前状态：** 两个样例的感知复跑结果见各自 README；Adnan 3090 的 FS 环境和真机流程尚待实验室验证。校外不能连接工作站，本次没有操作机器人。

完整历史流程保留在 [2026-10-16/R6_grasp_pipeline.md](../docs/daily/2026-10-16/R6_grasp_pipeline.md)。
