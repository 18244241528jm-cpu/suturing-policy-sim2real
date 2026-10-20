# 数据与模型

GitHub 保存代码、说明及来源；大体积数据和权重使用单独工件。**以下 ZIP 尚未提供在线下载地址，当前 clone 仓库不能完成全部离线分析。** 不将本地路径当作队友可访问的下载链接。

| 工件 | 字节数 | SHA256 | 在线状态 |
|---|---:|---|---|
| R6_Reproduction_Package_Xiangrui_20261019.zip | 652994179 | f16ddb5e8d00bc0f6cccd8a915efaaff3c7b07b3f0b6521288af107ca5d4ffbd | 下载地址待补 |
| R6_GPU_Supplement_Xiangrui_20261019.zip | 3831153144 | a894e514c494dd5de78686f57373d69c02dfd59b1c225fd35fb9901bb90f65e9 | 下载地址待补；旧笔记本环境材料 |

R6 approach policy 已在仓库 [models/rl](../../models/rl/r6_unified_single_goal_yaw15_seed1_final.zip)，SHA256 必须为 6286a88c21f04abfbc4b0747a87a67bc2c5dcba17f710692c6b5138f7776e525。不要用同名但不同哈希的模型代替。

主 ZIP 包含 home/r6off/input、归档结果/候选、M0 mesh、校准和历史日志；原始输入清单 956/956 在源处校验通过，交付 ZIP 另排除了视频及缓存。M0 mesh SHA256 为 2db5c6060c806498e3a18bde5fe4d7d7d53900dd1e167bcb61bc1c7873a8d7ca。

GPU 补充包保存 FoundationStereo（基线 6e8806816b533e4d13ddbb95ffa907b797060a62）、FoundationPose（基线 a1b694b83e633c2cb6115b9063d940a687759392）、现场额外脚本、模型及初始化缓存，各自许可随包。它不含可直接迁移的 CUDA 编译环境；3090 上应优先核对 R5 的已有环境。

原始全量数据及视频仍需实验室提供。禁止把精简版没有的图像或独立真值当作已交付。
