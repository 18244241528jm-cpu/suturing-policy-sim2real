# R6 Reproduction Package — 祥瑞入口

目标：把 R6 的脚本、跨目录依赖和复现说明集中到 GitHub，保留原始实验代码。此目录只新增文件；原来的 docs/daily、模型和实验实现均未修改。

**当前是交付草稿：代码已整理，数据下载地址待补，Adnan 3090 尚未验证。** 工作站只能从实验室网络连接，2026-10-19 在校外未能建立 SSH；没有执行 GPU 推理或改变工作站环境。接下来在实验室核对 R5 环境后再补运行记录，不把笔记本结果当作 3090 结果。

## 从哪里开始

- 原始 [R6 真机流程](../../docs/daily/2026-10-16/R6_grasp_pipeline.md)。
- [依赖审计](DEPENDENCY_AUDIT.md)：包含哪些文件、还缺什么、固定路径和格式问题。
- [数据与模型](DATA_AND_MODELS.md)：ZIP 名称、SHA256、所需目录及下载状态。
- [离线复现](REPRODUCE.md)：在新工作副本中运行，不覆盖原数据。
- [离线分析原报告与 10 月 19 日勘误](home/r6off/SurgicAI_R6_offline_20261016.md)。
- [源码来源及哈希](SOURCE_MANIFEST.json)。

## 目录

| 路径 | 内容 |
|---|---|
| home/r6off/exp0…exp4 | 原样复制的 CPU 回放、policy 推演、FP 准备/分析脚本与预注册 |
| home/r6off/input/code_snapshot | 当天 R6 快照以及 r2_arc、r5_grasp_v3、r4_ecm_move |
| home/r6off/input/policy_offline | 原始 policy 输入转换代码；完整部署循环未包含 |
| home/suturing-policy-sim2real/scripts | 将相同代码复制为 r2/r4/r5/r6_realrobot 布局，满足历史导入 |
| SOURCE_MANIFEST.json | 39 个原始代码文件的 SHA256 |

## 已有证据与本次边界

之前在笔记本的独立副本中完成 9 个 CPU 阶段：exp0、exp1/1b/1c、exp3、exp4、FP 输入准备、自检和归档候选分析。policy 三份结果、exp4、FP 指标与归档逐字段一致（rtol/atol=1e-5）；exp3 只有运行耗时不同；exp0 有数值差异，仍通过当天代码 11/11 的 0.1 mm 门槛。该记录不是 3090 的环境验证，也不是 FP/FS 网络重新推理。

本次 GitHub 整理没有运行机器人。真机控制仍需现场 ROS/驱动、操作者、新鲜感知和当前标定。历史相机/ECM/SUJ 标定不能用于新的摆放。
