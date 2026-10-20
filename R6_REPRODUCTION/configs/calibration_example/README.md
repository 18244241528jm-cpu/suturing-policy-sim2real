# 历史标定示例

这些是 2026-10-16、ProGrasp 400093 和当天 ECM/SUJ 摆放的文件。

| 文件 | 含义 |
|---|---|
| calib/manifest.json | 标定点、参考姿态和基准位置 |
| calib/calibration_r6.json | ECM cp → 相机坐标的仿射标定 |
| calib/check2_after_restart2.json | 当天重启后的检查记录 |
| beta/beta_sweep.json | 当天 beta 测量 |
| camera/ | 两个 topic 的相机参数 |
| calibration_summary.json | 坐标系、TCP 和标定摘要 |

位置单位看字段名：`*_mm` 为毫米，`*_m` 为米。物理左目对应 topic right。

这是一套**文件格式和历史结果示例**，不能直接当作新实验的有效标定。ECM、SUJ、器械或摆放变化后，需要重新标定或检查。
