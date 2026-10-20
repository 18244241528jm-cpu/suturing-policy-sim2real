# 失败样例：T3b_C

2026-10-16 正式计数尝试，条件 C；抓取针尾侧 `s=0.25`。记录结果：失败（无有效接触，夹爪尖落在圆弧内侧）。

这次复用了同摆放 T3_C 的感知，因此本次 snap0 没有 FS 视差，也没有 snap1。**这是原始记录的情况，不是漏打包。**

- snap0：T3b_C 自己的起始图像和状态。
- perception_reference/snap0、snap1：T3_C 被复用的图像和 FS 视差，供回放。
- clicks.json、perception.json：沿用的点选与当天保存的感知。
- trial_state.json、trial_log.jsonl、preclose_pose.json 和阶段图像：本次执行记录。

协议中的失败样例回放用 perception_reference 的两帧。感知可能通过检查；**感知通过不等于抓取成功**。这个失败用于说明两者的区别。

2026-10-20 回放验证：脚本正常退出，感知检查通过，抓取点与归档相差 0.00029 mm（门槛 0.1 mm）；历史抓取结果仍是失败。
