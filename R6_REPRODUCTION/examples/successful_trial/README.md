# 成功样例：Q1_C

2026-10-16 正式计数尝试，条件 C；抓取弧中点 `s=0.5`。记录结果：成功。

- snap0/snap1：左右图、状态、FS 视差；可直接复跑感知。
- clicks.json：人工从针尾到针尖点的折线。
- perception.json：当天保存的感知结果，用作回放参照。
- template_clicks.json：左右图上的夹爪模板点选。
- trial_state.json、trial_log.jsonl、preclose_pose.json：执行状态和闭合前位姿。
- pre_close_touch、closed、lifted_2s：闭合前、闭合后和抬起后的图像。

成功结论来自历史实验记录。单次感知回放只检查几何是否可重算，不会复现抓取动作或证明新的真机成功。

2026-10-20 回放验证：脚本正常退出，感知检查通过，抓取点与归档相差 0.00094 mm（门槛 0.1 mm）。
