# SurgicAI SIM-S6：PSM 多假设不确定性融合审计

## 结论

`[测量]` **高可见、GT-depth/特权 mask 上限中，MHT-EKF 没有显著优于 P11b 的固定启发式；去特权部署代理仍被候选生成卡死。** 在 held-out episode 2 的 50 帧、64 次 Monte Carlo、250 ms 延迟下，5 mm/5° 先验误差组的严格通过率为：运动学 22.1%、raw FP 47.9%、固定启发式 81.6%、概率单帧 80.2%、MHT-EKF 83.0%。MHT 只比最佳基线高 1.43 pp，未过预注册 +5 pp 门；翻转为 0，但声明的 95% 协方差只覆盖 68.1%，过度自信。（证据：`records/logs/SIM_S6_uncertainty_fusion_20260813/summary.json`；`pre_registration.json`）

`[测量]` 10 mm/10° 组中 MHT-EKF 仅 29.4%，平移/旋转 p95=12.54 mm/14.99°；它比固定启发式 26.3%略高，但不能满足 5 mm/15° 控制合同。（证据：同一 `summary.json:aggregate.adnan_upper`）

`[测量]` 高可见 held-out 的 candidate oracle=100%，但 SIM-S2 的 new-DA+biased-mask 部署代理 D 组 oracle=0/100。因此本轮分别判为 `FUSION_NOT_SUPPORTED` 与 `DEPLOYMENT_CANDIDATE_GENERATION_BLOCKED`。（证据：同一 `summary.json:high_visibility_oracle_fraction, deployment_proxy_D_oracle_fraction`；`records/logs/SIM_S2_PSM_hybrid_privilege_removal_20260811/results/summary.json`）

![SIM-S6 summary](SIM_S6_uncertainty_fusion_20260813/fusion_summary.png)

## 方法合同

- `[测量]` calibration=episode 1，held-out=episode 2，各 50 帧；每帧 252 个 FP 候选；64 次 Monte Carlo；延迟固定 250 ms。（证据：`records/logs/SIM_S6_uncertainty_fusion_20260813/pre_registration.json`）
- `[测量]` 误差分四档：2 mm/3°、5 mm/5°、10 mm/10°、15 mm/15°；误差是时间相关的 SE(3) 高斯-马尔可夫过程。（证据：`scripts/analyze_sim_s6_uncertainty_fusion.py:20,78-87`）
- `[测量]` MHT 保留 5 个候选分支，以运动学创新协方差、FP rank 和前一分支的运动连续性累计代价；选中分支再做 SE(3) 误差状态更新和延迟运动补偿。（证据：`scripts/analyze_sim_s6_uncertainty_fusion.py:95-137`）
- `[假设]` 这不是 dVRK 关节误差的实测重放。冻结数据没有同步 joint state/FK 链，故这里只能做末端 SE(3) 敏感性代理；真机后必须用同步运动学残差重新估计协方差。（证据：`summary.json:scope_warning`）

## 四档结果

| 先验误差 | fixed pass | MHT pass | MHT flip | MHT t p95 | MHT R p95 | 95% covariance coverage |
|---|---:|---:|---:|---:|---:|---:|
| 2 mm / 3° | 99.97% | 99.97% | 0% | 3.03 mm | 5.94° | 98.47% |
| 5 mm / 5° | 81.58% | 83.01% | 0% | 6.27 mm | 8.45° | 68.07% |
| 10 mm / 10° | 26.27% | 29.43% | 0% | 12.54 mm | 14.99° | 12.63% |
| 15 mm / 15° | 7.88% | 9.51% | 0% | 18.53 mm | 22.55° | 2.99% |

`[推断]` MHT 的主要收益是压住旋转尾部，不是恢复平移；误差越大，候选关联和未建模系统偏置让滤波器越过度自信。继续调固定权重不能解决 deployment D 组 oracle=0 的问题。（证据：上表；`summary.json:gates`）

## 无效首轮

`[测量]` 首轮错误地用 calibration episode 的全部帧估计视觉协方差，但该 episode 只有 24/50 帧含合格候选，导致 sigma 被失败帧拉到 33–105 mm。内部一致性检查发现它与 5 mm 候选合同冲突后，该轮被标成 invalid 并保留审计痕迹；正式轮只用 24 个有合格候选的 calibration 帧。（证据：`records/logs/SIM_S6_uncertainty_fusion_20260813/INVALID_V1.md`; `summary.json:calibration`）

## 决策

`[已证伪]` “在当前候选和误差模型下，上 MHT-EKF 会显著胜过简单 candidate+kinematics 排序”不成立。

`[推断]` 工程主线保持 kinematics+hand-eye；视觉只在 candidate oracle 先达到 ≥80% 后作为有界修正。本轮算法可保留为真机残差到位后的重测骨架，但现在不接控制器。（证据：预注册 stop rule；本报告结论）

