# SurgicAI SIM-S3：首帧针支撑面门 live 验证（2026-08-11）

## 1. 最终判定

`[测量]` **`LIVE_INITIAL_GATE_SUPPORTED`。** 主条件 `DEPLOYMENT_PROXY` 的 40 次独立 reset 全部完成一次 live FoundationPose register：raw top-1 有 1/40 个 >90° 翻转，固定 D7 支撑面门后接受 40/40、翻转 0/40、正确样本误拒 0/40；接受样本平移 p95=1.066 mm、旋转 p95=4.950°，reset hard failure=0，五项预注册门全部通过。（证据：`records/logs/SIM_S3_live_needle_initial_gate_20260811/raw_audit/result.json:4-112`）

`[测量]` `CONTROL` 同样接受40/40、门后0翻转，平移/旋转p95=1.042mm/4.251°；因此主结论不是依赖 perfect mask 才成立。（证据：同一`result.json:23-63`）

## 2. 数据、domain 与一次性 register 合同

`[测量]` 运行前 domain 229 除ROS内建`/rosout`、`/parameter_events`外无任务topic；建议的233超过当前Fast DDS支持的UDP端口范围，故预注册使用干净229。AMBF使用私有可执行文件`/home/jiaming/sim_s3_runtime/ambf_sim_s3_domain229`，原始日志与数据持久化于`/home/jiaming/sim_s3_runs/20260811/formal`。（证据：`records/logs/SIM_S3_live_needle_initial_gate_20260811/pre_registration.json`；`raw_audit/domain_preflight_topics.txt`）

`[测量]` 40次reset均使用x/y±3mm、runner参数yaw±15°，每次执行R1最多60步settle；`needle_reset_ranges.py`未改。每个reset只对同一RGB/new-DA depth的CONTROL和DEPLOYMENT_PROXY各调用一次cold register，共80次；无重复register、相邻帧投票或GT候选选择。（证据：`raw_audit/reset_bank.json`；`raw_audit/result.json:5-9`；`per_reset_condition.csv`）

`[测量]` 7个RL核心文件canonical/mirror仍全SAME：SHA逐文件保存在SIM-S1 preflight，包含`Model_evaluation.py=75c36…e2b`、`needle_reset_ranges.py=a546…c02`等7行。（证据：`records/logs/SIM_S1_camera_fov_contract_20260811/preflight.txt:1-7`）

## 3. Mask与new-DA输入

`[测量]` `CONTROL`为AMBF perfect semantic needle mask；`DEPLOYMENT_PROXY`使用运行前冻结的seed 20260811，逐reset独立采样x/y整数平移[-3,3]px、erosion/dilation/none和0–3px radius。代理mask像素p50/p95=589/1084.7，和CONTROL的IoU p50/p95=0.451/0.720。（证据：`pre_registration.json`；`raw_audit/perturbation_plan_frozen.json`；`raw_audit/result.json:87-99`）

`[假设]` **`DEPLOYMENT_PROXY`只是人工首帧mask误差代理，不是真实自动分割器；本阶段不能声称自动mask已完成。** 扰动不读取GT pose来决定方向，但其源mask仍来自仿真semantic。（证据：`pre_registration.json:mask_conditions`；`raw_audit/result.json:deployment_proxy_warning`）

`[测量]` P5a/new-DA checkpoint SHA为`fc46be…c88`，40帧推理延迟p50/p95=0.188/0.265s；仿真needle区域depth MAE p50/p95=0.680/1.060mm。GT depth只用于事后depth误差审计，没有送给FP。（证据：`raw_audit/da_result.json:4-25`；`raw_audit/per_condition.jsonl`）

## 4. 固定支撑面门

`[测量]` 完整沿用D7标定：world rest normal=`[0.092848,0.113400,0.989201]`、support height=0.742987m；候选必须同时满足法向偏差≤20°和高度偏差≤5mm，在通过者中取原始FP score最高者。无候选通过时拒绝，绝不回退raw top-1。（证据：`pre_registration.json:support_gate`；`raw_audit/result.json:11-20`）

`[测量]` 每次register固定产生252个候选，80次共保存20,160个候选的pose、raw score、score rank、normal/height gate与事后误差。（证据：`analysis.json:66`；`all_candidates_raw_score_rank.csv`；`raw_audit/candidates/`）

## 5. Raw翻转被门消除的直接证据

`[测量]` 唯一raw翻转是`frame_000008`：raw score rank 1候选旋转误差179.768°，固定物理门选择rank 2候选，旋转误差4.188°、平移误差0.686mm；该选择过程不读取GT，GT只用于这句话的事后评分。（证据：`per_reset_condition.csv:19`；`all_candidates_raw_score_rank.csv:4286-4288`；`failure_all_candidates.png`）

`[推断]` 在本仿真固定ECM、new-DA和冻结扰动范围内，D7的支撑面/自然静止姿态约束提供了raw FP score缺少的消歧信息；它验证的是“接受后0翻转”的live候选门，不是自动分割或真实域鲁棒性。（依据：raw 1 flip→gated 0 flip；§3边界）

## 6. Reset、接受/拒绝与误差

| 条件 | register/dropout | raw flip | 接受/拒绝 | 门后flip | 正确样本误拒 | 平移p50/p95 | 旋转p50/p95 |
|---|---:|---:|---:|---:|---:|---:|---:|
| CONTROL | 40/0 | 0/40 | 40/0 | 0/40 | 0/40 | 0.513/1.042mm | 2.239/4.251° |
| DEPLOYMENT_PROXY | 40/0 | 1/40 | 40/0 | **0/40** | **0/40** | 0.500/1.066mm | 2.564/4.950° |

`[测量]` reset settle drift p50/p95/max=0.315/0.755/1.106mm，settle steps p50/p95/max=11.5/16/19，40次均settled且无hard failure。（证据：`analysis.json:55-64`；`raw_audit/reset_bank.json`）

## 7. 判定边界与后续合同

`[测量]` 预注册五门均为true；没有接受翻转，因此不触发强制`HUMAN_CONFIRMATION_REQUIRED`分支。（证据：`raw_audit/result.json:108-114`）

`[假设]` semantic-derived扰动不能覆盖真实分割器的漏检、错类、反光、血液、遮挡和域偏移；new-DA也仍在仿真域。因此SIM-S4可使用该门作为`manual-mask-equivalent deployment proxy`，但不能把它改写成“全自动无特权mask”。

## 8. 产物、失败留痕与完整性

- 预注册：`records/logs/SIM_S3_live_needle_initial_gate_20260811/pre_registration.json`
- 逐reset/逐候选：`per_reset_condition.csv`、`all_candidates_raw_score_rank.csv`、`raw_audit/candidates/`
- 图：`40_reset_candidate_dashboard.png`、`raw_gated_flip_comparison.png`、`acceptance_rejection_confusion_matrix.png`、`mask_perturbation_visualization.png`、`failure_all_candidates.png`、`first_frame_overlay.png`
- 视频：`sim_s3_live_gate_h264.mp4`（40 reset，640×480，H.264）
- WSL raw：`/home/jiaming/sim_s3_runs/20260811/formal`
- `[测量]` 第一次FP编排在任何register前因OpenCV 4.13的`warpAffine`关键字兼容性退出；失败日志保留为`raw_audit/foundationpose_attempt1_opencv_api_failure.log`。随后仅修正`interpolation`→`flags`并对同一冻结40帧重跑FP，没有重采或调门。
- 修改前存档：`_patch_archive/sim_s3_prepatch_20260811_092807/`

