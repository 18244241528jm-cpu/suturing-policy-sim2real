# SurgicAI SIM-S5：Stereo、DA 与融合 depth 对 needle FP 的严格配对审计（2026-08-13）

## 1. 最终判定

`[测量]` **`FUSION_NOT_SUPPORTED`。** 主融合组 D2 相对 B 的 needle MAE p95 没有改善 20%，反而从 1.060 mm 增至 3.221 mm（仅 29/40 帧有任意有效 needle 融合像素）；needle coverage p50 只有 0.617%。D2 的 FP candidate oracle/selected pass 均为 36/40，低于 B 的 40/40；flip 均为 0、dropout 均为 0。（证据：`records/logs/SIM_S5_stereo_DA_depth_fusion_20260813/summary.json` 的 `depth.{B_DA,D2_ROBUST}.needle`、`foundationpose.{B_DA,D2_ROBUST}`、`decision_gates`）

`[测量]` `STEREO_INVALID_FOR_TASK` 的字面有效率门没有触发：C 在 needle 上的 disparity-valid fraction 为 100%。但它与 DA 的 >5 mm 强冲突率均值为 97.671%，needle MAE p50/p95=8.778/9.682 mm，FP oracle/selected=0/40、flip=4。因此“有 disparity”不是本任务中的可信几何锚点。（证据：同一 `summary.json` 的 `stereo_needle_valid_fraction`、`needle_DA_stereo_conflict_fraction`、`depth.C_STEREO.needle`、`foundationpose.C_STEREO`）

`[测量]` 预注册 Reach 触发条件为 false：D2 没有比 B 增加 selected pass 或 candidate oracle，且实际更差 4/40；所以未运行 40 个 frozen-goal D2 Reach，未调用 `learn()`。（证据：同一 `summary.json` 的 `reach_trigger=false`、`reach_action="skip by preregistration"`；`pre_registration.json:foundationpose,reach_trigger`）

## 2. 配对合同与输入边界

`[测量]` 复用 SIM-S3 的冻结采集，而不是重新驱动一批不同 reset：40 个独立 reset 的 seed 为 100000–100039，x/y ±3 mm、yaw ±15°，ECM 固定、针在拍摄时静止；每帧已有同步 L/R RGB、两眼 GT depth、两眼 semantic needle mask、K、左右相机外参和 GT needle pose。六流同步 spread 均为 0 ms。（证据：`raw_audit/reset_bank.json` 的 `command_range,entries[*].reset_seed`；`raw_audit/capture/capture_report.json` 的 `frames_completed,rows[*].six_stream_sync_spread_ms`；完整数组位置见 `raw_audit/RAW_DATA_LOCATION.json`）

`[测量]` A/B/C/D1/D2 对每个 frame 使用完全相同的左 RGB、perfect semantic mask、K、GT pose、FP iteration=5 与 seed=2718；每次 register 前重新设置 Python/NumPy/Torch/CUDA 的同一个 seed，且只调用一次 register。全部 200 次成功调用都保存了 252 个候选。（证据：`raw_audit/foundationpose/per_register.jsonl` 的 `frame_id,group,fp_seed,iteration,register_call_count,candidate_count`；`raw_audit/foundationpose/summary.json:all_successful_registers_have_252_candidates=true`）

`[测量]` GT depth 只进入 A 上限组以及所有组完成预测后的事后评分；B/C/D1/D2 算法不读取 GT depth。perfect semantic mask 是明确的仿真特权输入，只用于相同 FP mask 和事后区域评分，不能称为真机自动分割。（证据：`depth_audit_stage_a/run_sim_s5_depth_fusion.py:219-228`；`pre_registration.json:groups,regions`）

## 3. 双目几何确认

`[测量]` 当前配置是 **AMBF 仿真 stereo，不是真机 JHU ECM 标定**。K 的 fx=fy=358.807 px、cx=320、cy=240；逐帧外参给出 baseline=4.000 mm、relative rotation=0.241245°，右相机在左相机坐标中的平移约 `[-0.0084, 3.99999, 0]` mm。由于 baseline 主要落在左相机 Y 轴，先执行 OpenCV stereo rectification，再匹配和回投原左图。（证据：`raw_audit/depth/per_frame.jsonl` 的 `geometry`；`raw_audit/capture/world_depth_audit_stereo.yaml`；`depth_audit_stage_a/run_sim_s5_depth_fusion.py:88-130`）

`[推断]` 本轮没有建立或声称真机 stereo；4 mm 小基线、AMBF shader/纹理和当前工作距离共同构成本结果的适用边界。不能把仿真 C 组失败直接外推成所有真机双目都失败。

## 4. Depth 结果

表内为 40 帧“逐帧区域 MAE”的 p50/p95（mm）；括号内为 coverage p50。D1/D2 的 needle MAE 只有 29 个非空帧，coverage 仍保留全部 40 帧。

| 组 | full | phantom | needle | needle-edge |
|---|---:|---:|---:|---:|
| A GT upper | 0/0 (100%) | 0/0 (100%) | 0/0 (100%) | 0/0 (100%) |
| B new-DA | 1.124/1.239 (100%) | 0.836/0.965 (100%) | 0.680/1.060 (100%) | 0.704/0.998 (100%) |
| C stereo | 25.540/26.015 (66.63%) | 7.503/7.522 (85.45%) | 8.778/9.682 (100%) | 8.313/9.005 (100%) |
| D1 hard | 1.851/1.927 (15.01%) | 1.560/1.650 (20.01%) | 5.278/5.892 (0.62%) | 4.527/5.549 (3.54%) |
| D2 robust | 1.243/1.305 (15.01%) | 0.971/1.003 (20.01%) | 2.592/3.221 (0.62%) | 1.842/2.391 (3.54%) |

`[测量]` D1/D2 强冲突处严格输出 invalid，没有用 DA 或 GT 静默覆盖；这造成 11/40 帧的 needle 融合区域完全无值。无效帧没有被删除。（证据：`raw_audit/depth/summary.json:groups.{D1_HARD,D2_ROBUST}.needle` 的 `mae_mm.count=29,coverage.count=40`；`depth_audit_stage_a/run_sim_s5_depth_fusion.py:141-160`）

`[测量]` depth 端推理 p50/p95：B DA=0.188/0.265 s，C stereo=0.0149/0.0159 s，D1/D2 的 DA+stereo+fusion 总计=0.210/0.286 s。（证据：`per_frame.csv:depth_runtime_s`；原分量见 `raw_audit/depth/summary.json:runtime_s`）

## 5. FoundationPose 结果

| 组 | register | candidate oracle | selected 5mm/15° | flip | dropout | register p50/p95 (s) |
|---|---:|---:|---:|---:|---:|---:|
| A GT upper | 40/40 | 40/40 | 40/40 | 0 | 0 | 2.321/4.764 |
| B new-DA | 40/40 | 40/40 | 40/40 | 0 | 0 | 2.330/3.258 |
| C stereo | 40/40 | 0/40 | 0/40 | 4 | 0 | 2.311/3.498 |
| D1 hard | 40/40 | 32/40 | 21/40 | 1 | 0 | 2.294/2.632 |
| D2 robust | 40/40 | 36/40 | 36/40 | 0 | 0 | 2.312/2.379 |

`[测量]` C 的候选集合本身已经失败（oracle 0/40），因此不是只需换 top-1 排序器的问题。D2 虽比 C/D1 好，但仍把 B 的四个正确 selected/oracle 样本变坏。（证据：`raw_audit/foundationpose/summary.json:groups`；全部候选见 `raw_audit/foundationpose/candidates/`）

`[假设]` FoundationPose 在 mask 内有效 depth 为 0 时仍能返回 252 hypotheses；因此 `register_success=40/40` 只能表示 API 返回，不能证明融合提供了可用几何。候选生成内部可能使用 mask bbox/其他有效 depth 或默认初始化，本轮没有修改 FP checkpoint 或内部实现来分解该机制。（证据：`raw_audit/foundationpose/per_register.jsonl` 中部分 D1/D2 行 `mask_depth_valid_fraction=0,candidate_count=252`）

## 6. Fusion 实现与预注册门

`[测量]` D1 使用 rectification、SGBM uniqueness=15、speckle window=50、LR consistency≤1 px、0.03–0.20 m 范围门；stereo 无效时补 DA，中等冲突回 DA，>5 mm 强冲突 invalid。D2 仅在一致门内按 `sigma_stereo` 与冻结 `sigma_DA=1.6 mm` 做逆方差 Huber robust fusion；不是 0.5/0.5 平均，也没有按 FP 结果调权。（证据：`pre_registration.json:stereo_matcher,fusion`；`depth_audit_stage_a/run_sim_s5_depth_fusion.py:88-160`）

`[测量]` 四个 `FUSION_SUPPORTED` 子门结果依次为 false、false、true、true：depth p95 未改善、FP pass 下降、flip=0、dropout 未增加。因此不满足支持判据；depth 也没有改善，故不是 `FUSION_DEPTH_ONLY`。（证据：`summary.json:decision_gates`）

`[已证伪]` 对当前 4 mm AMBF stereo 与冻结参数，路线“把 stereo 当真实几何锚点，与 new-DA 融合即可改善 needle depth p95 且不伤 FP”已被本轮严格配对结果否定。后续不能在相同数据上继续事后调权、放宽强冲突门或只报全图 MAE来重复投入。

## 7. 运行约束与异常记录

`[测量]` ROS domain 228 在实验前只有 `/parameter_events`、`/rosout`；原始数据和日志保存在 `/home/jiaming/sim_s5_runs/20260813/formal`，没有写 `/tmp`。本轮没有启动 AMBF，因为严格复用已有 40 个同步 reset 比重新生成一批更能满足配对合同。（证据：`raw_audit/domain_preflight_topics.txt`；`raw_audit/RAW_DATA_LOCATION.json`）

`[测量]` 初次软件试跑发现“3σ 一致门可能与 >5 mm 强冲突门同时为真”；在正式 200 次 FP 完成前停止，修正为强冲突绝对优先，并从冻结输入重算全部 40 帧。没有改阈值或看 FP 后调参。（证据：最终代码 `run_sim_s5_depth_fusion.py:146-150`；预注册文件在首次 depth/FP 结果前已写入）

## 8. 产物与边界

- 汇总：`records/logs/SIM_S5_stereo_DA_depth_fusion_20260813/summary.json`
- 逐帧配对表：`per_frame.csv`
- 原始 depth/FP JSON 与 200 份候选：`raw_audit/`
- 大数组持久位置与内容：`raw_audit/RAW_DATA_LOCATION.json`
- 对比图：`depth_region_comparison.png`、`fp_group_comparison.png`
- needle 局部 depth/diff：`needle_local_depth_diff.png`
- 可播放 H.264 视频：`sim_s5_depth_comparison_h264.mp4`
- WSL 原始完整性：`raw_audit/WSL_RAW_MANIFEST.sha256`
- 项目产物完整性：`MANIFEST.sha256`

`[假设]` 结果不验证真机 stereo、真实自动 mask、真实 DA domain gap、hand-eye/flexibility、物理 grasp/close/lift。semantic mask 和 AMBF GT 都是仿真特权；只有 A 与事后评分可见 GT depth。
