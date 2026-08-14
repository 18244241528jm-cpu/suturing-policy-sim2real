# SurgicAI SIM-S4：部署代理 frozen-goal Reach（2026-08-11）

## 1. 最终判定

`[测量]` **`DEPLOYMENT_PROXY_REACH_SUPPORTED`。** B组在30个冻结配对reset中达到deployment Reach 29/30（96.67%，Wilson 95% CI 83.33%–99.41%），collision proxy=0、接受后的needle flip=0、方向反转=0、reset hard failure=0；五项预注册主门全部为true。（证据：`records/logs/SIM_S4_deployment_proxy_reach_20260811/summary.json:42-90`）

`[测量]` A组为30/30（Wilson 95% CI 88.65%–100%）。A没有覆盖B的结论；B的29/30独立满足≥27/30门。（证据：同一`summary.json:5-40`、`:42-86`）

`[测量]` 唯一B组失败是episode 11：最终GT事后误差2.935 mm/20.0369°，比固定20°门多0.0369°，termination=`stall_abort`，归因`D2 controller`；该失败完整保留，没有放宽阈值。（证据：`per_episode.csv:43`；`summary.json:67-77`）

## 2. 依赖、配对与运行合同

`[测量]` SIM-S1判`NO_SINGLE_VIEW_FEASIBLE`，因此这里只把MID作为较远全局运行回退，不能称其FOV合同通过；SIM-S2判`CANDIDATE_OR_SELECTION_LIMITED`，所以PSM FoundationPose top-1/候选均禁止进入控制，PSM保持kinematics+hand-eye；SIM-S3判`LIVE_INITIAL_GATE_SUPPORTED`，因此本仿真代理不强制人工确认。（证据：`pre_registration.json:6-10`）

`[测量]` A/B严格复用同一批30个SIM-S3冻结reset：eval seed 1，x/y±3 mm，yaw通过runner覆盖±15°，每次R1 settle最多60步；没有修改`needle_reset_ranges.py`。（证据：`pre_registration.json:16-24`；`raw_audit/banks/A_gt_frozen.json`；`raw_audit/banks/B_deployment_proxy_frozen_bias5.json`）

`[测量]` 建议domain 234超出当前Fast DDS UDP端口范围，运行前冻结改用干净domain 230；AMBF使用私有可执行文件`/home/jiaming/sim_s4_runtime/ambf_sim_s4_domain230`，原始持久目录为`/home/jiaming/sim_s4_runs/20260811/formal`。（证据：`pre_registration.json:11-15`；`raw_audit/domain_preflight_topics.txt`）

`[测量]` 7个RL核心文件canonical/mirror仍全部SAME；逐文件SHA256保存在SIM-S1 preflight，包括`Model_evaluation.py=75c36…e2b`、`needle_reset_ranges.py=a546…c02`等7行。（证据：`records/logs/SIM_S1_camera_fov_contract_20260811/preflight.txt:1-7`）

## 3. A/B输入与冻结目标

`[测量]` A组用needle GT pose构造首帧frozen goal、无PSM transform bias并执行D2 staged。B组使用锁定MID RGB→P5a new-DA→`manual-mask-equivalent deployment proxy`→一次live FP register→SIM-S3固定支撑面门；门通过后冻结needle goal，并给PSM kinematics/hand-eye代理注入每episode可复现的5 mm/5° goal-frame偏差。（证据：`pre_registration.json:25-30`；`per_episode.csv`的`mask_source`、`frozen_goal`、`psm_bias`列）

`[假设]` **`manual-mask-equivalent deployment proxy`由AMBF semantic/GT-derived mask作冻结扰动而来，不是真实自动分割器，也不能称“全自动无特权mask”。** 它只代理人工首帧mask的有限平移/形态误差。（证据：`pre_registration.json:27-30`；SIM-S3 `pre_registration.json`）

`[测量]` B组全部30个首帧均被固定门接受，事后needle flip为0；没有读取GT来纠正候选或frozen goal，needle GT只进入分析评分字段。（证据：`summary.json:83-88`；`per_episode.csv`的`gate_accepted`、`needle_flip_posthoc`列；`raw_audit/banks/B_deployment_proxy_frozen_bias5.json`）

## 4. D2控制与Reach评分

`[测量]` 沿用D7/D2固定合同：每步1.5 mm/3°，内部staged termination 3 mm/3°，stall abort 700步；没有修改`Approach_env.criteria()`、reward、observation或成功契约。（证据：`pre_registration.json:31-39`；`raw_audit/A_GT_frozen_D2/status.txt`；`raw_audit/B_deployment_proxy_D2/status.txt`）

`[测量]` 外部事后同时复评分10 mm/20° deployment Reach、5 mm/15°和1 cm/10°。B分别为29/30、4/30、26/30；A分别为30/30、30/30、29/30。（证据：`summary.json:8-17`、`:44-53`）

`[测量]` B终点平移误差p50/p95/max=6.790/8.009/8.127 mm，旋转误差p50/p95/max=6.154/15.579/20.037°；A分别为2.941/3.016/4.275 mm和2.203/9.826/10.043°。（证据：`summary.json:26-35`、`:62-70`）

## 5. 误差瀑布与失败归因

`[测量]` B的首帧perception goal相对needle GT误差为平移p50/p95=0.537/0.961 mm、旋转2.588/5.034°；加入冻结5 mm/5° transform bias后为5.072/5.462 mm、5.854/8.296°；最终控制端点为6.790/8.009 mm、6.154/15.579°。（证据：`summary.json:130-165`；`perception_transform_control_waterfall.png`）

`[推断]` 在本冻结代理内，首帧针候选门不是29/30的瓶颈；唯一外部门失败发生在D2 stalled endpoint，且与固定goal仍差8.124 mm/15.090°。这支持按预注册分箱为`D2 controller`，但不授权重训RL或改变D2门。（依据：`per_episode.csv:43`；`failure_attribution`=`summary.json:88-90`）

`[测量]` A环境`criteria()`成功21/30、stall 9；B环境成功24/30、stall 6。它们与外部Reach分数并列保留，不能用内部success或scripted attachment替换deployment Reach。（证据：`summary.json:18-24`、`:54-61`、`:168`）

## 6. 碰撞、反转、attachment与teardown

`[测量]` 预注册collision proxy只检查PSM2 finger sensor在首次GT事后Reach之前是否接触非Needle对象，并按对象identity排除PSM/ghost自体；A/B均为0。它不是全link刚体碰撞测量。（证据：`pre_registration.json:40-41`；`summary.json:20`、`:56`、`:92`）

`[测量]` 首次Reach之后的phantom contact为A 1/30、B 7/30；B scripted attach 24/30、jaw closed observed 30/30。这些都只作附加记录，既不撤销已经到达的Reach，也不构成真实close/lift或物理抓取证据。（证据：`summary.json:21-25`、`:57-61`、`:168`）

`[测量]` A/B方向反转均0，B所有失败只落入允许分箱中的`D2 controller` 1次。（证据：`summary.json:22`、`:58`、`:72-77`、`:88-90`）

`[测量]` evaluator exit 137和AMBF wait exit 137/143来自runner请求teardown；两组`artifact_complete=1`且`ambf_teardown_clean=true`。A完成30个episode后shell的`wait`+errexit中断了状态收尾，随后只恢复状态并从B继续，没有重跑A或选择性覆盖结果。（证据：`summary.json:93-128`；`raw_audit/A_GT_frozen_D2/status.txt`；`raw_audit/B_deployment_proxy_D2/status.txt`）

## 7. 判定边界

`[测量]` 本阶段只判定**固定ECM下的部署代理Reach**；不把scripted attachment、jaw、close/lift计入成功。B失败不能由A数字覆盖，episode 11也没有被删除。（证据：`summary.json:4`、`:42-90`、`:168`；`per_episode.csv:43`）

`[已证伪]` “需要把P11b PSM FP top-1接入控制才能达到本阶段Reach”在本实验中不成立：B明确使用kinematics+hand-eye且29/30；但这不证明真实hand-eye/flexibility足够准确。（证据：`pre_registration.json:8`、`:30`；`summary.json:44-50`）

`[假设]` 仿真没有验证真实DA domain gap、真实hand-eye/flexibility、真实close/lift，也没有验证真实自动分割；这些仍是第一次真机试验的独立硬门。

## 8. 产物与完整性

- 预注册：`records/logs/SIM_S4_deployment_proxy_reach_20260811/pre_registration.json`
- 逐episode/逐step：`per_episode.csv`、`per_step_trace.csv`
- 图：`ab_success_wilson.png`、`perception_transform_control_waterfall.png`、`per_episode_min_final_error.png`、`collision_reversal_stall.png`、`success_failure_trajectories.png`
- 视频：`success_episode_h264.mp4`（完整成功episode）与`typical_failure_episode_h264.mp4`（episode 11典型失败）
- 原始审计：`raw_audit/`；WSL持久目录：`/home/jiaming/sim_s4_runs/20260811/formal`
- 修改前存档：`_patch_archive/sim_s4_prepatch_20260811_094904/`
- 完整性：`records/logs/SIM_S4_deployment_proxy_reach_20260811/MANIFEST.sha256`
