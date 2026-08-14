# SurgicAI R6：单目标成功语义统一与 yaw 范围定向验证（2026-08-09）

## 结论先行

`[测量]` demonstration、reward/HER、termination、evaluation 已统一为每个 episode
一个 reset 后冻结的 `desired_goal`；25 个实时 grasp-angle 候选只保留作诊断，不再提供
额外成功出口。1 cm/10°、measured pose-close、奖励数值和观测均未修改。（证据：
`RL/Approach_env.py:679-731`；`RL/RL_training_online.py:200-228`；
`scripts/test_r6_goal_semantics.py:1-79`）

`[测量]` yaw±15° 新示范 50/50 接受、6341 transitions、settle drift p95=0.0862 cm、
reset rejection 0、hard failure 0，合并文件 SHA256 为
`8A1D7C7F5FB845EE61A5677117ED91645748B9031A50E1B3282C89BEA94E0C38`。
（证据：`records/logs/R6_goal_semantics_20260808/yaw15_collection_summary.json:1-75`）

`[测量]` 两次 150k 从零训练和公共 goal-bank 12/12 评估格均已完成。eval yaw±15°
的 135 个配对 episode 中，yaw±15° 模型为 132/135（97.78%），yaw±30° 模型为
57/135（42.22%），差值 +55.56pp；45-goal cluster bootstrap 95% CI=[40.00,
68.89]pp，goal-level sign-flip p=4.99998e-6，三 seed 同方向，判
**`YAW15_SUPPORTED`**。（证据：
`records/logs/R6_goal_semantics_20260808/final_eval_matrix/analysis.json:252-295`）

`[测量]` yaw±15° 模型在更宽的 eval yaw±30° 上仍为 124/135（91.85%），而 yaw±30°
模型为 53/135（39.26%）。因此当前证据支持把 Approach 训练/采集 yaw 收到真机实测
±15°；R2 的 x/y±3 mm 保持不变。预注册同时触发第三训练：用 yaw±15°、train seed 2
复现，排除单训练 seed 偶然性。（证据：同一 `analysis.json:219-296`）

## 1. 原结构性问题

`[测量]` R3 采集器显式 `freeze_live_goal=True`；历史训练没有显式冻结，环境默认
会不断生成 25 个候选；历史 independent evaluation 默认冻结。历史训练的 reward/HER
只面向一个 `desired_goal`，termination 却允许 25 选 1，因此两者优化的不是同一任务。
（证据：`RL/collect_measured_approach_demos.py:399-400`；
`RL/Approach_env.py:713-718`；`RL/Model_evaluation.py:610-611`）

`[推断]` 所以论文原始 95%、R4 在线 0.61、R5 frozen 0.570 不是可以直接串成一条
进步曲线；R6 的目标是先建立自己的干净单目标基线，而不是“追回 95%”。（依据：
`records/logs/SurgicAI_R5_independent_eval_20260728.md`；
`records/logs/SurgicAI_architecture_red_team_20260727.md`）

## 2. 修复内容

| 层 | R6 语义 | 证据 |
|---|---|---|
| 示范 | reset 后的一个 live needle goal 冻结到 episode 结束 | `RL/collect_measured_approach_demos.py:399-400` |
| reward/HER | 单一 `desired_goal` | `RL/Approach_env.py:713-718` |
| termination | 只检查 `[self.goal_obs]` | `RL/Approach_env.py:713-767` |
| 训练 | measured Approach 显式 `freeze_live_goal=True` | `RL/RL_training_online.py:200-228` |
| 评估 | frozen single-goal，1 cm/10° | `scripts/run_r6_single_goal_eval_cell.sh:75-105` |

`[测量]` 无 AMBF 回归同时验证：备选候选命中被拒绝、主目标命中被接受、诊断列表仍
存在、三处配置都显式单目标。（证据：`scripts/test_r6_goal_semantics.py:1-79`）

## 3. 旧 checkpoint 免费分箱

`[测量]` 使用 R5 已有的 M3-150k FROZEN 135 条 episode，不重跑策略：

| 分箱 | 成功 | 成功率 | Wilson 95% |
|---|---:|---:|---:|
| |yaw| 0–10° | 25/33 | 75.8% | [59.0%, 87.2%] |
| |yaw| 10–20° | 24/48 | 50.0% | [36.4%, 63.6%] |
| |yaw| 20–30° | 28/54 | 51.9% | [38.9%, 64.6%] |
| |yaw|≤15° | 43/57 | 75.4% | [62.9%, 84.8%] |
| |yaw|>15° | 34/78 | 43.6% | [33.1%, 54.6%] |

`[测量]` outer-inner=-31.85 个百分点，触发预注册的“范围假设”分支。（证据：
`records/logs/R6_goal_semantics_20260808/legacy_goal_bins.json:1-91`）

`[推断]` 这是旧 25-candidate 语义训练 checkpoint 在单目标评估上的条件相关，不是
“收窄训练范围必然提高成功率”的因果证据；因此 R6 仍从零训练两个模型作验证。

## 4. yaw±15° 示范质量

| 指标 | 实测 |
|---|---:|
| 接受 episode | 50/50 |
| transitions | 6341 |
| reset rejections / hard failures | 0 / 0 |
| integrator residual fraction≤1e-5 | 0.0 |
| goal 两两最大 | 1.0452 cm / 34.656° |
| settle drift p50/p95/max | 0.0372 / 0.0862 / 0.0956 cm |
| settle rotation error p95 | 2.091° |

（证据：`records/logs/R6_goal_semantics_20260808/yaw15_collection_summary.json:1-75`）

`[测量]` 新集仍使用 x/y±3 mm、yaw±15°、grasp angle 5–20°、LHS、measured
pose-close 与脚本 actuate 合同；R1 沉降和 R2 默认定义没有回退。（证据：
`scripts/run_r6_collect_yaw15.sh:69-101`；同一 collection summary）

## 5. 两次定向训练

| 训练 | steps | 墙钟 | expert transitions | 最后 rollout | final SHA256 |
|---|---:|---:|---:|---:|---|
| yaw±30° | 150917 | 21845.9 s（6h04m） | 8723 | 0.40 | `150363d3…6224407` |
| yaw±15° | 150012 | 17981.2 s（5h00m） | 6341 | 0.91 | `6286a88c…776e525` |

`[测量]` 两个 final ZIP 均通过 `unzip -t`，各自 MANIFEST 0 错；yaw±15° 比 yaw±30°
少用约 17.7% 墙钟。yaw±30° 在线 success 从早期约 0.20–0.25 缓慢升至 0.40；
yaw±15° 最后记录 0.91、峰值 0.96@138634。（证据：
`records/logs/R6_goal_semantics_20260808/training_comparison.json:1-999`；两个模型目录的
`training_run.json` 与 `MANIFEST.sha256`）

`[推断]` 收窄分布明显更容易优化，但在线 rollout 既与训练采样相关又不是独立数据，
所以范围决策仍只看 §6 的公共 goal-bank 配对评估。

`[测量]` 两个单元预注册为同算法 TD3+HER+BC、同 seed 1、同 150k、同 x/y±3 mm、
同 1 cm/10°；只改变 yaw±30° R3 数据/环境与 yaw±15° 新数据/环境。两个都从零
训练，不使用 resume。两组在独立 domain 230/231 与独立 AMBF 可执行副本上并行；
P7g/评估仍等待两组完成后串行启动。（证据：
`scripts/run_r6_single_goal_training.sh:46-149`；
`records/logs/r6_train_yaw15_seed1_launcher.stdout.log`；
`scripts/run_r6_posttraining_sim_queue.sh:24-38`）

## 6. 12 格独立评估

后台矩阵自然结束后一次性汇总如下；12 个格均为同一冻结单目标、同一 checkpoint goal
bank 和 fixed PSM reset，reset_invalid 全为 0。（证据：
`records/logs/R6_goal_semantics_20260808/final_eval_matrix/analysis.json:1-251`）

| 训练范围 | 评估范围 | seed | 已完成 | success | Wilson 95% CI | 状态 |
|---|---|---:|---:|---:|---:|---|
| yaw±30° | yaw±15° | 6101 | 45/45 | 19/45 = 42.22% | [28.97%, 56.70%] | complete |
| yaw±15° | yaw±15° | 6101 | 45/45 | 44/45 = 97.78% | [88.43%, 99.61%] | complete |
| yaw±30° | yaw±15° | 6102 | 45/45 | 19/45 = 42.22% | [28.97%, 56.70%] | complete |
| yaw±15° | yaw±15° | 6102 | 45/45 | 44/45 = 97.78% | [88.43%, 99.61%] | complete |
| yaw±30° | yaw±15° | 6103 | 45/45 | 19/45 = 42.22% | [28.97%, 56.70%] | complete |
| yaw±15° | yaw±15° | 6103 | 45/45 | 44/45 = 97.78% | [88.43%, 99.61%] | complete |
| yaw±30° | yaw±30° | 6101/6102/6103 | 135/135 | 53/135 = 39.26% | [31.43%, 47.68%] | complete |
| yaw±15° | yaw±30° | 6101/6102/6103 | 135/135 | 124/135 = 91.85% | [86.00%, 95.39%] | complete |

`[测量]` eval yaw±15° 的三个配对 seed 都是 19/45 对 44/45，差值均 +55.56pp。
分析没有把重复的 45 goals 冒充 135 个独立 goals：同时报告了 episode McNemar、
45-goal cluster bootstrap 和 goal-level sign-flip permutation。（证据：
`records/logs/R6_goal_semantics_20260808/final_eval_matrix/analysis.json:252-292`）

`[测量]` 第一版仅靠相同随机 seed 的评估已作废并移至
`/home/jiaming/r6_runs/20260808/single_goal_eval_unpaired_precheck/`：两进程 settled goal
最大逐元素差为 0.0321。正式轮改用同一 checkpoint goal bank + fixed PSM；2×2 smoke
逐元素 goal 差为 0.0。（证据：`docs/repro/R6_single_goal_semantics_and_range.md:153-163`）

`[测量]` 主比较通过全部预注册门：差值≥10pp、exact McNemar p=5.29e-23、cluster
bootstrap 下界>0、goal-level permutation p<0.05，且三 seed delta>0。（证据：
`analysis.json:252-295`；判定实现：`scripts/analyze_r6_single_goal_eval.py:145-230`）

## 7. 两个并行仿真结论

### 7.1 相对伺服 fallback

`[测量]` 10,000 组 SE(3) 审计中，共同 frame 误差在
`inv(T_CG)·T_CN` 中抵消到最大 3.42e-13 mm / 3.82e-6°；绝对目标误差则为
p50/p95 15.65/38.20 mm。D7 中 +10 mm 绝对偏差把 Reach 从 20/20 降到 6/20。
（证据：`records/logs/R6_goal_semantics_20260808/relative_servo_upper_bound.json:1-42`）

`[测量]` 当前工程前提失败：P7f gripper pure-FP pitch-only pass 0.54%、flip 96.79%。
因此相对公式正确，但无法以当前 PSM 视觉输入执行；主线仍是 kinematics+hand-eye。
（证据：同一 JSON `:43-51`；
`records/logs/SurgicAI_P7f_PSM_composite_tracking_20260807.md`）

### 7.2 Xiangrui 新 DA 候选资产门

`[测量]` 本机审计到的三个 checkpoint 是 P5a best、同轮 latest、旧 pre-P5a；没有
任何被标为新候选的权重，因此没有伪造“新模型 close-view A/B”。（证据：
`records/logs/R6_goal_semantics_20260808/da_candidate_asset_audit.json:1-43`）

`[推断]` 这只证明本机缺少外部资产，不是对 Xiangrui 尚未共享实验的负结论。候选到手
后直接运行 `scripts/run_p5b_da_candidate_benchmark.sh checkpoint NEW.pth NAME`，无需再写
测试器。（依据：同一 JSON `:39-43`；
`records/logs/SurgicAI_P5b_DA_benchmark_and_gate_review_20260801.md`）

## 8. 决策与限制

`[测量]` 正式矩阵 12/12 complete、540 episode、reset_invalid 0；逐格 `result.json`、
`episodes.jsonl` 和 manifest 已拷回持久目录。（证据：
`records/logs/R6_goal_semantics_20260808/final_eval_matrix/`）

`[推断]` 这支持 **Approach 的工程训练范围改为 x/y±3 mm、yaw±15°**，因为新输入的
真机测量是 yaw±10–15°，而且窄模型在±30°测试仍保持91.85%。它不证明所有后续缝合
子任务也应使用±15°，也不证明比原论文95%更强；契约不同，公平基线是本报告矩阵。

如需重新生成最终统计：

```bash
ROOT=/home/jiaming/r6_runs/20260808/single_goal_eval_matrix
N=$(find "$ROOT" -mindepth 2 -maxdepth 2 -name result.json | wc -l)
echo "completed result.json: $N/12"
test "$N" -eq 12 || { echo "matrix still incomplete" >&2; exit 2; }
python3 /home/jiaming/project34_windows/scripts/analyze_r6_single_goal_eval.py \
  --root "$ROOT" \
  --out "$ROOT/analysis.json"
(cd "$ROOT" && find . -type f ! -name MANIFEST.sha256 -print0 | sort -z | \
  xargs -0 sha256sum > MANIFEST.sha256)
(cd "$ROOT" && sha256sum -c MANIFEST.sha256)
```

`[测量]` R6 没有修改成功阈值、奖励、观测、R3 数据或旧模型目录；默认范围的代码
切换留到 RL8 在 seed2 复现后单独提交，避免把统计决定和实现混成一步。（代码 diff：
`_patch_archive/r6_goal_semantics_prepatch_20260808T234541/`）
