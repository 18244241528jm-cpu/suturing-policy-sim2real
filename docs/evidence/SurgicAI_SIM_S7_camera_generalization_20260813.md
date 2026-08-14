# SurgicAI SIM-S7：靠近且正对 phantom 的 DA 泛化审计

## 结论

`[测量]` **“靠近并正对 phantom 会让当前 DA 更准”被强烈证伪。** 5 个 episode、每格 100 个严格配对帧中，mid+oblique 基线针区 MAE p50/p95=0.769/1.696 mm；near+normal（约 40 mm）为 26.516/33.489 mm，分别恶化 34.47×/19.75×。只把角度改正视、距离仍约 82 mm，也恶化到 1.942/6.660 mm。（证据：`records/logs/SIM_S7_camera_generalization_20260813/summary.json:DA, comparisons`）

`[测量]` 几何上 near+normal 的确把针像素 p50 从 502 提到 1619、PSM 像素从 3374 提到 13908；同样近距离但保持旧斜视时针和 PSM 均为 0 像素。因此“拉近”必须同时重瞄准，但像素增大不保证 DA 泛化。（证据：`summary.json:geometry`）

`[测量]` near+normal 仍有 external background p50=29.54%，PSM 可见率 74.36%，未过预注册 5%/80% 门，故总体判定 `FOV_CONTRACT_FAILED`；局部 DA 结论仍可独立判为负。（证据：`summary.json:fov_gate_passed, decision`；`pre_registration.json:fov_gate`）

![SIM-S7 quantitative summary](SIM_S7_camera_generalization_20260813/camera_da_ab.png)

![SIM-S7 paired depth montage](SIM_S7_camera_generalization_20260813/paired_depth_montage.png)

## 实验设计

- `[测量]` 四个相机同一 AMBF step 同时渲染：mid/near × 当前约 33.8°斜视/世界 phantom 法向正视。（证据：`depth_audit_stage_a/world_sim_s7_camera_generalization.yaml:32-177`；`capture_sim_s7_camera_generalization.py:31-46`）
- `[测量]` 固定 5 个 Approach episode，每格统一抽 20 帧/episode，共 100；RGB、GT depth、针/PSM 区域、相机/目标位姿均按相同 episode/sequence 配对。（证据：`scripts/select_sim_s7_frames.py`; `summary.json:paired_contract`）
- `[测量]` DA checkpoint SHA256=`fc46bead...ec88`，严格复用 P5a ViT-L、518×518、FP32 推理合同，没有重训或改预处理。（证据：`records/logs/SIM_S7_camera_generalization_20260813/da/DA_CHECKPOINT.sha256`；`scripts/run_sim_s7_da_ab.sh`）
- `[假设]` AMBF semantic mask 只用于区域误差与可见性评分，不是现实自动 mask。（证据：`capture_sim_s7_camera_generalization.py:1-7`; `summary.json:semantic_mask_warning`）

## 为什么更大的针反而更差

`[推断]` 当前 DA 不是“逐像素从几何推深度”的算法；它学习了训练分布中的尺度、相机俯角、phantom 布局与目标上下文。正视改变布局，40 mm 又把尺度推到训练分布外，模型仍输出类似原分布的整体深度结构，造成针及边缘 20–30 mm 系统误差。图中的 DA−GT 整片同号偏差直接支持这是全局尺度/视角域偏移，不是“针太小看不清”这一单因子。（证据：`paired_depth_montage.png`; `summary.json:DA.near_normal`）

`[测量]` relief 正号率仍为 98%，说明模型知道针是凸起，但绝对 metric depth 错了几十毫米；“形状方向对”不能替代控制所需的 metric depth。（证据：`summary.json:DA.near_normal.relief_sign_match_fraction`）

## 被拒绝的 60 mm pilot

`[测量]` 首次 near-normal=60 mm 的 180×4 帧 pilot 留下 43.9%背景，未实现“phantom 铺满”的预注册意图，因此在 DA 前拒绝，没有混入正式统计；正式重采改为 40 mm。（证据：`records/logs/SIM_S7_camera_generalization_20260813/rejected_60mm_pilot.json`; `_patch_archive/sim_s7_geometry_prepatch_20260813T194305/`）

## 决策

`[已证伪]` 不应直接把当前 DA 用在近且正视 ECM，也不应因为针像素变大就宣称精度提高。

`[推断]` 若真机选择近/正视 ECM，必须采真实该视角数据，重训或至少重新标定 metric head 后再过同一针区 p50/p95 门；demo 前更保守的方案仍是锁定已验证的 mid+oblique 几何。（证据：本报告结果；`records/logs/SurgicAI_P11a_paired_work_distance_20260809.md`）

