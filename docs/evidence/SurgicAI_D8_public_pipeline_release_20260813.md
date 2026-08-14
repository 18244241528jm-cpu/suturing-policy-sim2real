# SurgicAI D8：公开 pipeline 代码仓库发布

日期：2026-08-13
目标仓库：<https://github.com/18244241528jm-cpu/suturing-policy-sim2real>
公开提交：`b31319ed145e85f949fcca022d6ef18bf0cea420`

## 结果

`[测量]` GitHub `main` 已推送到 `b31319e`，公开包包含88个本次新增/修改/重命名文件、约0.72 MB核心代码与文档；未包含DA 3.8 GB checkpoint、RL checkpoint、原始数据、rosbag、AMBF二进制、FoundationPose镜像或第三方仓库。（证据：公开提交`b31319e`；`packaging/public_repo_20260813/README.md`；`packaging/public_repo_20260813/docs/MODEL_ASSETS.md`）

`[测量]` 代码按RL、perception、control、hand-eye和runner五层组织；README明确区分已经验证的frozen-goal Reach子链与尚未验证的自动mask、真机DA、真机hand-eye和physical close/lift。（证据：`packaging/public_repo_20260813/README.md`；`packaging/public_repo_20260813/docs/ARCHITECTURE.md`）

`[测量]` 发布前通过：code-only preflight、2个控制器合同测试、Python compileall、全部Bash `-n`、大文件/常见凭据扫描，以及ECM/PSM clean+noisy四套hand-eye synthetic self-test；self-test最终`failures=[]`、`passed=true`。（证据：本轮终端输出；`packaging/public_repo_20260813/scripts/preflight.py`；`packaging/public_repo_20260813/tests/test_public_contract.py`）

`[测量]` 远端原有两个文件名含Windows非法字符`?`，已做100%内容保留的重命名为`README_zh.md`和`dVRK_Policy_Deployment_Checklist_zh.md`，使后续Windows clone不再被该文件名阻塞。（证据：公开提交`b31319e`的rename记录）

`[测量]` 推送后从公开URL重新执行Windows `git clone --depth 1`得到HEAD=`b31319e`，clean clone的code-only preflight和2个控制器合同测试再次通过，工作树无修改。（证据：本轮clean-clone终端输出；公开提交`b31319e`）

## 边界

`[推断]` 该仓库让其他人能够审核接口、运行硬件无关测试，并在自行取得外部checkpoint/场景/原始frame bank后复现仿真pipeline；它不是“一次clone即可复现所有统计数字”的全资产归档。（依据：`packaging/public_repo_20260813/docs/MODEL_ASSETS.md`）

`[测量]` 公开README没有把29/30 Reach写成完整抓取或缝合成功，并明确pure PSM FP、stereo+DA融合、MHT/EKF增益和near-normal DA均为负结果或未达部署门。（证据：`packaging/public_repo_20260813/README.md`；`packaging/public_repo_20260813/docs/KNOWN_LIMITATIONS.md`）
