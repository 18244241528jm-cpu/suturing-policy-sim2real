# 复现步骤

## A. 先复跑两个样例（不连接机器人）

在 `R6_REPRODUCTION/` 内，用已有 numpy、opencv、scipy 的 Python 环境执行。以下命令只在临时目录产生结果，不改 examples。

```bash
set -euo pipefail
export P="$PWD"
export PYTHONPATH="$P/scripts/required_dependencies${PYTHONPATH:+:$PYTHONPATH}"
RUN=$(mktemp -d)
export RUN
mkdir -p "$RUN/trials"
cp -R examples/successful_trial "$RUN/trials/successful_trial"
cp -R examples/failed_trial "$RUN/trials/failed_trial"

# 将保存的 npz 视差展开成原脚本需要的 npy；只写工作副本。
python3 - <<'PY'
import os
from pathlib import Path
import numpy as np
for p in Path(os.environ['RUN']).rglob('fs_disp.npz'):
    np.save(p.with_suffix('.npy'), np.load(p)['disp'].astype(np.float32))
PY

T="$RUN/trials/successful_trial"
python3 scripts/r6_perceive.py --trial "$T" \
  --snaps "$T/snap0" "$T/snap1" --clicks "$T/clicks.json" --s 0.5

T="$RUN/trials/failed_trial"
python3 scripts/r6_perceive.py --trial "$T" \
  --snaps "$T/perception_reference/snap0" "$T/perception_reference/snap1" \
  --clicks "$T/clicks.json" --s 0.25

echo "结果目录：$RUN/trials"
```

查看工作副本的 `perception.json` 和 `overlay_*.png`。与 examples 中保存的 p_star_mm 比较，回放门槛为差值 ≤ 0.1 mm；受 float16 保存和数值环境影响，不要求逐位相同。失败样例的感知通过也是正常的，其失败发生在抓取执行阶段。

**这里生成的是历史数据回放，不是新鲜的现场感知，不能传给 r6_grasp.py。**

## B. 到实验室后，核对 3090 的 R5 环境

使用 Adnan 工作站上的既有 FoundationStereo 环境，先确认 [requirements.md](requirements.md) 中的源码、权重、配置和缓存。不要覆盖已有安装。确认后记录 Python/PyTorch/CUDA 版本和模型 SHA256。

在工作站上，对一组新图像运行原推理脚本；下列变量需填入现场实际路径：

```bash
# FS_PYTHON：已有 FS 环境的 python；FS_ROOT：源码目录。
# FS_CKPT：23-51-11/model_best_bp2.pth；IN/OUT：本次输入和新输出目录。
PYTHONPATH="$FS_ROOT" "$FS_PYTHON" \
  "$P/scripts/required_dependencies/r6_fs_infer.py" \
  --in_dir "$IN" --out_dir "$OUT" --ckpt "$FS_CKPT"
```

IN 必须是 `left/f.png` 和 `right/f.png`：left 放 **topic right_rect.png**，right 放 **topic left_rect.png**。输出 `disp/f.npy` 应复制为对应 snapshot 的 `fs_disp.npy`。双帧分别运行并保留各自元数据，不串用视差。

先验证这一帧能正常输出，再继续现场采集。当前尚未在 3090 上完成该验证。

## C. 真机抓针（现场逐阶段）

先完成 [SAFETY_CHECKLIST.md](SAFETY_CHECKLIST.md)，并按 [原始流程](../docs/daily/2026-10-16/R6_grasp_pipeline.md) 启动 dVRK 和相机。

1. **标定。** 用 required_dependencies 下的 r6_calib.py 和 r6_beta.py；按原流程完成 manifest → probe → grid → fit → check2。当前标定检查通过后才继续。固定读取路径和所需文件见 requirements.md；示例目录不自动部署。
2. **建立新尝试。** 机器人端运行 `bash "$P/scripts/required_dependencies/r6_trial_start.sh" "$T" --nofs`，保存 start.json 和 snap0；然后用 `r6_snap.py "$T/snap1" -k 6` 拍第二帧。T 必须是新建的现场试验目录。
3. **计算感知。** 将两组图像送到 3090，按 B 节计算 FS 并送回各自 snapshot。在 snap0/right_rect.png 上从针尾到尖点折线，保存 clicks.json（格式看样例）。运行 r6_perceive.py，参数同 A 节但使用本次 T。确认所有检查通过。
4. **接近与模板。** 逐次运行 r6_grasp 的 approach（条件 C）→ wrist → open。每步确认后再继续。在 open25 的左右图上点夹爪特征和夹爪尖中点，保存 template_clicks.json（格式看样例）；然后运行 template。
5. **抓取。** 逐次运行 hover → descend → touch → close_lift；闭合前核对针移动。任一步失败就停止。原流程 touch 使用 `--mm 6`，是否适用于当前装置须由操作者确认。
6. **结束。** 按现场状态执行 putdown、release，记录成功／失败／中止及原因，保留日志和图像。

单阶段命令格式如下。phase 由操作者逐次填写，**不要自动串跑**：

```text
python3 "$P/scripts/r6_grasp.py" --trial "$T" --phase <phase>
approach 另外加 --cond C；touch 的参数见第 5 步。
```

真机端同样先设置 A 节的 PYTHONPATH。原脚本不读取 `--config`，也不会自动使用 calibration_example；当前标定必须出现在 requirements.md 列出的固定位置。

## D. 交回哪些结果

每次提交：输入双目图和视差、clicks.json、perception.json、template_clicks.json、trial_state.json、trial_log.jsonl、preclose_pose.json、闭合前／闭合后／抬起图像，以及结果标签和失败原因。环境首次验证另附版本与权重哈希。
