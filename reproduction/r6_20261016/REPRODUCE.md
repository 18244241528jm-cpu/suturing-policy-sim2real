# 独立工作副本的离线复现

当前需要先取得主 ZIP（地址待补）。以下命令在 Linux/WSL 使用；进入从 ZIP 解压的主包根目录。它只使用新的工作目录，所有脚本均保持原样。3090 的 Python/CUDA 环境待在实验室核对，本页不自动安装或重建已有环境。

CPU 依赖：numpy、opencv-python、scipy；policy 另需 torch、stable-baselines3、gymnasium、cloudpickle；制图另需 matplotlib。此前笔记本验证环境：Python 3.14.2、numpy 2.3.5、opencv-python 4.13.0.92、scipy 1.16.3、torch 2.10.0、SB3 2.7.1、gymnasium 1.0.0、cloudpickle 3.1.2。此清单是记录，不是 3090 的安装方案或原始环境锁。

```bash
set -euo pipefail
sha256sum -c SHA256SUMS.txt
test ! -e ../R6_work_home
cp -R home ../R6_work_home
export HOME="$(cd ../R6_work_home && pwd)"
export PYTHONDONTWRITEBYTECODE=1
python3 "$HOME/r6off/exp0/exp0.py"
python3 "$HOME/r6off/exp1/exp1.py"
python3 "$HOME/r6off/exp1/exp1b.py"
python3 "$HOME/r6off/exp1/exp1c.py"
python3 "$HOME/r6off/exp3/exp3.py"
python3 "$HOME/r6off/exp4/exp4.py"
python3 "$HOME/r6off/exp2/prep_inputs.py"
cp -R "$HOME/r6off/exp2/fp_out/M0" "$HOME/r6off_exp2/M0"
cp -R "$HOME/r6off/exp2/fp_out/M2" "$HOME/r6off_exp2/M2"
python3 "$HOME/r6off/exp2/analyze_fp.py" "$HOME/r6off_exp2" --selftest
python3 "$HOME/r6off/exp2/analyze_fp.py" "$HOME/r6off_exp2"
```

这里的 FP 阶段复算归档候选指标，不是重新运行 FP 模型。prep_inputs 会重建 bank.jsonl，不要直接沿用归档 bank 中的旧绝对路径。物理左目 = topic right_rect.png；物理右目 = topic left_rect.png。FS 输入方向按物理左右排列。

HOME 只用于当前 shell；完成后新开 shell 即恢复原设置。不要对原始 trial 直接运行会覆盖 perception.json 的真机脚本。

在 3090 重新推理 FS/FP 的命令需先确认 R5 环境、模型版本和挂载路径；这些命令还没有写成已验证入口。校外连接受实验室网络限制，不在本次离线 CPU 结果中假定 GPU 测试已经通过。
