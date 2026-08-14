# D3a 两套 hand-eye 实验室预备包报告（2026-08-06）

## 结论

- `[测量]` 两套采集器已能在无 ROS、无机器人 mock 模式生成各 **24 solve + 6 held-out** 样本，并完成 validator→solver→held-out residual→overlay 全链。（证据：`records/logs/D3a_handeye_preflight_20260806/synthetic_self_test/self_test_summary.json`）
- `[测量]` 无噪声已知真值测试中，ECM `T_control_point_from_camera` 与 PSM `T_camera_from_robot_base`/`T_control_point_from_marker` 的误差均低于 **2e−13 mm / 3e−14°**；说明矩阵方向与逆矩阵不是仅凭变量名猜测。（证据：同上 clean cases）
- `[测量]` 在 robot 0.2 mm/0.1°、PnP 0.3 mm/0.2°、corner 0.25 px 的合成噪声下，ECM held-out p95 为 **1.127 mm / 0.452° / 3.315 px**，PSM 为 **0.897 mm / 0.477° / 2.798 px**。（证据：同上 noisy cases）
- `[测量]` validator 自动检查分辨率/camera_info、marker size、时间差、米制单位、有限 SE(3)、重复姿态、平移/旋转覆盖、solve-held-out隔离和 frame 链方向。（证据：`scripts/real_robot_handeye/validate_handeye_dataset.py`；4 个 `validation_report.json`）
- `[测量]` 采集脚本只创建 ROS subscriber；代码中没有 publisher、motion action、power/home、`servo_cp` 或 jaw 命令。（证据：`scripts/real_robot_handeye/capture_handeye_sample.py`）
- `[推断]` 工具的软件部分已达到“带到实验室后填真实 topic/marker 参数即可采”的程度，但尚未达到“无需现场确认直接按回车”：真实 topic/frame、分辨率、marker规格、刚性夹具、camera-registration URL/branch和真机通过门仍缺。（依据：`scripts/real_robot_handeye/config/example_session.yaml`）

## 工具与数据合同

`[测量]` 目录 `scripts/real_robot_handeye/` 提供：`discover_topics.sh`、只读采集器、validator、两套 solver、overlay renderer、synthetic self-test、example YAML、README 和共享 frame-explicit 数学库。（证据：该目录）

`[测量]` ECM eye-in-hand 求解 `T_control_point_from_camera`，并同时输出 `T_camera_from_control_point`；PSM eye-to-hand联合求解 `T_camera_from_robot_base` 与 `T_control_point_from_marker`，并输出两者逆矩阵。（证据：`solve_ecm_camera_handeye.py`；`solve_psm_camera_extrinsic.py`）

`[测量]` 所有矩阵统一遵循 `T_A_from_B maps coordinates in B into A`；PSM显式方程是 `T_camera_from_marker_i = T_camera_from_robot_base @ T_robot_base_from_control_point_i @ T_control_point_from_marker`。（证据：`scripts/real_robot_handeye/README.md`）

## synthetic 自测数字

| 标定 | profile | 真值变换误差 | held-out p95 t/R | held-out p95 reproj |
|---|---|---:|---:|---:|
| ECM eye-in-hand | clean | 1.47e−13 mm / 5.79e−15° | 2.23e−13 mm / 1.70e−14° | 0 px |
| ECM eye-in-hand | noisy | 0.839 mm / 0.326° | 1.127 mm / 0.452° | 3.315 px |
| PSM eye-to-hand | clean | ≤1.96e−14 mm / ≤2.55e−14° | 4.44e−14 mm / 2.07e−14° | 0 px |
| PSM eye-to-hand | noisy | X:0.254 mm/0.189°；Y:0.444 mm/0.221° | 0.897 mm / 0.477° | 2.798 px |

`[假设]` noisy profile 是软件稳定性回归条件，不代表真实 dVRK 噪声模型或实机标定精度。（证据：`synthetic_self_test.py` 参数）

## 现场仍缺的具体信息

1. `[测量]` `dVRK camera registration` URL/branch 本机未找到；会议只提到 main/dev/ECM optical registration，不能猜 URL。（证据：本机 `rg` 搜索；会议记录附件）
2. `[测量]` 左/右 image、camera_info、ECM/PSM measured_cp 的真实 topic、消息类型和 frame_id 未确认。（证据：example YAML 的 `TO_CONFIRM`）
3. `[测量]` 真机分辨率、K、distortion、同步延迟分布未测。（证据：同上）
4. `[测量]` ArUco dictionary/ID/实际边长或 checkerboard 行列/square size 未拍板。（证据：example YAML 仅为示例）
5. `[测量]` PSM marker 刚性夹具、安装照片、marker↔control-point物理稳定性未验证。（证据：meeting record；README）
6. `[测量]` robot/camera配置路径、dVRK workspace source 路径、session双备份位置未确认。（证据：example YAML/README）
7. `[测量]` 真实 held-out translation/rotation/reprojection通过门尚未由Adnan/Ed批准。（证据：当前只有 synthetic self-test）

## 到实验室后的精确顺序

1. `[推断]` 负责人完成上电/home/器械检查；本工具不做这些动作。
2. `[测量]` 运行 `discover_topics.sh`，保存 node/topic/info；只读 echo/hz 一小段 RGB、camera_info、ECM/PSM measured_cp。（证据：README §2）
3. `[推断]` 把确认的 topic/type/frame、marker实测规格、配置路径、安装照片写入两份 YAML。
4. `[推断]` 先在静止状态验证 ArUco/PnP、分辨率和时间差；失败就停止采集。
5. `[假设]` ECM：标定板固定，操作者手动/批准的遥操作移动 ECM，采 24 solve + 6全新 held-out。
6. `[假设]` PSM：ECM固定、marker刚性夹在PSM，采 24+6；必须有多轴旋转。
7. `[测量]` 当场跑 validator；有 error 不求解、不把 rejected 样本改成 valid。（证据：validator fail-closed）
8. `[测量]` 离线求解并查看 6 个 held-out overlay；报告 t/R/reprojection p50/p95/max，不能只报 solve residual。（证据：solver/renderer）
9. `[推断]` 将 session 做两份备份并验证文件数/哈希。
10. `[推断]` hand-eye 门通过后才做 PSM kinematics overlay→needle DA/FP离线验收→D2 translation→orientation→approach→人工确认→单次 close/lift。

## 绝对停止线

`[测量]` 在第 8 步 held-out hand-eye/overlay 获得负责人明确通过之前，绝对不能发任何自动运动命令；本 D3a 采集全程也不发命令。（证据：`scripts/real_robot_handeye/README.md:8. Absolute stop line`）

`[测量]` 当前 `Image_IL/dvrk_policy_adapter.py` 不是安全 Approach runner，不能用于本流程驱动真机。（证据：`records/logs/SurgicAI_T13_deployment_contract_audit_20260730.md`）

## 五个必须回答

1. **现场缺什么？** `[测量]` 见“现场仍缺”7项，尤其 URL/branch、topic/frame/分辨率、marker/夹具和实机通过门。
2. **两套采集器可否无机器人运行？** `[测量]` 可以；4个30样本session全部通过，`passed=true`。
3. **方向、单位、held-out是否自动检查？** `[测量]` 是；clean真值误差近机器精度，逆矩阵乘积 max abs ≤9.99e−16，validator显式检查米制和split。
4. **实验室顺序？** `[推断]` 见10步清单。
5. **哪一步前不能发命令？** `[测量]` 至少在两套 held-out + overlay 被负责人批准前绝对不能发；D3a全程不发。

## 产物

- `[测量]` 工具：`scripts/real_robot_handeye/`
- `[测量]` 自测：`records/logs/D3a_handeye_preflight_20260806/synthetic_self_test/`
- `[测量]` manifest：`.../synthetic_self_test/MANIFEST.sha256`
- `[测量]` 复现：`docs/repro/D3a_handeye_preflight.md`

## 2026-08-06 ArUco API 兼容性复验追加

- `[测量]` Windows 隔离 Python 缺少 `cv2`；WSL 系统 Python 3.10.12 的 OpenCV 4.5.4 可导入 `cv2.aruco`，但原实现因缺少新版 `generateImageMarker` 在第一套 mock 采集前退出。（证据：本轮原始终端输出；`capture_handeye_sample.py` 的 prepatch 版本；归档 `environments/SurgicAI/_patch_archive/d3a_opencv_compat_prepatch_20260806T221730/`）
- `[测量]` 采集器现按 feature 检测选择 legacy `drawMarker`/module-level `detectMarkers` 或新版 `generateImageMarker`/`ArucoDetector`；无 contrib ArUco 时明确报错，未增加 publisher 或运动接口。（证据：`scripts/real_robot_handeye/capture_handeye_sample.py:69-103`）
- `[测量]` 在同一 WSL Python 3.10.12 + OpenCV 4.5.4 环境重新运行后，ECM/PSM clean+noisy 四套、每套 24 solve+6 held-out 全部通过，`failures=[]`、`passed=true`；noisy held-out p95 仍为 ECM **1.127 mm/0.452°/3.315 px**、PSM **0.897 mm/0.477°/2.798 px**。（证据：`/home/jiaming/d3a_reverify_20260806_codex_019fd9fe_r1/self_test_summary.json`；同目录 `MANIFEST.sha256`）
- `[推断]` 这消除了已发现的 OpenCV 新旧 API 环境阻塞，但不改变现场结论：真实 topic/frame/K/distortion、marker/夹具、registration URL/branch 与真机 held-out 通过门仍必须现场确认。（依据：本报告“现场仍缺的具体信息”）
