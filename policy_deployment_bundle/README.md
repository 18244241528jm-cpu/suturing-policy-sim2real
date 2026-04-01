# Policy Deployment Bundle

这个文件夹用于集中放置第一次 `policy -> ROS/CRTK -> PSM` 真机最小闭环相关文件，方便下次上机直接从根目录打开。

## 文件说明

- `dVRK_Policy真机测试Checklist_中英对照.md`
  - 第一次真机最小闭环 checklist
- `README_dvrk_adapter.md`
  - `dvrk_policy_adapter.py` 的使用说明和风险提醒
- `README_运行说明.md`
  - 仿真侧 `Image IL` / `Task_evaluation_R3M.py` 运行说明
- `dvrk_policy_adapter.py`
  - 项目内 `policy -> ROS/CRTK` 适配实现

## 原始位置

- `project34/dvrk讲义/dVRK_Policy真机测试Checklist_中英对照.md`
- `project34/SurgicAI-SurgicAI-DR-RAL/Image_IL/README_dvrk_adapter.md`
- `project34/SurgicAI-SurgicAI-DR-RAL/Image_IL/README_运行说明.md`
- `project34/SurgicAI-SurgicAI-DR-RAL/Image_IL/dvrk_policy_adapter.py`

## 使用建议

先读 `dVRK_Policy真机测试Checklist_中英对照.md`，再对照 `README_dvrk_adapter.md` 和 `dvrk_policy_adapter.py`。涉及接口语义、坐标系、视频 ROS 管线时，以官方 dVRK 文档为准。
