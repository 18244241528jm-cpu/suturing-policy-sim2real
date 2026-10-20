# 精简版说明（相对完整版）
- 逐帧图：悬停每次修正（hover_init / hover_c*）保留前 3 帧，闭合前（pre_close*）保留第 1 帧，以上为无损 PNG；抬起后（lifted_2s）保留第 1 帧，JPEG q92；其余逐帧图删除，各阶段的均值图仍在。
- 无损 PNG（最高压缩）：snap0/snap1（FS 和 FP 的输入）、模板 tpl_*、闭合前均值图、calib/pre3。其余均值图存为 JPEG q92。
- FS 视差：fs_disp.npz，键 disp，float16，单位为全分辨率像素，Z = f*B/d。
- 删除了 xfer/（只是 trials/ 里 snap 和 perception 的副本）。
- 调试尝试（debug1–6）不保留逐帧图。
- 视频：H.264 960 px，左右两路，放在 video/。全分辨率原始视频和完整数据（5.1 GB）留在 lcsr-dvrk-15 的 ~/suturing-policy-sim2real/records/logs/R6_20261016/。
统计：{"png_lossless": 320, "jpg": 355, "npz": 31, "dropped": 1114, "copied": 243}
