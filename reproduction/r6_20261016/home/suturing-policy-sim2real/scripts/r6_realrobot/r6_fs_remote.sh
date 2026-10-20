#!/bin/bash
# R6: run FoundationStereo for ONE snapshot dir on the user's laptop (on-demand single frame), fetch disparity back.
# usage: r6_fs_remote.sh <snapshot_dir> <tag>   (snapshot dir has left_rect.png/right_rect.png with TOPIC names; FS left = physical left = topic right)
set -e
D=$1; TAG=$2; X=/mnt/c/Users/30518/r6_xfer; K="-o BatchMode=yes -o ConnectTimeout=8 -i $HOME/.ssh/id_ed25519_jiaming_sim2real"
S="ssh $K -p 2222 jiaming@localhost"
rm -f "$D/fs_disp.npy" "$D/fs_meta.json"            # never reuse an old result
$S "rm -rf $X/$TAG && mkdir -p $X/$TAG/in/left $X/$TAG/in/right" 2>/dev/null
scp -q $K -P 2222 "$D/right_rect.png" jiaming@localhost:$X/$TAG/in/left/f.png
scp -q $K -P 2222 "$D/left_rect.png" jiaming@localhost:$X/$TAG/in/right/f.png
timeout 240 $S "cd /mnt/c && /mnt/c/Windows/System32/wsl.exe -d Ubuntu-22.04 --cd /home/jiaming -- bash -lc 'cd ~/FoundationStereo && PYTHONPATH=~/FoundationStereo ~/.venvs/foundationstereo-5080/bin/python $X/scripts/r6_fs_infer.py --in_dir $X/$TAG/in --out_dir $X/$TAG/out --ckpt ~/FoundationStereo/pretrained_models/23-51-11/model_best_bp2.pth 2>&1 | grep stem'" 2>/dev/null
scp -q $K -P 2222 jiaming@localhost:$X/$TAG/out/disp/f.npy "$D/fs_disp.npy"
scp -q $K -P 2222 jiaming@localhost:$X/$TAG/out/meta.json "$D/fs_meta.json"
echo "{\"host\": \"Deck-3-Jiaming (RTX 5080 Laptop, WSL Ubuntu-22.04)\", \"tag\": \"$TAG\", \"fetched\": \"$(date -Iseconds)\"}" > "$D/fs_host.json"
