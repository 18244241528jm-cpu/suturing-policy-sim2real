#!/bin/bash
# EXP2 GPU stage. usage: run_exp2_gpu.sh <smoke|full> <monitor_round>   (user present, monitor.ps1 -Round <monitor_round> running)
# smoke: 3 frames, M0 + D-raw, into M0_smoke/.  full: M0 (raw+fix) then M2 (raw+fix), one docker process at a time, resumable.
set -u
MODE=$1; ROUND=$2
P="/mnt/c/Users/30518/OneDrive - Johns Hopkins/Desktop/cis2/project34"; W=/home/jiaming/r6off_exp2; MON=/mnt/c/crashfix/stress_$ROUND.log
LL=/mnt/c/crashfix/load_log.txt
OFFS=$(date +%z | awk '{s=substr($1,1,1); h=substr($1,2,2)+0; print (s=="-" ? -h : h)}')
say() { echo "$(date '+%Y-%m-%d %H:%M:%S') EXP2 $*" >> $LL; sync; }
age=$(( $(date +%s) - $(stat -c %Y "$MON" 2>/dev/null || echo 0) ))
[ $age -le 30 ] || { echo "monitor log $MON stale (${age}s): start monitor.ps1 first"; exit 2; }
[ -z "$(docker ps -q)" ] || { echo "a container is already running"; exit 3; }
run() {  # run <mesh_path> <name> <outdir> <depths> [extra]
  docker run --rm --ipc host --device=/dev/dxg -v /usr/lib/wsl:/usr/lib/wsl -e LD_LIBRARY_PATH=/usr/lib/wsl/lib -e TZ=America/New_York \
    -v /home/jiaming:/home/jiaming -v "$P:/project" -v /home/jiaming/FoundationPose:/workspace -v /mnt/c/crashfix:/crashfix -w /project/depth_audit_stage_a \
    foundationpose:blackwell /opt/venv/bin/python $W/dump_r6_candidates.py --bank $W/bank.jsonl --mesh "$1" --mesh-name "$2" --out "$3" --depths $4 --stage EXP2_$MODE --crop 640 480 --utc-offset-hours $OFFS ${@:5}
}
if [ "$MODE" = smoke ]; then
  say "smoke start (3 frames, M0, D-raw)"
  run $W/meshes/M0_Needle.OBJ M0 $W/M0_smoke "raw" --only Q1_C_snap0 T4_C_snap0 debug7_P0e_S_snap0 > $W/smoke.out 2>&1; rc=$?
  say "smoke end rc=$rc"; tail -5 $W/smoke.out
else
  say "full start"
  run $W/meshes/M0_Needle.OBJ M0 $W/M0 "raw fix" > $W/full_M0.out 2>&1; rc=$?; say "M0 end rc=$rc"
  [ $rc -eq 0 ] || exit $rc
  run $W/meshes/M2_arc.obj M2 $W/M2 "raw fix" > $W/full_M2.out 2>&1; rc=$?; say "M2 end rc=$rc"
fi
