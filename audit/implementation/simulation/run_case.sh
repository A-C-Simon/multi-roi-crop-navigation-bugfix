#!/usr/bin/env bash
# usage: run_case.sh NAME WALL_TIMEOUT [launch args...]
set -u
NAME="$1"; WALL="$2"; shift 2
REPO=/home/ac/multi-roi-crop-navigation-bugfix
OUT=/tmp/mr_sim/runs/$NAME
rm -rf "$OUT"; mkdir -p "$OUT"
if pgrep -x gzserver >/dev/null; then echo "gzserver already running; refusing" | tee "$OUT/blocked.txt"; exit 3; fi
export ROS_DOMAIN_ID=87
export GAZEBO_MASTER_URI=http://localhost:11387
export ROS_LOG_DIR=/tmp/mr_sim/roslog
export MULTIROI_DIR=$REPO
export GAZEBO_MODEL_PATH=$REPO/tests/agribot/agribot_gazebo/models:/usr/share/gazebo-11/models
set +u; source /opt/ros/humble/setup.bash; source $REPO/tests/sim_ros2/install/setup.bash; set -u
cd "$REPO/tests/sim_ros2"
setsid ros2 launch mrsim farm.launch.py log_dir:="$OUT" "$@" > "$OUT/launch.log" 2>&1 &
LPID=$!
echo $LPID > "$OUT/launch.pid"
START=$(date +%s)
while kill -0 $LPID 2>/dev/null; do
  if (( $(date +%s) - START > WALL )); then
    echo "wall timeout, stopping launch group $LPID" >> "$OUT/launch.log"
    kill -INT -- -$LPID 2>/dev/null; sleep 8; kill -TERM -- -$LPID 2>/dev/null; sleep 3; kill -KILL -- -$LPID 2>/dev/null
    break
  fi
  sleep 2
done
wait $LPID 2>/dev/null; echo "launch_exit=$? wall_s=$(( $(date +%s) - START ))" >> "$OUT/launch.log"
# make sure no child of ours survived
sleep 1; pkill -INT -g $LPID 2>/dev/null; true
tail -2 "$OUT/launch.log"
