#!/usr/bin/env bash
# Failure-stop check: bad-encoding frames (perception failure) and a camera
# stall (physics paused) during a straight run with camera_timeout:=1.0.
set -u
OUT=/tmp/mr_sim/runs/failure_stop
/tmp/mr_sim/run_case.sh failure_stop 240 field:=straight max_seconds:=45 camera_timeout:=1.0 &
RUNPID=$!
export ROS_DOMAIN_ID=87 GAZEBO_MASTER_URI=http://localhost:11387 ROS_LOG_DIR=/tmp/mr_sim/roslog
set +u; source /opt/ros/humble/setup.bash; set -u
EV=/tmp/mr_sim/runs/failure_stop_events.log; : > $EV
# wait until the nav node is driving (nav_run.csv has > 300 rows)
for i in $(seq 1 120); do
  n=$(wc -l < $OUT/nav_run.csv 2>/dev/null || echo 0); [ "$n" -gt 300 ] && break; sleep 1
done
echo "$(date +%s.%N) bad_frames_start rows=$n" >> $EV
timeout 4 ros2 topic pub -r 20 /camera/image_raw sensor_msgs/msg/Image \
  "{encoding: mono16, height: 1, width: 1, step: 2, data: [0, 0]}" > /dev/null 2>&1
echo "$(date +%s.%N) bad_frames_end" >> $EV
sleep 3
echo "$(date +%s.%N) pause_physics" >> $EV
timeout 10 ros2 service call /pause_physics std_srvs/srv/Empty >> $EV 2>&1
sleep 2.5
echo "$(date +%s.%N) cmd_vel_during_pause:" >> $EV
timeout 5 ros2 topic echo --once /cmd_vel geometry_msgs/msg/Twist >> $EV 2>&1
sleep 1
echo "$(date +%s.%N) unpause_physics" >> $EV
timeout 10 ros2 service call /unpause_physics std_srvs/srv/Empty >> $EV 2>&1
wait $RUNPID
ros2 daemon stop >/dev/null 2>&1 || true
echo STALLDONE >> $EV
