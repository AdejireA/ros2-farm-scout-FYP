#!/bin/bash
#
# Automated trial runner for scout_mission.py evaluation.
#
# Usage:
#   ./run_trials.sh baseline 1 20
#   ./run_trials.sh all 1 20
#

SCENARIO="${1:-all}"
START="${2:-1}"
END="${3:-20}"
SCOUT_SCRIPT="$HOME/ros2_ws/src/ros2-farm-scout-FYP/robot_bringup/scripts/scout_mission.py"
TRIAL_DIR="$HOME/ros2_ws/src/ros2-farm-scout-FYP/trial_results"
TRIAL_TIMEOUT=300
LAUNCH_TIMEOUT=120
MY_PID=$$

mkdir -p "$TRIAL_DIR"

kill_all() {
    # Kill ROS/Gazebo processes but NOT this script
    for proc in "gz sim" "gzserver" "gzclient" "rviz2" "ruby" "nav2" "bt_navigator" "controller_server" "planner_server" "map_server" "lifecycle_manager" "robot_state_publisher" "slam_toolbox" "smoother_server" "waypoint_follower" "behavior_server" "velocity_smoother" "arbitration_node" "keyboard_teleop" "scout_mission" "basic_navigator" "static_transform_publisher"; do
        pkill -f "$proc" 2>/dev/null || true
    done
    # Kill ros2 launch specifically (not anything else with ros2 in the path)
    pkill -f "ros2.launch" 2>/dev/null || true
    pkill -f "ros2 launch" 2>/dev/null || true

    sleep 10

    # Force kill
    for proc in "gz sim" "gzserver" "gzclient" "rviz2" "ruby"; do
        pkill -9 -f "$proc" 2>/dev/null || true
    done
    pkill -9 -f "ros2.launch" 2>/dev/null || true
    pkill -9 -f "ros2 launch" 2>/dev/null || true

    sleep 5

    # Verify Gazebo is dead
    if pgrep -f "gz sim" > /dev/null 2>&1; then
        echo "[runner] WARNING: Gazebo still alive, extra wait..."
        sleep 10
        pkill -9 -f "gz sim" 2>/dev/null || true
        sleep 5
    fi
    echo "[runner] System killed."
}

launch_and_wait() {
    source /opt/ros/jazzy/setup.bash
    source "$HOME/ros2_ws/install/setup.bash" 2>/dev/null || true

    ros2 launch robot_bringup full_system.launch.py &
    LAUNCH_PID=$!
    disown $LAUNCH_PID

    echo "[runner] Launch PID: $LAUNCH_PID, waiting for Nav2..."
    local elapsed=0
    while [ $elapsed -lt $LAUNCH_TIMEOUT ]; do
        if ros2 node list 2>/dev/null | grep -q "bt_navigator"; then
            echo "[runner] Nav2 ready (${elapsed}s)"
            sleep 5
            return 0
        fi
        sleep 3
        elapsed=$((elapsed + 3))
    done
    echo "[runner] ERROR: Nav2 did not start within ${LAUNCH_TIMEOUT}s"
    return 1
}

run_one_trial() {
    local scenario="$1"
    local num="$2"
    local trial_name=$(printf "%s_%02d" "$scenario" "$num")
    local attempt=0
    local max_attempts=3

    while [ $attempt -lt $max_attempts ]; do
        attempt=$((attempt + 1))
        echo ""
        echo "============================================"
        echo "[runner] $trial_name (attempt $attempt/$max_attempts)"
        echo "============================================"

        kill_all
        if ! launch_and_wait; then
            echo "[runner] System failed to start. Retrying..."
            continue
        fi

        # Set mode — background it so it cannot hang the script
        timeout 15 ros2 topic pub /mode std_msgs/msg/String "data: 'auto'" --once 2>/dev/null &
        MODE_PID=$!
        sleep 3
        kill $MODE_PID 2>/dev/null || true
        wait $MODE_PID 2>/dev/null || true

        # Run trial
        timeout $TRIAL_TIMEOUT python3 "$SCOUT_SCRIPT" \
            --scenario "$scenario" \
            --auto \
            --log "$TRIAL_DIR/$trial_name" 2>&1

        local exit_code=$?

        if [ $exit_code -eq 0 ]; then
            echo "[runner] $trial_name DONE"
            return 0
        elif [ $exit_code -eq 124 ]; then
            echo "[runner] $trial_name TIMED OUT"
        else
            echo "[runner] $trial_name FAILED (exit $exit_code)"
        fi
    done

    echo "[runner] $trial_name FAILED after $max_attempts attempts. Skipping."
    return 1
}

run_scenario() {
    local scenario="$1"
    local start="$2"
    local end="$3"
    local passed=0
    local failed=0
    local total=$((end - start + 1))

    echo ""
    echo "================================================================"
    echo "[runner] SCENARIO: $scenario (trials $start to $end)"
    echo "================================================================"

    for i in $(seq "$start" "$end"); do
        if run_one_trial "$scenario" "$i"; then
            passed=$((passed + 1))
        else
            failed=$((failed + 1))
        fi
        echo "[runner] Progress: $passed passed, $failed failed, $((passed + failed))/$total done"
    done

    echo ""
    echo "[runner] $scenario COMPLETE: $passed/$total passed"
}

# Main
echo "================================================================"
echo "[runner] Automated Trial Runner (PID: $$)"
echo "[runner] Scenario: $SCENARIO | Trials: $START to $END"
echo "[runner] Kill + relaunch between every trial"
echo "[runner] $(date)"
echo "================================================================"

START_TIME=$(date +%s)

if [ "$SCENARIO" = "all" ]; then
    run_scenario "baseline" "$START" "$END"
    run_scenario "deadman" "$START" "$END"
    run_scenario "obstacle" "$START" "$END"
else
    run_scenario "$SCENARIO" "$START" "$END"
fi

END_TIME=$(date +%s)
ELAPSED=$(( (END_TIME - START_TIME) / 60 ))

kill_all

echo ""
echo "================================================================"
echo "[runner] ALL DONE"
echo "[runner] Total time: ${ELAPSED} minutes"
echo "[runner] $(date)"
echo "[runner] CSVs in: $TRIAL_DIR"
echo "================================================================"
echo ""
echo "CSV files:"
ls -1 "$TRIAL_DIR"/*.csv 2>/dev/null
echo ""
ls "$TRIAL_DIR"/*.csv 2>/dev/null | wc -l
echo " files total"
