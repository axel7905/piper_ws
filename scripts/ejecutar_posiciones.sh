#!/usr/bin/env bash

set -euo pipefail

ACTION_NAME="/arm_controller/follow_joint_trajectory"
ACTION_TYPE="control_msgs/action/FollowJointTrajectory"
JOINT_NAMES="[joint1, joint2, joint3, joint4, joint5, joint6]"
DURATION_SEC=4

POSE_A="[0.00, 0.15, -0.15, 0.00, 0.10, 0.00]"
POSE_B="[0.25, 0.30, -0.40, -0.15, 0.20, 0.25]"

GRIPPER7_ACTION_NAME="/gripper_controller/follow_joint_trajectory"
GRIPPER8_ACTION_NAME="/gripper8_controller/follow_joint_trajectory"
GRIPPER_OPEN_7="[0.02]"
GRIPPER_OPEN_8="[-0.02]"
GRIPPER_CLOSED_7="[0.0]"
GRIPPER_CLOSED_8="[0.0]"

send_goal() {
  local action_name="$1"
  local joint_names="$2"
  local positions="$3"

  ros2 action send_goal \
    "${action_name}" \
    "${ACTION_TYPE}" \
    "{trajectory: {joint_names: ${joint_names}, points: [{positions: ${positions}, time_from_start: {sec: ${DURATION_SEC}, nanosec: 0}}]}}"
}

send_pose() {
  local pose_name="$1"
  local positions="$2"

  echo "Ejecutando posición: ${pose_name}"
  send_goal "${ACTION_NAME}" "${JOINT_NAMES}" "${positions}"
}

send_gripper() {
  local state_name="$1"
  local joint7_position="$2"
  local joint8_position="$3"

  echo "Ejecutando gripper: ${state_name}"
  send_goal "${GRIPPER7_ACTION_NAME}" "[joint7]" "${joint7_position}"
  send_goal "${GRIPPER8_ACTION_NAME}" "[joint8]" "${joint8_position}"
}

send_pose "A" "${POSE_A}"
send_gripper "abrir" "${GRIPPER_OPEN_7}" "${GRIPPER_OPEN_8}"

send_pose "B" "${POSE_B}"
send_gripper "cerrar" "${GRIPPER_CLOSED_7}" "${GRIPPER_CLOSED_8}"

send_pose "A" "${POSE_A}"
send_gripper "abrir" "${GRIPPER_OPEN_7}" "${GRIPPER_OPEN_8}"
