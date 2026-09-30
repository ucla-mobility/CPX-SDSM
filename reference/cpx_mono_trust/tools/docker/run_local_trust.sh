#!/usr/bin/env bash
set -euo pipefail

tracked_objects_topic="${CPX_TRACKED_OBJECTS_TOPIC:-/vehicle/perception/tracked_objects}"
cloud_topic="${CPX_INPUT_TOPIC:-/vehicle/lidar/points}"
score_topic="${CPX_TRUST_SCORE_TOPIC:-/local_trust_estimation/score}"
sdsm_topic="${CPX_SDSM_TOPIC:-/perception/global_trustworthiness/sdsm}"
sensor_frame="${CPX_SENSOR_FRAME:-vehicle_lidar}"
global_frame="${CPX_GLOBAL_FRAME:-map}"

for topic in \
  "${tracked_objects_topic}" \
  "${cloud_topic}" \
  "${score_topic}" \
  "${sdsm_topic}"; do
  if [[ "${topic}" != /* || "${topic}" =~ [[:space:]] ]]; then
    printf 'ROS topics must be absolute and contain no whitespace: %s\n' \
      "${topic}" >&2
    exit 2
  fi
done

for frame in "${sensor_frame}" "${global_frame}"; do
  if [[ -z "${frame}" || "${frame}" =~ [[:space:]] ]]; then
    printf 'ROS frames must be nonempty and contain no whitespace: %s\n' \
      "${frame}" >&2
    exit 2
  fi
done

printf 'Starting local trust: %s + %s -> %s -> %s\n' \
  "${tracked_objects_topic}" "${cloud_topic}" "${score_topic}" "${sdsm_topic}"

exec ros2 launch \
  local_trust_estimation local_trust_estimation.launch.py \
  tracked_objects_topic:="${tracked_objects_topic}" \
  cloud_topic:="${cloud_topic}" \
  score_topic:="${score_topic}" \
  sdsm_topic:="${sdsm_topic}" \
  sensor_frame:="${sensor_frame}" \
  global_frame:="${global_frame}"
