# PFEIFER - MTSC - Modified by FrogAi for FrogPilot
import json
import math
from openpilot.common.params import Params

params_memory = Params("/dev/shm/params")

R = 6373000.0  # approximate radius of earth in meters
TO_RADIANS = math.pi / 180
TARGET_JERK = -0.6  # m/s^3 should match up with the long planner
TARGET_ACCEL = -1.2  # m/s^2 should match up with the long planner
TARGET_OFFSET = 1.0  # seconds - This controls how soon before the curve you reach the target velocity. It also helps
                     # reach the target velocity when innacuracies in the distance modeling logic would cause overshoot.
                     # The value is multiplied against the target velocity to determine the additional distance. This is
                     # done to keep the distance calculations consistent but results in the offset actually being less
                     # time than specified depending on how much of a speed diffrential there is between v_ego and the
                     # target velocity.

def calculate_accel(t, target_jerk, a_ego):
  return a_ego + target_jerk * t

def calculate_velocity(t, target_jerk, a_ego, v_ego):
  return v_ego + a_ego * t + target_jerk / 2 * (t ** 2)

def calculate_distance(t, target_jerk, a_ego, v_ego):
  return t * v_ego + a_ego / 2 * (t ** 2) + target_jerk / 6 * (t ** 3)

def distance_to_point(ax, ay, bx, by):
  a = (math.sin((bx - ax) / 2) ** 2 + math.cos(ax) * math.cos(bx) * math.sin((by - ay) / 2) ** 2)
  return R * 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

class MapTurnSpeedController:
  def __init__(self):
    self.target_lat = 0.0
    self.target_lon = 0.0
    self.target_v = 0.0

  def target_speed(self, v_ego, a_ego) -> float:
    try:
      position = json.loads(params_memory.get("LastGPSPosition"))
      lat, lon = position["latitude"], position["longitude"]
    except:
      return 0.0

    try:
      target_velocities = json.loads(params_memory.get("MapTargetVelocities"))
    except:
      return 0.0

    min_idx, min_dist = min(enumerate(target_velocities), key=lambda i_v:
      distance_to_point(lat * TO_RADIANS, lon * TO_RADIANS, i_v[1]["latitude"] * TO_RADIANS, i_v[1]["longitude"] * TO_RADIANS))

    forward_points = target_velocities[min_idx:]
    forward_distances = [
      distance_to_point(lat * TO_RADIANS, lon * TO_RADIANS, tv["latitude"] * TO_RADIANS, tv["longitude"] * TO_RADIANS)
      for tv in forward_points
    ]

    valid_velocities = []
    for i, target_velocity in enumerate(forward_points):
      tv = target_velocity["velocity"]
      if tv > v_ego:
        continue
      d = forward_distances[i]
      a_diff = (a_ego - TARGET_ACCEL)
      accel_t = abs(a_diff / TARGET_JERK)
      min_accel_v = calculate_velocity(accel_t, TARGET_JERK, a_ego, v_ego)
      max_d = calculate_distance(accel_t, TARGET_JERK, a_ego, v_ego) if tv > min_accel_v else calculate_distance(
        abs((min_accel_v - tv) / TARGET_ACCEL), 0, TARGET_ACCEL, min_accel_v)

      if d < max_d + tv * TARGET_OFFSET:
        valid_velocities.append((float(tv), target_velocity["latitude"], target_velocity["longitude"]))

    min_v = min(valid_velocities, default=(100.0, 0.0, 0.0))[0]
    if self.target_v < min_v and not (self.target_lat == 0 and self.target_lon == 0):
      for target_velocity in forward_points:
        if (target_velocity["latitude"], target_velocity["longitude"], target_velocity["velocity"]) == (self.target_lat, self.target_lon, self.target_v):
          return float(self.target_v)
      self.target_v = self.target_lat = self.target_lon = 0.0

    if valid_velocities:
      self.target_v, self.target_lat, self.target_lon = min(valid_velocities, key=lambda x: x[0])

    return min_v
