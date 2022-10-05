roughness_smoother.py is a node that takes the roughness_score from lidar, calculate the mode of 10 readings, then publishes /cmd_vel_smooth that is /cmd_vel*roughness_factor


roughness_smoother_og.py just takes the roughness score and applies a limiter on /cmd_vel