std_msgs/Header header

# Simulation time [s] for this command (mirrors header.stamp)
float64 time

# Bitmask declaring which fields are valid in this command payload.
# Bit mapping:
# 0x01 -> torque
# 0x02 -> wheel_speed
# 0x04 -> vehicle_speed
# 0x08 -> steering
# 0x10 -> brake
# 0x20 -> throttle
uint8 valid_fields

# Command values
float64 torque
float64 wheel_speed
float64 vehicle_speed
float64 steering
float64 brake
float64 throttle
