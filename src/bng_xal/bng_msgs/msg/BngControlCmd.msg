# Scalar low-level command for xlab LLC (no trajectory / MPC).
# Forwarded on control_listen as {"type":"cmd","data":{...}} by sensor_dispatcher
# (or VehicleSession.send_control_cmd from Python).
#
# OR valid_fields bits to declare which command values are active. The bridge
# includes only those keys in the JSON payload (controller_llc loadScalar).
# LLC replace semantics: keys not sent are cleared server-side.
#
# Steering axes (pick one per command):
#   steering        — roadwheel angle [rad] → LLC uses steering_to_input scale (+ optional PI)
#   steering_input  — BeamNG electrics [-1, 1] direct (calibration / unknown steering_to_input)
#
# valid_fields == 0 → bridge skips send (no-op).

std_msgs/Header header          # ROS time when command was issued

# --- field bit constants (OR together) ---
uint8 FIELD_THROTTLE=1          # 0x01 → data.throttle
uint8 FIELD_BRAKE=2             # 0x02 → data.brake
uint8 FIELD_STEERING=4          # 0x04 → data.steering
uint8 FIELD_TORQUE=8            # 0x08 → data.torque
uint8 FIELD_WHEEL_SPEED=16      # 0x10 → data.wheel_speed
uint8 FIELD_PARKING_BRAKE=32    # 0x20 → data.pbrake (reserved — LLC TBD)
uint8 FIELD_GEAR=64             # 0x40 → data.gear_index (reserved — LLC TBD)
uint8 FIELD_STEERING_INPUT=128  # 0x80 → data.steering_input (direct [-1,1], no steering_to_input)

uint8 valid_fields

# --- command values (active subset determined by valid_fields) ---
float64 throttle                # [0, 1]
float64 brake                   # [0, 1]
float64 steering                # roadwheel angle [rad]
float64 steering_input          # BeamNG steering_input [-1, 1] — bypasses steering_to_input
float64 torque                  # feedforward rear torque [N·m]
float64 wheel_speed             # rear wheel linear speed target [m/s]
float64 pbrake                  # parking brake [0, 1] — reserved
int32 gear_index                # target gear — reserved
