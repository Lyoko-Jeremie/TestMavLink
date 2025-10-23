// @ts-nocheck
import {MavLinkData, MavLinkPacketField} from 'node-mavlink';

/**
 * These values define the type of firmware release. These values indicate the first version or release
 * of this type. For example the first alpha release would be 64, the second would be 65.
 */
export enum FirmwareVersionType {
  /**
   * development release
   */
  'DEV'                                            = 0,

  /**
   * alpha release
   */
  'ALPHA'                                          = 64,

  /**
   * beta release
   */
  'BETA'                                           = 128,

  /**
   * release candidate
   */
  'RC'                                             = 192,

  /**
   * official stable release
   */
  'OFFICIAL'                                       = 255,
}

/**
 * Flags to report failure cases over the high latency telemtry.
 */
export enum HlFailureFlag {
  /**
   * GPS failure.
   */
  'GPS'                                            = 1,

  /**
   * Differential pressure sensor failure.
   */
  'DIFFERENTIAL_PRESSURE'                          = 2,

  /**
   * Absolute pressure sensor failure.
   */
  'ABSOLUTE_PRESSURE'                              = 4,

  /**
   * Accelerometer sensor failure.
   */
  'HL_FAILURE_FLAG_3D_ACCEL'                       = 8,

  /**
   * Gyroscope sensor failure.
   */
  'HL_FAILURE_FLAG_3D_GYRO'                        = 16,

  /**
   * Magnetometer sensor failure.
   */
  'HL_FAILURE_FLAG_3D_MAG'                         = 32,

  /**
   * Terrain subsystem failure.
   */
  'TERRAIN'                                        = 64,

  /**
   * Battery failure/critical low battery.
   */
  'BATTERY'                                        = 128,

  /**
   * RC receiver failure/no rc connection.
   */
  'RC_RECEIVER'                                    = 256,

  /**
   * Offboard link failure.
   */
  'OFFBOARD_LINK'                                  = 512,

  /**
   * Engine failure.
   */
  'ENGINE'                                         = 1024,

  /**
   * Geofence violation.
   */
  'GEOFENCE'                                       = 2048,

  /**
   * Estimator failure, for example measurement rejection or large variances.
   */
  'ESTIMATOR'                                      = 4096,

  /**
   * Mission failure.
   */
  'MISSION'                                        = 8192,
}

/**
 * Actions that may be specified in MAV_CMD_OVERRIDE_GOTO to override mission execution.
 */
export enum MavGoto {
  /**
   * Hold at the current position.
   */
  'DO_HOLD'                                        = 0,

  /**
   * Continue with the next item in mission execution.
   */
  'DO_CONTINUE'                                    = 1,

  /**
   * Hold at the current position of the system
   */
  'HOLD_AT_CURRENT_POSITION'                       = 2,

  /**
   * Hold at the position specified in the parameters of the DO_HOLD action
   */
  'HOLD_AT_SPECIFIED_POSITION'                     = 3,
}

/**
 * These defines are predefined OR-combined mode flags. There is no need to use values from this enum,
 * but it
 simplifies the use of the mode flags. Note that manual input is enabled in all modes as a
 * safety override.
 */
export enum MavMode {
  /**
   * System is not ready to fly, booting, calibrating, etc. No flag is set.
   */
  'PREFLIGHT'                                      = 0,

  /**
   * System is allowed to be active, under assisted RC control.
   */
  'STABILIZE_DISARMED'                             = 80,

  /**
   * System is allowed to be active, under assisted RC control.
   */
  'STABILIZE_ARMED'                                = 208,

  /**
   * System is allowed to be active, under manual (RC) control, no stabilization
   */
  'MANUAL_DISARMED'                                = 64,

  /**
   * System is allowed to be active, under manual (RC) control, no stabilization
   */
  'MANUAL_ARMED'                                   = 192,

  /**
   * System is allowed to be active, under autonomous control, manual setpoint
   */
  'GUIDED_DISARMED'                                = 88,

  /**
   * System is allowed to be active, under autonomous control, manual setpoint
   */
  'GUIDED_ARMED'                                   = 216,

  /**
   * System is allowed to be active, under autonomous control and navigation (the trajectory is decided
   * onboard and not pre-programmed by waypoints)
   */
  'AUTO_DISARMED'                                  = 92,

  /**
   * System is allowed to be active, under autonomous control and navigation (the trajectory is decided
   * onboard and not pre-programmed by waypoints)
   */
  'AUTO_ARMED'                                     = 220,

  /**
   * UNDEFINED mode. This solely depends on the autopilot - use with caution, intended for developers
   * only.
   */
  'TEST_DISARMED'                                  = 66,

  /**
   * UNDEFINED mode. This solely depends on the autopilot - use with caution, intended for developers
   * only.
   */
  'TEST_ARMED'                                     = 194,
}

/**
 * These encode the sensors whose status is sent as part of the SYS_STATUS message.
 */
export enum MavSysStatusSensor {
  /**
   * 0x01 3D gyro
   */
  'SENSOR_3D_GYRO'                                 = 1,

  /**
   * 0x02 3D accelerometer
   */
  'SENSOR_3D_ACCEL'                                = 2,

  /**
   * 0x04 3D magnetometer
   */
  'SENSOR_3D_MAG'                                  = 4,

  /**
   * 0x08 absolute pressure
   */
  'SENSOR_ABSOLUTE_PRESSURE'                       = 8,

  /**
   * 0x10 differential pressure
   */
  'SENSOR_DIFFERENTIAL_PRESSURE'                   = 16,

  /**
   * 0x20 GPS
   */
  'SENSOR_GPS'                                     = 32,

  /**
   * 0x40 optical flow
   */
  'SENSOR_OPTICAL_FLOW'                            = 64,

  /**
   * 0x80 computer vision position
   */
  'SENSOR_VISION_POSITION'                         = 128,

  /**
   * 0x100 laser based position
   */
  'SENSOR_LASER_POSITION'                          = 256,

  /**
   * 0x200 external ground truth (Vicon or Leica)
   */
  'SENSOR_EXTERNAL_GROUND_TRUTH'                   = 512,

  /**
   * 0x400 3D angular rate control
   */
  'SENSOR_ANGULAR_RATE_CONTROL'                    = 1024,

  /**
   * 0x800 attitude stabilization
   */
  'SENSOR_ATTITUDE_STABILIZATION'                  = 2048,

  /**
   * 0x1000 yaw position
   */
  'SENSOR_YAW_POSITION'                            = 4096,

  /**
   * 0x2000 z/altitude control
   */
  'SENSOR_Z_ALTITUDE_CONTROL'                      = 8192,

  /**
   * 0x4000 x/y position control
   */
  'SENSOR_XY_POSITION_CONTROL'                     = 16384,

  /**
   * 0x8000 motor outputs / control
   */
  'SENSOR_MOTOR_OUTPUTS'                           = 32768,

  /**
   * 0x10000 rc receiver
   */
  'SENSOR_RC_RECEIVER'                             = 65536,

  /**
   * 0x20000 2nd 3D gyro
   */
  'SENSOR_3D_GYRO2'                                = 131072,

  /**
   * 0x40000 2nd 3D accelerometer
   */
  'SENSOR_3D_ACCEL2'                               = 262144,

  /**
   * 0x80000 2nd 3D magnetometer
   */
  'SENSOR_3D_MAG2'                                 = 524288,

  /**
   * 0x100000 geofence
   */
  'GEOFENCE'                                       = 1048576,

  /**
   * 0x200000 AHRS subsystem health
   */
  'AHRS'                                           = 2097152,

  /**
   * 0x400000 Terrain subsystem health
   */
  'TERRAIN'                                        = 4194304,

  /**
   * 0x800000 Motors are reversed
   */
  'REVERSE_MOTOR'                                  = 8388608,

  /**
   * 0x1000000 Logging
   */
  'LOGGING'                                        = 16777216,

  /**
   * 0x2000000 Battery
   */
  'SENSOR_BATTERY'                                 = 33554432,

  /**
   * 0x4000000 Proximity
   */
  'SENSOR_PROXIMITY'                               = 67108864,

  /**
   * 0x8000000 Satellite Communication
   */
  'SENSOR_SATCOM'                                  = 134217728,

  /**
   * 0x10000000 pre-arm check status. Always healthy when armed
   */
  'PREARM_CHECK'                                   = 268435456,

  /**
   * 0x20000000 Avoidance/collision prevention
   */
  'OBSTACLE_AVOIDANCE'                             = 536870912,

  /**
   * 0x40000000 propulsion (actuator, esc, motor or propellor)
   */
  'SENSOR_PROPULSION'                              = 1073741824,

  /**
   * 0x80000000 Extended bit-field are used for further sensor status bits (needs to be set in
   * onboard_control_sensors_present only)
   */
  'EXTENSION_USED'                                 = 2147483648,
}

/**
 * These encode the sensors whose status is sent as part of the SYS_STATUS message in the extended
 * fields.
 */
export enum MavSysStatusSensorExtended {
  /**
   * 0x01 Recovery system (parachute, balloon, retracts etc)
   */
  'SYSTEM'                                         = 1,
}

/**
 * Co-ordinate frames used by MAVLink. Not all frames are supported by all commands, messages, or
 * vehicles.

 Global frames use the following naming conventions:
 - "GLOBAL": Global co-ordinate
 * frame with WGS84 latitude/longitude and altitude positive over mean sea level (MSL) by default.
 * The following modifiers may be used with "GLOBAL":
 - "RELATIVE_ALT": Altitude is relative to the
 * vehicle home position rather than MSL.
 - "TERRAIN_ALT": Altitude is relative to ground level rather
 * than MSL.
 - "INT": Latitude/longitude (in degrees) are scaled by multiplying by 1E7.

 Local
 * frames use the following naming conventions:
 - "LOCAL": Origin of local frame is fixed relative to
 * earth. Unless otherwise specified this origin is the origin of the vehicle position-estimator
 * ("EKF").
 - "BODY": Origin of local frame travels with the vehicle. NOTE, "BODY" does NOT indicate
 * alignment of frame axis with vehicle attitude.
 - "OFFSET": Deprecated synonym for "BODY" (origin
 * travels with the vehicle). Not to be used for new frames.

 Some deprecated frames do not follow
 * these conventions (e.g. MAV_FRAME_BODY_NED and MAV_FRAME_BODY_OFFSET_NED).
 */
export enum MavFrame {
  /**
   * Global (WGS84) coordinate frame + MSL altitude. First value / x: latitude, second value / y:
   * longitude, third value / z: positive altitude over mean sea level (MSL).
   */
  'GLOBAL'                                         = 0,

  /**
   * NED local tangent frame (x: North, y: East, z: Down) with origin fixed relative to earth.
   */
  'LOCAL_NED'                                      = 1,

  /**
   * NOT a coordinate frame, indicates a mission command.
   */
  'MISSION'                                        = 2,

  /**
   * Global (WGS84) coordinate frame + altitude relative to the home position. First value / x: latitude,
   * second value / y: longitude, third value / z: positive altitude with 0 being at the altitude of the
   * home location.
   */
  'GLOBAL_RELATIVE_ALT'                            = 3,

  /**
   * ENU local tangent frame (x: East, y: North, z: Up) with origin fixed relative to earth.
   */
  'LOCAL_ENU'                                      = 4,

  /**
   * Global (WGS84) coordinate frame (scaled) + MSL altitude. First value / x: latitude in degrees*1E7,
   * second value / y: longitude in degrees*1E7, third value / z: positive altitude over mean sea level
   * (MSL).
   */
  'GLOBAL_INT'                                     = 5,

  /**
   * Global (WGS84) coordinate frame (scaled) + altitude relative to the home position. First value / x:
   * latitude in degrees*1E7, second value / y: longitude in degrees*1E7, third value / z: positive
   * altitude with 0 being at the altitude of the home location.
   */
  'GLOBAL_RELATIVE_ALT_INT'                        = 6,

  /**
   * NED local tangent frame (x: North, y: East, z: Down) with origin that travels with the vehicle.
   */
  'LOCAL_OFFSET_NED'                               = 7,

  /**
   * Same as MAV_FRAME_LOCAL_NED when used to represent position values. Same as MAV_FRAME_BODY_FRD when
   * used with velocity/accelaration values.
   */
  'BODY_NED'                                       = 8,

  /**
   * This is the same as MAV_FRAME_BODY_FRD.
   */
  'BODY_OFFSET_NED'                                = 9,

  /**
   * Global (WGS84) coordinate frame with AGL altitude (at the waypoint coordinate). First value / x:
   * latitude in degrees, second value / y: longitude in degrees, third value / z: positive altitude in
   * meters with 0 being at ground level in terrain model.
   */
  'GLOBAL_TERRAIN_ALT'                             = 10,

  /**
   * Global (WGS84) coordinate frame (scaled) with AGL altitude (at the waypoint coordinate). First value
   * / x: latitude in degrees*1E7, second value / y: longitude in degrees*1E7, third value / z: positive
   * altitude in meters with 0 being at ground level in terrain model.
   */
  'GLOBAL_TERRAIN_ALT_INT'                         = 11,

  /**
   * FRD local tangent frame (x: Forward, y: Right, z: Down) with origin that travels with vehicle. The
   * forward axis is aligned to the front of the vehicle in the horizontal plane.
   */
  'BODY_FRD'                                       = 12,

  /**
   * MAV_FRAME_BODY_FLU - Body fixed frame of reference, Z-up (x: Forward, y: Left, z: Up).
   */
  'BODY_FLU'                                       = 13,

  /**
   * MAV_FRAME_MOCAP_NED - Odometry local coordinate frame of data given by a motion capture system,
   * Z-down (x: North, y: East, z: Down).
   */
  'RESERVED_14'                                    = 14,

  /**
   * MAV_FRAME_MOCAP_ENU - Odometry local coordinate frame of data given by a motion capture system, Z-up
   * (x: East, y: North, z: Up).
   */
  'RESERVED_15'                                    = 15,

  /**
   * MAV_FRAME_VISION_NED - Odometry local coordinate frame of data given by a vision estimation system,
   * Z-down (x: North, y: East, z: Down).
   */
  'RESERVED_16'                                    = 16,

  /**
   * MAV_FRAME_VISION_ENU - Odometry local coordinate frame of data given by a vision estimation system,
   * Z-up (x: East, y: North, z: Up).
   */
  'RESERVED_17'                                    = 17,

  /**
   * MAV_FRAME_ESTIM_NED - Odometry local coordinate frame of data given by an estimator running onboard
   * the vehicle, Z-down (x: North, y: East, z: Down).
   */
  'RESERVED_18'                                    = 18,

  /**
   * MAV_FRAME_ESTIM_ENU - Odometry local coordinate frame of data given by an estimator running onboard
   * the vehicle, Z-up (x: East, y: North, z: Up).
   */
  'RESERVED_19'                                    = 19,

  /**
   * FRD local tangent frame (x: Forward, y: Right, z: Down) with origin fixed relative to earth. The
   * forward axis is aligned to the front of the vehicle in the horizontal plane.
   */
  'LOCAL_FRD'                                      = 20,

  /**
   * FLU local tangent frame (x: Forward, y: Left, z: Up) with origin fixed relative to earth. The
   * forward axis is aligned to the front of the vehicle in the horizontal plane.
   */
  'LOCAL_FLU'                                      = 21,
}

/**
 * MAVLINK_DATA_STREAM_TYPE
 */
export enum MavlinkDataStreamType {
  'JPEG'                                           = 0,
  'BMP'                                            = 1,
  'RAW8U'                                          = 2,
  'RAW32U'                                         = 3,
  'PGM'                                            = 4,
  'PNG'                                            = 5,
}

/**
 * Actions following geofence breach.
 */
export enum FenceAction {
  /**
   * Disable fenced mode. If used in a plan this would mean the next fence is disabled.
   */
  'NONE'                                           = 0,

  /**
   * Fly to geofence MAV_CMD_NAV_FENCE_RETURN_POINT in GUIDED mode. Note: This action is only supported
   * by ArduPlane, and may not be supported in all versions.
   */
  'GUIDED'                                         = 1,

  /**
   * Report fence breach, but don't take action
   */
  'REPORT'                                         = 2,

  /**
   * Fly to geofence MAV_CMD_NAV_FENCE_RETURN_POINT with manual throttle control in GUIDED mode. Note:
   * This action is only supported by ArduPlane, and may not be supported in all versions.
   */
  'GUIDED_THR_PASS'                                = 3,

  /**
   * Return/RTL mode.
   */
  'RTL'                                            = 4,

  /**
   * Hold at current location.
   */
  'HOLD'                                           = 5,

  /**
   * Termination failsafe. Motors are shut down (some flight stacks may trigger other failsafe actions).
   */
  'TERMINATE'                                      = 6,

  /**
   * Land at current location.
   */
  'LAND'                                           = 7,
}

/**
 * FENCE_BREACH
 */
export enum FenceBreach {
  /**
   * No last fence breach
   */
  'NONE'                                           = 0,

  /**
   * Breached minimum altitude
   */
  'MINALT'                                         = 1,

  /**
   * Breached maximum altitude
   */
  'MAXALT'                                         = 2,

  /**
   * Breached fence boundary
   */
  'BOUNDARY'                                       = 3,
}

/**
 * Actions being taken to mitigate/prevent fence breach
 */
export enum FenceMitigate {
  /**
   * Unknown
   */
  'UNKNOWN'                                        = 0,

  /**
   * No actions being taken
   */
  'NONE'                                           = 1,

  /**
   * Velocity limiting active to prevent breach
   */
  'VEL_LIMIT'                                      = 2,
}

/**
 * Enumeration of possible mount operation modes. This message is used by obsolete/deprecated gimbal
 * messages.
 */
export enum MavMountMode {
  /**
   * Load and keep safe position (Roll,Pitch,Yaw) from permant memory and stop stabilization
   */
  'RETRACT'                                        = 0,

  /**
   * Load and keep neutral position (Roll,Pitch,Yaw) from permanent memory.
   */
  'NEUTRAL'                                        = 1,

  /**
   * Load neutral position and start MAVLink Roll,Pitch,Yaw control with stabilization
   */
  'MAVLINK_TARGETING'                              = 2,

  /**
   * Load neutral position and start RC Roll,Pitch,Yaw control with stabilization
   */
  'RC_TARGETING'                                   = 3,

  /**
   * Load neutral position and start to point to Lat,Lon,Alt
   */
  'GPS_POINT'                                      = 4,

  /**
   * Gimbal tracks system with specified system ID
   */
  'SYSID_TARGET'                                   = 5,

  /**
   * Gimbal tracks home location
   */
  'HOME_LOCATION'                                  = 6,
}

/**
 * Gimbal device (low level) capability flags (bitmap)
 */
export enum GimbalDeviceCapFlags {
  /**
   * Gimbal device supports a retracted position
   */
  'HAS_RETRACT'                                    = 1,

  /**
   * Gimbal device supports a horizontal, forward looking position, stabilized
   */
  'HAS_NEUTRAL'                                    = 2,

  /**
   * Gimbal device supports rotating around roll axis.
   */
  'HAS_ROLL_AXIS'                                  = 4,

  /**
   * Gimbal device supports to follow a roll angle relative to the vehicle
   */
  'HAS_ROLL_FOLLOW'                                = 8,

  /**
   * Gimbal device supports locking to an roll angle (generally that's the default with roll stabilized)
   */
  'HAS_ROLL_LOCK'                                  = 16,

  /**
   * Gimbal device supports rotating around pitch axis.
   */
  'HAS_PITCH_AXIS'                                 = 32,

  /**
   * Gimbal device supports to follow a pitch angle relative to the vehicle
   */
  'HAS_PITCH_FOLLOW'                               = 64,

  /**
   * Gimbal device supports locking to an pitch angle (generally that's the default with pitch
   * stabilized)
   */
  'HAS_PITCH_LOCK'                                 = 128,

  /**
   * Gimbal device supports rotating around yaw axis.
   */
  'HAS_YAW_AXIS'                                   = 256,

  /**
   * Gimbal device supports to follow a yaw angle relative to the vehicle (generally that's the default)
   */
  'HAS_YAW_FOLLOW'                                 = 512,

  /**
   * Gimbal device supports locking to an absolute heading (often this is an option available)
   */
  'HAS_YAW_LOCK'                                   = 1024,

  /**
   * Gimbal device supports yawing/panning infinetely (e.g. using slip disk).
   */
  'SUPPORTS_INFINITE_YAW'                          = 2048,
}

/**
 * Gimbal manager high level capability flags (bitmap). The first 16 bits are identical to the
 * GIMBAL_DEVICE_CAP_FLAGS. However, the gimbal manager does not need to copy the flags from the gimbal
 * but can also enhance the capabilities and thus add flags.
 */
export enum GimbalManagerCapFlags {
  /**
   * Based on GIMBAL_DEVICE_CAP_FLAGS_HAS_RETRACT.
   */
  'HAS_RETRACT'                                    = 1,

  /**
   * Based on GIMBAL_DEVICE_CAP_FLAGS_HAS_NEUTRAL.
   */
  'HAS_NEUTRAL'                                    = 2,

  /**
   * Based on GIMBAL_DEVICE_CAP_FLAGS_HAS_ROLL_AXIS.
   */
  'HAS_ROLL_AXIS'                                  = 4,

  /**
   * Based on GIMBAL_DEVICE_CAP_FLAGS_HAS_ROLL_FOLLOW.
   */
  'HAS_ROLL_FOLLOW'                                = 8,

  /**
   * Based on GIMBAL_DEVICE_CAP_FLAGS_HAS_ROLL_LOCK.
   */
  'HAS_ROLL_LOCK'                                  = 16,

  /**
   * Based on GIMBAL_DEVICE_CAP_FLAGS_HAS_PITCH_AXIS.
   */
  'HAS_PITCH_AXIS'                                 = 32,

  /**
   * Based on GIMBAL_DEVICE_CAP_FLAGS_HAS_PITCH_FOLLOW.
   */
  'HAS_PITCH_FOLLOW'                               = 64,

  /**
   * Based on GIMBAL_DEVICE_CAP_FLAGS_HAS_PITCH_LOCK.
   */
  'HAS_PITCH_LOCK'                                 = 128,

  /**
   * Based on GIMBAL_DEVICE_CAP_FLAGS_HAS_YAW_AXIS.
   */
  'HAS_YAW_AXIS'                                   = 256,

  /**
   * Based on GIMBAL_DEVICE_CAP_FLAGS_HAS_YAW_FOLLOW.
   */
  'HAS_YAW_FOLLOW'                                 = 512,

  /**
   * Based on GIMBAL_DEVICE_CAP_FLAGS_HAS_YAW_LOCK.
   */
  'HAS_YAW_LOCK'                                   = 1024,

  /**
   * Based on GIMBAL_DEVICE_CAP_FLAGS_SUPPORTS_INFINITE_YAW.
   */
  'SUPPORTS_INFINITE_YAW'                          = 2048,

  /**
   * Gimbal manager supports to point to a local position.
   */
  'CAN_POINT_LOCATION_LOCAL'                       = 65536,

  /**
   * Gimbal manager supports to point to a global latitude, longitude, altitude position.
   */
  'CAN_POINT_LOCATION_GLOBAL'                      = 131072,
}

/**
 * Flags for gimbal device (lower level) operation.
 */
export enum GimbalDeviceFlags {
  /**
   * Set to retracted safe position (no stabilization), takes presedence over all other flags.
   */
  'RETRACT'                                        = 1,

  /**
   * Set to neutral/default position, taking precedence over all other flags except RETRACT. Neutral is
   * commonly forward-facing and horizontal (pitch=yaw=0) but may be any orientation.
   */
  'NEUTRAL'                                        = 2,

  /**
   * Lock roll angle to absolute angle relative to horizon (not relative to drone). This is generally the
   * default with a stabilizing gimbal.
   */
  'ROLL_LOCK'                                      = 4,

  /**
   * Lock pitch angle to absolute angle relative to horizon (not relative to drone). This is generally
   * the default.
   */
  'PITCH_LOCK'                                     = 8,

  /**
   * Lock yaw angle to absolute angle relative to North (not relative to drone). If this flag is set, the
   * quaternion is in the Earth frame with the x-axis pointing North (yaw absolute). If this flag is not
   * set, the quaternion frame is in the Earth frame rotated so that the x-axis is pointing forward (yaw
   * relative to vehicle).
   */
  'YAW_LOCK'                                       = 16,
}

/**
 * Flags for high level gimbal manager operation The first 16 bits are identical to the
 * GIMBAL_DEVICE_FLAGS.
 */
export enum GimbalManagerFlags {
  /**
   * Based on GIMBAL_DEVICE_FLAGS_RETRACT
   */
  'RETRACT'                                        = 1,

  /**
   * Based on GIMBAL_DEVICE_FLAGS_NEUTRAL
   */
  'NEUTRAL'                                        = 2,

  /**
   * Based on GIMBAL_DEVICE_FLAGS_ROLL_LOCK
   */
  'ROLL_LOCK'                                      = 4,

  /**
   * Based on GIMBAL_DEVICE_FLAGS_PITCH_LOCK
   */
  'PITCH_LOCK'                                     = 8,

  /**
   * Based on GIMBAL_DEVICE_FLAGS_YAW_LOCK
   */
  'YAW_LOCK'                                       = 16,
}

/**
 * Gimbal device (low level) error flags (bitmap, 0 means no error)
 */
export enum GimbalDeviceErrorFlags {
  /**
   * Gimbal device is limited by hardware roll limit.
   */
  'AT_ROLL_LIMIT'                                  = 1,

  /**
   * Gimbal device is limited by hardware pitch limit.
   */
  'AT_PITCH_LIMIT'                                 = 2,

  /**
   * Gimbal device is limited by hardware yaw limit.
   */
  'AT_YAW_LIMIT'                                   = 4,

  /**
   * There is an error with the gimbal encoders.
   */
  'ENCODER_ERROR'                                  = 8,

  /**
   * There is an error with the gimbal power source.
   */
  'POWER_ERROR'                                    = 16,

  /**
   * There is an error with the gimbal motor's.
   */
  'MOTOR_ERROR'                                    = 32,

  /**
   * There is an error with the gimbal's software.
   */
  'SOFTWARE_ERROR'                                 = 64,

  /**
   * There is an error with the gimbal's communication.
   */
  'COMMS_ERROR'                                    = 128,

  /**
   * Gimbal is currently calibrating.
   */
  'CALIBRATION_RUNNING'                            = 256,
}

/**
 * Gripper actions.
 */
export enum GripperActions {
  /**
   * Gripper release cargo.
   */
  'RELEASE'                                        = 0,

  /**
   * Gripper grab onto cargo.
   */
  'GRAB'                                           = 1,
}

/**
 * Winch actions.
 */
export enum WinchActions {
  /**
   * Relax winch.
   */
  'RELAXED'                                        = 0,

  /**
   * Wind or unwind specified length of cable, optionally using specified rate.
   */
  'RELATIVE_LENGTH_CONTROL'                        = 1,

  /**
   * Wind or unwind cable at specified rate.
   */
  'RATE_CONTROL'                                   = 2,
}

/**
 * Generalized UAVCAN node health
 */
export enum UavcanNodeHealth {
  /**
   * The node is functioning properly.
   */
  'OK'                                             = 0,

  /**
   * A critical parameter went out of range or the node has encountered a minor failure.
   */
  'WARNING'                                        = 1,

  /**
   * The node has encountered a major failure.
   */
  'ERROR'                                          = 2,

  /**
   * The node has suffered a fatal malfunction.
   */
  'CRITICAL'                                       = 3,
}

/**
 * Generalized UAVCAN node mode
 */
export enum UavcanNodeMode {
  /**
   * The node is performing its primary functions.
   */
  'OPERATIONAL'                                    = 0,

  /**
   * The node is initializing; this mode is entered immediately after startup.
   */
  'INITIALIZATION'                                 = 1,

  /**
   * The node is under maintenance.
   */
  'MAINTENANCE'                                    = 2,

  /**
   * The node is in the process of updating its software.
   */
  'SOFTWARE_UPDATE'                                = 3,

  /**
   * The node is no longer available online.
   */
  'OFFLINE'                                        = 7,
}

/**
 * Indicates the ESC connection type.
 */
export enum EscConnectionType {
  /**
   * Traditional PPM ESC.
   */
  'PPM'                                            = 0,

  /**
   * Serial Bus connected ESC.
   */
  'SERIAL'                                         = 1,

  /**
   * One Shot PPM ESC.
   */
  'ONESHOT'                                        = 2,

  /**
   * I2C ESC.
   */
  'I2C'                                            = 3,

  /**
   * CAN-Bus ESC.
   */
  'CAN'                                            = 4,

  /**
   * DShot ESC.
   */
  'DSHOT'                                          = 5,
}

/**
 * Flags to report ESC failures.
 */
export enum EscFailureFlags {
  /**
   * No ESC failure.
   */
  'NONE'                                           = 0,

  /**
   * Over current failure.
   */
  'OVER_CURRENT'                                   = 1,

  /**
   * Over voltage failure.
   */
  'OVER_VOLTAGE'                                   = 2,

  /**
   * Over temperature failure.
   */
  'OVER_TEMPERATURE'                               = 4,

  /**
   * Over RPM failure.
   */
  'OVER_RPM'                                       = 8,

  /**
   * Inconsistent command failure i.e. out of bounds.
   */
  'INCONSISTENT_CMD'                               = 16,

  /**
   * Motor stuck failure.
   */
  'MOTOR_STUCK'                                    = 32,

  /**
   * Generic ESC failure.
   */
  'GENERIC'                                        = 64,
}

/**
 * Flags to indicate the status of camera storage.
 */
export enum StorageStatus {
  /**
   * Storage is missing (no microSD card loaded for example.)
   */
  'EMPTY'                                          = 0,

  /**
   * Storage present but unformatted.
   */
  'UNFORMATTED'                                    = 1,

  /**
   * Storage present and ready.
   */
  'READY'                                          = 2,

  /**
   * Camera does not supply storage status information. Capacity information in STORAGE_INFORMATION
   * fields will be ignored.
   */
  'NOT_SUPPORTED'                                  = 3,
}

/**
 * Flags to indicate the type of storage.
 */
export enum StorageType {
  /**
   * Storage type is not known.
   */
  'UNKNOWN'                                        = 0,

  /**
   * Storage type is USB device.
   */
  'USB_STICK'                                      = 1,

  /**
   * Storage type is SD card.
   */
  'SD'                                             = 2,

  /**
   * Storage type is microSD card.
   */
  'MICROSD'                                        = 3,

  /**
   * Storage type is CFast.
   */
  'CF'                                             = 4,

  /**
   * Storage type is CFexpress.
   */
  'CFE'                                            = 5,

  /**
   * Storage type is XQD.
   */
  'XQD'                                            = 6,

  /**
   * Storage type is HD mass storage type.
   */
  'HD'                                             = 7,

  /**
   * Storage type is other, not listed type.
   */
  'OTHER'                                          = 254,
}

/**
 * Flags to indicate usage for a particular storage (see STORAGE_INFORMATION.storage_usage and
 * MAV_CMD_SET_STORAGE_USAGE).
 */
export enum StorageUsageFlag {
  /**
   * Always set to 1 (indicates STORAGE_INFORMATION.storage_usage is supported).
   */
  'SET'                                            = 1,

  /**
   * Storage for saving photos.
   */
  'PHOTO'                                          = 2,

  /**
   * Storage for saving videos.
   */
  'VIDEO'                                          = 4,

  /**
   * Storage for saving logs.
   */
  'LOGS'                                           = 8,
}

/**
 * Yaw behaviour during orbit flight.
 */
export enum OrbitYawBehaviour {
  /**
   * Vehicle front points to the center (default).
   */
  'HOLD_FRONT_TO_CIRCLE_CENTER'                    = 0,

  /**
   * Vehicle front holds heading when message received.
   */
  'HOLD_INITIAL_HEADING'                           = 1,

  /**
   * Yaw uncontrolled.
   */
  'UNCONTROLLED'                                   = 2,

  /**
   * Vehicle front follows flight path (tangential to circle).
   */
  'HOLD_FRONT_TANGENT_TO_CIRCLE'                   = 3,

  /**
   * Yaw controlled by RC input.
   */
  'RC_CONTROLLED'                                  = 4,
}

/**
 * Possible responses from a WIFI_CONFIG_AP message.
 */
export enum WifiConfigApResponse {
  /**
   * Undefined response. Likely an indicative of a system that doesn't support this request.
   */
  'UNDEFINED'                                      = 0,

  /**
   * Changes accepted.
   */
  'ACCEPTED'                                       = 1,

  /**
   * Changes rejected.
   */
  'REJECTED'                                       = 2,

  /**
   * Invalid Mode.
   */
  'MODE_ERROR'                                     = 3,

  /**
   * Invalid SSID.
   */
  'SSID_ERROR'                                     = 4,

  /**
   * Invalid Password.
   */
  'PASSWORD_ERROR'                                 = 5,
}

/**
 * Possible responses from a CELLULAR_CONFIG message.
 */
export enum CellularConfigResponse {
  /**
   * Changes accepted.
   */
  'RESPONSE_ACCEPTED'                              = 0,

  /**
   * Invalid APN.
   */
  'RESPONSE_APN_ERROR'                             = 1,

  /**
   * Invalid PIN.
   */
  'RESPONSE_PIN_ERROR'                             = 2,

  /**
   * Changes rejected.
   */
  'RESPONSE_REJECTED'                              = 3,

  /**
   * PUK is required to unblock SIM card.
   */
  'BLOCKED_PUK_REQUIRED'                           = 4,
}

/**
 * WiFi Mode.
 */
export enum WifiConfigApMode {
  /**
   * WiFi mode is undefined.
   */
  'UNDEFINED'                                      = 0,

  /**
   * WiFi configured as an access point.
   */
  'AP'                                             = 1,

  /**
   * WiFi configured as a station connected to an existing local WiFi network.
   */
  'STATION'                                        = 2,

  /**
   * WiFi disabled.
   */
  'DISABLED'                                       = 3,
}

/**
 * Supported component metadata types. These are used in the "general" metadata file returned by
 * COMPONENT_INFORMATION to provide information about supported metadata types. The types are not used
 * directly in MAVLink messages.
 */
export enum CompMetadataType {
  /**
   * General information about the component. General metadata includes information about other
   * COMP_METADATA_TYPEs supported by the component. This type must be supported and must be downloadable
   * from vehicle.
   */
  'GENERAL'                                        = 0,

  /**
   * Parameter meta data.
   */
  'PARAMETER'                                      = 1,

  /**
   * Meta data that specifies which commands and command parameters the vehicle supports. (WIP)
   */
  'COMMANDS'                                       = 2,

  /**
   * Meta data that specifies external non-MAVLink peripherals.
   */
  'PERIPHERALS'                                    = 3,

  /**
   * Meta data for the events interface.
   */
  'EVENTS'                                         = 4,

  /**
   * Meta data for actuator configuration (motors, servos and vehicle geometry) and testing.
   */
  'ACTUATORS'                                      = 5,
}

/**
 * Actuator configuration, used to change a setting on an actuator. Component information metadata can
 * be used to know which outputs support which commands.
 */
export enum ActuatorConfiguration {
  /**
   * Do nothing.
   */
  'NONE'                                           = 0,

  /**
   * Command the actuator to beep now.
   */
  'BEEP'                                           = 1,

  /**
   * Permanently set the actuator (ESC) to 3D mode (reversible thrust).
   */
  'ACTUATOR_CONFIGURATION_3D_MODE_ON'              = 2,

  /**
   * Permanently set the actuator (ESC) to non 3D mode (non-reversible thrust).
   */
  'ACTUATOR_CONFIGURATION_3D_MODE_OFF'             = 3,

  /**
   * Permanently set the actuator (ESC) to spin direction 1 (which can be clockwise or
   * counter-clockwise).
   */
  'SPIN_DIRECTION1'                                = 4,

  /**
   * Permanently set the actuator (ESC) to spin direction 2 (opposite of direction 1).
   */
  'SPIN_DIRECTION2'                                = 5,
}

/**
 * Actuator output function. Values greater or equal to 1000 are autopilot-specific.
 */
export enum ActuatorOutputFunction {
  /**
   * No function (disabled).
   */
  'NONE'                                           = 0,

  /**
   * Motor 1
   */
  'MOTOR1'                                         = 1,

  /**
   * Motor 2
   */
  'MOTOR2'                                         = 2,

  /**
   * Motor 3
   */
  'MOTOR3'                                         = 3,

  /**
   * Motor 4
   */
  'MOTOR4'                                         = 4,

  /**
   * Motor 5
   */
  'MOTOR5'                                         = 5,

  /**
   * Motor 6
   */
  'MOTOR6'                                         = 6,

  /**
   * Motor 7
   */
  'MOTOR7'                                         = 7,

  /**
   * Motor 8
   */
  'MOTOR8'                                         = 8,

  /**
   * Motor 9
   */
  'MOTOR9'                                         = 9,

  /**
   * Motor 10
   */
  'MOTOR10'                                        = 10,

  /**
   * Motor 11
   */
  'MOTOR11'                                        = 11,

  /**
   * Motor 12
   */
  'MOTOR12'                                        = 12,

  /**
   * Motor 13
   */
  'MOTOR13'                                        = 13,

  /**
   * Motor 14
   */
  'MOTOR14'                                        = 14,

  /**
   * Motor 15
   */
  'MOTOR15'                                        = 15,

  /**
   * Motor 16
   */
  'MOTOR16'                                        = 16,

  /**
   * Servo 1
   */
  'SERVO1'                                         = 33,

  /**
   * Servo 2
   */
  'SERVO2'                                         = 34,

  /**
   * Servo 3
   */
  'SERVO3'                                         = 35,

  /**
   * Servo 4
   */
  'SERVO4'                                         = 36,

  /**
   * Servo 5
   */
  'SERVO5'                                         = 37,

  /**
   * Servo 6
   */
  'SERVO6'                                         = 38,

  /**
   * Servo 7
   */
  'SERVO7'                                         = 39,

  /**
   * Servo 8
   */
  'SERVO8'                                         = 40,

  /**
   * Servo 9
   */
  'SERVO9'                                         = 41,

  /**
   * Servo 10
   */
  'SERVO10'                                        = 42,

  /**
   * Servo 11
   */
  'SERVO11'                                        = 43,

  /**
   * Servo 12
   */
  'SERVO12'                                        = 44,

  /**
   * Servo 13
   */
  'SERVO13'                                        = 45,

  /**
   * Servo 14
   */
  'SERVO14'                                        = 46,

  /**
   * Servo 15
   */
  'SERVO15'                                        = 47,

  /**
   * Servo 16
   */
  'SERVO16'                                        = 48,
}

/**
 * Enable axes that will be tuned via autotuning. Used in MAV_CMD_DO_AUTOTUNE_ENABLE.
 */
export enum AutotuneAxis {
  /**
   * Flight stack tunes axis according to its default settings.
   */
  'DEFAULT'                                        = 0,

  /**
   * Autotune roll axis.
   */
  'ROLL'                                           = 1,

  /**
   * Autotune pitch axis.
   */
  'PITCH'                                          = 2,

  /**
   * Autotune yaw axis.
   */
  'YAW'                                            = 4,
}

/**
 * Commands to be executed by the MAV. They can be executed on user request, or as part of a mission
 * script. If the action is used in a mission, the parameter mapping to the waypoint/mission message is
 * as follows: Param 1, Param 2, Param 3, Param 4, X: Param 5, Y:Param 6, Z:Param 7. This command list
 * is similar what ARINC 424 is for commercial aircraft: A data format how to interpret
 * waypoint/mission data. NaN and INT32_MAX may be used in float/integer params (respectively) to
 * indicate optional/default values (e.g. to use the component's current yaw or latitude rather than a
 * specific value). See https://mavlink.io/en/guide/xml_schema.html#MAV_CMD for information about the
 * structure of the MAV_CMD entries
 */
export enum MavCmd {
  /**
   * request vehicle to send WGA code.
   * @param1 ind (uint32_t)0-9:WGA index  10-79:Clear
   * @param2 Empty
   * @param3 Empty
   * @param4 Empty
   * @param5 Empty
   * @param6 Empty
   * @param7 Empty
   */
  'SEND_WGA'                                       = 10,

  /**
   * request vehicle to write WGA password.
   * @param1 password1 password1
   * @param2 password2 password2
   * @param3 password3 password3
   * @param4 password4 password4
   * @param5 Empty
   * @param6 Empty
   * @param7 Empty
   */
  'WRITE_WGA'                                      = 11,

  /**
   * request vehicle to write WGA password.
   * @param1 rtc time bit25-31:year-1980  bit21-24:month  bit16-20:date  bit11-15:hour  bit5-10:minute  bit0-4:seconds
   * @param2 Empty
   * @param3 Empty
   * @param4 Empty
   * @param5 Empty
   * @param6 Empty
   * @param7 Empty
   */
  'SET_RTC'                                        = 12,

  /**
   * request vehicle to write WGA password.
   * @param1 ind possensor ind
   * @param2 rate positive:rate divider   negative:send times per second   equals 0:cancel message
   * @param3 req 1:cancel others(this only)
   * @param4 Empty
   * @param5 Empty
   * @param6 Empty
   * @param7 Empty
   */
  'REQUEST_ACFLY_POSSENSOR_INFO_STREAM'            = 14,

  /**
   * request vehicle to write WGA password.
   * @param1 rateDivider HTL time rate divider. Must be positive integer.
   * @param2 Empty
   * @param3 Empty
   * @param4 Empty
   * @param5 Empty
   * @param6 Empty
   * @param7 Empty
   */
  'REQUEST_HTL'                                    = 15,

  /**
   * Navigate to waypoint.
   *
   * @note has location and is destination
   *
   * @param1 Hold[s] (min: 0) Hold time. (ignored by fixed wing, time to stay at waypoint for rotary wing)
   * @param2 Accept Radius[m] (min: 0) Acceptance radius (if the sphere with this radius is hit, the waypoint counts as reached)
   * @param3 Pass Radius[m] 0 to pass through the WP, if > 0 radius to pass by WP. Positive value for clockwise orbit, negative value for counter-clockwise orbit. Allows trajectory control.
   * @param4 Yaw[deg] Desired yaw angle at waypoint (rotary wing). NaN to use the current system yaw heading mode (e.g. yaw towards next waypoint, yaw to home, etc.).
   * @param5 Latitude Latitude
   * @param6 Longitude Longitude
   * @param7 Altitude[m] Altitude
   */
  'NAV_WAYPOINT'                                   = 16,

  /**
   * Loiter around this waypoint an unlimited amount of time
   *
   * @note has location and is destination
   *
   * @param1 Empty
   * @param2 Empty
   * @param3 Radius[m] Loiter radius around waypoint for forward-only moving vehicles (not multicopters). If positive loiter clockwise, else counter-clockwise
   * @param4 Yaw[deg] Desired yaw angle. NaN to use the current system yaw heading mode (e.g. yaw towards next waypoint, yaw to home, etc.).
   * @param5 Latitude Latitude
   * @param6 Longitude Longitude
   * @param7 Altitude[m] Altitude
   */
  'NAV_LOITER_UNLIM'                               = 17,

  /**
   * Loiter around this waypoint for X turns
   *
   * @note has location and is destination
   *
   * @param1 Turns (min: 0) Number of turns.
   * @param2 Heading Required (min: 0, max: 1, increment: 1) Leave loiter circle only once heading towards the next waypoint (0 = False)
   * @param3 Radius[m] Loiter radius around waypoint for forward-only moving vehicles (not multicopters). If positive loiter clockwise, else counter-clockwise
   * @param4 Xtrack Location Loiter circle exit location and/or path to next waypoint ("xtrack") for forward-only moving vehicles (not multicopters). 0 for the vehicle to converge towards the center xtrack when it leaves the loiter (the line between the centers of the current and next waypoint), 1 to converge to the direct line between the location that the vehicle exits the loiter radius and the next waypoint. Otherwise the angle (in degrees) between the tangent of the loiter circle and the center xtrack at which the vehicle must leave the loiter (and converge to the center xtrack). NaN to use the current system default xtrack behaviour.
   * @param5 Latitude Latitude
   * @param6 Longitude Longitude
   * @param7 Altitude[m] Altitude
   */
  'NAV_LOITER_TURNS'                               = 18,

  /**
   * Loiter at the specified latitude, longitude and altitude for a certain amount of time. Multicopter
   * vehicles stop at the point (within a vehicle-specific acceptance radius). Forward-only moving
   * vehicles (e.g. fixed-wing) circle the point with the specified radius/direction. If the Heading
   * Required parameter (2) is non-zero forward moving aircraft will only leave the loiter circle once
   * heading towards the next waypoint.
   *
   * @note has location and is destination
   *
   * @param1 Time[s] (min: 0) Loiter time (only starts once Lat, Lon and Alt is reached).
   * @param2 Heading Required (min: 0, max: 1, increment: 1) Leave loiter circle only once heading towards the next waypoint (0 = False)
   * @param3 Radius[m] Loiter radius around waypoint for forward-only moving vehicles (not multicopters). If positive loiter clockwise, else counter-clockwise.
   * @param4 Xtrack Location Loiter circle exit location and/or path to next waypoint ("xtrack") for forward-only moving vehicles (not multicopters). 0 for the vehicle to converge towards the center xtrack when it leaves the loiter (the line between the centers of the current and next waypoint), 1 to converge to the direct line between the location that the vehicle exits the loiter radius and the next waypoint. Otherwise the angle (in degrees) between the tangent of the loiter circle and the center xtrack at which the vehicle must leave the loiter (and converge to the center xtrack). NaN to use the current system default xtrack behaviour.
   * @param5 Latitude Latitude
   * @param6 Longitude Longitude
   * @param7 Altitude[m] Altitude
   */
  'NAV_LOITER_TIME'                                = 19,

  /**
   * Return to launch location
   * @param1 Empty
   * @param2 Empty
   * @param3 Empty
   * @param4 Empty
   * @param5 Empty
   * @param6 Empty
   * @param7 Empty
   */
  'NAV_RETURN_TO_LAUNCH'                           = 20,

  /**
   * Land at location.
   *
   * @note has location and is destination
   *
   * @param1 Abort Alt[m] Minimum target altitude if landing is aborted (0 = undefined/use system default).
   * @param2 Land Mode Precision land mode.
   * @param3 Empty.
   * @param4 Yaw Angle[deg] Desired yaw angle. NaN to use the current system yaw heading mode (e.g. yaw towards next waypoint, yaw to home, etc.).
   * @param5 Latitude Latitude.
   * @param6 Longitude Longitude.
   * @param7 Altitude[m] Landing altitude (ground level in current frame).
   */
  'NAV_LAND'                                       = 21,

  /**
   * Takeoff from ground / hand. Vehicles that support multiple takeoff modes (e.g. VTOL quadplane)
   * should take off using the currently configured mode.
   *
   * @note has location and is destination
   *
   * @param1 Pitch[deg] Minimum pitch (if airspeed sensor present), desired pitch without sensor
   * @param2 Empty
   * @param3 Empty
   * @param4 Yaw[deg] Yaw angle (if magnetometer present), ignored without magnetometer. NaN to use the current system yaw heading mode (e.g. yaw towards next waypoint, yaw to home, etc.).
   * @param5 Latitude Latitude
   * @param6 Longitude Longitude
   * @param7 Altitude[m] Altitude
   */
  'NAV_TAKEOFF'                                    = 22,

  /**
   * Land at local position (local frame only)
   *
   * @note has location and is destination
   *
   * @param1 Target (min: 0, increment: 1) Landing target number (if available)
   * @param2 Offset[m] (min: 0) Maximum accepted offset from desired landing position - computed magnitude from spherical coordinates: d = sqrt(x^2 + y^2 + z^2), which gives the maximum accepted distance between the desired landing position and the position where the vehicle is about to land
   * @param3 Descend Rate[m/s] Landing descend rate
   * @param4 Yaw[rad] Desired yaw angle
   * @param5 Y Position[m] Y-axis position
   * @param6 X Position[m] X-axis position
   * @param7 Z Position[m] Z-axis / ground level position
   */
  'NAV_LAND_LOCAL'                                 = 23,

  /**
   * Takeoff from local position (local frame only)
   *
   * @note has location and is destination
   *
   * @param1 Pitch[rad] Minimum pitch (if airspeed sensor present), desired pitch without sensor
   * @param2 Empty
   * @param3 Ascend Rate[m/s] Takeoff ascend rate
   * @param4 Yaw[rad] Yaw angle (if magnetometer or another yaw estimation source present), ignored without one of these
   * @param5 Y Position[m] Y-axis position
   * @param6 X Position[m] X-axis position
   * @param7 Z Position[m] Z-axis position
   */
  'NAV_TAKEOFF_LOCAL'                              = 24,

  /**
   * Vehicle following, i.e. this waypoint represents the position of a moving vehicle
   *
   * @note has location
   *
   * @param1 Following (increment: 1) Following logic to use (e.g. loitering or sinusoidal following) - depends on specific autopilot implementation
   * @param2 Ground Speed[m/s] Ground speed of vehicle to be followed
   * @param3 Radius[m] Radius around waypoint. If positive loiter clockwise, else counter-clockwise
   * @param4 Yaw[deg] Desired yaw angle.
   * @param5 Latitude Latitude
   * @param6 Longitude Longitude
   * @param7 Altitude[m] Altitude
   */
  'NAV_FOLLOW'                                     = 25,

  /**
   * Continue on the current course and climb/descend to specified altitude. When the altitude is reached
   * continue to the next command (i.e., don't proceed to the next command until the desired altitude is
   * reached.
   *
   * @note is destination
   *
   * @param1 Action (min: 0, max: 2, increment: 1) Climb or Descend (0 = Neutral, command completes when within 5m of this command's altitude, 1 = Climbing, command completes when at or above this command's altitude, 2 = Descending, command completes when at or below this command's altitude.
   * @param2 Empty
   * @param3 Empty
   * @param4 Empty
   * @param5 Empty
   * @param6 Empty
   * @param7 Altitude[m] Desired altitude
   */
  'NAV_CONTINUE_AND_CHANGE_ALT'                    = 30,

  /**
   * Begin loiter at the specified Latitude and Longitude. If Lat=Lon=0, then loiter at the current
   * position. Don't consider the navigation command complete (don't leave loiter) until the altitude has
   * been reached. Additionally, if the Heading Required parameter is non-zero the aircraft will not
   * leave the loiter until heading toward the next waypoint.
   *
   * @note has location and is destination
   *
   * @param1 Heading Required (min: 0, max: 1, increment: 1) Leave loiter circle only once heading towards the next waypoint (0 = False)
   * @param2 Radius[m] Loiter radius around waypoint for forward-only moving vehicles (not multicopters). If positive loiter clockwise, negative counter-clockwise, 0 means no change to standard loiter.
   * @param3 Empty
   * @param4 Xtrack Location (min: 0, max: 1, increment: 1) Loiter circle exit location and/or path to next waypoint ("xtrack") for forward-only moving vehicles (not multicopters). 0 for the vehicle to converge towards the center xtrack when it leaves the loiter (the line between the centers of the current and next waypoint), 1 to converge to the direct line between the location that the vehicle exits the loiter radius and the next waypoint. Otherwise the angle (in degrees) between the tangent of the loiter circle and the center xtrack at which the vehicle must leave the loiter (and converge to the center xtrack). NaN to use the current system default xtrack behaviour.
   * @param5 Latitude Latitude
   * @param6 Longitude Longitude
   * @param7 Altitude[m] Altitude
   */
  'NAV_LOITER_TO_ALT'                              = 31,

  /**
   * Begin following a target
   * @param1 System ID (min: 0, max: 255, increment: 1) System ID (of the FOLLOW_TARGET beacon). Send 0 to disable follow-me and return to the default position hold mode.
   * @param2 Reserved
   * @param3 Reserved
   * @param4 Altitude Mode (min: 0, max: 2, increment: 1) Altitude mode: 0: Keep current altitude, 1: keep altitude difference to target, 2: go to a fixed altitude above home.
   * @param5 Altitude[m] Altitude above home. (used if mode=2)
   * @param6 Reserved
   * @param7 Time to Land[s] (min: 0) Time to land in which the MAV should go to the default position hold mode after a message RX timeout.
   */
  'DO_FOLLOW'                                      = 32,

  /**
   * Reposition the MAV after a follow target command has been sent
   * @param1 Camera Q1 Camera q1 (where 0 is on the ray from the camera to the tracking device)
   * @param2 Camera Q2 Camera q2
   * @param3 Camera Q3 Camera q3
   * @param4 Camera Q4 Camera q4
   * @param5 Altitude Offset[m] altitude offset from target
   * @param6 X Offset[m] X offset from target
   * @param7 Y Offset[m] Y offset from target
   */
  'DO_FOLLOW_REPOSITION'                           = 33,

  /**
   * Start orbiting on the circumference of a circle defined by the parameters. Setting values to
   * NaN/INT32_MAX (as appropriate) results in using defaults.
   *
   * @note has location and is destination
   *
   * @param1 Radius[m] Radius of the circle. Positive: orbit clockwise. Negative: orbit counter-clockwise. NaN: Use vehicle default radius, or current radius if already orbiting.
   * @param2 Velocity[m/s] Tangential Velocity. NaN: Use vehicle default velocity, or current velocity if already orbiting.
   * @param3 Yaw Behavior Yaw behavior of the vehicle.
   * @param4 Orbits[rad] (min: 0) Orbit around the centre point for this many radians (i.e. for a three-quarter orbit set 270*Pi/180). 0: Orbit forever. NaN: Use vehicle default, or current value if already orbiting.
   * @param5 Latitude/X Center point latitude (if no MAV_FRAME specified) / X coordinate according to MAV_FRAME. INT32_MAX (or NaN if sent in COMMAND_LONG): Use current vehicle position, or current center if already orbiting.
   * @param6 Longitude/Y Center point longitude (if no MAV_FRAME specified) / Y coordinate according to MAV_FRAME. INT32_MAX (or NaN if sent in COMMAND_LONG): Use current vehicle position, or current center if already orbiting.
   * @param7 Altitude/Z Center point altitude (MSL) (if no MAV_FRAME specified) / Z coordinate according to MAV_FRAME. NaN: Use current vehicle altitude.
   */
  'DO_ORBIT'                                       = 34,

  /**
   * Sets the region of interest (ROI) for a sensor set or the vehicle itself. This can then be used by
   * the vehicle's control system to control the vehicle attitude and the attitude of various sensors
   * such as cameras.
   *
   * @note has location
   *
   * @param1 ROI Mode Region of interest mode.
   * @param2 WP Index (min: 0, increment: 1) Waypoint index/ target ID. (see MAV_ROI enum)
   * @param3 ROI Index (min: 0, increment: 1) ROI index (allows a vehicle to manage multiple ROI's)
   * @param4 Empty
   * @param5 X x the location of the fixed ROI (see MAV_FRAME)
   * @param6 Y y
   * @param7 Z z
   */
  'NAV_ROI'                                        = 80,

  /**
   * Control autonomous path planning on the MAV.
   *
   * @note has location and is destination
   *
   * @param1 Local Ctrl (min: 0, max: 2, increment: 1) 0: Disable local obstacle avoidance / local path planning (without resetting map), 1: Enable local path planning, 2: Enable and reset local path planning
   * @param2 Global Ctrl (min: 0, max: 3, increment: 1) 0: Disable full path planning (without resetting map), 1: Enable, 2: Enable and reset map/occupancy grid, 3: Enable and reset planned route, but not occupancy grid
   * @param3 Empty
   * @param4 Yaw[deg] Yaw angle at goal
   * @param5 Latitude/X Latitude/X of goal
   * @param6 Longitude/Y Longitude/Y of goal
   * @param7 Altitude/Z Altitude/Z of goal
   */
  'NAV_PATHPLANNING'                               = 81,

  /**
   * Navigate to waypoint using a spline path.
   *
   * @note has location and is destination
   *
   * @param1 Hold[s] (min: 0) Hold time. (ignored by fixed wing, time to stay at waypoint for rotary wing)
   * @param2 Empty
   * @param3 Empty
   * @param4 Empty
   * @param5 Latitude/X Latitude/X of goal
   * @param6 Longitude/Y Longitude/Y of goal
   * @param7 Altitude/Z Altitude/Z of goal
   */
  'NAV_SPLINE_WAYPOINT'                            = 82,

  /**
   * Takeoff from ground using VTOL mode, and transition to forward flight with specified heading. The
   * command should be ignored by vehicles that dont support both VTOL and fixed-wing flight
   * (multicopters, boats,etc.).
   *
   * @note has location and is destination
   *
   * @param1 Empty
   * @param2 Transition Heading Front transition heading.
   * @param3 Empty
   * @param4 Yaw Angle[deg] Yaw angle. NaN to use the current system yaw heading mode (e.g. yaw towards next waypoint, yaw to home, etc.).
   * @param5 Latitude Latitude
   * @param6 Longitude Longitude
   * @param7 Altitude[m] Altitude
   */
  'NAV_VTOL_TAKEOFF'                               = 84,

  /**
   * Land using VTOL mode
   *
   * @note has location and is destination
   *
   * @param1 Land Options Landing behaviour.
   * @param2 Empty
   * @param3 Approach Altitude[m] Approach altitude (with the same reference as the Altitude field). NaN if unspecified.
   * @param4 Yaw[deg] Yaw angle. NaN to use the current system yaw heading mode (e.g. yaw towards next waypoint, yaw to home, etc.).
   * @param5 Latitude Latitude
   * @param6 Longitude Longitude
   * @param7 Ground Altitude[m] Altitude (ground level) relative to the current coordinate frame. NaN to use system default landing altitude (ignore value).
   */
  'NAV_VTOL_LAND'                                  = 85,

  /**
   * hand control over to an external controller
   * @param1 Enable (min: 0, max: 1, increment: 1) On / Off (> 0.5f on)
   * @param2 Empty
   * @param3 Empty
   * @param4 Empty
   * @param5 Empty
   * @param6 Empty
   * @param7 Empty
   */
  'NAV_GUIDED_ENABLE'                              = 92,

  /**
   * Delay the next navigation command a number of seconds or until a specified time
   * @param1 Delay[s] (min: -1, increment: 1) Delay (-1 to enable time-of-day fields)
   * @param2 Hour (min: -1, max: 23, increment: 1) hour (24h format, UTC, -1 to ignore)
   * @param3 Minute (min: -1, max: 59, increment: 1) minute (24h format, UTC, -1 to ignore)
   * @param4 Second (min: -1, max: 59, increment: 1) second (24h format, UTC, -1 to ignore)
   * @param5 Empty
   * @param6 Empty
   * @param7 Empty
   */
  'NAV_DELAY'                                      = 93,

  /**
   * Descend and place payload. Vehicle moves to specified location, descends until it detects a hanging
   * payload has reached the ground, and then releases the payload. If ground is not detected before the
   * reaching the maximum descent value (param1), the command will complete without releasing the
   * payload.
   *
   * @note has location and is destination
   *
   * @param1 Max Descent[m] (min: 0) Maximum distance to descend.
   * @param2 Empty
   * @param3 Empty
   * @param4 Empty
   * @param5 Latitude Latitude
   * @param6 Longitude Longitude
   * @param7 Altitude[m] Altitude
   */
  'NAV_PAYLOAD_PLACE'                              = 94,

  /**
   * NOP - This command is only used to mark the upper limit of the NAV/ACTION commands in the
   * enumeration
   * @param1 Empty
   * @param2 Empty
   * @param3 Empty
   * @param4 Empty
   * @param5 Empty
   * @param6 Empty
   * @param7 Empty
   */
  'NAV_LAST'                                       = 95,

  /**
   * Delay mission state machine.
   * @param1 Delay[s] (min: 0) Delay
   * @param2 Empty
   * @param3 Empty
   * @param4 Empty
   * @param5 Empty
   * @param6 Empty
   * @param7 Empty
   */
  'CONDITION_DELAY'                                = 112,

  /**
   * Ascend/descend to target altitude at specified rate. Delay mission state machine until desired
   * altitude reached.
   *
   * @note is destination
   *
   * @param1 Rate[m/s] Descent / Ascend rate.
   * @param2 Empty
   * @param3 Empty
   * @param4 Empty
   * @param5 Empty
   * @param6 Empty
   * @param7 Altitude[m] Target Altitude
   */
  'CONDITION_CHANGE_ALT'                           = 113,

  /**
   * Delay mission state machine until within desired distance of next NAV point.
   * @param1 Distance[m] (min: 0) Distance.
   * @param2 Empty
   * @param3 Empty
   * @param4 Empty
   * @param5 Empty
   * @param6 Empty
   * @param7 Empty
   */
  'CONDITION_DISTANCE'                             = 114,

  /**
   * Reach a certain target angle.
   * @param1 Angle[deg] target angle, 0 is north
   * @param2 Angular Speed[deg/s] angular speed
   * @param3 Direction (min: -1, max: 1, increment: 2) direction: -1: counter clockwise, 1: clockwise
   * @param4 Relative (min: 0, max: 1, increment: 1) 0: absolute angle, 1: relative offset
   * @param5 Empty
   * @param6 Empty
   * @param7 Empty
   */
  'CONDITION_YAW'                                  = 115,

  /**
   * NOP - This command is only used to mark the upper limit of the CONDITION commands in the enumeration
   * @param1 Empty
   * @param2 Empty
   * @param3 Empty
   * @param4 Empty
   * @param5 Empty
   * @param6 Empty
   * @param7 Empty
   */
  'CONDITION_LAST'                                 = 159,

  /**
   * Set system mode.
   * @param1 Mode Mode
   * @param2 Custom Mode Custom mode - this is system specific, please refer to the individual autopilot specifications for details.
   * @param3 Custom Submode Custom sub mode - this is system specific, please refer to the individual autopilot specifications for details.
   * @param4 Empty
   * @param5 Empty
   * @param6 Empty
   * @param7 Empty
   */
  'DO_SET_MODE'                                    = 176,

  /**
   * Jump to the desired command in the mission list. Repeat this action only the specified number of
   * times
   * @param1 Number (min: 0, increment: 1) Sequence number
   * @param2 Repeat (min: 0, increment: 1) Repeat count
   * @param3 Empty
   * @param4 Empty
   * @param5 Empty
   * @param6 Empty
   * @param7 Empty
   */
  'DO_JUMP'                                        = 177,

  /**
   * Change speed and/or throttle set points.
   * @param1 Speed Type (min: 0, max: 3, increment: 1) Speed type (0=Airspeed, 1=Ground Speed, 2=Climb Speed, 3=Descent Speed)
   * @param2 Speed[m/s] (min: -1) Speed (-1 indicates no change)
   * @param3 Throttle[%] (min: -1) Throttle (-1 indicates no change)
   * @param4 Relative (min: 0, max: 1, increment: 1) 0: absolute, 1: relative
   * @param5 Empty
   * @param6 Empty
   * @param7 Empty
   */
  'DO_CHANGE_SPEED'                                = 178,

  /**
   * Changes the home location either to the current location or a specified location.
   *
   * @note has location
   *
   * @param1 Use Current (min: 0, max: 1, increment: 1) Use current (1=use current location, 0=use specified location)
   * @param2 Empty
   * @param3 Empty
   * @param4 Yaw[deg] Yaw angle. NaN to use default heading
   * @param5 Latitude Latitude
   * @param6 Longitude Longitude
   * @param7 Altitude[m] Altitude
   */
  'DO_SET_HOME'                                    = 179,

  /**
   * Set a system parameter. Caution! Use of this command requires knowledge of the numeric enumeration
   * value of the parameter.
   * @param1 Number (min: 0, increment: 1) Parameter number
   * @param2 Value Parameter value
   * @param3 Empty
   * @param4 Empty
   * @param5 Empty
   * @param6 Empty
   * @param7 Empty
   */
  'DO_SET_PARAMETER'                               = 180,

  /**
   * Set a relay to a condition.
   * @param1 Instance (min: 0, increment: 1) Relay instance number.
   * @param2 Setting (min: 0, increment: 1) Setting. (1=on, 0=off, others possible depending on system hardware)
   * @param3 Empty
   * @param4 Empty
   * @param5 Empty
   * @param6 Empty
   * @param7 Empty
   */
  'DO_SET_RELAY'                                   = 181,

  /**
   * Cycle a relay on and off for a desired number of cycles with a desired period.
   * @param1 Instance (min: 0, increment: 1) Relay instance number.
   * @param2 Count (min: 1, increment: 1) Cycle count.
   * @param3 Time[s] (min: 0) Cycle time.
   * @param4 Empty
   * @param5 Empty
   * @param6 Empty
   * @param7 Empty
   */
  'DO_REPEAT_RELAY'                                = 182,

  /**
   * Set a servo to a desired PWM value.
   * @param1 Instance (min: 0, increment: 1) Servo instance number.
   * @param2 PWM[us] (min: 0, increment: 1) Pulse Width Modulation.
   * @param3 Empty
   * @param4 Empty
   * @param5 Empty
   * @param6 Empty
   * @param7 Empty
   */
  'DO_SET_SERVO'                                   = 183,

  /**
   * Cycle a between its nominal setting and a desired PWM for a desired number of cycles with a desired
   * period.
   * @param1 Instance (min: 0, increment: 1) Servo instance number.
   * @param2 PWM[us] (min: 0, increment: 1) Pulse Width Modulation.
   * @param3 Count (min: 1, increment: 1) Cycle count.
   * @param4 Time[s] (min: 0) Cycle time.
   * @param5 Empty
   * @param6 Empty
   * @param7 Empty
   */
  'DO_REPEAT_SERVO'                                = 184,

  /**
   * Terminate flight immediately
   * @param1 Terminate (min: 0, max: 1, increment: 1) Flight termination activated if > 0.5
   * @param2 Empty
   * @param3 Empty
   * @param4 Empty
   * @param5 Empty
   * @param6 Empty
   * @param7 Empty
   */
  'DO_FLIGHTTERMINATION'                           = 185,

  /**
   * Change altitude set point.
   * @param1 Altitude[m] Altitude.
   * @param2 Frame Frame of new altitude.
   * @param3 Empty
   * @param4 Empty
   * @param5 Empty
   * @param6 Empty
   * @param7 Empty
   */
  'DO_CHANGE_ALTITUDE'                             = 186,

  /**
   * Sets actuators (e.g. servos) to a desired value. The actuator numbers are mapped to specific outputs
   * (e.g. on any MAIN or AUX PWM or UAVCAN) using a flight-stack specific mechanism (i.e. a parameter).
   * @param1 Actuator 1 (min: -1, max: 1) Actuator 1 value, scaled from [-1 to 1]. NaN to ignore.
   * @param2 Actuator 2 (min: -1, max: 1) Actuator 2 value, scaled from [-1 to 1]. NaN to ignore.
   * @param3 Actuator 3 (min: -1, max: 1) Actuator 3 value, scaled from [-1 to 1]. NaN to ignore.
   * @param4 Actuator 4 (min: -1, max: 1) Actuator 4 value, scaled from [-1 to 1]. NaN to ignore.
   * @param5 Actuator 5 (min: -1, max: 1) Actuator 5 value, scaled from [-1 to 1]. NaN to ignore.
   * @param6 Actuator 6 (min: -1, max: 1) Actuator 6 value, scaled from [-1 to 1]. NaN to ignore.
   * @param7 Index (min: 0, increment: 1) Index of actuator set (i.e if set to 1, Actuator 1 becomes Actuator 7)
   */
  'DO_SET_ACTUATOR'                                = 187,

  /**
   * Mission command to perform a landing. This is used as a marker in a mission to tell the autopilot
   * where a sequence of mission items that represents a landing starts. It may also be sent via a
   * COMMAND_LONG to trigger a landing, in which case the nearest (geographically) landing sequence in
   * the mission will be used. The Latitude/Longitude is optional, and may be set to 0 if not needed. If
   * specified then it will be used to help find the closest landing sequence.
   *
   * @note has location
   *
   * @param1 Empty
   * @param2 Empty
   * @param3 Empty
   * @param4 Empty
   * @param5 Latitude Latitude
   * @param6 Longitude Longitude
   * @param7 Empty
   */
  'DO_LAND_START'                                  = 189,

  /**
   * Mission command to perform a landing from a rally point.
   * @param1 Altitude[m] Break altitude
   * @param2 Speed[m/s] Landing speed
   * @param3 Empty
   * @param4 Empty
   * @param5 Empty
   * @param6 Empty
   * @param7 Empty
   */
  'DO_RALLY_LAND'                                  = 190,

  /**
   * Mission command to safely abort an autonomous landing.
   * @param1 Altitude[m] Altitude
   * @param2 Empty
   * @param3 Empty
   * @param4 Empty
   * @param5 Empty
   * @param6 Empty
   * @param7 Empty
   */
  'DO_GO_AROUND'                                   = 191,

  /**
   * Reposition the vehicle to a specific WGS84 global position.
   *
   * @note has location and is destination
   *
   * @param1 Speed[m/s] (min: -1) Ground speed, less than 0 (-1) for default
   * @param2 Bitmask Bitmask of option flags.
   * @param3 Reserved
   * @param4 Yaw[deg] Yaw heading. NaN to use the current system yaw heading mode (e.g. yaw towards next waypoint, yaw to home, etc.). For planes indicates loiter direction (0: clockwise, 1: counter clockwise)
   * @param5 Latitude Latitude
   * @param6 Longitude Longitude
   * @param7 Altitude[m] Altitude
   */
  'DO_REPOSITION'                                  = 192,

  /**
   * If in a GPS controlled position mode, hold the current position or continue.
   * @param1 Continue (min: 0, max: 1, increment: 1) 0: Pause current mission or reposition command, hold current position. 1: Continue mission. A VTOL capable vehicle should enter hover mode (multicopter and VTOL planes). A plane should loiter with the default loiter radius.
   * @param2 Reserved
   * @param3 Reserved
   * @param4 Reserved
   * @param5 Reserved
   * @param6 Reserved
   * @param7 Reserved
   */
  'DO_PAUSE_CONTINUE'                              = 193,

  /**
   * Set moving direction to forward or reverse.
   * @param1 Reverse (min: 0, max: 1, increment: 1) Direction (0=Forward, 1=Reverse)
   * @param2 Empty
   * @param3 Empty
   * @param4 Empty
   * @param5 Empty
   * @param6 Empty
   * @param7 Empty
   */
  'DO_SET_REVERSE'                                 = 194,

  /**
   * Sets the region of interest (ROI) to a location. This can then be used by the vehicle's control
   * system to control the vehicle attitude and the attitude of various sensors such as cameras. This
   * command can be sent to a gimbal manager but not to a gimbal device. A gimbal is not to react to this
   * message.
   *
   * @note has location
   *
   * @param1 Gimbal device ID Component ID of gimbal device to address (or 1-6 for non-MAVLink gimbal), 0 for all gimbal device components. Send command multiple times for more than one gimbal (but not all gimbals).
   * @param2 Empty
   * @param3 Empty
   * @param4 Empty
   * @param5 Latitude[degE7] Latitude of ROI location
   * @param6 Longitude[degE7] Longitude of ROI location
   * @param7 Altitude[m] Altitude of ROI location
   */
  'DO_SET_ROI_LOCATION'                            = 195,

  /**
   * Sets the region of interest (ROI) to be toward next waypoint, with optional pitch/roll/yaw offset.
   * This can then be used by the vehicle's control system to control the vehicle attitude and the
   * attitude of various sensors such as cameras. This command can be sent to a gimbal manager but not to
   * a gimbal device. A gimbal device is not to react to this message.
   * @param1 Gimbal device ID Component ID of gimbal device to address (or 1-6 for non-MAVLink gimbal), 0 for all gimbal device components. Send command multiple times for more than one gimbal (but not all gimbals).
   * @param2 Empty
   * @param3 Empty
   * @param4 Empty
   * @param5 Pitch Offset Pitch offset from next waypoint, positive pitching up
   * @param6 Roll Offset Roll offset from next waypoint, positive rolling to the right
   * @param7 Yaw Offset Yaw offset from next waypoint, positive yawing to the right
   */
  'DO_SET_ROI_WPNEXT_OFFSET'                       = 196,

  /**
   * Cancels any previous ROI command returning the vehicle/sensors to default flight characteristics.
   * This can then be used by the vehicle's control system to control the vehicle attitude and the
   * attitude of various sensors such as cameras. This command can be sent to a gimbal manager but not to
   * a gimbal device. A gimbal device is not to react to this message. After this command the gimbal
   * manager should go back to manual input if available, and otherwise assume a neutral position.
   * @param1 Gimbal device ID Component ID of gimbal device to address (or 1-6 for non-MAVLink gimbal), 0 for all gimbal device components. Send command multiple times for more than one gimbal (but not all gimbals).
   * @param2 Empty
   * @param3 Empty
   * @param4 Empty
   * @param5 Empty
   * @param6 Empty
   * @param7 Empty
   */
  'DO_SET_ROI_NONE'                                = 197,

  /**
   * Mount tracks system with specified system ID. Determination of target vehicle position may be done
   * with GLOBAL_POSITION_INT or any other means. This command can be sent to a gimbal manager but not to
   * a gimbal device. A gimbal device is not to react to this message.
   * @param1 System ID (min: 1, max: 255, increment: 1) System ID
   * @param2 Gimbal device ID Component ID of gimbal device to address (or 1-6 for non-MAVLink gimbal), 0 for all gimbal device components. Send command multiple times for more than one gimbal (but not all gimbals).
   */
  'DO_SET_ROI_SYSID'                               = 198,

  /**
   * Control onboard camera system.
   * @param1 ID (min: -1, increment: 1) Camera ID (-1 for all)
   * @param2 Transmission (min: 0, max: 2, increment: 1) Transmission: 0: disabled, 1: enabled compressed, 2: enabled raw
   * @param3 Interval[s] (min: 0) Transmission mode: 0: video stream, >0: single images every n seconds
   * @param4 Recording (min: 0, max: 2, increment: 1) Recording: 0: disabled, 1: enabled compressed, 2: enabled raw
   * @param5 Empty
   * @param6 Empty
   * @param7 Empty
   */
  'DO_CONTROL_VIDEO'                               = 200,

  /**
   * Sets the region of interest (ROI) for a sensor set or the vehicle itself. This can then be used by
   * the vehicle's control system to control the vehicle attitude and the attitude of various sensors
   * such as cameras.
   *
   * @note has location
   *
   * @param1 ROI Mode Region of interest mode.
   * @param2 WP Index (min: 0, increment: 1) Waypoint index/ target ID (depends on param 1).
   * @param3 ROI Index (min: 0, increment: 1) Region of interest index. (allows a vehicle to manage multiple ROI's)
   * @param4 Empty
   * @param5 MAV_ROI_WPNEXT: pitch offset from next waypoint, MAV_ROI_LOCATION: latitude
   * @param6 MAV_ROI_WPNEXT: roll offset from next waypoint, MAV_ROI_LOCATION: longitude
   * @param7 MAV_ROI_WPNEXT: yaw offset from next waypoint, MAV_ROI_LOCATION: altitude
   */
  'DO_SET_ROI'                                     = 201,

  /**
   * Configure digital camera. This is a fallback message for systems that have not yet implemented
   * PARAM_EXT_XXX messages and camera definition files (see
   * https://mavlink.io/en/services/camera_def.html ).
   * @param1 Mode (min: 0, increment: 1) Modes: P, TV, AV, M, Etc.
   * @param2 Shutter Speed (min: 0, increment: 1) Shutter speed: Divisor number for one second.
   * @param3 Aperture (min: 0) Aperture: F stop number.
   * @param4 ISO (min: 0, increment: 1) ISO number e.g. 80, 100, 200, Etc.
   * @param5 Exposure Exposure type enumerator.
   * @param6 Command Identity Command Identity.
   * @param7 Engine Cut-off[ds] (min: 0, increment: 1) Main engine cut-off time before camera trigger. (0 means no cut-off)
   */
  'DO_DIGICAM_CONFIGURE'                           = 202,

  /**
   * Control digital camera. This is a fallback message for systems that have not yet implemented
   * PARAM_EXT_XXX messages and camera definition files (see
   * https://mavlink.io/en/services/camera_def.html ).
   * @param1 Session Control Session control e.g. show/hide lens
   * @param2 Zoom Absolute Zoom's absolute position
   * @param3 Zoom Relative Zooming step value to offset zoom from the current position
   * @param4 Focus Focus Locking, Unlocking or Re-locking
   * @param5 Shoot Command Shooting Command
   * @param6 Command Identity Command Identity
   * @param7 Shot ID Test shot identifier. If set to 1, image will only be captured, but not counted towards internal frame count.
   */
  'DO_DIGICAM_CONTROL'                             = 203,

  /**
   * Mission command to configure a camera or antenna mount
   * @param1 Mode Mount operation mode
   * @param2 Stabilize Roll (min: 0, max: 1, increment: 1) stabilize roll? (1 = yes, 0 = no)
   * @param3 Stabilize Pitch (min: 0, max: 1, increment: 1) stabilize pitch? (1 = yes, 0 = no)
   * @param4 Stabilize Yaw (min: 0, max: 1, increment: 1) stabilize yaw? (1 = yes, 0 = no)
   * @param5 Roll Input Mode roll input (0 = angle body frame, 1 = angular rate, 2 = angle absolute frame)
   * @param6 Pitch Input Mode pitch input (0 = angle body frame, 1 = angular rate, 2 = angle absolute frame)
   * @param7 Yaw Input Mode yaw input (0 = angle body frame, 1 = angular rate, 2 = angle absolute frame)
   */
  'DO_MOUNT_CONFIGURE'                             = 204,

  /**
   * Mission command to control a camera or antenna mount
   * @param1 Pitch pitch depending on mount mode (degrees or degrees/second depending on pitch input).
   * @param2 Roll roll depending on mount mode (degrees or degrees/second depending on roll input).
   * @param3 Yaw yaw depending on mount mode (degrees or degrees/second depending on yaw input).
   * @param4 Altitude[m] altitude depending on mount mode.
   * @param5 Latitude latitude, set if appropriate mount mode.
   * @param6 Longitude longitude, set if appropriate mount mode.
   * @param7 Mode Mount mode.
   */
  'DO_MOUNT_CONTROL'                               = 205,

  /**
   * Mission command to set camera trigger distance for this flight. The camera is triggered each time
   * this distance is exceeded. This command can also be used to set the shutter integration time for the
   * camera.
   * @param1 Distance[m] (min: 0) Camera trigger distance. 0 to stop triggering.
   * @param2 Shutter[ms] (min: -1, increment: 1) Camera shutter integration time. -1 or 0 to ignore
   * @param3 Trigger (min: 0, max: 1, increment: 1) Trigger camera once immediately. (0 = no trigger, 1 = trigger)
   * @param4 Empty
   * @param5 Empty
   * @param6 Empty
   * @param7 Empty
   */
  'DO_SET_CAM_TRIGG_DIST'                          = 206,

  /**
   * Mission command to enable the geofence
   * @param1 Enable (min: 0, max: 2, increment: 1) enable? (0=disable, 1=enable, 2=disable_floor_only)
   * @param2 Empty
   * @param3 Empty
   * @param4 Empty
   * @param5 Empty
   * @param6 Empty
   * @param7 Empty
   */
  'DO_FENCE_ENABLE'                                = 207,

  /**
   * Mission item/command to release a parachute or enable/disable auto release.
   * @param1 Action Action
   * @param2 Empty
   * @param3 Empty
   * @param4 Empty
   * @param5 Empty
   * @param6 Empty
   * @param7 Empty
   */
  'DO_PARACHUTE'                                   = 208,

  /**
   * Command to perform motor test.
   * @param1 Instance (min: 1, increment: 1) Motor instance number (from 1 to max number of motors on the vehicle).
   * @param2 Throttle Type Throttle type (whether the Throttle Value in param3 is a percentage, PWM value, etc.)
   * @param3 Throttle Throttle value.
   * @param4 Timeout[s] (min: 0) Timeout between tests that are run in sequence.
   * @param5 Motor Count (min: 0, increment: 1) Motor count. Number of motors to test in sequence: 0/1=one motor, 2= two motors, etc. The Timeout (param4) is used between tests.
   * @param6 Test Order Motor test order.
   * @param7 Empty
   */
  'DO_MOTOR_TEST'                                  = 209,

  /**
   * Change to/from inverted flight.
   * @param1 Inverted (min: 0, max: 1, increment: 1) Inverted flight. (0=normal, 1=inverted)
   * @param2 Empty
   * @param3 Empty
   * @param4 Empty
   * @param5 Empty
   * @param6 Empty
   * @param7 Empty
   */
  'DO_INVERTED_FLIGHT'                             = 210,

  /**
   * Mission command to operate a gripper.
   * @param1 Instance (min: 1, increment: 1) Gripper instance number.
   * @param2 Action Gripper action to perform.
   * @param3 Empty
   * @param4 Empty
   * @param5 Empty
   * @param6 Empty
   * @param7 Empty
   */
  'DO_GRIPPER'                                     = 211,

  /**
   * Enable/disable autotune.
   * @param1 Enable (min: 0, max: 1, increment: 1) Enable (1: enable, 0:disable).
   * @param2 Axis Specify which axis are autotuned. 0 indicates autopilot default settings.
   * @param3 Empty.
   * @param4 Empty.
   * @param5 Empty.
   * @param6 Empty.
   * @param7 Empty.
   */
  'DO_AUTOTUNE_ENABLE'                             = 212,

  /**
   * Sets a desired vehicle turn angle and speed change.
   * @param1 Yaw[deg] Yaw angle to adjust steering by.
   * @param2 Speed[m/s] Speed.
   * @param3 Angle (min: 0, max: 1, increment: 1) Final angle. (0=absolute, 1=relative)
   * @param4 Empty
   * @param5 Empty
   * @param6 Empty
   * @param7 Empty
   */
  'NAV_SET_YAW_SPEED'                              = 213,

  /**
   * Mission command to set camera trigger interval for this flight. If triggering is enabled, the camera
   * is triggered each time this interval expires. This command can also be used to set the shutter
   * integration time for the camera.
   * @param1 Trigger Cycle[ms] (min: -1, increment: 1) Camera trigger cycle time. -1 or 0 to ignore.
   * @param2 Shutter Integration[ms] (min: -1, increment: 1) Camera shutter integration time. Should be less than trigger cycle time. -1 or 0 to ignore.
   * @param3 Empty
   * @param4 Empty
   * @param5 Empty
   * @param6 Empty
   * @param7 Empty
   */
  'DO_SET_CAM_TRIGG_INTERVAL'                      = 214,

  /**
   * Mission command to control a camera or antenna mount, using a quaternion as reference.
   * @param1 Q1 quaternion param q1, w (1 in null-rotation)
   * @param2 Q2 quaternion param q2, x (0 in null-rotation)
   * @param3 Q3 quaternion param q3, y (0 in null-rotation)
   * @param4 Q4 quaternion param q4, z (0 in null-rotation)
   * @param5 Empty
   * @param6 Empty
   * @param7 Empty
   */
  'DO_MOUNT_CONTROL_QUAT'                          = 220,

  /**
   * set id of master controller
   * @param1 System ID (min: 0, max: 255, increment: 1) System ID
   * @param2 Component ID (min: 0, max: 255, increment: 1) Component ID
   * @param3 Empty
   * @param4 Empty
   * @param5 Empty
   * @param6 Empty
   * @param7 Empty
   */
  'DO_GUIDED_MASTER'                               = 221,

  /**
   * Set limits for external control
   * @param1 Timeout[s] (min: 0) Timeout - maximum time that external controller will be allowed to control vehicle. 0 means no timeout.
   * @param2 Min Altitude[m] Altitude (MSL) min - if vehicle moves below this alt, the command will be aborted and the mission will continue. 0 means no lower altitude limit.
   * @param3 Max Altitude[m] Altitude (MSL) max - if vehicle moves above this alt, the command will be aborted and the mission will continue. 0 means no upper altitude limit.
   * @param4 Horiz. Move Limit[m] (min: 0) Horizontal move limit - if vehicle moves more than this distance from its location at the moment the command was executed, the command will be aborted and the mission will continue. 0 means no horizontal move limit.
   * @param5 Empty
   * @param6 Empty
   * @param7 Empty
   */
  'DO_GUIDED_LIMITS'                               = 222,

  /**
   * Control vehicle engine. This is interpreted by the vehicles engine controller to change the target
   * engine state. It is intended for vehicles with internal combustion engines
   * @param1 Start Engine (min: 0, max: 1, increment: 1) 0: Stop engine, 1:Start Engine
   * @param2 Cold Start (min: 0, max: 1, increment: 1) 0: Warm start, 1:Cold start. Controls use of choke where applicable
   * @param3 Height Delay[m] (min: 0) Height delay. This is for commanding engine start only after the vehicle has gained the specified height. Used in VTOL vehicles during takeoff to start engine after the aircraft is off the ground. Zero for no delay.
   * @param4 Empty
   * @param5 Empty
   * @param5 Empty
   * @param6 Empty
   * @param7 Empty
   */
  'DO_ENGINE_CONTROL'                              = 223,

  /**
   * Set the mission item with sequence number seq as current item. This means that the MAV will continue
   * to this mission item on the shortest path (not following the mission items in-between).
   * @param1 Number (min: 0, increment: 1) Mission sequence value to set
   * @param2 Empty
   * @param3 Empty
   * @param4 Empty
   * @param5 Empty
   * @param5 Empty
   * @param6 Empty
   * @param7 Empty
   */
  'DO_SET_MISSION_CURRENT'                         = 224,

  /**
   * NOP - This command is only used to mark the upper limit of the DO commands in the enumeration
   * @param1 Empty
   * @param2 Empty
   * @param3 Empty
   * @param4 Empty
   * @param5 Empty
   * @param6 Empty
   * @param7 Empty
   */
  'DO_LAST'                                        = 240,

  /**
   * Trigger calibration. This command will be only accepted if in pre-flight mode. Except for
   * Temperature Calibration, only one sensor should be set in a single message and all others should be
   * zero.
   * @param1 Gyro Temperature (min: 0, max: 3, increment: 1) 1: gyro calibration, 3: gyro temperature calibration
   * @param2 Magnetometer (min: 0, max: 1, increment: 1) 1: magnetometer calibration
   * @param3 Ground Pressure (min: 0, max: 1, increment: 1) 1: ground pressure calibration
   * @param4 Remote Control (min: 0, max: 1, increment: 1) 1: radio RC calibration, 2: RC trim calibration
   * @param5 Accelerometer (min: 0, max: 4, increment: 1) 1: accelerometer calibration, 2: board level calibration, 3: accelerometer temperature calibration, 4: simple accelerometer calibration
   * @param6 Compmot or Airspeed (min: 0, max: 2, increment: 1) 1: APM: compass/motor interference calibration (PX4: airspeed calibration, deprecated), 2: airspeed calibration
   * @param7 ESC or Baro (min: 0, max: 3, increment: 1) 1: ESC calibration, 3: barometer temperature calibration
   */
  'PREFLIGHT_CALIBRATION'                          = 241,

  /**
   * Set sensor offsets. This command will be only accepted if in pre-flight mode.
   * @param1 Sensor Type (min: 0, max: 6, increment: 1) Sensor to adjust the offsets for: 0: gyros, 1: accelerometer, 2: magnetometer, 3: barometer, 4: optical flow, 5: second magnetometer, 6: third magnetometer
   * @param2 X Offset X axis offset (or generic dimension 1), in the sensor's raw units
   * @param3 Y Offset Y axis offset (or generic dimension 2), in the sensor's raw units
   * @param4 Z Offset Z axis offset (or generic dimension 3), in the sensor's raw units
   * @param5 4th Dimension Generic dimension 4, in the sensor's raw units
   * @param6 5th Dimension Generic dimension 5, in the sensor's raw units
   * @param7 6th Dimension Generic dimension 6, in the sensor's raw units
   */
  'PREFLIGHT_SET_SENSOR_OFFSETS'                   = 242,

  /**
   * Trigger UAVCAN configuration (actuator ID assignment and direction mapping). Note that this maps to
   * the legacy UAVCAN v0 function UAVCAN_ENUMERATE, which is intended to be executed just once during
   * initial vehicle configuration (it is not a normal pre-flight command and has been poorly named).
   * @param1 Actuator ID 1: Trigger actuator ID assignment and direction mapping. 0: Cancel command.
   * @param2 Reserved
   * @param3 Reserved
   * @param4 Reserved
   * @param5 Reserved
   * @param6 Reserved
   * @param7 Reserved
   */
  'PREFLIGHT_UAVCAN'                               = 243,

  /**
   * Request storage of different parameter values and logs. This command will be only accepted if in
   * pre-flight mode.
   * @param1 Parameter Storage (min: 0, max: 3, increment: 1) Parameter storage: 0: READ FROM FLASH/EEPROM, 1: WRITE CURRENT TO FLASH/EEPROM, 2: Reset to defaults, 3: Reset sensor calibration parameter data to factory default (or firmware default if not available)
   * @param2 Mission Storage (min: 0, max: 2, increment: 1) Mission storage: 0: READ FROM FLASH/EEPROM, 1: WRITE CURRENT TO FLASH/EEPROM, 2: Reset to defaults
   * @param3 Logging Rate[Hz] (min: -1, increment: 1) Onboard logging: 0: Ignore, 1: Start default rate logging, -1: Stop logging, > 1: logging rate (e.g. set to 1000 for 1000 Hz logging)
   * @param4 Reserved
   * @param5 Empty
   * @param6 Empty
   * @param7 Empty
   */
  'PREFLIGHT_STORAGE'                              = 245,

  /**
   * Request the reboot or shutdown of system components.
   * @param1 Autopilot (min: 0, max: 3, increment: 1) 0: Do nothing for autopilot, 1: Reboot autopilot, 2: Shutdown autopilot, 3: Reboot autopilot and keep it in the bootloader until upgraded.
   * @param2 Companion (min: 0, max: 3, increment: 1) 0: Do nothing for onboard computer, 1: Reboot onboard computer, 2: Shutdown onboard computer, 3: Reboot onboard computer and keep it in the bootloader until upgraded.
   * @param3 Component action (min: 0, max: 3, increment: 1) 0: Do nothing for component, 1: Reboot component, 2: Shutdown component, 3: Reboot component and keep it in the bootloader until upgraded
   * @param4 Component ID (min: 0, max: 255, increment: 1) MAVLink Component ID targeted in param3 (0 for all components).
   * @param5 Reserved (set to 0)
   * @param6 Reserved (set to 0)
   * @param7 WIP: ID (e.g. camera ID -1 for all IDs)
   */
  'PREFLIGHT_REBOOT_SHUTDOWN'                      = 246,

  /**
   * Override current mission with command to pause mission, pause mission and move to position,
   * continue/resume mission. When param 1 indicates that the mission is paused (MAV_GOTO_DO_HOLD), param
   * 2 defines whether it holds in place or moves to another position.
   *
   * @note has location and is destination
   *
   * @param1 Continue MAV_GOTO_DO_HOLD: pause mission and either hold or move to specified position (depending on param2), MAV_GOTO_DO_CONTINUE: resume mission.
   * @param2 Position MAV_GOTO_HOLD_AT_CURRENT_POSITION: hold at current position, MAV_GOTO_HOLD_AT_SPECIFIED_POSITION: hold at specified position.
   * @param3 Frame Coordinate frame of hold point.
   * @param4 Yaw[deg] Desired yaw angle.
   * @param5 Latitude/X Latitude/X position.
   * @param6 Longitude/Y Longitude/Y position.
   * @param7 Altitude/Z Altitude/Z position.
   */
  'OVERRIDE_GOTO'                                  = 252,

  /**
   * Mission command to set a Camera Auto Mount Pivoting Oblique Survey (Replaces CAM_TRIGG_DIST for this
   * purpose). The camera is triggered each time this distance is exceeded, then the mount moves to the
   * next position. Params 4~6 set-up the angle limits and number of positions for oblique survey, where
   * mount-enabled vehicles automatically roll the camera between shots to emulate an oblique camera
   * setup (providing an increased HFOV). This command can also be used to set the shutter integration
   * time for the camera.
   * @param1 Distance[m] (min: 0) Camera trigger distance. 0 to stop triggering.
   * @param2 Shutter[ms] (min: 0, increment: 1) Camera shutter integration time. 0 to ignore
   * @param3 Min Interval[ms] (min: 0, max: 10000, increment: 1) The minimum interval in which the camera is capable of taking subsequent pictures repeatedly. 0 to ignore.
   * @param4 Positions (min: 2, increment: 1) Total number of roll positions at which the camera will capture photos (images captures spread evenly across the limits defined by param5).
   * @param5 Roll Angle[deg] (min: 0) Angle limits that the camera can be rolled to left and right of center.
   * @param6 Pitch Angle[deg] (min: -180, max: 180) Fixed pitch angle that the camera will hold in oblique mode if the mount is actuated in the pitch axis.
   * @param7 Empty
   */
  'OBLIQUE_SURVEY'                                 = 260,

  /**
   * xingangfei ext command take off
   * @param1 height[m] (min: 0, max: 2) front distance
   * @param2 Empty
   * @param3 Empty
   * @param4 Empty
   * @param5 Empty
   * @param6 Empty
   * @param7 Empty
   */
  'EXT_DRONE_TAKEOFF'                              = 270,

  /**
   * xingangfei ext command take off
   * @param1 land_mode (min: 0, max: 1) 0:normal land    1: qrcode land
   * @param2 landSpeed[cm/s] (min: 0, max: 200) land speed
   * @param3 Empty
   * @param4 Empty
   * @param5 Empty
   * @param6 Empty
   * @param7 Empty
   */
  'EXT_DRONE_LAND'                                 = 271,

  /**
   * xingangfei ext command move
   * @param1 direction (min: 0, max: 6) //front:03/behind:04/left:05/right:06/up:01/down:02
   * @param2 distance[cm] (min: 0, max: 1000) distance
   * @param2 speed[cm/s] (min: 0, max: 200) speed
   * @param4 Empty
   * @param5 Empty
   * @param6 Empty
   * @param7 Empty
   */
  'EXT_DRONE_MOVE'                                 = 272,

  /**
   * xingangfei ext command circle
   * @param1 direction (min: 0, max: 1) 0 Counterclockwise  1 clockwise
   * @param2 degrees (min: 0, max: 360) degrees
   * @param3 Empty
   * @param4 Empty
   * @param5 Empty
   * @param6 Empty
   * @param7 Empty
   */
  'EXT_DRONE_CIRCLE'                               = 273,

  /**
   * xinguangfei ext command waypoint
   * @param1 x Distance[m] (min: -1000, max: 1000) x Distance
   * @param2 y Distance[m] (min: -1000, max: 1000) y Distance
   * @param3 z Distance[m] (min: -200, max: 200) z Distance
   * @param4 Empty
   * @param5 Empty
   * @param6 Empty
   * @param7 Empty
   */
  'EXT_DRONE_WAYPOINT'                             = 274,

  /**
   * xinguangfei ext change speed
   * @param1 speed[cm/s] (min: 0, max: 200) speed
   * @param2 Empty
   * @param3 Empty
   * @param4 Empty
   * @param5 Empty
   * @param6 Empty
   * @param7 Empty
   */
  'EXT_DRONE_CHANGE_SPEED'                         = 275,

  /**
   * xinguangfei ext change speed
   * @param1 R (min: 0, max: 255) R
   * @param2 G (min: 0, max: 255) G
   * @param3 B (min: 0, max: 255) b
   * @param4 Breathe (min: 0, max: 1) Breathe
   * @param5 rainbow (min: 0, max: 1) rainbow
   * @param6 Empty
   * @param7 Empty
   */
  'EXT_DRONE_LIGHT_RGB'                            = 276,

  /**
   * xinguangfei ext set mode
   * @param1 mode 1:normal 2:track line 3:follow
   * @param2 Empty
   * @param3 Empty
   * @param4 Empty
   * @param5 Empty
   * @param6 Empty
   * @param7 Empty
   */
  'EXT_DRONE_SET_MODE'                             = 277,

  /**
   * xinguangfei ext set mode
   * @param1 L_L Min detection value of color block L channel
   * @param2 L_H Max detection value of color block L channel
   * @param3 A_L Min detection value of color block A channel
   * @param4 A_H Max detection value of color block A channel
   * @param5 B_L Min detection value of color block B channel
   * @param6 B_H Max detection value of color block B channel
   * @param7 Empty
   */
  'EXT_DRONE_VERSION_DETECT_MODE_SET'              = 278,

  /**
   * xinguangfei ext goto target loc
   * @param1 target_X (min: 0, max: 1) 0:fllowline 1:lock apritag
   * @param2 target_Y null
   * @param3 target_Z null
   * @param4 Empty
   * @param5 Empty
   * @param6 Empty
   * @param7 Empty
   */
  'EXT_DRONE_GOTO_CMD'                             = 279,

  /**
   * xinguangfei ext goto target loc
   * @param1 cmd null
   * @param2 Empty
   * @param3 Empty
   * @param4 Empty
   * @param5 Empty
   * @param6 Empty
   * @param7 Empty
   */
  'EXT_DRONE_OPEMMV_CMD'                           = 280,

  /**
   * xinguangfei ext set mode
   * @param1 Empty
   * @param2 Empty
   * @param3 Empty
   * @param4 Empty
   * @param5 Empty
   * @param6 Empty
   * @param7 Empty
   */
  'EXT_DRONE_TOTAL'                                = 290,

  /**
   * Actuator testing command. This is similar to MAV_CMD_DO_MOTOR_TEST but operates on the level of
   * output functions, i.e. it is possible to test Motor1 independent from which output it is configured
   * on. Autopilots typically refuse this command while armed.
   * @param1 Value (min: -1, max: 1) Output value: 1 means maximum positive output, 0 to center servos or minimum motor thrust (expected to spin), -1 for maximum negative (if not supported by the motors, i.e. motor is not reversible, smaller than 0 maps to NaN). And NaN maps to disarmed (stop the motors).
   * @param2 Timeout[s] (min: 0, max: 3) Timeout after which the test command expires and the output is restored to the previous value. A timeout has to be set for safety reasons. A timeout of 0 means to restore the previous value immediately.
   * @param5 Output Function Actuator Output function
   */
  'ACTUATOR_TEST'                                  = 310,

  /**
   * Actuator configuration command.
   * @param1 Configuration Actuator configuration action
   * @param5 Output Function Actuator Output function
   */
  'CONFIGURE_ACTUATOR'                             = 311,

  /**
   * Arms / Disarms a component
   * @param1 Arm (min: 0, max: 1, increment: 1) 0: disarm, 1: arm
   * @param2 Force (min: 0, max: 21196, increment: 21196) 0: arm-disarm unless prevented by safety checks (i.e. when landed), 21196: force arming/disarming (e.g. allow arming to override preflight checks and disarming in flight)
   */
  'COMPONENT_ARM_DISARM'                           = 400,

  /**
   * Instructs system to run pre-arm checks. This command should return MAV_RESULT_TEMPORARILY_REJECTED
   * in the case the system is armed, otherwise MAV_RESULT_ACCEPTED. Note that the return value from
   * executing this command does not indicate whether the vehicle is armable or not, just whether the
   * system has successfully run/is currently running the checks. The result of the checks is reflected
   * in the SYS_STATUS message.
   */
  'RUN_PREARM_CHECKS'                              = 401,

  /**
   * Turns illuminators ON/OFF. An illuminator is a light source that is used for lighting up dark areas
   * external to the sytstem: e.g. a torch or searchlight (as opposed to a light source for illuminating
   * the system itself, e.g. an indicator light).
   * @param1 Enable (min: 0, max: 1, increment: 1) 0: Illuminators OFF, 1: Illuminators ON
   */
  'ILLUMINATOR_ON_OFF'                             = 405,

  /**
   * Request the home position from the vehicle.
   * @param1 Reserved
   * @param2 Reserved
   * @param3 Reserved
   * @param4 Reserved
   * @param5 Reserved
   * @param6 Reserved
   * @param7 Reserved
   */
  'GET_HOME_POSITION'                              = 410,

  /**
   * Inject artificial failure for testing purposes. Note that autopilots should implement an additional
   * protection before accepting this command such as a specific param setting.
   * @param1 Failure unit The unit which is affected by the failure.
   * @param2 Failure type The type how the failure manifests itself.
   * @param3 Instance Instance affected by failure (0 to signal all).
   */
  'INJECT_FAILURE'                                 = 420,

  /**
   * Starts receiver pairing.
   * @param1 Spektrum 0:Spektrum.
   * @param2 RC Type RC type.
   */
  'START_RX_PAIR'                                  = 500,

  /**
   * Request the interval between messages for a particular MAVLink message ID. The receiver should ACK
   * the command and then emit its response in a MESSAGE_INTERVAL message.
   * @param1 Message ID (min: 0, max: 16777215, increment: 1) The MAVLink message ID
   */
  'GET_MESSAGE_INTERVAL'                           = 510,

  /**
   * Set the interval between messages for a particular MAVLink message ID. This interface replaces
   * REQUEST_DATA_STREAM.
   * @param1 Message ID (min: 0, max: 16777215, increment: 1) The MAVLink message ID
   * @param2 Interval[us] (min: -1, increment: 1) The interval between two messages. Set to -1 to disable and 0 to request default rate.
   * @param7 Response Target (min: 0, max: 2, increment: 1) Target address of message stream (if message has target address fields). 0: Flight-stack default (recommended), 1: address of requestor, 2: broadcast.
   */
  'SET_MESSAGE_INTERVAL'                           = 511,

  /**
   * Request the target system(s) emit a single instance of a specified message (i.e. a "one-shot"
   * version of MAV_CMD_SET_MESSAGE_INTERVAL).
   * @param1 Message ID (min: 0, max: 16777215, increment: 1) The MAVLink message ID of the requested message.
   * @param2 Req Param 1 Use for index ID, if required. Otherwise, the use of this parameter (if any) must be defined in the requested message. By default assumed not used (0).
   * @param3 Req Param 2 The use of this parameter (if any), must be defined in the requested message. By default assumed not used (0).
   * @param4 Req Param 3 The use of this parameter (if any), must be defined in the requested message. By default assumed not used (0).
   * @param5 Req Param 4 The use of this parameter (if any), must be defined in the requested message. By default assumed not used (0).
   * @param6 Req Param 5 The use of this parameter (if any), must be defined in the requested message. By default assumed not used (0).
   * @param7 Response Target (min: 0, max: 2, increment: 1) Target address for requested message (if message has target address fields). 0: Flight-stack default, 1: address of requestor, 2: broadcast.
   */
  'REQUEST_MESSAGE'                                = 512,

  /**
   * Request MAVLink protocol version compatibility. All receivers should ACK the command and then emit
   * their capabilities in an PROTOCOL_VERSION message
   * @param1 Protocol (min: 0, max: 1, increment: 1) 1: Request supported protocol versions by all nodes on the network
   * @param2 Reserved (all remaining params)
   */
  'REQUEST_PROTOCOL_VERSION'                       = 519,

  /**
   * Request autopilot capabilities. The receiver should ACK the command and then emit its capabilities
   * in an AUTOPILOT_VERSION message
   * @param1 Version (min: 0, max: 1, increment: 1) 1: Request autopilot version
   * @param2 Reserved (all remaining params)
   */
  'REQUEST_AUTOPILOT_CAPABILITIES'                 = 520,

  /**
   * Request camera information (CAMERA_INFORMATION).
   * @param1 Capabilities (min: 0, max: 1, increment: 1) 0: No action 1: Request camera capabilities
   * @param2 Reserved (all remaining params)
   */
  'REQUEST_CAMERA_INFORMATION'                     = 521,

  /**
   * Request camera settings (CAMERA_SETTINGS).
   * @param1 Settings (min: 0, max: 1, increment: 1) 0: No Action 1: Request camera settings
   * @param2 Reserved (all remaining params)
   */
  'REQUEST_CAMERA_SETTINGS'                        = 522,

  /**
   * Request storage information (STORAGE_INFORMATION). Use the command's target_component to target a
   * specific component's storage.
   * @param1 Storage ID (min: 0, increment: 1) Storage ID (0 for all, 1 for first, 2 for second, etc.)
   * @param2 Information (min: 0, max: 1, increment: 1) 0: No Action 1: Request storage information
   * @param3 Reserved (all remaining params)
   */
  'REQUEST_STORAGE_INFORMATION'                    = 525,

  /**
   * Format a storage medium. Once format is complete, a STORAGE_INFORMATION message is sent. Use the
   * command's target_component to target a specific component's storage.
   * @param1 Storage ID (min: 0, increment: 1) Storage ID (1 for first, 2 for second, etc.)
   * @param2 Format (min: 0, max: 1, increment: 1) Format storage (and reset image log). 0: No action 1: Format storage
   * @param3 Reset Image Log (min: 0, max: 1, increment: 1) Reset Image Log (without formatting storage medium). This will reset CAMERA_CAPTURE_STATUS.image_count and CAMERA_IMAGE_CAPTURED.image_index. 0: No action 1: Reset Image Log
   * @param4 Reserved (all remaining params)
   */
  'STORAGE_FORMAT'                                 = 526,

  /**
   * Request camera capture status (CAMERA_CAPTURE_STATUS)
   * @param1 Capture Status (min: 0, max: 1, increment: 1) 0: No Action 1: Request camera capture status
   * @param2 Reserved (all remaining params)
   */
  'REQUEST_CAMERA_CAPTURE_STATUS'                  = 527,

  /**
   * Request flight information (FLIGHT_INFORMATION)
   * @param1 Flight Information (min: 0, max: 1, increment: 1) 1: Request flight information
   * @param2 Reserved (all remaining params)
   */
  'REQUEST_FLIGHT_INFORMATION'                     = 528,

  /**
   * Reset all camera settings to Factory Default
   * @param1 Reset (min: 0, max: 1, increment: 1) 0: No Action 1: Reset all settings
   * @param2 Reserved (all remaining params)
   */
  'RESET_CAMERA_SETTINGS'                          = 529,

  /**
   * Set camera running mode. Use NaN for reserved values. GCS will send a
   * MAV_CMD_REQUEST_VIDEO_STREAM_STATUS command after a mode change if the camera supports video
   * streaming.
   * @param1 Reserved (Set to 0)
   * @param2 Camera Mode Camera mode
   */
  'SET_CAMERA_MODE'                                = 530,

  /**
   * Set camera zoom. Camera must respond with a CAMERA_SETTINGS message (on success).
   * @param1 Zoom Type Zoom type
   * @param2 Zoom Value Zoom value. The range of valid values depend on the zoom type.
   */
  'SET_CAMERA_ZOOM'                                = 531,

  /**
   * Set camera focus. Camera must respond with a CAMERA_SETTINGS message (on success).
   * @param1 Focus Type Focus type
   * @param2 Focus Value Focus value
   */
  'SET_CAMERA_FOCUS'                               = 532,

  /**
   * Set that a particular storage is the preferred location for saving photos, videos, and/or other
   * media (e.g. to set that an SD card is used for storing videos).
 There can only be one preferred
   * save location for each particular media type: setting a media usage flag will clear/reset that same
   * flag if set on any other storage.
 If no flag is set the system should use its default storage.
 A
   * target system can choose to always use default storage, in which case it should ACK the command with
   * MAV_RESULT_UNSUPPORTED.
 A target system can choose to not allow a particular storage to be set as
   * preferred storage, in which case it should ACK the command with MAV_RESULT_DENIED.
   * @param1 Storage ID (min: 0, increment: 1) Storage ID (1 for first, 2 for second, etc.)
   * @param2 Usage Usage flags
   */
  'SET_STORAGE_USAGE'                              = 533,

  /**
   * Tagged jump target. Can be jumped to with MAV_CMD_DO_JUMP_TAG.
   * @param1 Tag (min: 0, increment: 1) Tag.
   */
  'JUMP_TAG'                                       = 600,

  /**
   * Jump to the matching tag in the mission list. Repeat this action for the specified number of times.
   * A mission should contain a single matching tag for each jump. If this is not the case then a jump to
   * a missing tag should complete the mission, and a jump where there are multiple matching tags should
   * always select the one with the lowest mission sequence number.
   * @param1 Tag (min: 0, increment: 1) Target tag to jump to.
   * @param2 Repeat (min: 0, increment: 1) Repeat count.
   */
  'DO_JUMP_TAG'                                    = 601,

  /**
   * High level setpoint to be sent to a gimbal manager to set a gimbal attitude. It is possible to set
   * combinations of the values below. E.g. an angle as well as a desired angular rate can be used to get
   * to this angle at a certain angular rate, or an angular rate only will result in continuous turning.
   * NaN is to be used to signal unset. Note: a gimbal is never to react to this command but only the
   * gimbal manager.
   * @param1 Pitch angle[deg] (min: -180, max: 180) Pitch angle (positive to pitch up, relative to vehicle for FOLLOW mode, relative to world horizon for LOCK mode).
   * @param2 Yaw angle[deg] (min: -180, max: 180) Yaw angle (positive to yaw to the right, relative to vehicle for FOLLOW mode, absolute to North for LOCK mode).
   * @param3 Pitch rate[deg/s] Pitch rate (positive to pitch up).
   * @param4 Yaw rate[deg/s] Yaw rate (positive to yaw to the right).
   * @param5 Gimbal manager flags Gimbal manager flags to use.
   * @param7 Gimbal device ID Component ID of gimbal device to address (or 1-6 for non-MAVLink gimbal), 0 for all gimbal device components. Send command multiple times for more than one gimbal (but not all gimbals).
   */
  'DO_GIMBAL_MANAGER_PITCHYAW'                     = 1000,

  /**
   * Gimbal configuration to set which sysid/compid is in primary and secondary control.
   * @param1 sysid primary control Sysid for primary control (0: no one in control, -1: leave unchanged, -2: set itself in control (for missions where the own sysid is still unknown), -3: remove control if currently in control).
   * @param2 compid primary control Compid for primary control (0: no one in control, -1: leave unchanged, -2: set itself in control (for missions where the own sysid is still unknown), -3: remove control if currently in control).
   * @param3 sysid secondary control Sysid for secondary control (0: no one in control, -1: leave unchanged, -2: set itself in control (for missions where the own sysid is still unknown), -3: remove control if currently in control).
   * @param4 compid secondary control Compid for secondary control (0: no one in control, -1: leave unchanged, -2: set itself in control (for missions where the own sysid is still unknown), -3: remove control if currently in control).
   * @param7 Gimbal device ID Component ID of gimbal device to address (or 1-6 for non-MAVLink gimbal), 0 for all gimbal device components. Send command multiple times for more than one gimbal (but not all gimbals).
   */
  'DO_GIMBAL_MANAGER_CONFIGURE'                    = 1001,

  /**
   * Gimbal configuration to set which sysid/compid is in primary and secondary control.
   * @param1 sysid primary control Sysid for primary control (0: no one in control, -1: leave unchanged, -2: set itself in control (for missions where the own sysid is still unknown), -3: remove control if currently in control).
   * @param2 compid primary control Compid for primary control (0: no one in control, -1: leave unchanged, -2: set itself in control (for missions where the own sysid is still unknown), -3: remove control if currently in control).
   * @param3 sysid secondary control Sysid for secondary control (0: no one in control, -1: leave unchanged, -2: set itself in control (for missions where the own sysid is still unknown), -3: remove control if currently in control).
   * @param4 compid secondary control Compid for secondary control (0: no one in control, -1: leave unchanged, -2: set itself in control (for missions where the own sysid is still unknown), -3: remove control if currently in control).
   * @param7 Gimbal device ID Component ID of gimbal device to address (or 1-6 for non-MAVLink gimbal), 0 for all gimbal device components. Send command multiple times for more than one gimbal (but not all gimbals).
   */
  'DO_GIMBAL_MANAGER_LASER'                        = 1002,

  /**
   * Gimbal configuration to set which sysid/compid is in primary and secondary control.
   * @param1 sysid primary control Sysid for primary control (0: no one in control, -1: leave unchanged, -2: set itself in control (for missions where the own sysid is still unknown), -3: remove control if currently in control).
   * @param2 compid primary control Compid for primary control (0: no one in control, -1: leave unchanged, -2: set itself in control (for missions where the own sysid is still unknown), -3: remove control if currently in control).
   * @param3 sysid secondary control Sysid for secondary control (0: no one in control, -1: leave unchanged, -2: set itself in control (for missions where the own sysid is still unknown), -3: remove control if currently in control).
   * @param4 compid secondary control Compid for secondary control (0: no one in control, -1: leave unchanged, -2: set itself in control (for missions where the own sysid is still unknown), -3: remove control if currently in control).
   * @param7 Gimbal device ID Component ID of gimbal device to address (or 1-6 for non-MAVLink gimbal), 0 for all gimbal device components. Send command multiple times for more than one gimbal (but not all gimbals).
   */
  'DO_GIMBAL_MANAGER_THERMAL'                      = 1003,

  /**
   * Gimbal configuration to set which sysid/compid is in primary and secondary control.
   * @param1 sysid primary control Sysid for primary control (0: no one in control, -1: leave unchanged, -2: set itself in control (for missions where the own sysid is still unknown), -3: remove control if currently in control).
   * @param2 compid primary control Compid for primary control (0: no one in control, -1: leave unchanged, -2: set itself in control (for missions where the own sysid is still unknown), -3: remove control if currently in control).
   * @param3 sysid secondary control Sysid for secondary control (0: no one in control, -1: leave unchanged, -2: set itself in control (for missions where the own sysid is still unknown), -3: remove control if currently in control).
   * @param4 compid secondary control Compid for secondary control (0: no one in control, -1: leave unchanged, -2: set itself in control (for missions where the own sysid is still unknown), -3: remove control if currently in control).
   * @param7 Gimbal device ID Component ID of gimbal device to address (or 1-6 for non-MAVLink gimbal), 0 for all gimbal device components. Send command multiple times for more than one gimbal (but not all gimbals).
   */
  'DO_GIMBAL_MANAGER_CENTER'                       = 1004,

  /**
   * Gimbal configuration to set which sysid/compid is in primary and secondary control.
   * @param1 sysid primary control Sysid for primary control (0: no one in control, -1: leave unchanged, -2: set itself in control (for missions where the own sysid is still unknown), -3: remove control if currently in control).
   * @param2 compid primary control Compid for primary control (0: no one in control, -1: leave unchanged, -2: set itself in control (for missions where the own sysid is still unknown), -3: remove control if currently in control).
   * @param3 sysid secondary control Sysid for secondary control (0: no one in control, -1: leave unchanged, -2: set itself in control (for missions where the own sysid is still unknown), -3: remove control if currently in control).
   * @param4 compid secondary control Compid for secondary control (0: no one in control, -1: leave unchanged, -2: set itself in control (for missions where the own sysid is still unknown), -3: remove control if currently in control).
   * @param7 Gimbal device ID Component ID of gimbal device to address (or 1-6 for non-MAVLink gimbal), 0 for all gimbal device components. Send command multiple times for more than one gimbal (but not all gimbals).
   */
  'DO_GIMBAL_MANAGER_UP'                           = 1005,

  /**
   * Gimbal configuration to set which sysid/compid is in primary and secondary control.
   * @param1 sysid primary control Sysid for primary control (0: no one in control, -1: leave unchanged, -2: set itself in control (for missions where the own sysid is still unknown), -3: remove control if currently in control).
   * @param2 compid primary control Compid for primary control (0: no one in control, -1: leave unchanged, -2: set itself in control (for missions where the own sysid is still unknown), -3: remove control if currently in control).
   * @param3 sysid secondary control Sysid for secondary control (0: no one in control, -1: leave unchanged, -2: set itself in control (for missions where the own sysid is still unknown), -3: remove control if currently in control).
   * @param4 compid secondary control Compid for secondary control (0: no one in control, -1: leave unchanged, -2: set itself in control (for missions where the own sysid is still unknown), -3: remove control if currently in control).
   * @param7 Gimbal device ID Component ID of gimbal device to address (or 1-6 for non-MAVLink gimbal), 0 for all gimbal device components. Send command multiple times for more than one gimbal (but not all gimbals).
   */
  'DO_GIMBAL_MANAGER_DOWN'                         = 1006,

  /**
   * Gimbal configuration to set which sysid/compid is in primary and secondary control.
   * @param1 sysid primary control Sysid for primary control (0: no one in control, -1: leave unchanged, -2: set itself in control (for missions where the own sysid is still unknown), -3: remove control if currently in control).
   * @param2 compid primary control Compid for primary control (0: no one in control, -1: leave unchanged, -2: set itself in control (for missions where the own sysid is still unknown), -3: remove control if currently in control).
   * @param3 sysid secondary control Sysid for secondary control (0: no one in control, -1: leave unchanged, -2: set itself in control (for missions where the own sysid is still unknown), -3: remove control if currently in control).
   * @param4 compid secondary control Compid for secondary control (0: no one in control, -1: leave unchanged, -2: set itself in control (for missions where the own sysid is still unknown), -3: remove control if currently in control).
   * @param7 Gimbal device ID Component ID of gimbal device to address (or 1-6 for non-MAVLink gimbal), 0 for all gimbal device components. Send command multiple times for more than one gimbal (but not all gimbals).
   */
  'DO_GIMBAL_START_TARGET_DETECTION'               = 1007,

  /**
   * Gimbal configuration to set which sysid/compid is in primary and secondary control.
   * @param1 sysid primary control Sysid for primary control (0: no one in control, -1: leave unchanged, -2: set itself in control (for missions where the own sysid is still unknown), -3: remove control if currently in control).
   * @param2 compid primary control Compid for primary control (0: no one in control, -1: leave unchanged, -2: set itself in control (for missions where the own sysid is still unknown), -3: remove control if currently in control).
   * @param3 sysid secondary control Sysid for secondary control (0: no one in control, -1: leave unchanged, -2: set itself in control (for missions where the own sysid is still unknown), -3: remove control if currently in control).
   * @param4 compid secondary control Compid for secondary control (0: no one in control, -1: leave unchanged, -2: set itself in control (for missions where the own sysid is still unknown), -3: remove control if currently in control).
   * @param7 Gimbal device ID Component ID of gimbal device to address (or 1-6 for non-MAVLink gimbal), 0 for all gimbal device components. Send command multiple times for more than one gimbal (but not all gimbals).
   */
  'DO_GIMBAL_STOP_TARGET_DETECTION'                = 1008,

  /**
   * Gimbal configuration to set which sysid/compid is in primary and secondary control.
   * @param1 sysid primary control Sysid for primary control (0: no one in control, -1: leave unchanged, -2: set itself in control (for missions where the own sysid is still unknown), -3: remove control if currently in control).
   * @param2 compid primary control Compid for primary control (0: no one in control, -1: leave unchanged, -2: set itself in control (for missions where the own sysid is still unknown), -3: remove control if currently in control).
   * @param3 sysid secondary control Sysid for secondary control (0: no one in control, -1: leave unchanged, -2: set itself in control (for missions where the own sysid is still unknown), -3: remove control if currently in control).
   * @param4 compid secondary control Compid for secondary control (0: no one in control, -1: leave unchanged, -2: set itself in control (for missions where the own sysid is still unknown), -3: remove control if currently in control).
   * @param7 Gimbal device ID Component ID of gimbal device to address (or 1-6 for non-MAVLink gimbal), 0 for all gimbal device components. Send command multiple times for more than one gimbal (but not all gimbals).
   */
  'DO_GIMBAL_ENABLE_OSD'                           = 1009,

  /**
   * Gimbal configuration to set which sysid/compid is in primary and secondary control.
   * @param1 sysid primary control Sysid for primary control (0: no one in control, -1: leave unchanged, -2: set itself in control (for missions where the own sysid is still unknown), -3: remove control if currently in control).
   * @param2 compid primary control Compid for primary control (0: no one in control, -1: leave unchanged, -2: set itself in control (for missions where the own sysid is still unknown), -3: remove control if currently in control).
   * @param3 sysid secondary control Sysid for secondary control (0: no one in control, -1: leave unchanged, -2: set itself in control (for missions where the own sysid is still unknown), -3: remove control if currently in control).
   * @param4 compid secondary control Compid for secondary control (0: no one in control, -1: leave unchanged, -2: set itself in control (for missions where the own sysid is still unknown), -3: remove control if currently in control).
   * @param7 Gimbal device ID Component ID of gimbal device to address (or 1-6 for non-MAVLink gimbal), 0 for all gimbal device components. Send command multiple times for more than one gimbal (but not all gimbals).
   */
  'DO_GIMBAL_DISABLE_OSD'                          = 1010,

  /**
   * Start image capture sequence. Sends CAMERA_IMAGE_CAPTURED after each capture. Use NaN for reserved
   * values.
   * @param1 Reserved (Set to 0)
   * @param2 Interval[s] (min: 0) Desired elapsed time between two consecutive pictures (in seconds). Minimum values depend on hardware (typically greater than 2 seconds).
   * @param3 Total Images (min: 0, increment: 1) Total number of images to capture. 0 to capture forever/until MAV_CMD_IMAGE_STOP_CAPTURE.
   * @param4 Sequence Number (min: 1, increment: 1) Capture sequence number starting from 1. This is only valid for single-capture (param3 == 1), otherwise set to 0. Increment the capture ID for each capture command to prevent double captures when a command is re-transmitted.
   */
  'IMAGE_START_CAPTURE'                            = 2000,

  /**
   * Stop image capture sequence Use NaN for reserved values.
   * @param1 Reserved (Set to 0)
   */
  'IMAGE_STOP_CAPTURE'                             = 2001,

  /**
   * Re-request a CAMERA_IMAGE_CAPTURED message.
   * @param1 Number (min: 0, increment: 1) Sequence number for missing CAMERA_IMAGE_CAPTURED message
   */
  'REQUEST_CAMERA_IMAGE_CAPTURE'                   = 2002,

  /**
   * Enable or disable on-board camera triggering system.
   * @param1 Enable (min: -1, max: 1, increment: 1) Trigger enable/disable (0 for disable, 1 for start), -1 to ignore
   * @param2 Reset (min: -1, max: 1, increment: 1) 1 to reset the trigger sequence, -1 or 0 to ignore
   * @param3 Pause (min: -1, max: 1, increment: 2) 1 to pause triggering, but without switching the camera off or retracting it. -1 to ignore
   */
  'DO_TRIGGER_CONTROL'                             = 2003,

  /**
   * If the camera supports point visual tracking (CAMERA_CAP_FLAGS_HAS_TRACKING_POINT is set), this
   * command allows to initiate the tracking.
   * @param1 Point x (min: 0, max: 1) Point to track x value (normalized 0..1, 0 is left, 1 is right).
   * @param2 Point y (min: 0, max: 1) Point to track y value (normalized 0..1, 0 is top, 1 is bottom).
   * @param3 Radius (min: 0, max: 1) Point radius (normalized 0..1, 0 is image left, 1 is image right).
   */
  'CAMERA_TRACK_POINT'                             = 2004,

  /**
   * If the camera supports rectangle visual tracking (CAMERA_CAP_FLAGS_HAS_TRACKING_RECTANGLE is set),
   * this command allows to initiate the tracking.
   * @param1 Top left corner x (min: 0, max: 1) Top left corner of rectangle x value (normalized 0..1, 0 is left, 1 is right).
   * @param2 Top left corner y (min: 0, max: 1) Top left corner of rectangle y value (normalized 0..1, 0 is top, 1 is bottom).
   * @param3 Bottom right corner x (min: 0, max: 1) Bottom right corner of rectangle x value (normalized 0..1, 0 is left, 1 is right).
   * @param4 Bottom right corner y (min: 0, max: 1) Bottom right corner of rectangle y value (normalized 0..1, 0 is top, 1 is bottom).
   */
  'CAMERA_TRACK_RECTANGLE'                         = 2005,

  /**
   * Stops ongoing tracking.
   */
  'CAMERA_STOP_TRACKING'                           = 2010,

  /**
   * Starts video capture (recording).
   * @param1 Stream ID (min: 0, increment: 1) Video Stream ID (0 for all streams)
   * @param2 Status Frequency[Hz] (min: 0) Frequency CAMERA_CAPTURE_STATUS messages should be sent while recording (0 for no messages, otherwise frequency)
   */
  'VIDEO_START_CAPTURE'                            = 2500,

  /**
   * Stop the current video capture (recording).
   * @param1 Stream ID (min: 0, increment: 1) Video Stream ID (0 for all streams)
   */
  'VIDEO_STOP_CAPTURE'                             = 2501,

  /**
   * Start video streaming
   * @param1 Stream ID (min: 0, increment: 1) Video Stream ID (0 for all streams, 1 for first, 2 for second, etc.)
   */
  'VIDEO_START_STREAMING'                          = 2502,

  /**
   * Stop the given video stream
   * @param1 Stream ID (min: 0, increment: 1) Video Stream ID (0 for all streams, 1 for first, 2 for second, etc.)
   */
  'VIDEO_STOP_STREAMING'                           = 2503,

  /**
   * Request video stream information (VIDEO_STREAM_INFORMATION)
   * @param1 Stream ID (min: 0, increment: 1) Video Stream ID (0 for all streams, 1 for first, 2 for second, etc.)
   */
  'REQUEST_VIDEO_STREAM_INFORMATION'               = 2504,

  /**
   * Request video stream status (VIDEO_STREAM_STATUS)
   * @param1 Stream ID (min: 0, increment: 1) Video Stream ID (0 for all streams, 1 for first, 2 for second, etc.)
   */
  'REQUEST_VIDEO_STREAM_STATUS'                    = 2505,

  /**
   * Request to start streaming logging data over MAVLink (see also LOGGING_DATA message)
   * @param1 Format (min: 0, increment: 1) Format: 0: ULog
   * @param2 Reserved (set to 0)
   * @param3 Reserved (set to 0)
   * @param4 Reserved (set to 0)
   * @param5 Reserved (set to 0)
   * @param6 Reserved (set to 0)
   * @param7 Reserved (set to 0)
   */
  'LOGGING_START'                                  = 2510,

  /**
   * Request to stop streaming log data over MAVLink
   * @param1 Reserved (set to 0)
   * @param2 Reserved (set to 0)
   * @param3 Reserved (set to 0)
   * @param4 Reserved (set to 0)
   * @param5 Reserved (set to 0)
   * @param6 Reserved (set to 0)
   * @param7 Reserved (set to 0)
   */
  'LOGGING_STOP'                                   = 2511,

  /**
   * @param1 Landing Gear ID (min: -1, increment: 1) Landing gear ID (default: 0, -1 for all)
   * @param2 Landing Gear Position Landing gear position (Down: 0, Up: 1, NaN for no change)
   */
  'AIRFRAME_CONFIGURATION'                         = 2520,

  /**
   * Request to start/stop transmitting over the high latency telemetry
   * @param1 Enable (min: 0, max: 1, increment: 1) Control transmission over high latency telemetry (0: stop, 1: start)
   * @param2 Empty
   * @param3 Empty
   * @param4 Empty
   * @param5 Empty
   * @param6 Empty
   * @param7 Empty
   */
  'CONTROL_HIGH_LATENCY'                           = 2600,

  /**
   * Create a panorama at the current position
   * @param1 Horizontal Angle[deg] Viewing angle horizontal of the panorama (+- 0.5 the total angle)
   * @param2 Vertical Angle[deg] Viewing angle vertical of panorama.
   * @param3 Horizontal Speed[deg/s] Speed of the horizontal rotation.
   * @param4 Vertical Speed[deg/s] Speed of the vertical rotation.
   */
  'PANORAMA_CREATE'                                = 2800,

  /**
   * Request VTOL transition
   * @param1 State The target VTOL state. For normal transitions, only MAV_VTOL_STATE_MC and MAV_VTOL_STATE_FW can be used.
   * @param2 Immediate Force immediate transition to the specified MAV_VTOL_STATE. 1: Force immediate, 0: normal transition. Can be used, for example, to trigger an emergency "Quadchute". Caution: Can be dangerous/damage vehicle, depending on autopilot implementation of this command.
   */
  'DO_VTOL_TRANSITION'                             = 3000,

  /**
   * Request authorization to arm the vehicle to a external entity, the arm authorizer is responsible to
   * request all data that is needs from the vehicle before authorize or deny the request. If approved
   * the progress of command_ack message should be set with period of time that this authorization is
   * valid in seconds or in case it was denied it should be set with one of the reasons in
   * ARM_AUTH_DENIED_REASON.
   * @param1 System ID (min: 0, max: 255, increment: 1) Vehicle system id, this way ground station can request arm authorization on behalf of any vehicle
   */
  'ARM_AUTHORIZATION_REQUEST'                      = 3001,

  /**
   * This command sets the submode to standard guided when vehicle is in guided mode. The vehicle holds
   * position and altitude and the user can input the desired velocities along all three axes.
   */
  'SET_GUIDED_SUBMODE_STANDARD'                    = 4000,

  /**
   * This command sets submode circle when vehicle is in guided mode. Vehicle flies along a circle facing
   * the center of the circle. The user can input the velocity along the circle and change the radius. If
   * no input is given the vehicle will hold position.
   *
   * @note has location
   *
   * @param1 Radius[m] Radius of desired circle in CIRCLE_MODE
   * @param2 User defined
   * @param3 User defined
   * @param4 User defined
   * @param5 Latitude[degE7] Target latitude of center of circle in CIRCLE_MODE
   * @param6 Longitude[degE7] Target longitude of center of circle in CIRCLE_MODE
   */
  'SET_GUIDED_SUBMODE_CIRCLE'                      = 4001,

  /**
   * Delay mission state machine until gate has been reached.
   *
   * @note has location and is destination
   *
   * @param1 Geometry (min: 0, increment: 1) Geometry: 0: orthogonal to path between previous and next waypoint.
   * @param2 UseAltitude (min: 0, max: 1, increment: 1) Altitude: 0: ignore altitude
   * @param3 Empty
   * @param4 Empty
   * @param5 Latitude Latitude
   * @param6 Longitude Longitude
   * @param7 Altitude[m] Altitude
   */
  'CONDITION_GATE'                                 = 4501,

  /**
   * Fence return point (there can only be one such point in a geofence definition). If rally points are
   * supported they should be used instead.
   *
   * @note has location and is destination
   *
   * @param1 Reserved
   * @param2 Reserved
   * @param3 Reserved
   * @param4 Reserved
   * @param5 Latitude Latitude
   * @param6 Longitude Longitude
   * @param7 Altitude[m] Altitude
   */
  'NAV_FENCE_RETURN_POINT'                         = 5000,

  /**
   * Fence vertex for an inclusion polygon (the polygon must not be self-intersecting). The vehicle must
   * stay within this area. Minimum of 3 vertices required.
   *
   * @note has location
   *
   * @param1 Vertex Count (min: 3, increment: 1) Polygon vertex count
   * @param2 Inclusion Group (min: 0, increment: 1) Vehicle must be inside ALL inclusion zones in a single group, vehicle must be inside at least one group, must be the same for all points in each polygon
   * @param3 Reserved
   * @param4 Reserved
   * @param5 Latitude Latitude
   * @param6 Longitude Longitude
   * @param7 Reserved
   */
  'NAV_FENCE_POLYGON_VERTEX_INCLUSION'             = 5001,

  /**
   * Fence vertex for an exclusion polygon (the polygon must not be self-intersecting). The vehicle must
   * stay outside this area. Minimum of 3 vertices required.
   *
   * @note has location
   *
   * @param1 Vertex Count (min: 3, increment: 1) Polygon vertex count
   * @param2 Reserved
   * @param3 Reserved
   * @param4 Reserved
   * @param5 Latitude Latitude
   * @param6 Longitude Longitude
   * @param7 Reserved
   */
  'NAV_FENCE_POLYGON_VERTEX_EXCLUSION'             = 5002,

  /**
   * Circular fence area. The vehicle must stay inside this area.
   *
   * @note has location
   *
   * @param1 Radius[m] Radius.
   * @param2 Inclusion Group (min: 0, increment: 1) Vehicle must be inside ALL inclusion zones in a single group, vehicle must be inside at least one group
   * @param3 Reserved
   * @param4 Reserved
   * @param5 Latitude Latitude
   * @param6 Longitude Longitude
   * @param7 Reserved
   */
  'NAV_FENCE_CIRCLE_INCLUSION'                     = 5003,

  /**
   * Circular fence area. The vehicle must stay outside this area.
   *
   * @note has location
   *
   * @param1 Radius[m] Radius.
   * @param2 Reserved
   * @param3 Reserved
   * @param4 Reserved
   * @param5 Latitude Latitude
   * @param6 Longitude Longitude
   * @param7 Reserved
   */
  'NAV_FENCE_CIRCLE_EXCLUSION'                     = 5004,

  /**
   * Rally point. You can have multiple rally points defined.
   *
   * @note has location
   *
   * @param1 Reserved
   * @param2 Reserved
   * @param3 Reserved
   * @param4 Reserved
   * @param5 Latitude Latitude
   * @param6 Longitude Longitude
   * @param7 Altitude[m] Altitude
   */
  'NAV_RALLY_POINT'                                = 5100,

  /**
   * Commands the vehicle to respond with a sequence of messages UAVCAN_NODE_INFO, one message per every
   * UAVCAN node that is online. Note that some of the response messages can be lost, which the receiver
   * can detect easily by checking whether every received UAVCAN_NODE_STATUS has a matching message
   * UAVCAN_NODE_INFO received earlier; if not, this command should be sent again in order to request
   * re-transmission of the node information messages.
   * @param1 Reserved (set to 0)
   * @param2 Reserved (set to 0)
   * @param3 Reserved (set to 0)
   * @param4 Reserved (set to 0)
   * @param5 Reserved (set to 0)
   * @param6 Reserved (set to 0)
   * @param7 Reserved (set to 0)
   */
  'UAVCAN_GET_NODE_INFO'                           = 5200,

  /**
   * Trigger the start of an ADSB-out IDENT. This should only be used when requested to do so by an Air
   * Traffic Controller in controlled airspace. This starts the IDENT which is then typically held for 18
   * seconds by the hardware per the Mode A, C, and S transponder spec.
   * @param1 Reserved (set to 0)
   * @param2 Reserved (set to 0)
   * @param3 Reserved (set to 0)
   * @param4 Reserved (set to 0)
   * @param5 Reserved (set to 0)
   * @param6 Reserved (set to 0)
   * @param7 Reserved (set to 0)
   */
  'DO_ADSB_OUT_IDENT'                              = 10001,

  /**
   * Deploy payload on a Lat / Lon / Alt position. This includes the navigation to reach the required
   * release position and velocity.
   *
   * @note has location and is destination
   *
   * @param1 Operation Mode (min: 0, max: 2, increment: 1) Operation mode. 0: prepare single payload deploy (overwriting previous requests), but do not execute it. 1: execute payload deploy immediately (rejecting further deploy commands during execution, but allowing abort). 2: add payload deploy to existing deployment list.
   * @param2 Approach Vector[deg] (min: -1, max: 360) Desired approach vector in compass heading. A negative value indicates the system can define the approach vector at will.
   * @param3 Ground Speed (min: -1) Desired ground speed at release time. This can be overridden by the airframe in case it needs to meet minimum airspeed. A negative value indicates the system can define the ground speed at will.
   * @param4 Altitude Clearance[m] (min: -1) Minimum altitude clearance to the release position. A negative value indicates the system can define the clearance at will.
   * @param5 Latitude[degE7] Latitude. Note, if used in MISSION_ITEM (deprecated) the units are degrees (unscaled)
   * @param6 Longitude[degE7] Longitude. Note, if used in MISSION_ITEM (deprecated) the units are degrees (unscaled)
   * @param7 Altitude[m] Altitude (MSL)
   */
  'PAYLOAD_PREPARE_DEPLOY'                         = 30001,

  /**
   * Control the payload deployment.
   * @param1 Operation Mode (min: 0, max: 101, increment: 1) Operation mode. 0: Abort deployment, continue normal mission. 1: switch to payload deployment mode. 100: delete first payload deployment request. 101: delete all payload deployment requests.
   * @param2 Reserved
   * @param3 Reserved
   * @param4 Reserved
   * @param5 Reserved
   * @param6 Reserved
   * @param7 Reserved
   */
  'PAYLOAD_CONTROL_DEPLOY'                         = 30002,

  /**
   * Magnetometer calibration based on provided known yaw. This allows for fast calibration using WMM
   * field tables in the vehicle, given only the known yaw of the vehicle. If Latitude and longitude are
   * both zero then use the current vehicle location.
   *
   * @note has location
   *
   * @param1 Yaw[deg] Yaw of vehicle in earth frame.
   * @param2 CompassMask CompassMask, 0 for all.
   * @param3 Latitude[deg] Latitude.
   * @param4 Longitude[deg] Longitude.
   * @param5 Empty.
   * @param6 Empty.
   * @param7 Empty.
   */
  'FIXED_MAG_CAL_YAW'                              = 42006,

  /**
   * Command to operate winch.
   * @param1 Instance (min: 1, increment: 1) Winch instance number.
   * @param2 Action Action to perform.
   * @param3 Length[m] Length of cable to release (negative to wind).
   * @param4 Rate[m/s] Release rate (negative to wind).
   * @param5 Empty.
   * @param6 Empty.
   * @param7 Empty.
   */
  'DO_WINCH'                                       = 42600,

  /**
   * User defined waypoint item. Ground Station will show the Vehicle as flying through this item.
   *
   * @note has location and is destination
   *
   * @param1 User defined
   * @param2 User defined
   * @param3 User defined
   * @param4 User defined
   * @param5 Latitude Latitude unscaled
   * @param6 Longitude Longitude unscaled
   * @param7 Altitude[m] Altitude (MSL)
   */
  'WAYPOINT_USER_1'                                = 31000,

  /**
   * User defined waypoint item. Ground Station will show the Vehicle as flying through this item.
   *
   * @note has location and is destination
   *
   * @param1 User defined
   * @param2 User defined
   * @param3 User defined
   * @param4 User defined
   * @param5 Latitude Latitude unscaled
   * @param6 Longitude Longitude unscaled
   * @param7 Altitude[m] Altitude (MSL)
   */
  'WAYPOINT_USER_2'                                = 31001,

  /**
   * User defined waypoint item. Ground Station will show the Vehicle as flying through this item.
   *
   * @note has location and is destination
   *
   * @param1 User defined
   * @param2 User defined
   * @param3 User defined
   * @param4 User defined
   * @param5 Latitude Latitude unscaled
   * @param6 Longitude Longitude unscaled
   * @param7 Altitude[m] Altitude (MSL)
   */
  'WAYPOINT_USER_3'                                = 31002,

  /**
   * User defined waypoint item. Ground Station will show the Vehicle as flying through this item.
   *
   * @note has location and is destination
   *
   * @param1 User defined
   * @param2 User defined
   * @param3 User defined
   * @param4 User defined
   * @param5 Latitude Latitude unscaled
   * @param6 Longitude Longitude unscaled
   * @param7 Altitude[m] Altitude (MSL)
   */
  'WAYPOINT_USER_4'                                = 31003,

  /**
   * User defined waypoint item. Ground Station will show the Vehicle as flying through this item.
   *
   * @note has location and is destination
   *
   * @param1 User defined
   * @param2 User defined
   * @param3 User defined
   * @param4 User defined
   * @param5 Latitude Latitude unscaled
   * @param6 Longitude Longitude unscaled
   * @param7 Altitude[m] Altitude (MSL)
   */
  'WAYPOINT_USER_5'                                = 31004,

  /**
   * User defined spatial item. Ground Station will not show the Vehicle as flying through this item.
   * Example: ROI item.
   *
   * @note has location
   *
   * @param1 User defined
   * @param2 User defined
   * @param3 User defined
   * @param4 User defined
   * @param5 Latitude Latitude unscaled
   * @param6 Longitude Longitude unscaled
   * @param7 Altitude[m] Altitude (MSL)
   */
  'SPATIAL_USER_1'                                 = 31005,

  /**
   * User defined spatial item. Ground Station will not show the Vehicle as flying through this item.
   * Example: ROI item.
   *
   * @note has location
   *
   * @param1 User defined
   * @param2 User defined
   * @param3 User defined
   * @param4 User defined
   * @param5 Latitude Latitude unscaled
   * @param6 Longitude Longitude unscaled
   * @param7 Altitude[m] Altitude (MSL)
   */
  'SPATIAL_USER_2'                                 = 31006,

  /**
   * User defined spatial item. Ground Station will not show the Vehicle as flying through this item.
   * Example: ROI item.
   *
   * @note has location
   *
   * @param1 User defined
   * @param2 User defined
   * @param3 User defined
   * @param4 User defined
   * @param5 Latitude Latitude unscaled
   * @param6 Longitude Longitude unscaled
   * @param7 Altitude[m] Altitude (MSL)
   */
  'SPATIAL_USER_3'                                 = 31007,

  /**
   * User defined spatial item. Ground Station will not show the Vehicle as flying through this item.
   * Example: ROI item.
   *
   * @note has location
   *
   * @param1 User defined
   * @param2 User defined
   * @param3 User defined
   * @param4 User defined
   * @param5 Latitude Latitude unscaled
   * @param6 Longitude Longitude unscaled
   * @param7 Altitude[m] Altitude (MSL)
   */
  'SPATIAL_USER_4'                                 = 31008,

  /**
   * User defined spatial item. Ground Station will not show the Vehicle as flying through this item.
   * Example: ROI item.
   *
   * @note has location
   *
   * @param1 User defined
   * @param2 User defined
   * @param3 User defined
   * @param4 User defined
   * @param5 Latitude Latitude unscaled
   * @param6 Longitude Longitude unscaled
   * @param7 Altitude[m] Altitude (MSL)
   */
  'SPATIAL_USER_5'                                 = 31009,

  /**
   * User defined command. Ground Station will not show the Vehicle as flying through this item. Example:
   * MAV_CMD_DO_SET_PARAMETER item.
   * @param1 User defined
   * @param2 User defined
   * @param3 User defined
   * @param4 User defined
   * @param5 User defined
   * @param6 User defined
   * @param7 User defined
   */
  'USER_1'                                         = 31010,

  /**
   * User defined command. Ground Station will not show the Vehicle as flying through this item. Example:
   * MAV_CMD_DO_SET_PARAMETER item.
   * @param1 User defined
   * @param2 User defined
   * @param3 User defined
   * @param4 User defined
   * @param5 User defined
   * @param6 User defined
   * @param7 User defined
   */
  'USER_2'                                         = 31011,

  /**
   * User defined command. Ground Station will not show the Vehicle as flying through this item. Example:
   * MAV_CMD_DO_SET_PARAMETER item.
   * @param1 User defined
   * @param2 User defined
   * @param3 User defined
   * @param4 User defined
   * @param5 User defined
   * @param6 User defined
   * @param7 User defined
   */
  'USER_3'                                         = 31012,

  /**
   * User defined command. Ground Station will not show the Vehicle as flying through this item. Example:
   * MAV_CMD_DO_SET_PARAMETER item.
   * @param1 User defined
   * @param2 User defined
   * @param3 User defined
   * @param4 User defined
   * @param5 User defined
   * @param6 User defined
   * @param7 User defined
   */
  'USER_4'                                         = 31013,

  /**
   * User defined command. Ground Station will not show the Vehicle as flying through this item. Example:
   * MAV_CMD_DO_SET_PARAMETER item.
   * @param1 User defined
   * @param2 User defined
   * @param3 User defined
   * @param4 User defined
   * @param5 User defined
   * @param6 User defined
   * @param7 User defined
   */
  'USER_5'                                         = 31014,
}

/**
 * A data stream is not a fixed set of messages, but rather a
 recommendation to the autopilot
 * software. Individual autopilots may or may not obey
 the recommended messages.
 */
export enum MavDataStream {
  /**
   * Enable all data streams
   */
  'ALL'                                            = 0,

  /**
   * Enable IMU_RAW, GPS_RAW, GPS_STATUS packets.
   */
  'RAW_SENSORS'                                    = 1,

  /**
   * Enable GPS_STATUS, CONTROL_STATUS, AUX_STATUS
   */
  'EXTENDED_STATUS'                                = 2,

  /**
   * Enable RC_CHANNELS_SCALED, RC_CHANNELS_RAW, SERVO_OUTPUT_RAW
   */
  'RC_CHANNELS'                                    = 3,

  /**
   * Enable ATTITUDE_CONTROLLER_OUTPUT, POSITION_CONTROLLER_OUTPUT, NAV_CONTROLLER_OUTPUT.
   */
  'RAW_CONTROLLER'                                 = 4,

  /**
   * Enable LOCAL_POSITION, GLOBAL_POSITION_INT messages.
   */
  'POSITION'                                       = 6,

  /**
   * Dependent on the autopilot
   */
  'EXTRA1'                                         = 10,

  /**
   * Dependent on the autopilot
   */
  'EXTRA2'                                         = 11,

  /**
   * Dependent on the autopilot
   */
  'EXTRA3'                                         = 12,
}

/**
 * The ROI (region of interest) for the vehicle. This can be
 be used by the vehicle for camera/vehicle
 * attitude alignment (see
 MAV_CMD_NAV_ROI).
 */
export enum MavRoi {
  /**
   * No region of interest.
   */
  'NONE'                                           = 0,

  /**
   * Point toward next waypoint, with optional pitch/roll/yaw offset.
   */
  'WPNEXT'                                         = 1,

  /**
   * Point toward given waypoint.
   */
  'WPINDEX'                                        = 2,

  /**
   * Point toward fixed location.
   */
  'LOCATION'                                       = 3,

  /**
   * Point toward of given id.
   */
  'TARGET'                                         = 4,
}

/**
 * ACK / NACK / ERROR values as a result of MAV_CMDs and for mission item transmission.
 */
export enum MavCmdAck {
  /**
   * Command / mission item is ok.
   */
  'OK'                                             = 0,

  /**
   * Generic error message if none of the other reasons fails or if no detailed error reporting is
   * implemented.
   */
  'ERR_FAIL'                                       = 1,

  /**
   * The system is refusing to accept this command from this source / communication partner.
   */
  'ERR_ACCESS_DENIED'                              = 2,

  /**
   * Command or mission item is not supported, other commands would be accepted.
   */
  'ERR_NOT_SUPPORTED'                              = 3,

  /**
   * The coordinate frame of this command / mission item is not supported.
   */
  'ERR_COORDINATE_FRAME_NOT_SUPPORTED'             = 4,

  /**
   * The coordinate frame of this command is ok, but he coordinate values exceed the safety limits of
   * this system. This is a generic error, please use the more specific error messages below if possible.
   */
  'ERR_COORDINATES_OUT_OF_RANGE'                   = 5,

  /**
   * The X or latitude value is out of range.
   */
  'ERR_X_LAT_OUT_OF_RANGE'                         = 6,

  /**
   * The Y or longitude value is out of range.
   */
  'ERR_Y_LON_OUT_OF_RANGE'                         = 7,

  /**
   * The Z or altitude value is out of range.
   */
  'ERR_Z_ALT_OUT_OF_RANGE'                         = 8,
}

/**
 * Specifies the datatype of a MAVLink parameter.
 */
export enum MavParamType {
  /**
   * 8-bit unsigned integer
   */
  'UINT8'                                          = 1,

  /**
   * 8-bit signed integer
   */
  'INT8'                                           = 2,

  /**
   * 16-bit unsigned integer
   */
  'UINT16'                                         = 3,

  /**
   * 16-bit signed integer
   */
  'INT16'                                          = 4,

  /**
   * 32-bit unsigned integer
   */
  'UINT32'                                         = 5,

  /**
   * 32-bit signed integer
   */
  'INT32'                                          = 6,

  /**
   * 64-bit unsigned integer
   */
  'UINT64'                                         = 7,

  /**
   * 64-bit signed integer
   */
  'INT64'                                          = 8,

  /**
   * 32-bit floating-point
   */
  'REAL32'                                         = 9,

  /**
   * 64-bit floating-point
   */
  'REAL64'                                         = 10,
}

/**
 * Specifies the datatype of a MAVLink extended parameter.
 */
export enum MavParamExtType {
  /**
   * 8-bit unsigned integer
   */
  'UINT8'                                          = 1,

  /**
   * 8-bit signed integer
   */
  'INT8'                                           = 2,

  /**
   * 16-bit unsigned integer
   */
  'UINT16'                                         = 3,

  /**
   * 16-bit signed integer
   */
  'INT16'                                          = 4,

  /**
   * 32-bit unsigned integer
   */
  'UINT32'                                         = 5,

  /**
   * 32-bit signed integer
   */
  'INT32'                                          = 6,

  /**
   * 64-bit unsigned integer
   */
  'UINT64'                                         = 7,

  /**
   * 64-bit signed integer
   */
  'INT64'                                          = 8,

  /**
   * 32-bit floating-point
   */
  'REAL32'                                         = 9,

  /**
   * 64-bit floating-point
   */
  'REAL64'                                         = 10,

  /**
   * Custom Type
   */
  'CUSTOM'                                         = 11,
}

/**
 * Result from a MAVLink command (MAV_CMD)
 */
export enum MavResult {
  /**
   * Command is valid (is supported and has valid parameters), and was executed.
   */
  'ACCEPTED'                                       = 0,

  /**
   * Command is valid, but cannot be executed at this time. This is used to indicate a problem that
   * should be fixed just by waiting (e.g. a state machine is busy, can't arm because have not got GPS
   * lock, etc.). Retrying later should work.
   */
  'TEMPORARILY_REJECTED'                           = 1,

  /**
   * Command is invalid (is supported but has invalid parameters). Retrying same command and parameters
   * will not work.
   */
  'DENIED'                                         = 2,

  /**
   * Command is not supported (unknown).
   */
  'UNSUPPORTED'                                    = 3,

  /**
   * Command is valid, but execution has failed. This is used to indicate any non-temporary or unexpected
   * problem, i.e. any problem that must be fixed before the command can succeed/be retried. For example,
   * attempting to write a file when out of memory, attempting to arm when sensors are not calibrated,
   * etc.
   */
  'FAILED'                                         = 4,

  /**
   * Command is valid and is being executed. This will be followed by further progress updates, i.e. the
   * component may send further COMMAND_ACK messages with result MAV_RESULT_IN_PROGRESS (at a rate
   * decided by the implementation), and must terminate by sending a COMMAND_ACK message with final
   * result of the operation. The COMMAND_ACK.progress field can be used to indicate the progress of the
   * operation.
   */
  'IN_PROGRESS'                                    = 5,

  /**
   * Command has been cancelled (as a result of receiving a COMMAND_CANCEL message).
   */
  'CANCELLED'                                      = 6,
}

/**
 * Result of mission operation (in a MISSION_ACK message).
 */
export enum MavMissionResult {
  /**
   * mission accepted OK
   */
  'ACCEPTED'                                       = 0,

  /**
   * Generic error / not accepting mission commands at all right now.
   */
  'ERROR'                                          = 1,

  /**
   * Coordinate frame is not supported.
   */
  'UNSUPPORTED_FRAME'                              = 2,

  /**
   * Command is not supported.
   */
  'UNSUPPORTED'                                    = 3,

  /**
   * Mission items exceed storage space.
   */
  'NO_SPACE'                                       = 4,

  /**
   * One of the parameters has an invalid value.
   */
  'INVALID'                                        = 5,

  /**
   * param1 has an invalid value.
   */
  'INVALID_PARAM1'                                 = 6,

  /**
   * param2 has an invalid value.
   */
  'INVALID_PARAM2'                                 = 7,

  /**
   * param3 has an invalid value.
   */
  'INVALID_PARAM3'                                 = 8,

  /**
   * param4 has an invalid value.
   */
  'INVALID_PARAM4'                                 = 9,

  /**
   * x / param5 has an invalid value.
   */
  'INVALID_PARAM5_X'                               = 10,

  /**
   * y / param6 has an invalid value.
   */
  'INVALID_PARAM6_Y'                               = 11,

  /**
   * z / param7 has an invalid value.
   */
  'INVALID_PARAM7'                                 = 12,

  /**
   * Mission item received out of sequence
   */
  'INVALID_SEQUENCE'                               = 13,

  /**
   * Not accepting any mission commands from this communication partner.
   */
  'DENIED'                                         = 14,

  /**
   * Current mission operation cancelled (e.g. mission upload, mission download).
   */
  'OPERATION_CANCELLED'                            = 15,
}

/**
 * Indicates the severity level, generally used for status messages to indicate their relative urgency.
 * Based on RFC-5424 using expanded definitions at:
 * http://www.kiwisyslog.com/kb/info:-syslog-message-levels/.
 */
export enum MavSeverity {
  /**
   * System is unusable. This is a "panic" condition.
   */
  'EMERGENCY'                                      = 0,

  /**
   * Action should be taken immediately. Indicates error in non-critical systems.
   */
  'ALERT'                                          = 1,

  /**
   * Action must be taken immediately. Indicates failure in a primary system.
   */
  'CRITICAL'                                       = 2,

  /**
   * Indicates an error in secondary/redundant systems.
   */
  'ERROR'                                          = 3,

  /**
   * Indicates about a possible future error if this is not resolved within a given timeframe. Example
   * would be a low battery warning.
   */
  'WARNING'                                        = 4,

  /**
   * An unusual event has occurred, though not an error condition. This should be investigated for the
   * root cause.
   */
  'NOTICE'                                         = 5,

  /**
   * Normal operational messages. Useful for logging. No action is required for these messages.
   */
  'INFO'                                           = 6,

  /**
   * Useful non-operational messages that can assist in debugging. These should not occur during normal
   * operation.
   */
  'DEBUG'                                          = 7,
}

/**
 * Power supply status flags (bitmask)
 */
export enum MavPowerStatus {
  /**
   * main brick power supply valid
   */
  'BRICK_VALID'                                    = 1,

  /**
   * main servo power supply valid for FMU
   */
  'SERVO_VALID'                                    = 2,

  /**
   * USB power is connected
   */
  'USB_CONNECTED'                                  = 4,

  /**
   * peripheral supply is in over-current state
   */
  'PERIPH_OVERCURRENT'                             = 8,

  /**
   * hi-power peripheral supply is in over-current state
   */
  'PERIPH_HIPOWER_OVERCURRENT'                     = 16,

  /**
   * Power status has changed since boot
   */
  'CHANGED'                                        = 32,
}

/**
 * SERIAL_CONTROL device types
 */
export enum SerialControlDev {
  /**
   * First telemetry port
   */
  'DEV_TELEM1'                                     = 0,

  /**
   * Second telemetry port
   */
  'DEV_TELEM2'                                     = 1,

  /**
   * First GPS port
   */
  'DEV_GPS1'                                       = 2,

  /**
   * Second GPS port
   */
  'DEV_GPS2'                                       = 3,

  /**
   * system shell
   */
  'DEV_SHELL'                                      = 10,

  /**
   * SERIAL0
   */
  'SERIAL0'                                        = 100,

  /**
   * SERIAL1
   */
  'SERIAL1'                                        = 101,

  /**
   * SERIAL2
   */
  'SERIAL2'                                        = 102,

  /**
   * SERIAL3
   */
  'SERIAL3'                                        = 103,

  /**
   * SERIAL4
   */
  'SERIAL4'                                        = 104,

  /**
   * SERIAL5
   */
  'SERIAL5'                                        = 105,

  /**
   * SERIAL6
   */
  'SERIAL6'                                        = 106,

  /**
   * SERIAL7
   */
  'SERIAL7'                                        = 107,

  /**
   * SERIAL8
   */
  'SERIAL8'                                        = 108,

  /**
   * SERIAL9
   */
  'SERIAL9'                                        = 109,
}

/**
 * SERIAL_CONTROL flags (bitmask)
 */
export enum SerialControlFlag {
  /**
   * Set if this is a reply
   */
  'REPLY'                                          = 1,

  /**
   * Set if the sender wants the receiver to send a response as another SERIAL_CONTROL message
   */
  'RESPOND'                                        = 2,

  /**
   * Set if access to the serial port should be removed from whatever driver is currently using it,
   * giving exclusive access to the SERIAL_CONTROL protocol. The port can be handed back by sending a
   * request without this flag set
   */
  'EXCLUSIVE'                                      = 4,

  /**
   * Block on writes to the serial port
   */
  'BLOCKING'                                       = 8,

  /**
   * Send multiple replies until port is drained
   */
  'MULTI'                                          = 16,
}

/**
 * Enumeration of distance sensor types
 */
export enum MavDistanceSensor {
  /**
   * Laser rangefinder, e.g. LightWare SF02/F or PulsedLight units
   */
  'LASER'                                          = 0,

  /**
   * Ultrasound rangefinder, e.g. MaxBotix units
   */
  'ULTRASOUND'                                     = 1,

  /**
   * Infrared rangefinder, e.g. Sharp units
   */
  'INFRARED'                                       = 2,

  /**
   * Radar type, e.g. uLanding units
   */
  'RADAR'                                          = 3,

  /**
   * Broken or unknown type, e.g. analog units
   */
  'UNKNOWN'                                        = 4,
}

/**
 * Enumeration of sensor orientation, according to its rotations
 */
export enum MavSensorOrientation {
  /**
   * Roll: 0, Pitch: 0, Yaw: 0
   */
  'NONE'                                           = 0,

  /**
   * Roll: 0, Pitch: 0, Yaw: 45
   */
  'YAW_45'                                         = 1,

  /**
   * Roll: 0, Pitch: 0, Yaw: 90
   */
  'YAW_90'                                         = 2,

  /**
   * Roll: 0, Pitch: 0, Yaw: 135
   */
  'YAW_135'                                        = 3,

  /**
   * Roll: 0, Pitch: 0, Yaw: 180
   */
  'YAW_180'                                        = 4,

  /**
   * Roll: 0, Pitch: 0, Yaw: 225
   */
  'YAW_225'                                        = 5,

  /**
   * Roll: 0, Pitch: 0, Yaw: 270
   */
  'YAW_270'                                        = 6,

  /**
   * Roll: 0, Pitch: 0, Yaw: 315
   */
  'YAW_315'                                        = 7,

  /**
   * Roll: 180, Pitch: 0, Yaw: 0
   */
  'ROLL_180'                                       = 8,

  /**
   * Roll: 180, Pitch: 0, Yaw: 45
   */
  'ROLL_180_YAW_45'                                = 9,

  /**
   * Roll: 180, Pitch: 0, Yaw: 90
   */
  'ROLL_180_YAW_90'                                = 10,

  /**
   * Roll: 180, Pitch: 0, Yaw: 135
   */
  'ROLL_180_YAW_135'                               = 11,

  /**
   * Roll: 0, Pitch: 180, Yaw: 0
   */
  'PITCH_180'                                      = 12,

  /**
   * Roll: 180, Pitch: 0, Yaw: 225
   */
  'ROLL_180_YAW_225'                               = 13,

  /**
   * Roll: 180, Pitch: 0, Yaw: 270
   */
  'ROLL_180_YAW_270'                               = 14,

  /**
   * Roll: 180, Pitch: 0, Yaw: 315
   */
  'ROLL_180_YAW_315'                               = 15,

  /**
   * Roll: 90, Pitch: 0, Yaw: 0
   */
  'ROLL_90'                                        = 16,

  /**
   * Roll: 90, Pitch: 0, Yaw: 45
   */
  'ROLL_90_YAW_45'                                 = 17,

  /**
   * Roll: 90, Pitch: 0, Yaw: 90
   */
  'ROLL_90_YAW_90'                                 = 18,

  /**
   * Roll: 90, Pitch: 0, Yaw: 135
   */
  'ROLL_90_YAW_135'                                = 19,

  /**
   * Roll: 270, Pitch: 0, Yaw: 0
   */
  'ROLL_270'                                       = 20,

  /**
   * Roll: 270, Pitch: 0, Yaw: 45
   */
  'ROLL_270_YAW_45'                                = 21,

  /**
   * Roll: 270, Pitch: 0, Yaw: 90
   */
  'ROLL_270_YAW_90'                                = 22,

  /**
   * Roll: 270, Pitch: 0, Yaw: 135
   */
  'ROLL_270_YAW_135'                               = 23,

  /**
   * Roll: 0, Pitch: 90, Yaw: 0
   */
  'PITCH_90'                                       = 24,

  /**
   * Roll: 0, Pitch: 270, Yaw: 0
   */
  'PITCH_270'                                      = 25,

  /**
   * Roll: 0, Pitch: 180, Yaw: 90
   */
  'PITCH_180_YAW_90'                               = 26,

  /**
   * Roll: 0, Pitch: 180, Yaw: 270
   */
  'PITCH_180_YAW_270'                              = 27,

  /**
   * Roll: 90, Pitch: 90, Yaw: 0
   */
  'ROLL_90_PITCH_90'                               = 28,

  /**
   * Roll: 180, Pitch: 90, Yaw: 0
   */
  'ROLL_180_PITCH_90'                              = 29,

  /**
   * Roll: 270, Pitch: 90, Yaw: 0
   */
  'ROLL_270_PITCH_90'                              = 30,

  /**
   * Roll: 90, Pitch: 180, Yaw: 0
   */
  'ROLL_90_PITCH_180'                              = 31,

  /**
   * Roll: 270, Pitch: 180, Yaw: 0
   */
  'ROLL_270_PITCH_180'                             = 32,

  /**
   * Roll: 90, Pitch: 270, Yaw: 0
   */
  'ROLL_90_PITCH_270'                              = 33,

  /**
   * Roll: 180, Pitch: 270, Yaw: 0
   */
  'ROLL_180_PITCH_270'                             = 34,

  /**
   * Roll: 270, Pitch: 270, Yaw: 0
   */
  'ROLL_270_PITCH_270'                             = 35,

  /**
   * Roll: 90, Pitch: 180, Yaw: 90
   */
  'ROLL_90_PITCH_180_YAW_90'                       = 36,

  /**
   * Roll: 90, Pitch: 0, Yaw: 270
   */
  'ROLL_90_YAW_270'                                = 37,

  /**
   * Roll: 90, Pitch: 68, Yaw: 293
   */
  'ROLL_90_PITCH_68_YAW_293'                       = 38,

  /**
   * Pitch: 315
   */
  'PITCH_315'                                      = 39,

  /**
   * Roll: 90, Pitch: 315
   */
  'ROLL_90_PITCH_315'                              = 40,

  /**
   * Custom orientation
   */
  'CUSTOM'                                         = 100,
}

/**
 * Bitmask of (optional) autopilot capabilities (64 bit). If a bit is set, the autopilot supports this
 * capability.
 */
export enum MavProtocolCapability {
  /**
   * Autopilot supports MISSION float message type.
   */
  'MISSION_FLOAT'                                  = 1,

  /**
   * Autopilot supports the new param float message type.
   */
  'PARAM_FLOAT'                                    = 2,

  /**
   * Autopilot supports MISSION_ITEM_INT scaled integer message type.
   */
  'MISSION_INT'                                    = 4,

  /**
   * Autopilot supports COMMAND_INT scaled integer message type.
   */
  'COMMAND_INT'                                    = 8,

  /**
   * Autopilot supports the new param union message type.
   */
  'PARAM_UNION'                                    = 16,

  /**
   * Autopilot supports the new FILE_TRANSFER_PROTOCOL message type.
   */
  'FTP'                                            = 32,

  /**
   * Autopilot supports commanding attitude offboard.
   */
  'SET_ATTITUDE_TARGET'                            = 64,

  /**
   * Autopilot supports commanding position and velocity targets in local NED frame.
   */
  'SET_POSITION_TARGET_LOCAL_NED'                  = 128,

  /**
   * Autopilot supports commanding position and velocity targets in global scaled integers.
   */
  'SET_POSITION_TARGET_GLOBAL_INT'                 = 256,

  /**
   * Autopilot supports terrain protocol / data handling.
   */
  'TERRAIN'                                        = 512,

  /**
   * Autopilot supports direct actuator control.
   */
  'SET_ACTUATOR_TARGET'                            = 1024,

  /**
   * Autopilot supports the flight termination command.
   */
  'FLIGHT_TERMINATION'                             = 2048,

  /**
   * Autopilot supports onboard compass calibration.
   */
  'COMPASS_CALIBRATION'                            = 4096,

  /**
   * Autopilot supports MAVLink version 2.
   */
  'MAVLINK2'                                       = 8192,

  /**
   * Autopilot supports mission fence protocol.
   */
  'MISSION_FENCE'                                  = 16384,

  /**
   * Autopilot supports mission rally point protocol.
   */
  'MISSION_RALLY'                                  = 32768,

  /**
   * Autopilot supports the flight information protocol.
   */
  'FLIGHT_INFORMATION'                             = 65536,
}

/**
 * Type of mission items being requested/sent in mission protocol.
 */
export enum MavMissionType {
  /**
   * Items are mission commands for main mission.
   */
  'MISSION'                                        = 0,

  /**
   * Specifies GeoFence area(s). Items are MAV_CMD_NAV_FENCE_ GeoFence items.
   */
  'FENCE'                                          = 1,

  /**
   * Specifies the rally points for the vehicle. Rally points are alternative RTL points. Items are
   * MAV_CMD_NAV_RALLY_POINT rally point items.
   */
  'RALLY'                                          = 2,

  /**
   * Only used in MISSION_CLEAR_ALL to clear all mission types.
   */
  'ALL'                                            = 255,
}

/**
 * Enumeration of estimator types
 */
export enum MavEstimatorType {
  /**
   * Unknown type of the estimator.
   */
  'UNKNOWN'                                        = 0,

  /**
   * This is a naive estimator without any real covariance feedback.
   */
  'NAIVE'                                          = 1,

  /**
   * Computer vision based estimate. Might be up to scale.
   */
  'VISION'                                         = 2,

  /**
   * Visual-inertial estimate.
   */
  'VIO'                                            = 3,

  /**
   * Plain GPS estimate.
   */
  'GPS'                                            = 4,

  /**
   * Estimator integrating GPS and inertial sensing.
   */
  'GPS_INS'                                        = 5,

  /**
   * Estimate from external motion capturing system.
   */
  'MOCAP'                                          = 6,

  /**
   * Estimator based on lidar sensor input.
   */
  'LIDAR'                                          = 7,

  /**
   * Estimator on autopilot.
   */
  'AUTOPILOT'                                      = 8,
}

/**
 * Enumeration of battery types
 */
export enum MavBatteryType {
  /**
   * Not specified.
   */
  'UNKNOWN'                                        = 0,

  /**
   * Lithium polymer battery
   */
  'LIPO'                                           = 1,

  /**
   * Lithium-iron-phosphate battery
   */
  'LIFE'                                           = 2,

  /**
   * Lithium-ION battery
   */
  'LION'                                           = 3,

  /**
   * Nickel metal hydride battery
   */
  'NIMH'                                           = 4,
}

/**
 * Enumeration of battery functions
 */
export enum MavBatteryFunction {
  /**
   * Battery function is unknown
   */
  'FUNCTION_UNKNOWN'                               = 0,

  /**
   * Battery supports all flight systems
   */
  'FUNCTION_ALL'                                   = 1,

  /**
   * Battery for the propulsion system
   */
  'FUNCTION_PROPULSION'                            = 2,

  /**
   * Avionics battery
   */
  'FUNCTION_AVIONICS'                              = 3,

  /**
   * Payload battery
   */
  'TYPE_PAYLOAD'                                   = 4,
}

/**
 * Enumeration for battery charge states.
 */
export enum MavBatteryChargeState {
  /**
   * Low battery state is not provided
   */
  'UNDEFINED'                                      = 0,

  /**
   * Battery is not in low state. Normal operation.
   */
  'OK'                                             = 1,

  /**
   * Battery state is low, warn and monitor close.
   */
  'LOW'                                            = 2,

  /**
   * Battery state is critical, return or abort immediately.
   */
  'CRITICAL'                                       = 3,

  /**
   * Battery state is too low for ordinary abort sequence. Perform fastest possible emergency stop to
   * prevent damage.
   */
  'EMERGENCY'                                      = 4,

  /**
   * Battery failed, damage unavoidable. Possible causes (faults) are listed in MAV_BATTERY_FAULT.
   */
  'FAILED'                                         = 5,

  /**
   * Battery is diagnosed to be defective or an error occurred, usage is discouraged / prohibited.
   * Possible causes (faults) are listed in MAV_BATTERY_FAULT.
   */
  'UNHEALTHY'                                      = 6,

  /**
   * Battery is charging.
   */
  'CHARGING'                                       = 7,
}

/**
 * Battery mode. Note, the normal operation mode (i.e. when flying) should be reported as
 * MAV_BATTERY_MODE_UNKNOWN to allow message trimming in normal flight.
 */
export enum MavBatteryMode {
  /**
   * Battery mode not supported/unknown battery mode/normal operation.
   */
  'UNKNOWN'                                        = 0,

  /**
   * Battery is auto discharging (towards storage level).
   */
  'AUTO_DISCHARGING'                               = 1,

  /**
   * Battery in hot-swap mode (current limited to prevent spikes that might damage sensitive electrical
   * circuits).
   */
  'HOT_SWAP'                                       = 2,
}

/**
 * Smart battery supply status/fault flags (bitmask) for health indication. The battery must also
 * report either MAV_BATTERY_CHARGE_STATE_FAILED or MAV_BATTERY_CHARGE_STATE_UNHEALTHY if any of these
 * are set.
 */
export enum MavBatteryFault {
  /**
   * Battery has deep discharged.
   */
  'DEEP_DISCHARGE'                                 = 1,

  /**
   * Voltage spikes.
   */
  'SPIKES'                                         = 2,

  /**
   * One or more cells have failed. Battery should also report MAV_BATTERY_CHARGE_STATE_FAILE (and should
   * not be used).
   */
  'CELL_FAIL'                                      = 4,

  /**
   * Over-current fault.
   */
  'OVER_CURRENT'                                   = 8,

  /**
   * Over-temperature fault.
   */
  'OVER_TEMPERATURE'                               = 16,

  /**
   * Under-temperature fault.
   */
  'UNDER_TEMPERATURE'                              = 32,

  /**
   * Vehicle voltage is not compatible with this battery (batteries on same power rail should have
   * similar voltage).
   */
  'INCOMPATIBLE_VOLTAGE'                           = 64,

  /**
   * Battery firmware is not compatible with current autopilot firmware.
   */
  'INCOMPATIBLE_FIRMWARE'                          = 128,

  /**
   * Battery is not compatible due to cell configuration (e.g. 5s1p when vehicle requires 6s).
   */
  'BATTERY_FAULT_INCOMPATIBLE_CELLS_CONFIGURATION' = 256,
}

/**
 * Flags to report status/failure cases for a power generator (used in GENERATOR_STATUS). Note that
 * FAULTS are conditions that cause the generator to fail. Warnings are conditions that require
 * attention before the next use (they indicate the system is not operating properly).
 */
export enum MavGeneratorStatusFlag {
  /**
   * Generator is off.
   */
  'OFF'                                            = 1,

  /**
   * Generator is ready to start generating power.
   */
  'READY'                                          = 2,

  /**
   * Generator is generating power.
   */
  'GENERATING'                                     = 4,

  /**
   * Generator is charging the batteries (generating enough power to charge and provide the load).
   */
  'CHARGING'                                       = 8,

  /**
   * Generator is operating at a reduced maximum power.
   */
  'REDUCED_POWER'                                  = 16,

  /**
   * Generator is providing the maximum output.
   */
  'MAXPOWER'                                       = 32,

  /**
   * Generator is near the maximum operating temperature, cooling is insufficient.
   */
  'OVERTEMP_WARNING'                               = 64,

  /**
   * Generator hit the maximum operating temperature and shutdown.
   */
  'OVERTEMP_FAULT'                                 = 128,

  /**
   * Power electronics are near the maximum operating temperature, cooling is insufficient.
   */
  'ELECTRONICS_OVERTEMP_WARNING'                   = 256,

  /**
   * Power electronics hit the maximum operating temperature and shutdown.
   */
  'ELECTRONICS_OVERTEMP_FAULT'                     = 512,

  /**
   * Power electronics experienced a fault and shutdown.
   */
  'ELECTRONICS_FAULT'                              = 1024,

  /**
   * The power source supplying the generator failed e.g. mechanical generator stopped, tether is no
   * longer providing power, solar cell is in shade, hydrogen reaction no longer happening.
   */
  'POWERSOURCE_FAULT'                              = 2048,

  /**
   * Generator controller having communication problems.
   */
  'COMMUNICATION_WARNING'                          = 4096,

  /**
   * Power electronic or generator cooling system error.
   */
  'COOLING_WARNING'                                = 8192,

  /**
   * Generator controller power rail experienced a fault.
   */
  'POWER_RAIL_FAULT'                               = 16384,

  /**
   * Generator controller exceeded the overcurrent threshold and shutdown to prevent damage.
   */
  'OVERCURRENT_FAULT'                              = 32768,

  /**
   * Generator controller detected a high current going into the batteries and shutdown to prevent
   * battery damage.
   */
  'BATTERY_OVERCHARGE_CURRENT_FAULT'               = 65536,

  /**
   * Generator controller exceeded it's overvoltage threshold and shutdown to prevent it exceeding the
   * voltage rating.
   */
  'OVERVOLTAGE_FAULT'                              = 131072,

  /**
   * Batteries are under voltage (generator will not start).
   */
  'BATTERY_UNDERVOLT_FAULT'                        = 262144,

  /**
   * Generator start is inhibited by e.g. a safety switch.
   */
  'START_INHIBITED'                                = 524288,

  /**
   * Generator requires maintenance.
   */
  'MAINTENANCE_REQUIRED'                           = 1048576,

  /**
   * Generator is not ready to generate yet.
   */
  'WARMING_UP'                                     = 2097152,

  /**
   * Generator is idle.
   */
  'IDLE'                                           = 4194304,
}

/**
 * Enumeration of VTOL states
 */
export enum MavVtolState {
  /**
   * MAV is not configured as VTOL
   */
  'UNDEFINED'                                      = 0,

  /**
   * VTOL is in transition from multicopter to fixed-wing
   */
  'TRANSITION_TO_FW'                               = 1,

  /**
   * VTOL is in transition from fixed-wing to multicopter
   */
  'TRANSITION_TO_MC'                               = 2,

  /**
   * VTOL is in multicopter state
   */
  'MC'                                             = 3,

  /**
   * VTOL is in fixed-wing state
   */
  'FW'                                             = 4,
}

/**
 * Enumeration of landed detector states
 */
export enum MavLandedState {
  /**
   * MAV landed state is unknown
   */
  'UNDEFINED'                                      = 0,

  /**
   * MAV is landed (on ground)
   */
  'ON_GROUND'                                      = 1,

  /**
   * MAV is in air
   */
  'IN_AIR'                                         = 2,

  /**
   * MAV currently taking off
   */
  'TAKEOFF'                                        = 3,

  /**
   * MAV currently landing
   */
  'LANDING'                                        = 4,
}

/**
 * Enumeration of the ADSB altimeter types
 */
export enum AdsbAltitudeType {
  /**
   * Altitude reported from a Baro source using QNH reference
   */
  'PRESSURE_QNH'                                   = 0,

  /**
   * Altitude reported from a GNSS source
   */
  'GEOMETRIC'                                      = 1,
}

/**
 * ADSB classification for the type of vehicle emitting the transponder signal
 */
export enum AdsbEmitterType {
  'NO_INFO'                                        = 0,
  'LIGHT'                                          = 1,
  'SMALL'                                          = 2,
  'LARGE'                                          = 3,
  'HIGH_VORTEX_LARGE'                              = 4,
  'HEAVY'                                          = 5,
  'HIGHLY_MANUV'                                   = 6,
  'ROTOCRAFT'                                      = 7,
  'UNASSIGNED'                                     = 8,
  'GLIDER'                                         = 9,
  'LIGHTER_AIR'                                    = 10,
  'PARACHUTE'                                      = 11,
  'ULTRA_LIGHT'                                    = 12,
  'UNASSIGNED2'                                    = 13,
  'UAV'                                            = 14,
  'SPACE'                                          = 15,
  'UNASSGINED3'                                    = 16,
  'EMERGENCY_SURFACE'                              = 17,
  'SERVICE_SURFACE'                                = 18,
  'POINT_OBSTACLE'                                 = 19,
}

/**
 * These flags indicate status such as data validity of each data source. Set = data valid
 */
export enum AdsbFlags {
  'VALID_COORDS'                                   = 1,
  'VALID_ALTITUDE'                                 = 2,
  'VALID_HEADING'                                  = 4,
  'VALID_VELOCITY'                                 = 8,
  'VALID_CALLSIGN'                                 = 16,
  'VALID_SQUAWK'                                   = 32,
  'SIMULATED'                                      = 64,
  'VERTICAL_VELOCITY_VALID'                        = 128,
  'BARO_VALID'                                     = 256,
  'SOURCE_UAT'                                     = 32768,
}

/**
 * Bitmap of options for the MAV_CMD_DO_REPOSITION
 */
export enum MavDoRepositionFlags {
  /**
   * The aircraft should immediately transition into guided. This should not be set for follow me
   * applications
   */
  'CHANGE_MODE'                                    = 1,

  /**
   * The aircraft should change altitude first and then flight to the coordinate
   */
  'ALTITUDE_MODE'                                  = 2,
}

/**
 * These flags indicate status such as data validity of each data source. Set = data valid
 */
export enum AcflyImuFlags {
  'ACCEL0'                                         = 1,
  'ACCEL1'                                         = 2,
  'ACCEL2'                                         = 4,
  'GYRO0'                                          = 16,
  'GYRO1'                                          = 32,
  'GYRO2'                                          = 64,
  'COMPASS0'                                       = 256,
  'COMPASS1'                                       = 512,
  'COMPASS2'                                       = 1024,
}

/**
 * These flags indicate status such as data validity of each data source. Set = data valid
 */
export enum AcflyPostypeFlags {
  'GlobalPositioning'                              = 0,
  'RelativePositioning'                            = 1,
  'RangePositioning'                               = 2,
}

/**
 * These flags indicate status such as data validity of each data source. Set = data valid
 */
export enum AcflyPosdatatypeFlags {
  's_xy'                                           = 0,
  's_z'                                            = 1,
  's_xyz'                                          = 2,
  'v_xy'                                           = 8,
  'v_z'                                            = 9,
  'v_xyz'                                          = 10,
  'xy'                                             = 16,
  'z'                                              = 17,
  'xyz'                                            = 18,
}

/**
 * Flags in ESTIMATOR_STATUS message
 */
export enum EstimatorStatusFlags {
  /**
   * True if the attitude estimate is good
   */
  'ATTITUDE'                                       = 1,

  /**
   * True if the horizontal velocity estimate is good
   */
  'VELOCITY_HORIZ'                                 = 2,

  /**
   * True if the vertical velocity estimate is good
   */
  'VELOCITY_VERT'                                  = 4,

  /**
   * True if the horizontal position (relative) estimate is good
   */
  'POS_HORIZ_REL'                                  = 8,

  /**
   * True if the horizontal position (absolute) estimate is good
   */
  'POS_HORIZ_ABS'                                  = 16,

  /**
   * True if the vertical position (absolute) estimate is good
   */
  'POS_VERT_ABS'                                   = 32,

  /**
   * True if the vertical position (above ground) estimate is good
   */
  'POS_VERT_AGL'                                   = 64,

  /**
   * True if the EKF is in a constant position mode and is not using external measurements (eg GPS or
   * optical flow)
   */
  'CONST_POS_MODE'                                 = 128,

  /**
   * True if the EKF has sufficient data to enter a mode that will provide a (relative) position estimate
   */
  'PRED_POS_HORIZ_REL'                             = 256,

  /**
   * True if the EKF has sufficient data to enter a mode that will provide a (absolute) position estimate
   */
  'PRED_POS_HORIZ_ABS'                             = 512,

  /**
   * True if the EKF has detected a GPS glitch
   */
  'GPS_GLITCH'                                     = 1024,

  /**
   * True if the EKF has detected bad accelerometer data
   */
  'ACCEL_ERROR'                                    = 2048,
}

/**
 * Sequence that motors are tested when using MAV_CMD_DO_MOTOR_TEST.
 */
export enum MotorTestOrder {
  /**
   * Default autopilot motor test method.
   */
  'DEFAULT'                                        = 0,

  /**
   * Motor numbers are specified as their index in a predefined vehicle-specific sequence.
   */
  'SEQUENCE'                                       = 1,

  /**
   * Motor numbers are specified as the output as labeled on the board.
   */
  'BOARD'                                          = 2,
}

/**
 * Defines how throttle value is represented in MAV_CMD_DO_MOTOR_TEST.
 */
export enum MotorTestThrottleType {
  /**
   * Throttle as a percentage (0 ~ 100)
   */
  'THROTTLE_PERCENT'                               = 0,

  /**
   * Throttle as an absolute PWM value (normally in range of 1000~2000).
   */
  'THROTTLE_PWM'                                   = 1,

  /**
   * Throttle pass-through from pilot's transmitter.
   */
  'THROTTLE_PILOT'                                 = 2,

  /**
   * Per-motor compass calibration test.
   */
  'COMPASS_CAL'                                    = 3,
}

/**
 * GPS_INPUT_IGNORE_FLAGS
 */
export enum GpsInputIgnoreFlags {
  /**
   * ignore altitude field
   */
  'ALT'                                            = 1,

  /**
   * ignore hdop field
   */
  'HDOP'                                           = 2,

  /**
   * ignore vdop field
   */
  'VDOP'                                           = 4,

  /**
   * ignore horizontal velocity field (vn and ve)
   */
  'VEL_HORIZ'                                      = 8,

  /**
   * ignore vertical velocity field (vd)
   */
  'VEL_VERT'                                       = 16,

  /**
   * ignore speed accuracy field
   */
  'SPEED_ACCURACY'                                 = 32,

  /**
   * ignore horizontal accuracy field
   */
  'HORIZONTAL_ACCURACY'                            = 64,

  /**
   * ignore vertical accuracy field
   */
  'VERTICAL_ACCURACY'                              = 128,
}

/**
 * Possible actions an aircraft can take to avoid a collision.
 */
export enum MavCollisionAction {
  /**
   * Ignore any potential collisions
   */
  'NONE'                                           = 0,

  /**
   * Report potential collision
   */
  'REPORT'                                         = 1,

  /**
   * Ascend or Descend to avoid threat
   */
  'ASCEND_OR_DESCEND'                              = 2,

  /**
   * Move horizontally to avoid threat
   */
  'MOVE_HORIZONTALLY'                              = 3,

  /**
   * Aircraft to move perpendicular to the collision's velocity vector
   */
  'MOVE_PERPENDICULAR'                             = 4,

  /**
   * Aircraft to fly directly back to its launch point
   */
  'RTL'                                            = 5,

  /**
   * Aircraft to stop in place
   */
  'HOVER'                                          = 6,
}

/**
 * Aircraft-rated danger from this threat.
 */
export enum MavCollisionThreatLevel {
  /**
   * Not a threat
   */
  'NONE'                                           = 0,

  /**
   * Craft is mildly concerned about this threat
   */
  'LOW'                                            = 1,

  /**
   * Craft is panicking, and may take actions to avoid threat
   */
  'HIGH'                                           = 2,
}

/**
 * Source of information about this collision.
 */
export enum MavCollisionSrc {
  /**
   * ID field references ADSB_VEHICLE packets
   */
  'ADSB'                                           = 0,

  /**
   * ID field references MAVLink SRC ID
   */
  'MAVLINK_GPS_GLOBAL_INT'                         = 1,
}

/**
 * Type of GPS fix
 */
export enum GpsFixType {
  /**
   * No GPS connected
   */
  'NO_GPS'                                         = 0,

  /**
   * No position information, GPS is connected
   */
  'NO_FIX'                                         = 1,

  /**
   * 2D position
   */
  'GPS_FIX_TYPE_2D_FIX'                            = 2,

  /**
   * 3D position
   */
  'GPS_FIX_TYPE_3D_FIX'                            = 3,

  /**
   * DGPS/SBAS aided 3D position
   */
  'DGPS'                                           = 4,

  /**
   * RTK float, 3D position
   */
  'RTK_FLOAT'                                      = 5,

  /**
   * RTK Fixed, 3D position
   */
  'RTK_FIXED'                                      = 6,

  /**
   * Static fixed, typically used for base stations
   */
  'STATIC'                                         = 7,

  /**
   * PPP, 3D position.
   */
  'PPP'                                            = 8,
}

/**
 * RTK GPS baseline coordinate system, used for RTK corrections
 */
export enum RtkBaselineCoordinateSystem {
  /**
   * Earth-centered, Earth-fixed
   */
  'ECEF'                                           = 0,

  /**
   * RTK basestation centered, north, east, down
   */
  'NED'                                            = 1,
}

/**
 * Type of landing target
 */
export enum LandingTargetType {
  /**
   * Landing target signaled by light beacon (ex: IR-LOCK)
   */
  'LIGHT_BEACON'                                   = 0,

  /**
   * Landing target signaled by radio beacon (ex: ILS, NDB)
   */
  'RADIO_BEACON'                                   = 1,

  /**
   * Landing target represented by a fiducial marker (ex: ARTag)
   */
  'VISION_FIDUCIAL'                                = 2,

  /**
   * Landing target represented by a pre-defined visual shape/feature (ex: X-marker, H-marker, square)
   */
  'VISION_OTHER'                                   = 3,
}

/**
 * Direction of VTOL transition
 */
export enum VtolTransitionHeading {
  /**
   * Respect the heading configuration of the vehicle.
   */
  'VEHICLE_DEFAULT'                                = 0,

  /**
   * Use the heading pointing towards the next waypoint.
   */
  'NEXT_WAYPOINT'                                  = 1,

  /**
   * Use the heading on takeoff (while sitting on the ground).
   */
  'TAKEOFF'                                        = 2,

  /**
   * Use the specified heading in parameter 4.
   */
  'SPECIFIED'                                      = 3,

  /**
   * Use the current heading when reaching takeoff altitude (potentially facing the wind when
   * weather-vaning is active).
   */
  'ANY'                                            = 4,
}

/**
 * Camera capability flags (Bitmap)
 */
export enum CameraCapFlags {
  /**
   * Camera is able to record video
   */
  'CAPTURE_VIDEO'                                  = 1,

  /**
   * Camera is able to capture images
   */
  'CAPTURE_IMAGE'                                  = 2,

  /**
   * Camera has separate Video and Image/Photo modes (MAV_CMD_SET_CAMERA_MODE)
   */
  'HAS_MODES'                                      = 4,

  /**
   * Camera can capture images while in video mode
   */
  'CAN_CAPTURE_IMAGE_IN_VIDEO_MODE'                = 8,

  /**
   * Camera can capture videos while in Photo/Image mode
   */
  'CAN_CAPTURE_VIDEO_IN_IMAGE_MODE'                = 16,

  /**
   * Camera has image survey mode (MAV_CMD_SET_CAMERA_MODE)
   */
  'HAS_IMAGE_SURVEY_MODE'                          = 32,

  /**
   * Camera has basic zoom control (MAV_CMD_SET_CAMERA_ZOOM)
   */
  'HAS_BASIC_ZOOM'                                 = 64,

  /**
   * Camera has basic focus control (MAV_CMD_SET_CAMERA_FOCUS)
   */
  'HAS_BASIC_FOCUS'                                = 128,

  /**
   * Camera has video streaming capabilities (request VIDEO_STREAM_INFORMATION with
   * MAV_CMD_REQUEST_MESSAGE for video streaming info)
   */
  'HAS_VIDEO_STREAM'                               = 256,

  /**
   * Camera supports tracking of a point on the camera view.
   */
  'HAS_TRACKING_POINT'                             = 512,

  /**
   * Camera supports tracking of a selection rectangle on the camera view.
   */
  'HAS_TRACKING_RECTANGLE'                         = 1024,

  /**
   * Camera supports tracking geo status (CAMERA_TRACKING_GEO_STATUS).
   */
  'HAS_TRACKING_GEO_STATUS'                        = 2048,
  'HAS_LASER'                                      = 4096,
  'HAS_THERMAL'                                    = 8192,
  'HAS_DOUBLE_VIDEO'                               = 16384,
  'HAS_GIMBAL_PITCH'                               = 32768,
  'HAS_GIMBAL_YAW'                                 = 65536,
  'HAS_GIMBAL_UP'                                  = 131072,
  'HAS_GIMBAL_DOWN'                                = 262144,
  'HAS_GIMBAL_CENTER'                              = 524288,
  'HAS_LED'                                        = 1048576,
  'HAS_FLIP'                                       = 2097152,
  'HAS_SWITCH'                                     = 4194304,
  'HAS_CALIBRATION'                                = 8388608,
  'HAS_ACK'                                        = 16777216,
  'SHOW_UI_BUTTON'                                 = 33554432,
  'GIMBAL_CTRL_SEND_WHEN_CHANGED'                  = 67108864,
  'TARGET_DETECTION'                               = 134217728,
  'HAS_OSD'                                        = 268435456,
  'SHOW_CAMERA_AREA'                               = 536870912,
  'TARGET_SELECTION_RECTANGLE'                     = 1073741824,
}

/**
 * Stream status flags (Bitmap)
 */
export enum VideoStreamStatusFlags {
  /**
   * Stream is active (running)
   */
  'RUNNING'                                        = 1,

  /**
   * Stream is thermal imaging
   */
  'THERMAL'                                        = 2,
}

/**
 * Video stream types
 */
export enum VideoStreamType {
  /**
   * Stream is RTSP
   */
  'RTSP'                                           = 0,

  /**
   * Stream is RTP UDP (URI gives the port number)
   */
  'RTPUDP'                                         = 1,

  /**
   * Stream is MPEG on TCP
   */
  'TCP_MPEG'                                       = 2,

  /**
   * Stream is h.264 on MPEG TS (URI gives the port number)
   */
  'MPEG_TS_H264'                                   = 3,
}

/**
 * Camera tracking status flags
 */
export enum CameraTrackingStatusFlags {
  /**
   * Camera is not tracking
   */
  'IDLE'                                           = 0,

  /**
   * Camera is tracking
   */
  'ACTIVE'                                         = 1,

  /**
   * Camera tracking in error state
   */
  'ERROR'                                          = 2,
}

/**
 * Camera tracking modes
 */
export enum CameraTrackingMode {
  /**
   * Not tracking
   */
  'NONE'                                           = 0,

  /**
   * Target is a point
   */
  'POINT'                                          = 1,

  /**
   * Target is a rectangle
   */
  'RECTANGLE'                                      = 2,
}

/**
 * Camera tracking target data (shows where tracked target is within image)
 */
export enum CameraTrackingTargetData {
  /**
   * No target data
   */
  'NONE'                                           = 0,

  /**
   * Target data embedded in image data (proprietary)
   */
  'EMBEDDED'                                       = 1,

  /**
   * Target data rendered in image
   */
  'RENDERED'                                       = 2,

  /**
   * Target data within status message (Point or Rectangle)
   */
  'IN_STATUS'                                      = 4,
}

/**
 * Zoom types for MAV_CMD_SET_CAMERA_ZOOM
 */
export enum CameraZoomType {
  /**
   * Zoom one step increment (-1 for wide, 1 for tele)
   */
  'STEP'                                           = 0,

  /**
   * Continuous zoom up/down until stopped (-1 for wide, 1 for tele, 0 to stop zooming)
   */
  'CONTINUOUS'                                     = 1,

  /**
   * Zoom value as proportion of full camera range (a value between 0.0 and 100.0)
   */
  'RANGE'                                          = 2,

  /**
   * Zoom value/variable focal length in milimetres. Note that there is no message to get the valid zoom
   * range of the camera, so this can type can only be used for cameras where the zoom range is known
   * (implying that this cannot reliably be used in a GCS for an arbitrary camera)
   */
  'FOCAL_LENGTH'                                   = 3,
}

/**
 * Focus types for MAV_CMD_SET_CAMERA_FOCUS
 */
export enum SetFocusType {
  /**
   * Focus one step increment (-1 for focusing in, 1 for focusing out towards infinity).
   */
  'STEP'                                           = 0,

  /**
   * Continuous focus up/down until stopped (-1 for focusing in, 1 for focusing out towards infinity, 0
   * to stop focusing)
   */
  'CONTINUOUS'                                     = 1,

  /**
   * Focus value as proportion of full camera focus range (a value between 0.0 and 100.0)
   */
  'RANGE'                                          = 2,

  /**
   * Focus value in metres. Note that there is no message to get the valid focus range of the camera, so
   * this can type can only be used for cameras where the range is known (implying that this cannot
   * reliably be used in a GCS for an arbitrary camera).
   */
  'METERS'                                         = 3,

  /**
   * Focus automatically.
   */
  'AUTO'                                           = 4,

  /**
   * Single auto focus. Mainly used for still pictures. Usually abbreviated as AF-S.
   */
  'AUTO_SINGLE'                                    = 5,

  /**
   * Continuous auto focus. Mainly used for dynamic scenes. Abbreviated as AF-C.
   */
  'AUTO_CONTINUOUS'                                = 6,
}

/**
 * Result from PARAM_EXT_SET message (or a PARAM_SET within a transaction).
 */
export enum ParamAck {
  /**
   * Parameter value ACCEPTED and SET
   */
  'ACCEPTED'                                       = 0,

  /**
   * Parameter value UNKNOWN/UNSUPPORTED
   */
  'VALUE_UNSUPPORTED'                              = 1,

  /**
   * Parameter failed to set
   */
  'FAILED'                                         = 2,

  /**
   * Parameter value received but not yet set/accepted. A subsequent PARAM_ACK_TRANSACTION or
   * PARAM_EXT_ACK with the final result will follow once operation is completed. This is returned
   * immediately for parameters that take longer to set, indicating taht the the parameter was recieved
   * and does not need to be resent.
   */
  'IN_PROGRESS'                                    = 3,
}

/**
 * Camera Modes.
 */
export enum CameraMode {
  /**
   * Camera is in image/photo capture mode.
   */
  'IMAGE'                                          = 0,

  /**
   * Camera is in video capture mode.
   */
  'VIDEO'                                          = 1,

  /**
   * Camera is in image survey capture mode. It allows for camera controller to do specific settings for
   * surveys.
   */
  'IMAGE_SURVEY'                                   = 2,
}

/**
 * MAV_ARM_AUTH_DENIED_REASON
 */
export enum MavArmAuthDeniedReason {
  /**
   * Not a specific reason
   */
  'GENERIC'                                        = 0,

  /**
   * Authorizer will send the error as string to GCS
   */
  'NONE'                                           = 1,

  /**
   * At least one waypoint have a invalid value
   */
  'INVALID_WAYPOINT'                               = 2,

  /**
   * Timeout in the authorizer process(in case it depends on network)
   */
  'TIMEOUT'                                        = 3,

  /**
   * Airspace of the mission in use by another vehicle, second result parameter can have the waypoint id
   * that caused it to be denied.
   */
  'AIRSPACE_IN_USE'                                = 4,

  /**
   * Weather is not good to fly
   */
  'BAD_WEATHER'                                    = 5,
}

/**
 * RC type
 */
export enum RcType {
  /**
   * Spektrum DSM2
   */
  'SPEKTRUM_DSM2'                                  = 0,

  /**
   * Spektrum DSMX
   */
  'SPEKTRUM_DSMX'                                  = 1,
}

/**
 * Bitmap to indicate which dimensions should be ignored by the vehicle: a value of 0b0000000000000000
 * or 0b0000001000000000 indicates that none of the setpoint dimensions should be ignored. If bit 9 is
 * set the floats afx afy afz should be interpreted as force instead of acceleration.
 */
export enum PositionTargetTypemask {
  /**
   * Ignore position x
   */
  'X_IGNORE'                                       = 1,

  /**
   * Ignore position y
   */
  'Y_IGNORE'                                       = 2,

  /**
   * Ignore position z
   */
  'Z_IGNORE'                                       = 4,

  /**
   * Ignore velocity x
   */
  'VX_IGNORE'                                      = 8,

  /**
   * Ignore velocity y
   */
  'VY_IGNORE'                                      = 16,

  /**
   * Ignore velocity z
   */
  'VZ_IGNORE'                                      = 32,

  /**
   * Ignore acceleration x
   */
  'AX_IGNORE'                                      = 64,

  /**
   * Ignore acceleration y
   */
  'AY_IGNORE'                                      = 128,

  /**
   * Ignore acceleration z
   */
  'AZ_IGNORE'                                      = 256,

  /**
   * Use force instead of acceleration
   */
  'FORCE_SET'                                      = 512,

  /**
   * Ignore yaw
   */
  'YAW_IGNORE'                                     = 1024,

  /**
   * Ignore yaw rate
   */
  'YAW_RATE_IGNORE'                                = 2048,
}

/**
 * Bitmap to indicate which dimensions should be ignored by the vehicle: a value of 0b00000000
 * indicates that none of the setpoint dimensions should be ignored.
 */
export enum AttitudeTargetTypemask {
  /**
   * Ignore body roll rate
   */
  'BODY_ROLL_RATE_IGNORE'                          = 1,

  /**
   * Ignore body pitch rate
   */
  'BODY_PITCH_RATE_IGNORE'                         = 2,

  /**
   * Ignore body yaw rate
   */
  'BODY_YAW_RATE_IGNORE'                           = 4,

  /**
   * Use 3D body thrust setpoint instead of throttle
   */
  'THRUST_BODY_SET'                                = 32,

  /**
   * Ignore throttle
   */
  'THROTTLE_IGNORE'                                = 64,

  /**
   * Ignore attitude
   */
  'ATTITUDE_IGNORE'                                = 128,
}

/**
 * Airborne status of UAS.
 */
export enum UtmFlightState {
  /**
   * The flight state can't be determined.
   */
  'UNKNOWN'                                        = 1,

  /**
   * UAS on ground.
   */
  'GROUND'                                         = 2,

  /**
   * UAS airborne.
   */
  'AIRBORNE'                                       = 3,

  /**
   * UAS is in an emergency flight state.
   */
  'EMERGENCY'                                      = 16,

  /**
   * UAS has no active controls.
   */
  'NOCTRL'                                         = 32,
}

/**
 * Flags for the global position report.
 */
export enum UtmDataAvailFlags {
  /**
   * The field time contains valid data.
   */
  'TIME_VALID'                                     = 1,

  /**
   * The field uas_id contains valid data.
   */
  'UAS_ID_AVAILABLE'                               = 2,

  /**
   * The fields lat, lon and h_acc contain valid data.
   */
  'POSITION_AVAILABLE'                             = 4,

  /**
   * The fields alt and v_acc contain valid data.
   */
  'ALTITUDE_AVAILABLE'                             = 8,

  /**
   * The field relative_alt contains valid data.
   */
  'RELATIVE_ALTITUDE_AVAILABLE'                    = 16,

  /**
   * The fields vx and vy contain valid data.
   */
  'HORIZONTAL_VELO_AVAILABLE'                      = 32,

  /**
   * The field vz contains valid data.
   */
  'VERTICAL_VELO_AVAILABLE'                        = 64,

  /**
   * The fields next_lat, next_lon and next_alt contain valid data.
   */
  'NEXT_WAYPOINT_AVAILABLE'                        = 128,
}

/**
 * These flags encode the cellular network status
 */
export enum CellularStatusFlag {
  /**
   * State unknown or not reportable.
   */
  'UNKNOWN'                                        = 0,

  /**
   * Modem is unusable
   */
  'FAILED'                                         = 1,

  /**
   * Modem is being initialized
   */
  'INITIALIZING'                                   = 2,

  /**
   * Modem is locked
   */
  'LOCKED'                                         = 3,

  /**
   * Modem is not enabled and is powered down
   */
  'DISABLED'                                       = 4,

  /**
   * Modem is currently transitioning to the CELLULAR_STATUS_FLAG_DISABLED state
   */
  'DISABLING'                                      = 5,

  /**
   * Modem is currently transitioning to the CELLULAR_STATUS_FLAG_ENABLED state
   */
  'ENABLING'                                       = 6,

  /**
   * Modem is enabled and powered on but not registered with a network provider and not available for
   * data connections
   */
  'ENABLED'                                        = 7,

  /**
   * Modem is searching for a network provider to register
   */
  'SEARCHING'                                      = 8,

  /**
   * Modem is registered with a network provider, and data connections and messaging may be available for
   * use
   */
  'REGISTERED'                                     = 9,

  /**
   * Modem is disconnecting and deactivating the last active packet data bearer. This state will not be
   * entered if more than one packet data bearer is active and one of the active bearers is deactivated
   */
  'DISCONNECTING'                                  = 10,

  /**
   * Modem is activating and connecting the first packet data bearer. Subsequent bearer activations when
   * another bearer is already active do not cause this state to be entered
   */
  'CONNECTING'                                     = 11,

  /**
   * One or more packet data bearers is active and connected
   */
  'CONNECTED'                                      = 12,
}

/**
 * These flags are used to diagnose the failure state of CELLULAR_STATUS
 */
export enum CellularNetworkFailedReason {
  /**
   * No error
   */
  'NONE'                                           = 0,

  /**
   * Error state is unknown
   */
  'UNKNOWN'                                        = 1,

  /**
   * SIM is required for the modem but missing
   */
  'SIM_MISSING'                                    = 2,

  /**
   * SIM is available, but not usuable for connection
   */
  'SIM_ERROR'                                      = 3,
}

/**
 * Cellular network radio type
 */
export enum CellularNetworkRadioType {
  'NONE'                                           = 0,
  'GSM'                                            = 1,
  'CDMA'                                           = 2,
  'WCDMA'                                          = 3,
  'LTE'                                            = 4,
}

/**
 * Precision land modes (used in MAV_CMD_NAV_LAND).
 */
export enum PrecisionLandMode {
  /**
   * Normal (non-precision) landing.
   */
  'DISABLED'                                       = 0,

  /**
   * Use precision landing if beacon detected when land command accepted, otherwise land normally.
   */
  'OPPORTUNISTIC'                                  = 1,

  /**
   * Use precision landing, searching for beacon if not found when land command accepted (land normally
   * if beacon cannot be found).
   */
  'REQUIRED'                                       = 2,
}

/**
 * Parachute actions. Trigger release and enable/disable auto-release.
 */
export enum ParachuteAction {
  /**
   * Disable auto-release of parachute (i.e. release triggered by crash detectors).
   */
  'DISABLE'                                        = 0,

  /**
   * Enable auto-release of parachute.
   */
  'ENABLE'                                         = 1,

  /**
   * Release parachute and kill motors.
   */
  'RELEASE'                                        = 2,
}

/**
 * MAV_TUNNEL_PAYLOAD_TYPE
 */
export enum MavTunnelPayloadType {
  /**
   * Encoding of payload unknown.
   */
  'UNKNOWN'                                        = 0,

  /**
   * Registered for STorM32 gimbal controller.
   */
  'STORM32_RESERVED0'                              = 200,

  /**
   * Registered for STorM32 gimbal controller.
   */
  'STORM32_RESERVED1'                              = 201,

  /**
   * Registered for STorM32 gimbal controller.
   */
  'STORM32_RESERVED2'                              = 202,

  /**
   * Registered for STorM32 gimbal controller.
   */
  'STORM32_RESERVED3'                              = 203,

  /**
   * Registered for STorM32 gimbal controller.
   */
  'STORM32_RESERVED4'                              = 204,

  /**
   * Registered for STorM32 gimbal controller.
   */
  'STORM32_RESERVED5'                              = 205,

  /**
   * Registered for STorM32 gimbal controller.
   */
  'STORM32_RESERVED6'                              = 206,

  /**
   * Registered for STorM32 gimbal controller.
   */
  'STORM32_RESERVED7'                              = 207,

  /**
   * Registered for STorM32 gimbal controller.
   */
  'STORM32_RESERVED8'                              = 208,

  /**
   * Registered for STorM32 gimbal controller.
   */
  'STORM32_RESERVED9'                              = 209,
}

/**
 * MAV_ODID_ID_TYPE
 */
export enum MavOdidIdType {
  /**
   * No type defined.
   */
  'NONE'                                           = 0,

  /**
   * Manufacturer Serial Number (ANSI/CTA-2063 format).
   */
  'SERIAL_NUMBER'                                  = 1,

  /**
   * CAA (Civil Aviation Authority) registered ID. Format: [ICAO Country Code].[CAA Assigned ID].
   */
  'CAA_REGISTRATION_ID'                            = 2,

  /**
   * UTM (Unmanned Traffic Management) assigned UUID (RFC4122).
   */
  'UTM_ASSIGNED_UUID'                              = 3,

  /**
   * A 20 byte ID for a specific flight/session. The exact ID type is indicated by the first byte of
   * uas_id and these type values are managed by ICAO.
   */
  'SPECIFIC_SESSION_ID'                            = 4,
}

/**
 * MAV_ODID_UA_TYPE
 */
export enum MavOdidUaType {
  /**
   * No UA (Unmanned Aircraft) type defined.
   */
  'NONE'                                           = 0,

  /**
   * Aeroplane/Airplane. Fixed wing.
   */
  'AEROPLANE'                                      = 1,

  /**
   * Helicopter or multirotor.
   */
  'HELICOPTER_OR_MULTIROTOR'                       = 2,

  /**
   * Gyroplane.
   */
  'GYROPLANE'                                      = 3,

  /**
   * VTOL (Vertical Take-Off and Landing). Fixed wing aircraft that can take off vertically.
   */
  'HYBRID_LIFT'                                    = 4,

  /**
   * Ornithopter.
   */
  'ORNITHOPTER'                                    = 5,

  /**
   * Glider.
   */
  'GLIDER'                                         = 6,

  /**
   * Kite.
   */
  'KITE'                                           = 7,

  /**
   * Free Balloon.
   */
  'FREE_BALLOON'                                   = 8,

  /**
   * Captive Balloon.
   */
  'CAPTIVE_BALLOON'                                = 9,

  /**
   * Airship. E.g. a blimp.
   */
  'AIRSHIP'                                        = 10,

  /**
   * Free Fall/Parachute (unpowered).
   */
  'FREE_FALL_PARACHUTE'                            = 11,

  /**
   * Rocket.
   */
  'ROCKET'                                         = 12,

  /**
   * Tethered powered aircraft.
   */
  'TETHERED_POWERED_AIRCRAFT'                      = 13,

  /**
   * Ground Obstacle.
   */
  'GROUND_OBSTACLE'                                = 14,

  /**
   * Other type of aircraft not listed earlier.
   */
  'OTHER'                                          = 15,
}

/**
 * MAV_ODID_STATUS
 */
export enum MavOdidStatus {
  /**
   * The status of the (UA) Unmanned Aircraft is undefined.
   */
  'UNDECLARED'                                     = 0,

  /**
   * The UA is on the ground.
   */
  'GROUND'                                         = 1,

  /**
   * The UA is in the air.
   */
  'AIRBORNE'                                       = 2,

  /**
   * The UA is having an emergency.
   */
  'EMERGENCY'                                      = 3,
}

/**
 * MAV_ODID_HEIGHT_REF
 */
export enum MavOdidHeightRef {
  /**
   * The height field is relative to the take-off location.
   */
  'OVER_TAKEOFF'                                   = 0,

  /**
   * The height field is relative to ground.
   */
  'OVER_GROUND'                                    = 1,
}

/**
 * MAV_ODID_HOR_ACC
 */
export enum MavOdidHorAcc {
  /**
   * The horizontal accuracy is unknown.
   */
  'UNKNOWN'                                        = 0,

  /**
   * The horizontal accuracy is smaller than 10 Nautical Miles. 18.52 km.
   */
  'MAV_ODID_HOR_ACC_10NM'                          = 1,

  /**
   * The horizontal accuracy is smaller than 4 Nautical Miles. 7.408 km.
   */
  'MAV_ODID_HOR_ACC_4NM'                           = 2,

  /**
   * The horizontal accuracy is smaller than 2 Nautical Miles. 3.704 km.
   */
  'MAV_ODID_HOR_ACC_2NM'                           = 3,

  /**
   * The horizontal accuracy is smaller than 1 Nautical Miles. 1.852 km.
   */
  'MAV_ODID_HOR_ACC_1NM'                           = 4,

  /**
   * The horizontal accuracy is smaller than 0.5 Nautical Miles. 926 m.
   */
  'MAV_ODID_HOR_ACC_0_5NM'                         = 5,

  /**
   * The horizontal accuracy is smaller than 0.3 Nautical Miles. 555.6 m.
   */
  'MAV_ODID_HOR_ACC_0_3NM'                         = 6,

  /**
   * The horizontal accuracy is smaller than 0.1 Nautical Miles. 185.2 m.
   */
  'MAV_ODID_HOR_ACC_0_1NM'                         = 7,

  /**
   * The horizontal accuracy is smaller than 0.05 Nautical Miles. 92.6 m.
   */
  'MAV_ODID_HOR_ACC_0_05NM'                        = 8,

  /**
   * The horizontal accuracy is smaller than 30 meter.
   */
  'MAV_ODID_HOR_ACC_30_METER'                      = 9,

  /**
   * The horizontal accuracy is smaller than 10 meter.
   */
  'MAV_ODID_HOR_ACC_10_METER'                      = 10,

  /**
   * The horizontal accuracy is smaller than 3 meter.
   */
  'MAV_ODID_HOR_ACC_3_METER'                       = 11,

  /**
   * The horizontal accuracy is smaller than 1 meter.
   */
  'MAV_ODID_HOR_ACC_1_METER'                       = 12,
}

/**
 * MAV_ODID_VER_ACC
 */
export enum MavOdidVerAcc {
  /**
   * The vertical accuracy is unknown.
   */
  'UNKNOWN'                                        = 0,

  /**
   * The vertical accuracy is smaller than 150 meter.
   */
  'MAV_ODID_VER_ACC_150_METER'                     = 1,

  /**
   * The vertical accuracy is smaller than 45 meter.
   */
  'MAV_ODID_VER_ACC_45_METER'                      = 2,

  /**
   * The vertical accuracy is smaller than 25 meter.
   */
  'MAV_ODID_VER_ACC_25_METER'                      = 3,

  /**
   * The vertical accuracy is smaller than 10 meter.
   */
  'MAV_ODID_VER_ACC_10_METER'                      = 4,

  /**
   * The vertical accuracy is smaller than 3 meter.
   */
  'MAV_ODID_VER_ACC_3_METER'                       = 5,

  /**
   * The vertical accuracy is smaller than 1 meter.
   */
  'MAV_ODID_VER_ACC_1_METER'                       = 6,
}

/**
 * MAV_ODID_SPEED_ACC
 */
export enum MavOdidSpeedAcc {
  /**
   * The speed accuracy is unknown.
   */
  'UNKNOWN'                                        = 0,

  /**
   * The speed accuracy is smaller than 10 meters per second.
   */
  'MAV_ODID_SPEED_ACC_10_METERS_PER_SECOND'        = 1,

  /**
   * The speed accuracy is smaller than 3 meters per second.
   */
  'MAV_ODID_SPEED_ACC_3_METERS_PER_SECOND'         = 2,

  /**
   * The speed accuracy is smaller than 1 meters per second.
   */
  'MAV_ODID_SPEED_ACC_1_METERS_PER_SECOND'         = 3,

  /**
   * The speed accuracy is smaller than 0.3 meters per second.
   */
  'MAV_ODID_SPEED_ACC_0_3_METERS_PER_SECOND'       = 4,
}

/**
 * MAV_ODID_TIME_ACC
 */
export enum MavOdidTimeAcc {
  /**
   * The timestamp accuracy is unknown.
   */
  'UNKNOWN'                                        = 0,

  /**
   * The timestamp accuracy is smaller than or equal to 0.1 second.
   */
  'MAV_ODID_TIME_ACC_0_1_SECOND'                   = 1,

  /**
   * The timestamp accuracy is smaller than or equal to 0.2 second.
   */
  'MAV_ODID_TIME_ACC_0_2_SECOND'                   = 2,

  /**
   * The timestamp accuracy is smaller than or equal to 0.3 second.
   */
  'MAV_ODID_TIME_ACC_0_3_SECOND'                   = 3,

  /**
   * The timestamp accuracy is smaller than or equal to 0.4 second.
   */
  'MAV_ODID_TIME_ACC_0_4_SECOND'                   = 4,

  /**
   * The timestamp accuracy is smaller than or equal to 0.5 second.
   */
  'MAV_ODID_TIME_ACC_0_5_SECOND'                   = 5,

  /**
   * The timestamp accuracy is smaller than or equal to 0.6 second.
   */
  'MAV_ODID_TIME_ACC_0_6_SECOND'                   = 6,

  /**
   * The timestamp accuracy is smaller than or equal to 0.7 second.
   */
  'MAV_ODID_TIME_ACC_0_7_SECOND'                   = 7,

  /**
   * The timestamp accuracy is smaller than or equal to 0.8 second.
   */
  'MAV_ODID_TIME_ACC_0_8_SECOND'                   = 8,

  /**
   * The timestamp accuracy is smaller than or equal to 0.9 second.
   */
  'MAV_ODID_TIME_ACC_0_9_SECOND'                   = 9,

  /**
   * The timestamp accuracy is smaller than or equal to 1.0 second.
   */
  'MAV_ODID_TIME_ACC_1_0_SECOND'                   = 10,

  /**
   * The timestamp accuracy is smaller than or equal to 1.1 second.
   */
  'MAV_ODID_TIME_ACC_1_1_SECOND'                   = 11,

  /**
   * The timestamp accuracy is smaller than or equal to 1.2 second.
   */
  'MAV_ODID_TIME_ACC_1_2_SECOND'                   = 12,

  /**
   * The timestamp accuracy is smaller than or equal to 1.3 second.
   */
  'MAV_ODID_TIME_ACC_1_3_SECOND'                   = 13,

  /**
   * The timestamp accuracy is smaller than or equal to 1.4 second.
   */
  'MAV_ODID_TIME_ACC_1_4_SECOND'                   = 14,

  /**
   * The timestamp accuracy is smaller than or equal to 1.5 second.
   */
  'MAV_ODID_TIME_ACC_1_5_SECOND'                   = 15,
}

/**
 * MAV_ODID_AUTH_TYPE
 */
export enum MavOdidAuthType {
  /**
   * No authentication type is specified.
   */
  'NONE'                                           = 0,

  /**
   * Signature for the UAS (Unmanned Aircraft System) ID.
   */
  'UAS_ID_SIGNATURE'                               = 1,

  /**
   * Signature for the Operator ID.
   */
  'OPERATOR_ID_SIGNATURE'                          = 2,

  /**
   * Signature for the entire message set.
   */
  'MESSAGE_SET_SIGNATURE'                          = 3,

  /**
   * Authentication is provided by Network Remote ID.
   */
  'NETWORK_REMOTE_ID'                              = 4,

  /**
   * The exact authentication type is indicated by the first byte of authentication_data and these type
   * values are managed by ICAO.
   */
  'SPECIFIC_AUTHENTICATION'                        = 5,
}

/**
 * MAV_ODID_DESC_TYPE
 */
export enum MavOdidDescType {
  /**
   * Free-form text description of the purpose of the flight.
   */
  'TEXT'                                           = 0,
}

/**
 * MAV_ODID_OPERATOR_LOCATION_TYPE
 */
export enum MavOdidOperatorLocationType {
  /**
   * The location of the operator is the same as the take-off location.
   */
  'TAKEOFF'                                        = 0,

  /**
   * The location of the operator is based on live GNSS data.
   */
  'LIVE_GNSS'                                      = 1,

  /**
   * The location of the operator is a fixed location.
   */
  'FIXED'                                          = 2,
}

/**
 * MAV_ODID_CLASSIFICATION_TYPE
 */
export enum MavOdidClassificationType {
  /**
   * The classification type for the UA is undeclared.
   */
  'UNDECLARED'                                     = 0,

  /**
   * The classification type for the UA follows EU (European Union) specifications.
   */
  'EU'                                             = 1,
}

/**
 * MAV_ODID_CATEGORY_EU
 */
export enum MavOdidCategoryEu {
  /**
   * The category for the UA, according to the EU specification, is undeclared.
   */
  'UNDECLARED'                                     = 0,

  /**
   * The category for the UA, according to the EU specification, is the Open category.
   */
  'OPEN'                                           = 1,

  /**
   * The category for the UA, according to the EU specification, is the Specific category.
   */
  'SPECIFIC'                                       = 2,

  /**
   * The category for the UA, according to the EU specification, is the Certified category.
   */
  'CERTIFIED'                                      = 3,
}

/**
 * MAV_ODID_CLASS_EU
 */
export enum MavOdidClassEu {
  /**
   * The class for the UA, according to the EU specification, is undeclared.
   */
  'UNDECLARED'                                     = 0,

  /**
   * The class for the UA, according to the EU specification, is Class 0.
   */
  'CLASS_0'                                        = 1,

  /**
   * The class for the UA, according to the EU specification, is Class 1.
   */
  'CLASS_1'                                        = 2,

  /**
   * The class for the UA, according to the EU specification, is Class 2.
   */
  'CLASS_2'                                        = 3,

  /**
   * The class for the UA, according to the EU specification, is Class 3.
   */
  'CLASS_3'                                        = 4,

  /**
   * The class for the UA, according to the EU specification, is Class 4.
   */
  'CLASS_4'                                        = 5,

  /**
   * The class for the UA, according to the EU specification, is Class 5.
   */
  'CLASS_5'                                        = 6,

  /**
   * The class for the UA, according to the EU specification, is Class 6.
   */
  'CLASS_6'                                        = 7,
}

/**
 * MAV_ODID_OPERATOR_ID_TYPE
 */
export enum MavOdidOperatorIdType {
  /**
   * CAA (Civil Aviation Authority) registered operator ID.
   */
  'CAA'                                            = 0,
}

/**
 * Tune formats (used for vehicle buzzer/tone generation).
 */
export enum TuneFormat {
  /**
   * Format is QBasic 1.1 Play: https://www.qbasic.net/en/reference/qb11/Statement/PLAY-006.htm.
   */
  'QBASIC1_1'                                      = 1,

  /**
   * Format is Modern Music Markup Language (MML):
   * https://en.wikipedia.org/wiki/Music_Macro_Language#Modern_MML.
   */
  'MML_MODERN'                                     = 2,
}

/**
 * Type of AIS vessel, enum duplicated from AIS standard, https://gpsd.gitlab.io/gpsd/AIVDM.html
 */
export enum AisType {
  /**
   * Not available (default).
   */
  'UNKNOWN'                                        = 0,
  'RESERVED_1'                                     = 1,
  'RESERVED_2'                                     = 2,
  'RESERVED_3'                                     = 3,
  'RESERVED_4'                                     = 4,
  'RESERVED_5'                                     = 5,
  'RESERVED_6'                                     = 6,
  'RESERVED_7'                                     = 7,
  'RESERVED_8'                                     = 8,
  'RESERVED_9'                                     = 9,
  'RESERVED_10'                                    = 10,
  'RESERVED_11'                                    = 11,
  'RESERVED_12'                                    = 12,
  'RESERVED_13'                                    = 13,
  'RESERVED_14'                                    = 14,
  'RESERVED_15'                                    = 15,
  'RESERVED_16'                                    = 16,
  'RESERVED_17'                                    = 17,
  'RESERVED_18'                                    = 18,
  'RESERVED_19'                                    = 19,

  /**
   * Wing In Ground effect.
   */
  'WIG'                                            = 20,
  'WIG_HAZARDOUS_A'                                = 21,
  'WIG_HAZARDOUS_B'                                = 22,
  'WIG_HAZARDOUS_C'                                = 23,
  'WIG_HAZARDOUS_D'                                = 24,
  'WIG_RESERVED_1'                                 = 25,
  'WIG_RESERVED_2'                                 = 26,
  'WIG_RESERVED_3'                                 = 27,
  'WIG_RESERVED_4'                                 = 28,
  'WIG_RESERVED_5'                                 = 29,
  'FISHING'                                        = 30,
  'TOWING'                                         = 31,

  /**
   * Towing: length exceeds 200m or breadth exceeds 25m.
   */
  'TOWING_LARGE'                                   = 32,

  /**
   * Dredging or other underwater ops.
   */
  'DREDGING'                                       = 33,
  'DIVING'                                         = 34,
  'MILITARY'                                       = 35,
  'SAILING'                                        = 36,
  'PLEASURE'                                       = 37,
  'RESERVED_20'                                    = 38,
  'RESERVED_21'                                    = 39,

  /**
   * High Speed Craft.
   */
  'HSC'                                            = 40,
  'HSC_HAZARDOUS_A'                                = 41,
  'HSC_HAZARDOUS_B'                                = 42,
  'HSC_HAZARDOUS_C'                                = 43,
  'HSC_HAZARDOUS_D'                                = 44,
  'HSC_RESERVED_1'                                 = 45,
  'HSC_RESERVED_2'                                 = 46,
  'HSC_RESERVED_3'                                 = 47,
  'HSC_RESERVED_4'                                 = 48,
  'HSC_UNKNOWN'                                    = 49,
  'PILOT'                                          = 50,

  /**
   * Search And Rescue vessel.
   */
  'SAR'                                            = 51,
  'TUG'                                            = 52,
  'PORT_TENDER'                                    = 53,

  /**
   * Anti-pollution equipment.
   */
  'ANTI_POLLUTION'                                 = 54,
  'LAW_ENFORCEMENT'                                = 55,
  'SPARE_LOCAL_1'                                  = 56,
  'SPARE_LOCAL_2'                                  = 57,
  'MEDICAL_TRANSPORT'                              = 58,

  /**
   * Noncombatant ship according to RR Resolution No. 18.
   */
  'NONECOMBATANT'                                  = 59,
  'PASSENGER'                                      = 60,
  'PASSENGER_HAZARDOUS_A'                          = 61,
  'PASSENGER_HAZARDOUS_B'                          = 62,
  'AIS_TYPE_PASSENGER_HAZARDOUS_C'                 = 63,
  'PASSENGER_HAZARDOUS_D'                          = 64,
  'PASSENGER_RESERVED_1'                           = 65,
  'PASSENGER_RESERVED_2'                           = 66,
  'PASSENGER_RESERVED_3'                           = 67,
  'AIS_TYPE_PASSENGER_RESERVED_4'                  = 68,
  'PASSENGER_UNKNOWN'                              = 69,
  'CARGO'                                          = 70,
  'CARGO_HAZARDOUS_A'                              = 71,
  'CARGO_HAZARDOUS_B'                              = 72,
  'CARGO_HAZARDOUS_C'                              = 73,
  'CARGO_HAZARDOUS_D'                              = 74,
  'CARGO_RESERVED_1'                               = 75,
  'CARGO_RESERVED_2'                               = 76,
  'CARGO_RESERVED_3'                               = 77,
  'CARGO_RESERVED_4'                               = 78,
  'CARGO_UNKNOWN'                                  = 79,
  'TANKER'                                         = 80,
  'TANKER_HAZARDOUS_A'                             = 81,
  'TANKER_HAZARDOUS_B'                             = 82,
  'TANKER_HAZARDOUS_C'                             = 83,
  'TANKER_HAZARDOUS_D'                             = 84,
  'TANKER_RESERVED_1'                              = 85,
  'TANKER_RESERVED_2'                              = 86,
  'TANKER_RESERVED_3'                              = 87,
  'TANKER_RESERVED_4'                              = 88,
  'TANKER_UNKNOWN'                                 = 89,
  'OTHER'                                          = 90,
  'OTHER_HAZARDOUS_A'                              = 91,
  'OTHER_HAZARDOUS_B'                              = 92,
  'OTHER_HAZARDOUS_C'                              = 93,
  'OTHER_HAZARDOUS_D'                              = 94,
  'OTHER_RESERVED_1'                               = 95,
  'OTHER_RESERVED_2'                               = 96,
  'OTHER_RESERVED_3'                               = 97,
  'OTHER_RESERVED_4'                               = 98,
  'OTHER_UNKNOWN'                                  = 99,
}

/**
 * Navigational status of AIS vessel, enum duplicated from AIS standard,
 * https://gpsd.gitlab.io/gpsd/AIVDM.html
 */
export enum AisNavStatus {
  /**
   * Under way using engine.
   */
  'UNDER_WAY'                                      = 0,
  'ANCHORED'                                       = 1,
  'UN_COMMANDED'                                   = 2,
  'RESTRICTED_MANOEUVERABILITY'                    = 3,
  'DRAUGHT_CONSTRAINED'                            = 4,
  'MOORED'                                         = 5,
  'AGROUND'                                        = 6,
  'FISHING'                                        = 7,
  'SAILING'                                        = 8,
  'RESERVED_HSC'                                   = 9,
  'RESERVED_WIG'                                   = 10,
  'RESERVED_1'                                     = 11,
  'RESERVED_2'                                     = 12,
  'RESERVED_3'                                     = 13,

  /**
   * Search And Rescue Transponder.
   */
  'AIS_SART'                                       = 14,

  /**
   * Not available (default).
   */
  'UNKNOWN'                                        = 15,
}

/**
 * These flags are used in the AIS_VESSEL.fields bitmask to indicate validity of data in the other
 * message fields. When set, the data is valid.
 */
export enum AisFlags {
  /**
   * 1 = Position accuracy less than 10m, 0 = position accuracy greater than 10m.
   */
  'POSITION_ACCURACY'                              = 1,
  'VALID_COG'                                      = 2,
  'VALID_VELOCITY'                                 = 4,

  /**
   * 1 = Velocity over 52.5765m/s (102.2 knots)
   */
  'HIGH_VELOCITY'                                  = 8,
  'VALID_TURN_RATE'                                = 16,

  /**
   * Only the sign of the returned turn rate value is valid, either greater than 5deg/30s or less than
   * -5deg/30s
   */
  'TURN_RATE_SIGN_ONLY'                            = 32,
  'VALID_DIMENSIONS'                               = 64,

  /**
   * Distance to bow is larger than 511m
   */
  'LARGE_BOW_DIMENSION'                            = 128,

  /**
   * Distance to stern is larger than 511m
   */
  'LARGE_STERN_DIMENSION'                          = 256,

  /**
   * Distance to port side is larger than 63m
   */
  'LARGE_PORT_DIMENSION'                           = 512,

  /**
   * Distance to starboard side is larger than 63m
   */
  'LARGE_STARBOARD_DIMENSION'                      = 1024,
  'VALID_CALLSIGN'                                 = 2048,
  'VALID_NAME'                                     = 4096,
}

/**
 * List of possible units where failures can be injected.
 */
export enum FailureUnit {
  'SENSOR_GYRO'                                    = 0,
  'SENSOR_ACCEL'                                   = 1,
  'SENSOR_MAG'                                     = 2,
  'SENSOR_BARO'                                    = 3,
  'SENSOR_GPS'                                     = 4,
  'SENSOR_OPTICAL_FLOW'                            = 5,
  'SENSOR_VIO'                                     = 6,
  'SENSOR_DISTANCE_SENSOR'                         = 7,
  'SENSOR_AIRSPEED'                                = 8,
  'SYSTEM_BATTERY'                                 = 100,
  'SYSTEM_MOTOR'                                   = 101,
  'SYSTEM_SERVO'                                   = 102,
  'SYSTEM_AVOIDANCE'                               = 103,
  'SYSTEM_RC_SIGNAL'                               = 104,
  'SYSTEM_MAVLINK_SIGNAL'                          = 105,
}

/**
 * List of possible failure type to inject.
 */
export enum FailureType {
  /**
   * No failure injected, used to reset a previous failure.
   */
  'OK'                                             = 0,

  /**
   * Sets unit off, so completely non-responsive.
   */
  'OFF'                                            = 1,

  /**
   * Unit is stuck e.g. keeps reporting the same value.
   */
  'STUCK'                                          = 2,

  /**
   * Unit is reporting complete garbage.
   */
  'GARBAGE'                                        = 3,

  /**
   * Unit is consistently wrong.
   */
  'WRONG'                                          = 4,

  /**
   * Unit is slow, so e.g. reporting at slower than expected rate.
   */
  'SLOW'                                           = 5,

  /**
   * Data of unit is delayed in time.
   */
  'DELAYED'                                        = 6,

  /**
   * Unit is sometimes working, sometimes not.
   */
  'INTERMITTENT'                                   = 7,
}

/**
 * NAV_VTOL_LAND_OPTIONS
 */
export enum NavVtolLandOptions {
  /**
   * Default autopilot landing behaviour.
   */
  'DEFAULT'                                        = 0,

  /**
   * Descend in fixed wing mode, transitioning to multicopter mode for vertical landing when close to the
   * ground.
 The fixed wing descent pattern is at the discretion of the vehicle (e.g. transition
   * altitude, loiter direction, radius, and speed, etc.).
   */
  'FW_DESCENT'                                     = 1,

  /**
   * Land in multicopter mode on reaching the landing co-ordinates (the whole landing is by "hover
   * descent").
   */
  'HOVER_DESCENT'                                  = 2,
}

/**
 * Winch status flags used in WINCH_STATUS
 */
export enum MavWinchStatusFlag {
  /**
   * Winch is healthy
   */
  'HEALTHY'                                        = 1,

  /**
   * Winch thread is fully retracted
   */
  'FULLY_RETRACTED'                                = 2,

  /**
   * Winch motor is moving
   */
  'MOVING'                                         = 4,

  /**
   * Winch clutch is engaged allowing motor to move freely
   */
  'CLUTCH_ENGAGED'                                 = 8,
}

/**
 * MAG_CAL_STATUS
 */
export enum MagCalStatus {
  'NOT_STARTED'                                    = 0,
  'WAITING_TO_START'                               = 1,
  'RUNNING_STEP_ONE'                               = 2,
  'RUNNING_STEP_TWO'                               = 3,
  'SUCCESS'                                        = 4,
  'FAILED'                                         = 5,
  'BAD_ORIENTATION'                                = 6,
  'BAD_RADIUS'                                     = 7,
}

/**
 * Reason for an event error response.
 */
export enum MavEventErrorReason {
  /**
   * The requested event is not available (anymore).
   */
  'UNAVAILABLE'                                    = 0,
}

/**
 * Flags for CURRENT_EVENT_SEQUENCE.
 */
export enum MavEventCurrentSequenceFlags {
  /**
   * A sequence reset has happened (e.g. vehicle reboot).
   */
  'RESET'                                          = 1,
}

/**
 * Flags in the HIL_SENSOR message indicate which fields have updated since the last message
 */
export enum HilSensorUpdatedFlags {
  /**
   * None of the fields in HIL_SENSOR have been updated
   */
  'NONE'                                           = 0,

  /**
   * The value in the xacc field has been updated
   */
  'XACC'                                           = 1,

  /**
   * The value in the yacc field has been updated
   */
  'YACC'                                           = 2,

  /**
   * The value in the zacc field has been updated
   */
  'ZACC'                                           = 4,

  /**
   * The value in the xgyro field has been updated
   */
  'XGYRO'                                          = 8,

  /**
   * The value in the ygyro field has been updated
   */
  'YGYRO'                                          = 16,

  /**
   * The value in the zgyro field has been updated
   */
  'ZGYRO'                                          = 32,

  /**
   * The value in the xmag field has been updated
   */
  'XMAG'                                           = 64,

  /**
   * The value in the ymag field has been updated
   */
  'YMAG'                                           = 128,

  /**
   * The value in the zmag field has been updated
   */
  'ZMAG'                                           = 256,

  /**
   * The value in the abs_pressure field has been updated
   */
  'ABS_PRESSURE'                                   = 512,

  /**
   * The value in the diff_pressure field has been updated
   */
  'DIFF_PRESSURE'                                  = 1024,

  /**
   * The value in the pressure_alt field has been updated
   */
  'PRESSURE_ALT'                                   = 2048,

  /**
   * The value in the temperature field has been updated
   */
  'TEMPERATURE'                                    = 4096,

  /**
   * Full reset of attitude/position/velocities/etc was performed in sim (Bit 31).
   */
  'RESET'                                          = 2147483648,
}

/**
 * Flags in the HIGHRES_IMU message indicate which fields have updated since the last message
 */
export enum HighresImuUpdatedFlags {
  /**
   * None of the fields in HIGHRES_IMU have been updated
   */
  'NONE'                                           = 0,

  /**
   * The value in the xacc field has been updated
   */
  'XACC'                                           = 1,

  /**
   * The value in the yacc field has been updated
   */
  'YACC'                                           = 2,

  /**
   * The value in the zacc field has been updated since
   */
  'ZACC'                                           = 4,

  /**
   * The value in the xgyro field has been updated
   */
  'XGYRO'                                          = 8,

  /**
   * The value in the ygyro field has been updated
   */
  'YGYRO'                                          = 16,

  /**
   * The value in the zgyro field has been updated
   */
  'ZGYRO'                                          = 32,

  /**
   * The value in the xmag field has been updated
   */
  'XMAG'                                           = 64,

  /**
   * The value in the ymag field has been updated
   */
  'YMAG'                                           = 128,

  /**
   * The value in the zmag field has been updated
   */
  'ZMAG'                                           = 256,

  /**
   * The value in the abs_pressure field has been updated
   */
  'ABS_PRESSURE'                                   = 512,

  /**
   * The value in the diff_pressure field has been updated
   */
  'DIFF_PRESSURE'                                  = 1024,

  /**
   * The value in the pressure_alt field has been updated
   */
  'PRESSURE_ALT'                                   = 2048,

  /**
   * The value in the temperature field has been updated
   */
  'TEMPERATURE'                                    = 4096,

  /**
   * All fields in HIGHRES_IMU have been updated.
   */
  'ALL'                                            = 65535,
}

/**
 * [object Object]
 */
export enum MavMissionCurrentFlag {
  /**
   * vehicle is inFlight
   */
  'IN_MISSION_FLIGHT'                              = 1,

  /**
   * resume waypoint info exist
   */
  'RESUME_WAYPOINT_EXIST'                          = 2,
}

/**
 * The general system state. If the system is following the MAVLink standard, the system state is
 * mainly defined by three orthogonal states/modes: The system mode, which is either LOCKED (motors
 * shut down and locked), MANUAL (system under RC control), GUIDED (system with autonomous position
 * control, position setpoint controlled manually) or AUTO (system guided by path/waypoint planner).
 * The NAV_MODE defined the current flight state: LIFTOFF (often an open-loop maneuver), LANDING,
 * WAYPOINTS or VECTOR. This represents the internal navigation state machine. The system status shows
 * whether the system is currently active or not and if an emergency occurred. During the CRITICAL and
 * EMERGENCY states the MAV is still considered to be active, but should start emergency procedures
 * autonomously. After a failure occurred it should first move from active to critical to allow manual
 * intervention and then move to emergency after a certain timeout.
 */
export class SysStatus extends MavLinkData {
  static MSG_ID = 1
  static MSG_NAME = 'SYS_STATUS'
  static PAYLOAD_LENGTH = 55
  static MAGIC_NUMBER = 124

  static FIELDS = [
    new MavLinkPacketField('onboard_control_sensors_present', 'onboardControlSensorsPresent', 0, false, 4, 'uint32_t', ''),
    new MavLinkPacketField('onboard_control_sensors_enabled', 'onboardControlSensorsEnabled', 4, false, 4, 'uint32_t', ''),
    new MavLinkPacketField('onboard_control_sensors_health', 'onboardControlSensorsHealth', 8, false, 4, 'uint32_t', ''),
    new MavLinkPacketField('load', 'load', 12, false, 2, 'uint16_t', 'd%'),
    new MavLinkPacketField('voltage_battery', 'voltageBattery', 14, false, 2, 'uint16_t', 'mV'),
    new MavLinkPacketField('current_battery', 'currentBattery', 16, false, 2, 'int16_t', 'cA'),
    new MavLinkPacketField('drop_rate_comm', 'dropRateComm', 18, false, 2, 'uint16_t', 'c%'),
    new MavLinkPacketField('errors_comm', 'errorsComm', 20, false, 2, 'uint16_t', ''),
    new MavLinkPacketField('errors_count1', 'errorsCount1', 22, false, 2, 'uint16_t', ''),
    new MavLinkPacketField('errors_count2', 'errorsCount2', 24, false, 2, 'uint16_t', ''),
    new MavLinkPacketField('errors_count3', 'errorsCount3', 26, false, 2, 'uint16_t', ''),
    new MavLinkPacketField('errors_count4', 'errorsCount4', 28, false, 2, 'uint16_t', ''),
    new MavLinkPacketField('battery_remaining', 'batteryRemaining', 30, false, 1, 'int8_t', '%'),
    new MavLinkPacketField('onboard_control_sensors_present_extended', 'onboardControlSensorsPresentExtended', 31, true, 4, 'uint32_t', ''),
    new MavLinkPacketField('onboard_control_sensors_enabled_extended', 'onboardControlSensorsEnabledExtended', 35, true, 4, 'uint32_t', ''),
    new MavLinkPacketField('onboard_control_sensors_health_extended', 'onboardControlSensorsHealthExtended', 39, true, 4, 'uint32_t', ''),
    new MavLinkPacketField('pos_Cflags', 'posCflags', 43, true, 4, 'uint32_t', ''),
    new MavLinkPacketField('pos_Aflags', 'posAflags', 47, true, 4, 'uint32_t', ''),
    new MavLinkPacketField('IMU_Cflags', 'IMUCflags', 51, true, 2, 'uint16_t', ''),
    new MavLinkPacketField('IMU_Aflags', 'IMUAflags', 53, true, 2, 'uint16_t', ''),
  ]

  constructor() {
    super()
    this.onboardControlSensorsPresent = MavSysStatusSensor[Object.keys(MavSysStatusSensor)[0]]
    this.onboardControlSensorsEnabled = MavSysStatusSensor[Object.keys(MavSysStatusSensor)[0]]
    this.onboardControlSensorsHealth = MavSysStatusSensor[Object.keys(MavSysStatusSensor)[0]]
    this.load = 0
    this.voltageBattery = 0
    this.currentBattery = 0
    this.batteryRemaining = 0
    this.dropRateComm = 0
    this.errorsComm = 0
    this.errorsCount1 = 0
    this.errorsCount2 = 0
    this.errorsCount3 = 0
    this.errorsCount4 = 0
    this.onboardControlSensorsPresentExtended = MavSysStatusSensorExtended[Object.keys(MavSysStatusSensorExtended)[0]]
    this.onboardControlSensorsEnabledExtended = MavSysStatusSensorExtended[Object.keys(MavSysStatusSensorExtended)[0]]
    this.onboardControlSensorsHealthExtended = MavSysStatusSensorExtended[Object.keys(MavSysStatusSensorExtended)[0]]
    this.posCflags = 0
    this.posAflags = 0
    this.IMUCflags = AcflyImuFlags[Object.keys(AcflyImuFlags)[0]]
    this.IMUAflags = AcflyImuFlags[Object.keys(AcflyImuFlags)[0]]
  }

  /**
   * Bitmap showing which onboard controllers and sensors are present. Value of 0: not present. Value of
   * 1: present.
   */
  onboardControlSensorsPresent: MavSysStatusSensor

  /**
   * Bitmap showing which onboard controllers and sensors are enabled: Value of 0: not enabled. Value of
   * 1: enabled.
   */
  onboardControlSensorsEnabled: MavSysStatusSensor

  /**
   * Bitmap showing which onboard controllers and sensors have an error (or are operational). Value of 0:
   * error. Value of 1: healthy.
   */
  onboardControlSensorsHealth: MavSysStatusSensor

  /**
   * Maximum usage in percent of the mainloop time. Values: [0-1000] - should always be below 1000
   * Units: d%
   */
  load: uint16_t

  /**
   * Battery voltage, UINT16_MAX: Voltage not sent by autopilot
   * Units: mV
   */
  voltageBattery: uint16_t

  /**
   * Battery current, -1: Current not sent by autopilot
   * Units: cA
   */
  currentBattery: int16_t

  /**
   * Battery energy remaining, -1: Battery remaining energy not sent by autopilot
   * Units: %
   */
  batteryRemaining: int8_t

  /**
   * Communication drop rate, (UART, I2C, SPI, CAN), dropped packets on all links (packets that were
   * corrupted on reception on the MAV)
   * Units: c%
   */
  dropRateComm: uint16_t

  /**
   * Communication errors (UART, I2C, SPI, CAN), dropped packets on all links (packets that were
   * corrupted on reception on the MAV)
   */
  errorsComm: uint16_t

  /**
   * Autopilot-specific errors
   */
  errorsCount1: uint16_t

  /**
   * Autopilot-specific errors
   */
  errorsCount2: uint16_t

  /**
   * Autopilot-specific errors
   */
  errorsCount3: uint16_t

  /**
   * Autopilot-specific errors
   */
  errorsCount4: uint16_t

  /**
   * Bitmap showing which onboard controllers and sensors are present. Value of 0: not present. Value of
   * 1: present.
   */
  onboardControlSensorsPresentExtended: MavSysStatusSensorExtended

  /**
   * Bitmap showing which onboard controllers and sensors are enabled: Value of 0: not enabled. Value of
   * 1: enabled.
   */
  onboardControlSensorsEnabledExtended: MavSysStatusSensorExtended

  /**
   * Bitmap showing which onboard controllers and sensors have an error (or are operational). Value of 0:
   * error. Value of 1: healthy.
   */
  onboardControlSensorsHealthExtended: MavSysStatusSensorExtended

  /**
   * Bitmap indicating which position sensor is connected.
   */
  posCflags: uint32_t

  /**
   * Bitmap indicating which position sensor is available.
   */
  posAflags: uint32_t

  /**
   * Bitmap indicating which IMU sensor is connected.
   */
  IMUCflags: AcflyImuFlags

  /**
   * Bitmap indicating which IMU sensor is calibrated.
   */
  IMUAflags: AcflyImuFlags
}

/**
 * The system time is the time of the master clock, typically the computer clock of the main onboard
 * computer.
 */
export class SystemTime extends MavLinkData {
  static MSG_ID = 2
  static MSG_NAME = 'SYSTEM_TIME'
  static PAYLOAD_LENGTH = 12
  static MAGIC_NUMBER = 137

  static FIELDS = [
    new MavLinkPacketField('time_unix_usec', 'timeUnixUsec', 0, false, 8, 'uint64_t', 'us'),
    new MavLinkPacketField('time_boot_ms', 'timeBootMs', 8, false, 4, 'uint32_t', 'ms'),
  ]

  constructor() {
    super()
    this.timeUnixUsec = BigInt(0)
    this.timeBootMs = 0
  }

  /**
   * Timestamp (UNIX epoch time).
   * Units: us
   */
  timeUnixUsec: uint64_t

  /**
   * Timestamp (time since system boot).
   * Units: ms
   */
  timeBootMs: uint32_t
}

/**
 * A ping message either requesting or responding to a ping. This allows to measure the system
 * latencies, including serial port, radio modem and UDP connections. The ping microservice is
 * documented at https://mavlink.io/en/services/ping.html
 *
 * @deprecated since 2011-08, replaced by SYSTEM_TIME; to be removed / merged with SYSTEM_TIME
 */
export class Ping extends MavLinkData {
  static MSG_ID = 4
  static MSG_NAME = 'PING'
  static PAYLOAD_LENGTH = 14
  static MAGIC_NUMBER = 237

  static FIELDS = [
    new MavLinkPacketField('time_usec', 'timeUsec', 0, false, 8, 'uint64_t', 'us'),
    new MavLinkPacketField('seq', 'seq', 8, false, 4, 'uint32_t', ''),
    new MavLinkPacketField('target_system', 'targetSystem', 12, false, 1, 'uint8_t', ''),
    new MavLinkPacketField('target_component', 'targetComponent', 13, false, 1, 'uint8_t', ''),
  ]

  constructor() {
    super()
    this.timeUsec = BigInt(0)
    this.seq = 0
    this.targetSystem = 0
    this.targetComponent = 0
  }

  /**
   * Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format
   * (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
   * Units: us
   */
  timeUsec: uint64_t

  /**
   * PING sequence
   */
  seq: uint32_t

  /**
   * 0: request ping from all receiving systems. If greater than 0: message is a ping response and number
   * is the system id of the requesting system
   */
  targetSystem: uint8_t

  /**
   * 0: request ping from all receiving components. If greater than 0: message is a ping response and
   * number is the component id of the requesting component.
   */
  targetComponent: uint8_t
}

/**
 * Request to control this MAV
 */
export class ChangeOperatorControl extends MavLinkData {
  static MSG_ID = 5
  static MSG_NAME = 'CHANGE_OPERATOR_CONTROL'
  static PAYLOAD_LENGTH = 28
  static MAGIC_NUMBER = 217

  static FIELDS = [
    new MavLinkPacketField('target_system', 'targetSystem', 0, false, 1, 'uint8_t', ''),
    new MavLinkPacketField('control_request', 'controlRequest', 1, false, 1, 'uint8_t', ''),
    new MavLinkPacketField('version', 'version', 2, false, 1, 'uint8_t', 'rad'),
    new MavLinkPacketField('passkey', 'passkey', 3, false, 1, 'char[]', '', 25),
  ]

  constructor() {
    super()
    this.targetSystem = 0
    this.controlRequest = 0
    this.version = 0
    this.passkey = ''
  }

  /**
   * System the GCS requests control for
   */
  targetSystem: uint8_t

  /**
   * 0: request control of this MAV, 1: Release control of this MAV
   */
  controlRequest: uint8_t

  /**
   * 0: key as plaintext, 1-255: future, different hashing/encryption variants. The GCS should in general
   * use the safest mode possible initially and then gradually move down the encryption level if it gets
   * a NACK message indicating an encryption mismatch.
   * Units: rad
   */
  version: uint8_t

  /**
   * Password / Key, depending on version plaintext or encrypted. 25 or less characters, NULL terminated.
   * The characters may involve A-Z, a-z, 0-9, and "!?,.-"
   */
  passkey: string
}

/**
 * Accept / deny control of this MAV
 */
export class ChangeOperatorControlAck extends MavLinkData {
  static MSG_ID = 6
  static MSG_NAME = 'CHANGE_OPERATOR_CONTROL_ACK'
  static PAYLOAD_LENGTH = 3
  static MAGIC_NUMBER = 104

  static FIELDS = [
    new MavLinkPacketField('gcs_system_id', 'gcsSystemId', 0, false, 1, 'uint8_t', ''),
    new MavLinkPacketField('control_request', 'controlRequest', 1, false, 1, 'uint8_t', ''),
    new MavLinkPacketField('ack', 'ack', 2, false, 1, 'uint8_t', ''),
  ]

  constructor() {
    super()
    this.gcsSystemId = 0
    this.controlRequest = 0
    this.ack = 0
  }

  /**
   * ID of the GCS this message
   */
  gcsSystemId: uint8_t

  /**
   * 0: request control of this MAV, 1: Release control of this MAV
   */
  controlRequest: uint8_t

  /**
   * 0: ACK, 1: NACK: Wrong passkey, 2: NACK: Unsupported passkey encryption method, 3: NACK: Already
   * under control
   */
  ack: uint8_t
}

/**
 * Emit an encrypted signature / key identifying this system. PLEASE NOTE: This protocol has been kept
 * simple, so transmitting the key requires an encrypted channel for true safety.
 */
export class AuthKey extends MavLinkData {
  static MSG_ID = 7
  static MSG_NAME = 'AUTH_KEY'
  static PAYLOAD_LENGTH = 32
  static MAGIC_NUMBER = 119

  static FIELDS = [
    new MavLinkPacketField('key', 'key', 0, false, 1, 'char[]', '', 32),
  ]

  constructor() {
    super()
    this.key = ''
  }

  /**
   * key
   */
  key: string
}

/**
 * Set the system mode, as defined by enum MAV_MODE. There is no target component id as the mode is by
 * definition for the overall aircraft, not only for one component.
 *
 * @deprecated since 2015-12, replaced by MAV_CMD_DO_SET_MODE; Use COMMAND_LONG with MAV_CMD_DO_SET_MODE instead
 */
export class SetMode extends MavLinkData {
  static MSG_ID = 11
  static MSG_NAME = 'SET_MODE'
  static PAYLOAD_LENGTH = 6
  static MAGIC_NUMBER = 89

  static FIELDS = [
    new MavLinkPacketField('custom_mode', 'customMode', 0, false, 4, 'uint32_t', ''),
    new MavLinkPacketField('target_system', 'targetSystem', 4, false, 1, 'uint8_t', ''),
    new MavLinkPacketField('base_mode', 'baseMode', 5, false, 1, 'uint8_t', ''),
  ]

  constructor() {
    super()
    this.targetSystem = 0
    this.baseMode = MavMode[Object.keys(MavMode)[0]]
    this.customMode = 0
  }

  /**
   * The system setting the mode
   */
  targetSystem: uint8_t

  /**
   * The new base mode.
   */
  baseMode: MavMode

  /**
   * The new autopilot-specific mode. This field can be ignored by an autopilot.
   */
  customMode: uint32_t
}

/**
 * Request to read the onboard parameter with the param_id string id. Onboard parameters are stored as
 * key[const char*] -> value[float]. This allows to send a parameter to any other component (such as
 * the GCS) without the need of previous knowledge of possible parameter names. Thus the same GCS can
 * store different parameters for different autopilots. See also
 * https://mavlink.io/en/services/parameter.html for a full documentation of QGroundControl and IMU
 * code.
 */
export class ParamRequestRead extends MavLinkData {
  static MSG_ID = 20
  static MSG_NAME = 'PARAM_REQUEST_READ'
  static PAYLOAD_LENGTH = 20
  static MAGIC_NUMBER = 214

  static FIELDS = [
    new MavLinkPacketField('param_index', 'paramIndex', 0, false, 2, 'int16_t', ''),
    new MavLinkPacketField('target_system', 'targetSystem', 2, false, 1, 'uint8_t', ''),
    new MavLinkPacketField('target_component', 'targetComponent', 3, false, 1, 'uint8_t', ''),
    new MavLinkPacketField('param_id', 'paramId', 4, false, 1, 'char[]', '', 16),
  ]

  constructor() {
    super()
    this.targetSystem = 0
    this.targetComponent = 0
    this.paramId = ''
    this.paramIndex = 0
  }

  /**
   * System ID
   */
  targetSystem: uint8_t

  /**
   * Component ID
   */
  targetComponent: uint8_t

  /**
   * Onboard parameter id, terminated by NULL if the length is less than 16 human-readable chars and
   * WITHOUT null termination (NULL) byte if the length is exactly 16 chars - applications have to
   * provide 16+1 bytes storage if the ID is stored as string
   */
  paramId: string

  /**
   * Parameter index. Send -1 to use the param ID field as identifier (else the param id will be ignored)
   */
  paramIndex: int16_t
}

/**
 * Request all parameters of this component. After this request, all parameters are emitted. The
 * parameter microservice is documented at https://mavlink.io/en/services/parameter.html
 */
export class ParamRequestList extends MavLinkData {
  static MSG_ID = 21
  static MSG_NAME = 'PARAM_REQUEST_LIST'
  static PAYLOAD_LENGTH = 2
  static MAGIC_NUMBER = 159

  static FIELDS = [
    new MavLinkPacketField('target_system', 'targetSystem', 0, false, 1, 'uint8_t', ''),
    new MavLinkPacketField('target_component', 'targetComponent', 1, false, 1, 'uint8_t', ''),
  ]

  constructor() {
    super()
    this.targetSystem = 0
    this.targetComponent = 0
  }

  /**
   * System ID
   */
  targetSystem: uint8_t

  /**
   * Component ID
   */
  targetComponent: uint8_t
}

/**
 * Emit the value of a onboard parameter. The inclusion of param_count and param_index in the message
 * allows the recipient to keep track of received parameters and allows him to re-request missing
 * parameters after a loss or timeout. The parameter microservice is documented at
 * https://mavlink.io/en/services/parameter.html
 */
export class ParamValue extends MavLinkData {
  static MSG_ID = 22
  static MSG_NAME = 'PARAM_VALUE'
  static PAYLOAD_LENGTH = 25
  static MAGIC_NUMBER = 220

  static FIELDS = [
    new MavLinkPacketField('param_value', 'paramValue', 0, false, 4, 'float', ''),
    new MavLinkPacketField('param_count', 'paramCount', 4, false, 2, 'uint16_t', ''),
    new MavLinkPacketField('param_index', 'paramIndex', 6, false, 2, 'uint16_t', ''),
    new MavLinkPacketField('param_id', 'paramId', 8, false, 1, 'char[]', '', 16),
    new MavLinkPacketField('param_type', 'paramType', 24, false, 1, 'uint8_t', ''),
  ]

  constructor() {
    super()
    this.paramId = ''
    this.paramValue = 0
    this.paramType = MavParamType[Object.keys(MavParamType)[0]]
    this.paramCount = 0
    this.paramIndex = 0
  }

  /**
   * Onboard parameter id, terminated by NULL if the length is less than 16 human-readable chars and
   * WITHOUT null termination (NULL) byte if the length is exactly 16 chars - applications have to
   * provide 16+1 bytes storage if the ID is stored as string
   */
  paramId: string

  /**
   * Onboard parameter value
   */
  paramValue: float

  /**
   * Onboard parameter type.
   */
  paramType: MavParamType

  /**
   * Total number of onboard parameters
   */
  paramCount: uint16_t

  /**
   * Index of this onboard parameter
   */
  paramIndex: uint16_t
}

/**
 * Set a parameter value (write new value to permanent storage).
 The receiving component should
 * acknowledge the new parameter value by broadcasting a PARAM_VALUE message (broadcasting ensures that
 * multiple GCS all have an up-to-date list of all parameters). If the sending GCS did not receive a
 * PARAM_VALUE within its timeout time, it should re-send the PARAM_SET message. The parameter
 * microservice is documented at https://mavlink.io/en/services/parameter.html.
 PARAM_SET may also be
 * called within the context of a transaction (started with MAV_CMD_PARAM_TRANSACTION). Within a
 * transaction the receiving component should respond with PARAM_ACK_TRANSACTION to the setter
 * component (instead of broadcasting PARAM_VALUE), and PARAM_SET should be re-sent if this is ACK not
 * received.
 */
export class ParamSet extends MavLinkData {
  static MSG_ID = 23
  static MSG_NAME = 'PARAM_SET'
  static PAYLOAD_LENGTH = 23
  static MAGIC_NUMBER = 168

  static FIELDS = [
    new MavLinkPacketField('param_value', 'paramValue', 0, false, 4, 'float', ''),
    new MavLinkPacketField('target_system', 'targetSystem', 4, false, 1, 'uint8_t', ''),
    new MavLinkPacketField('target_component', 'targetComponent', 5, false, 1, 'uint8_t', ''),
    new MavLinkPacketField('param_id', 'paramId', 6, false, 1, 'char[]', '', 16),
    new MavLinkPacketField('param_type', 'paramType', 22, false, 1, 'uint8_t', ''),
  ]

  constructor() {
    super()
    this.targetSystem = 0
    this.targetComponent = 0
    this.paramId = ''
    this.paramValue = 0
    this.paramType = MavParamType[Object.keys(MavParamType)[0]]
  }

  /**
   * System ID
   */
  targetSystem: uint8_t

  /**
   * Component ID
   */
  targetComponent: uint8_t

  /**
   * Onboard parameter id, terminated by NULL if the length is less than 16 human-readable chars and
   * WITHOUT null termination (NULL) byte if the length is exactly 16 chars - applications have to
   * provide 16+1 bytes storage if the ID is stored as string
   */
  paramId: string

  /**
   * Onboard parameter value
   */
  paramValue: float

  /**
   * Onboard parameter type.
   */
  paramType: MavParamType
}

/**
 * The global position, as returned by the Global Positioning System (GPS). This is
 NOT the global
 * position estimate of the system, but rather a RAW sensor value. See message GLOBAL_POSITION_INT for
 * the global position estimate.
 */
export class GpsRawInt extends MavLinkData {
  static MSG_ID = 24
  static MSG_NAME = 'GPS_RAW_INT'
  static PAYLOAD_LENGTH = 52
  static MAGIC_NUMBER = 24

  static FIELDS = [
    new MavLinkPacketField('time_usec', 'timeUsec', 0, false, 8, 'uint64_t', 'us'),
    new MavLinkPacketField('lat', 'lat', 8, false, 4, 'int32_t', 'degE7'),
    new MavLinkPacketField('lon', 'lon', 12, false, 4, 'int32_t', 'degE7'),
    new MavLinkPacketField('alt', 'alt', 16, false, 4, 'int32_t', 'mm'),
    new MavLinkPacketField('eph', 'eph', 20, false, 2, 'uint16_t', ''),
    new MavLinkPacketField('epv', 'epv', 22, false, 2, 'uint16_t', ''),
    new MavLinkPacketField('vel', 'vel', 24, false, 2, 'uint16_t', 'cm/s'),
    new MavLinkPacketField('cog', 'cog', 26, false, 2, 'uint16_t', 'cdeg'),
    new MavLinkPacketField('fix_type', 'fixType', 28, false, 1, 'uint8_t', ''),
    new MavLinkPacketField('satellites_visible', 'satellitesVisible', 29, false, 1, 'uint8_t', ''),
    new MavLinkPacketField('alt_ellipsoid', 'altEllipsoid', 30, true, 4, 'int32_t', 'mm'),
    new MavLinkPacketField('h_acc', 'hAcc', 34, true, 4, 'uint32_t', 'mm'),
    new MavLinkPacketField('v_acc', 'vAcc', 38, true, 4, 'uint32_t', 'mm'),
    new MavLinkPacketField('vel_acc', 'velAcc', 42, true, 4, 'uint32_t', 'mm'),
    new MavLinkPacketField('hdg_acc', 'hdgAcc', 46, true, 4, 'uint32_t', 'degE5'),
    new MavLinkPacketField('yaw', 'yaw', 50, true, 2, 'uint16_t', 'cdeg'),
  ]

  constructor() {
    super()
    this.timeUsec = BigInt(0)
    this.fixType = GpsFixType[Object.keys(GpsFixType)[0]]
    this.lat = 0
    this.lon = 0
    this.alt = 0
    this.eph = 0
    this.epv = 0
    this.vel = 0
    this.cog = 0
    this.satellitesVisible = 0
    this.altEllipsoid = 0
    this.hAcc = 0
    this.vAcc = 0
    this.velAcc = 0
    this.hdgAcc = 0
    this.yaw = 0
  }

  /**
   * Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format
   * (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
   * Units: us
   */
  timeUsec: uint64_t

  /**
   * GPS fix type.
   */
  fixType: GpsFixType

  /**
   * Latitude (WGS84, EGM96 ellipsoid)
   * Units: degE7
   */
  lat: int32_t

  /**
   * Longitude (WGS84, EGM96 ellipsoid)
   * Units: degE7
   */
  lon: int32_t

  /**
   * Altitude (MSL). Positive for up. Note that virtually all GPS modules provide the MSL altitude in
   * addition to the WGS84 altitude.
   * Units: mm
   */
  alt: int32_t

  /**
   * GPS HDOP horizontal dilution of position (unitless * 100). If unknown, set to: UINT16_MAX
   */
  eph: uint16_t

  /**
   * GPS VDOP vertical dilution of position (unitless * 100). If unknown, set to: UINT16_MAX
   */
  epv: uint16_t

  /**
   * GPS ground speed. If unknown, set to: UINT16_MAX
   * Units: cm/s
   */
  vel: uint16_t

  /**
   * Course over ground (NOT heading, but direction of movement) in degrees * 100, 0.0..359.99 degrees.
   * If unknown, set to: UINT16_MAX
   * Units: cdeg
   */
  cog: uint16_t

  /**
   * Number of satellites visible. If unknown, set to UINT8_MAX
   */
  satellitesVisible: uint8_t

  /**
   * Altitude (above WGS84, EGM96 ellipsoid). Positive for up.
   * Units: mm
   */
  altEllipsoid: int32_t

  /**
   * Position uncertainty.
   * Units: mm
   */
  hAcc: uint32_t

  /**
   * Altitude uncertainty.
   * Units: mm
   */
  vAcc: uint32_t

  /**
   * Speed uncertainty.
   * Units: mm
   */
  velAcc: uint32_t

  /**
   * Heading / track uncertainty
   * Units: degE5
   */
  hdgAcc: uint32_t

  /**
   * Yaw in earth frame from north. Use 0 if this GPS does not provide yaw. Use UINT16_MAX if this GPS is
   * configured to provide yaw and is currently unable to provide it. Use 36000 for north.
   * Units: cdeg
   */
  yaw: uint16_t
}

/**
 * The positioning status, as reported by GPS. This message is intended to display status information
 * about each satellite visible to the receiver. See message GLOBAL_POSITION_INT for the global
 * position estimate. This message can contain information for up to 20 satellites.
 */
export class GpsStatus extends MavLinkData {
  static MSG_ID = 25
  static MSG_NAME = 'GPS_STATUS'
  static PAYLOAD_LENGTH = 101
  static MAGIC_NUMBER = 23

  static FIELDS = [
    new MavLinkPacketField('satellites_visible', 'satellitesVisible', 0, false, 1, 'uint8_t', ''),
    new MavLinkPacketField('satellite_prn', 'satellitePrn', 1, false, 1, 'uint8_t[]', '', 20),
    new MavLinkPacketField('satellite_used', 'satelliteUsed', 21, false, 1, 'uint8_t[]', '', 20),
    new MavLinkPacketField('satellite_elevation', 'satelliteElevation', 41, false, 1, 'uint8_t[]', 'deg', 20),
    new MavLinkPacketField('satellite_azimuth', 'satelliteAzimuth', 61, false, 1, 'uint8_t[]', 'deg', 20),
    new MavLinkPacketField('satellite_snr', 'satelliteSnr', 81, false, 1, 'uint8_t[]', 'dB', 20),
  ]

  constructor() {
    super()
    this.satellitesVisible = 0
    this.satellitePrn = []
    this.satelliteUsed = []
    this.satelliteElevation = []
    this.satelliteAzimuth = []
    this.satelliteSnr = []
  }

  /**
   * Number of satellites visible
   */
  satellitesVisible: uint8_t

  /**
   * Global satellite ID
   */
  satellitePrn: uint8_t[]

  /**
   * 0: Satellite not used, 1: used for localization
   */
  satelliteUsed: uint8_t[]

  /**
   * Elevation (0: right on top of receiver, 90: on the horizon) of satellite
   * Units: deg
   */
  satelliteElevation: uint8_t[]

  /**
   * Direction of satellite, 0: 0 deg, 255: 360 deg.
   * Units: deg
   */
  satelliteAzimuth: uint8_t[]

  /**
   * Signal to noise ratio of satellite
   * Units: dB
   */
  satelliteSnr: uint8_t[]
}

/**
 * The RAW IMU readings for the usual 9DOF sensor setup. This message should contain the scaled values
 * to the described units
 */
export class ScaledImu extends MavLinkData {
  static MSG_ID = 26
  static MSG_NAME = 'SCALED_IMU'
  static PAYLOAD_LENGTH = 26
  static MAGIC_NUMBER = 170

  static FIELDS = [
    new MavLinkPacketField('time_boot_ms', 'timeBootMs', 0, false, 4, 'uint32_t', 'ms'),
    new MavLinkPacketField('xacc', 'xacc', 4, false, 2, 'int16_t', 'mG'),
    new MavLinkPacketField('yacc', 'yacc', 6, false, 2, 'int16_t', 'mG'),
    new MavLinkPacketField('zacc', 'zacc', 8, false, 2, 'int16_t', 'mG'),
    new MavLinkPacketField('xgyro', 'xgyro', 10, false, 2, 'int16_t', 'mrad/s'),
    new MavLinkPacketField('ygyro', 'ygyro', 12, false, 2, 'int16_t', 'mrad/s'),
    new MavLinkPacketField('zgyro', 'zgyro', 14, false, 2, 'int16_t', 'mrad/s'),
    new MavLinkPacketField('xmag', 'xmag', 16, false, 2, 'int16_t', 'mgauss'),
    new MavLinkPacketField('ymag', 'ymag', 18, false, 2, 'int16_t', 'mgauss'),
    new MavLinkPacketField('zmag', 'zmag', 20, false, 2, 'int16_t', 'mgauss'),
    new MavLinkPacketField('temperature', 'temperature', 22, true, 2, 'int16_t', 'cdegC'),
    new MavLinkPacketField('daoYaw', 'daoYaw', 24, true, 2, 'int16_t', '0.1deg'),
  ]

  constructor() {
    super()
    this.timeBootMs = 0
    this.xacc = 0
    this.yacc = 0
    this.zacc = 0
    this.xgyro = 0
    this.ygyro = 0
    this.zgyro = 0
    this.xmag = 0
    this.ymag = 0
    this.zmag = 0
    this.temperature = 0
    this.daoYaw = 0
  }

  /**
   * Timestamp (time since system boot).
   * Units: ms
   */
  timeBootMs: uint32_t

  /**
   * X acceleration
   * Units: mG
   */
  xacc: int16_t

  /**
   * Y acceleration
   * Units: mG
   */
  yacc: int16_t

  /**
   * Z acceleration
   * Units: mG
   */
  zacc: int16_t

  /**
   * Angular speed around X axis
   * Units: mrad/s
   */
  xgyro: int16_t

  /**
   * Angular speed around Y axis
   * Units: mrad/s
   */
  ygyro: int16_t

  /**
   * Angular speed around Z axis
   * Units: mrad/s
   */
  zgyro: int16_t

  /**
   * X Magnetic field
   * Units: mgauss
   */
  xmag: int16_t

  /**
   * Y Magnetic field
   * Units: mgauss
   */
  ymag: int16_t

  /**
   * Z Magnetic field
   * Units: mgauss
   */
  zmag: int16_t

  /**
   * Temperature, 0: IMU does not provide temperature values. If the IMU is at 0C it must send 1 (0.01C).
   * Units: cdegC
   */
  temperature: int16_t

  /**
   * 双天线传感器1角度
   * Units: 0.1deg
   */
  daoYaw: int16_t
}

/**
 * The RAW IMU readings for a 9DOF sensor, which is identified by the id (default IMU1). This message
 * should always contain the true raw values without any scaling to allow data capture and system
 * debugging.
 */
export class RawImu extends MavLinkData {
  static MSG_ID = 27
  static MSG_NAME = 'RAW_IMU'
  static PAYLOAD_LENGTH = 29
  static MAGIC_NUMBER = 144

  static FIELDS = [
    new MavLinkPacketField('time_usec', 'timeUsec', 0, false, 8, 'uint64_t', 'us'),
    new MavLinkPacketField('xacc', 'xacc', 8, false, 2, 'int16_t', ''),
    new MavLinkPacketField('yacc', 'yacc', 10, false, 2, 'int16_t', ''),
    new MavLinkPacketField('zacc', 'zacc', 12, false, 2, 'int16_t', ''),
    new MavLinkPacketField('xgyro', 'xgyro', 14, false, 2, 'int16_t', ''),
    new MavLinkPacketField('ygyro', 'ygyro', 16, false, 2, 'int16_t', ''),
    new MavLinkPacketField('zgyro', 'zgyro', 18, false, 2, 'int16_t', ''),
    new MavLinkPacketField('xmag', 'xmag', 20, false, 2, 'int16_t', ''),
    new MavLinkPacketField('ymag', 'ymag', 22, false, 2, 'int16_t', ''),
    new MavLinkPacketField('zmag', 'zmag', 24, false, 2, 'int16_t', ''),
    new MavLinkPacketField('id', 'id', 26, true, 1, 'uint8_t', ''),
    new MavLinkPacketField('temperature', 'temperature', 27, true, 2, 'int16_t', 'cdegC'),
  ]

  constructor() {
    super()
    this.timeUsec = BigInt(0)
    this.xacc = 0
    this.yacc = 0
    this.zacc = 0
    this.xgyro = 0
    this.ygyro = 0
    this.zgyro = 0
    this.xmag = 0
    this.ymag = 0
    this.zmag = 0
    this.id = 0
    this.temperature = 0
  }

  /**
   * Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format
   * (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
   * Units: us
   */
  timeUsec: uint64_t

  /**
   * X acceleration (raw)
   */
  xacc: int16_t

  /**
   * Y acceleration (raw)
   */
  yacc: int16_t

  /**
   * Z acceleration (raw)
   */
  zacc: int16_t

  /**
   * Angular speed around X axis (raw)
   */
  xgyro: int16_t

  /**
   * Angular speed around Y axis (raw)
   */
  ygyro: int16_t

  /**
   * Angular speed around Z axis (raw)
   */
  zgyro: int16_t

  /**
   * X Magnetic field (raw)
   */
  xmag: int16_t

  /**
   * Y Magnetic field (raw)
   */
  ymag: int16_t

  /**
   * Z Magnetic field (raw)
   */
  zmag: int16_t

  /**
   * Id. Ids are numbered from 0 and map to IMUs numbered from 1 (e.g. IMU1 will have a message with
   * id=0)
   */
  id: uint8_t

  /**
   * Temperature, 0: IMU does not provide temperature values. If the IMU is at 0C it must send 1 (0.01C).
   * Units: cdegC
   */
  temperature: int16_t
}

/**
 * The RAW pressure readings for the typical setup of one absolute pressure and one differential
 * pressure sensor. The sensor values should be the raw, UNSCALED ADC values.
 */
export class RawPressure extends MavLinkData {
  static MSG_ID = 28
  static MSG_NAME = 'RAW_PRESSURE'
  static PAYLOAD_LENGTH = 16
  static MAGIC_NUMBER = 67

  static FIELDS = [
    new MavLinkPacketField('time_usec', 'timeUsec', 0, false, 8, 'uint64_t', 'us'),
    new MavLinkPacketField('press_abs', 'pressAbs', 8, false, 2, 'int16_t', ''),
    new MavLinkPacketField('press_diff1', 'pressDiff1', 10, false, 2, 'int16_t', ''),
    new MavLinkPacketField('press_diff2', 'pressDiff2', 12, false, 2, 'int16_t', ''),
    new MavLinkPacketField('temperature', 'temperature', 14, false, 2, 'int16_t', ''),
  ]

  constructor() {
    super()
    this.timeUsec = BigInt(0)
    this.pressAbs = 0
    this.pressDiff1 = 0
    this.pressDiff2 = 0
    this.temperature = 0
  }

  /**
   * Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format
   * (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
   * Units: us
   */
  timeUsec: uint64_t

  /**
   * Absolute pressure (raw)
   */
  pressAbs: int16_t

  /**
   * Differential pressure 1 (raw, 0 if nonexistent)
   */
  pressDiff1: int16_t

  /**
   * Differential pressure 2 (raw, 0 if nonexistent)
   */
  pressDiff2: int16_t

  /**
   * Raw Temperature measurement (raw)
   */
  temperature: int16_t
}

/**
 * The pressure readings for the typical setup of one absolute and differential pressure sensor. The
 * units are as specified in each field.
 */
export class ScaledPressure extends MavLinkData {
  static MSG_ID = 29
  static MSG_NAME = 'SCALED_PRESSURE'
  static PAYLOAD_LENGTH = 16
  static MAGIC_NUMBER = 115

  static FIELDS = [
    new MavLinkPacketField('time_boot_ms', 'timeBootMs', 0, false, 4, 'uint32_t', 'ms'),
    new MavLinkPacketField('press_abs', 'pressAbs', 4, false, 4, 'float', 'hPa'),
    new MavLinkPacketField('press_diff', 'pressDiff', 8, false, 4, 'float', 'hPa'),
    new MavLinkPacketField('temperature', 'temperature', 12, false, 2, 'int16_t', 'cdegC'),
    new MavLinkPacketField('temperature_press_diff', 'temperaturePressDiff', 14, true, 2, 'int16_t', 'cdegC'),
  ]

  constructor() {
    super()
    this.timeBootMs = 0
    this.pressAbs = 0
    this.pressDiff = 0
    this.temperature = 0
    this.temperaturePressDiff = 0
  }

  /**
   * Timestamp (time since system boot).
   * Units: ms
   */
  timeBootMs: uint32_t

  /**
   * Absolute pressure
   * Units: hPa
   */
  pressAbs: float

  /**
   * Differential pressure 1
   * Units: hPa
   */
  pressDiff: float

  /**
   * Absolute pressure temperature
   * Units: cdegC
   */
  temperature: int16_t

  /**
   * Differential pressure temperature (0, if not available). Report values of 0 (or 1) as 1 cdegC.
   * Units: cdegC
   */
  temperaturePressDiff: int16_t
}

/**
 * The attitude in the aeronautical frame (right-handed, Z-down, X-front, Y-right).
 */
export class Attitude extends MavLinkData {
  static MSG_ID = 30
  static MSG_NAME = 'ATTITUDE'
  static PAYLOAD_LENGTH = 28
  static MAGIC_NUMBER = 39

  static FIELDS = [
    new MavLinkPacketField('time_boot_ms', 'timeBootMs', 0, false, 4, 'uint32_t', 'ms'),
    new MavLinkPacketField('roll', 'roll', 4, false, 4, 'float', 'rad'),
    new MavLinkPacketField('pitch', 'pitch', 8, false, 4, 'float', 'rad'),
    new MavLinkPacketField('yaw', 'yaw', 12, false, 4, 'float', 'rad'),
    new MavLinkPacketField('rollspeed', 'rollspeed', 16, false, 4, 'float', 'rad/s'),
    new MavLinkPacketField('pitchspeed', 'pitchspeed', 20, false, 4, 'float', 'rad/s'),
    new MavLinkPacketField('yawspeed', 'yawspeed', 24, false, 4, 'float', 'rad/s'),
  ]

  constructor() {
    super()
    this.timeBootMs = 0
    this.roll = 0
    this.pitch = 0
    this.yaw = 0
    this.rollspeed = 0
    this.pitchspeed = 0
    this.yawspeed = 0
  }

  /**
   * Timestamp (time since system boot).
   * Units: ms
   */
  timeBootMs: uint32_t

  /**
   * Roll angle (-pi..+pi)
   * Units: rad
   */
  roll: float

  /**
   * Pitch angle (-pi..+pi)
   * Units: rad
   */
  pitch: float

  /**
   * Yaw angle (-pi..+pi)
   * Units: rad
   */
  yaw: float

  /**
   * Roll angular speed
   * Units: rad/s
   */
  rollspeed: float

  /**
   * Pitch angular speed
   * Units: rad/s
   */
  pitchspeed: float

  /**
   * Yaw angular speed
   * Units: rad/s
   */
  yawspeed: float
}

/**
 * The attitude in the aeronautical frame (right-handed, Z-down, X-front, Y-right), expressed as
 * quaternion. Quaternion order is w, x, y, z and a zero rotation would be expressed as (1 0 0 0).
 */
export class AttitudeQuaternion extends MavLinkData {
  static MSG_ID = 31
  static MSG_NAME = 'ATTITUDE_QUATERNION'
  static PAYLOAD_LENGTH = 48
  static MAGIC_NUMBER = 246

  static FIELDS = [
    new MavLinkPacketField('time_boot_ms', 'timeBootMs', 0, false, 4, 'uint32_t', 'ms'),
    new MavLinkPacketField('q1', 'q1', 4, false, 4, 'float', ''),
    new MavLinkPacketField('q2', 'q2', 8, false, 4, 'float', ''),
    new MavLinkPacketField('q3', 'q3', 12, false, 4, 'float', ''),
    new MavLinkPacketField('q4', 'q4', 16, false, 4, 'float', ''),
    new MavLinkPacketField('rollspeed', 'rollspeed', 20, false, 4, 'float', 'rad/s'),
    new MavLinkPacketField('pitchspeed', 'pitchspeed', 24, false, 4, 'float', 'rad/s'),
    new MavLinkPacketField('yawspeed', 'yawspeed', 28, false, 4, 'float', 'rad/s'),
    new MavLinkPacketField('repr_offset_q', 'reprOffsetQ', 32, true, 4, 'float[]', '', 4),
  ]

  constructor() {
    super()
    this.timeBootMs = 0
    this.q1 = 0
    this.q2 = 0
    this.q3 = 0
    this.q4 = 0
    this.rollspeed = 0
    this.pitchspeed = 0
    this.yawspeed = 0
    this.reprOffsetQ = []
  }

  /**
   * Timestamp (time since system boot).
   * Units: ms
   */
  timeBootMs: uint32_t

  /**
   * Quaternion component 1, w (1 in null-rotation)
   */
  q1: float

  /**
   * Quaternion component 2, x (0 in null-rotation)
   */
  q2: float

  /**
   * Quaternion component 3, y (0 in null-rotation)
   */
  q3: float

  /**
   * Quaternion component 4, z (0 in null-rotation)
   */
  q4: float

  /**
   * Roll angular speed
   * Units: rad/s
   */
  rollspeed: float

  /**
   * Pitch angular speed
   * Units: rad/s
   */
  pitchspeed: float

  /**
   * Yaw angular speed
   * Units: rad/s
   */
  yawspeed: float

  /**
   * Rotation offset by which the attitude quaternion and angular speed vector should be rotated for user
   * display (quaternion with [w, x, y, z] order, zero-rotation is [1, 0, 0, 0], send [0, 0, 0, 0] if
   * field not supported). This field is intended for systems in which the reference attitude may change
   * during flight. For example, tailsitters VTOLs rotate their reference attitude by 90 degrees between
   * hover mode and fixed wing mode, thus repr_offset_q is equal to [1, 0, 0, 0] in hover mode and equal
   * to [0.7071, 0, 0.7071, 0] in fixed wing mode.
   */
  reprOffsetQ: float[]
}

/**
 * The filtered local position (e.g. fused computer vision and accelerometers). Coordinate frame is
 * right-handed, Z-axis down (aeronautical frame, NED / north-east-down convention)
 */
export class LocalPositionNed extends MavLinkData {
  static MSG_ID = 32
  static MSG_NAME = 'LOCAL_POSITION_NED'
  static PAYLOAD_LENGTH = 28
  static MAGIC_NUMBER = 185

  static FIELDS = [
    new MavLinkPacketField('time_boot_ms', 'timeBootMs', 0, false, 4, 'uint32_t', 'ms'),
    new MavLinkPacketField('x', 'x', 4, false, 4, 'float', 'm'),
    new MavLinkPacketField('y', 'y', 8, false, 4, 'float', 'm'),
    new MavLinkPacketField('z', 'z', 12, false, 4, 'float', 'm'),
    new MavLinkPacketField('vx', 'vx', 16, false, 4, 'float', 'm/s'),
    new MavLinkPacketField('vy', 'vy', 20, false, 4, 'float', 'm/s'),
    new MavLinkPacketField('vz', 'vz', 24, false, 4, 'float', 'm/s'),
  ]

  constructor() {
    super()
    this.timeBootMs = 0
    this.x = 0
    this.y = 0
    this.z = 0
    this.vx = 0
    this.vy = 0
    this.vz = 0
  }

  /**
   * Timestamp (time since system boot).
   * Units: ms
   */
  timeBootMs: uint32_t

  /**
   * X Position
   * Units: m
   */
  x: float

  /**
   * Y Position
   * Units: m
   */
  y: float

  /**
   * Z Position
   * Units: m
   */
  z: float

  /**
   * X Speed
   * Units: m/s
   */
  vx: float

  /**
   * Y Speed
   * Units: m/s
   */
  vy: float

  /**
   * Z Speed
   * Units: m/s
   */
  vz: float
}

/**
 * The filtered global position (e.g. fused GPS and accelerometers). The position is in GPS-frame
 * (right-handed, Z-up). It
 is designed as scaled integer message since the resolution of float is not
 * sufficient.
 */
export class GlobalPositionInt extends MavLinkData {
  static MSG_ID = 33
  static MSG_NAME = 'GLOBAL_POSITION_INT'
  static PAYLOAD_LENGTH = 28
  static MAGIC_NUMBER = 104

  static FIELDS = [
    new MavLinkPacketField('time_boot_ms', 'timeBootMs', 0, false, 4, 'uint32_t', 'ms'),
    new MavLinkPacketField('lat', 'lat', 4, false, 4, 'int32_t', 'degE7'),
    new MavLinkPacketField('lon', 'lon', 8, false, 4, 'int32_t', 'degE7'),
    new MavLinkPacketField('alt', 'alt', 12, false, 4, 'int32_t', 'mm'),
    new MavLinkPacketField('relative_alt', 'relativeAlt', 16, false, 4, 'int32_t', 'mm'),
    new MavLinkPacketField('vx', 'vx', 20, false, 2, 'int16_t', 'cm/s'),
    new MavLinkPacketField('vy', 'vy', 22, false, 2, 'int16_t', 'cm/s'),
    new MavLinkPacketField('vz', 'vz', 24, false, 2, 'int16_t', 'cm/s'),
    new MavLinkPacketField('hdg', 'hdg', 26, false, 2, 'uint16_t', 'cdeg'),
  ]

  constructor() {
    super()
    this.timeBootMs = 0
    this.lat = 0
    this.lon = 0
    this.alt = 0
    this.relativeAlt = 0
    this.vx = 0
    this.vy = 0
    this.vz = 0
    this.hdg = 0
  }

  /**
   * Timestamp (time since system boot).
   * Units: ms
   */
  timeBootMs: uint32_t

  /**
   * Latitude, expressed
   * Units: degE7
   */
  lat: int32_t

  /**
   * Longitude, expressed
   * Units: degE7
   */
  lon: int32_t

  /**
   * Altitude (MSL). Note that virtually all GPS modules provide both WGS84 and MSL.
   * Units: mm
   */
  alt: int32_t

  /**
   * Altitude above ground
   * Units: mm
   */
  relativeAlt: int32_t

  /**
   * Ground X Speed (Latitude, positive north)
   * Units: cm/s
   */
  vx: int16_t

  /**
   * Ground Y Speed (Longitude, positive east)
   * Units: cm/s
   */
  vy: int16_t

  /**
   * Ground Z Speed (Altitude, positive down)
   * Units: cm/s
   */
  vz: int16_t

  /**
   * Vehicle heading (yaw angle), 0.0..359.99 degrees. If unknown, set to: UINT16_MAX
   * Units: cdeg
   */
  hdg: uint16_t
}

/**
 * The scaled values of the RC channels received: (-100%) -10000, (0%) 0, (100%) 10000. Channels that
 * are inactive should be set to UINT16_MAX.
 */
export class RcChannelsScaled extends MavLinkData {
  static MSG_ID = 34
  static MSG_NAME = 'RC_CHANNELS_SCALED'
  static PAYLOAD_LENGTH = 22
  static MAGIC_NUMBER = 237

  static FIELDS = [
    new MavLinkPacketField('time_boot_ms', 'timeBootMs', 0, false, 4, 'uint32_t', 'ms'),
    new MavLinkPacketField('chan1_scaled', 'chan1Scaled', 4, false, 2, 'int16_t', ''),
    new MavLinkPacketField('chan2_scaled', 'chan2Scaled', 6, false, 2, 'int16_t', ''),
    new MavLinkPacketField('chan3_scaled', 'chan3Scaled', 8, false, 2, 'int16_t', ''),
    new MavLinkPacketField('chan4_scaled', 'chan4Scaled', 10, false, 2, 'int16_t', ''),
    new MavLinkPacketField('chan5_scaled', 'chan5Scaled', 12, false, 2, 'int16_t', ''),
    new MavLinkPacketField('chan6_scaled', 'chan6Scaled', 14, false, 2, 'int16_t', ''),
    new MavLinkPacketField('chan7_scaled', 'chan7Scaled', 16, false, 2, 'int16_t', ''),
    new MavLinkPacketField('chan8_scaled', 'chan8Scaled', 18, false, 2, 'int16_t', ''),
    new MavLinkPacketField('port', 'port', 20, false, 1, 'uint8_t', ''),
    new MavLinkPacketField('rssi', 'rssi', 21, false, 1, 'uint8_t', ''),
  ]

  constructor() {
    super()
    this.timeBootMs = 0
    this.port = 0
    this.chan1Scaled = 0
    this.chan2Scaled = 0
    this.chan3Scaled = 0
    this.chan4Scaled = 0
    this.chan5Scaled = 0
    this.chan6Scaled = 0
    this.chan7Scaled = 0
    this.chan8Scaled = 0
    this.rssi = 0
  }

  /**
   * Timestamp (time since system boot).
   * Units: ms
   */
  timeBootMs: uint32_t

  /**
   * Servo output port (set of 8 outputs = 1 port). Flight stacks running on Pixhawk should use: 0 =
   * MAIN, 1 = AUX.
   */
  port: uint8_t

  /**
   * RC channel 1 value scaled.
   */
  chan1Scaled: int16_t

  /**
   * RC channel 2 value scaled.
   */
  chan2Scaled: int16_t

  /**
   * RC channel 3 value scaled.
   */
  chan3Scaled: int16_t

  /**
   * RC channel 4 value scaled.
   */
  chan4Scaled: int16_t

  /**
   * RC channel 5 value scaled.
   */
  chan5Scaled: int16_t

  /**
   * RC channel 6 value scaled.
   */
  chan6Scaled: int16_t

  /**
   * RC channel 7 value scaled.
   */
  chan7Scaled: int16_t

  /**
   * RC channel 8 value scaled.
   */
  chan8Scaled: int16_t

  /**
   * Receive signal strength indicator in device-dependent units/scale. Values: [0-254], UINT8_MAX:
   * invalid/unknown.
   */
  rssi: uint8_t
}

/**
 * The RAW values of the RC channels received. The standard PPM modulation is as follows: 1000
 * microseconds: 0%, 2000 microseconds: 100%. A value of UINT16_MAX implies the channel is unused.
 * Individual receivers/transmitters might violate this specification.
 */
export class RcChannelsRaw extends MavLinkData {
  static MSG_ID = 35
  static MSG_NAME = 'RC_CHANNELS_RAW'
  static PAYLOAD_LENGTH = 22
  static MAGIC_NUMBER = 244

  static FIELDS = [
    new MavLinkPacketField('time_boot_ms', 'timeBootMs', 0, false, 4, 'uint32_t', 'ms'),
    new MavLinkPacketField('chan1_raw', 'chan1Raw', 4, false, 2, 'uint16_t', 'us'),
    new MavLinkPacketField('chan2_raw', 'chan2Raw', 6, false, 2, 'uint16_t', 'us'),
    new MavLinkPacketField('chan3_raw', 'chan3Raw', 8, false, 2, 'uint16_t', 'us'),
    new MavLinkPacketField('chan4_raw', 'chan4Raw', 10, false, 2, 'uint16_t', 'us'),
    new MavLinkPacketField('chan5_raw', 'chan5Raw', 12, false, 2, 'uint16_t', 'us'),
    new MavLinkPacketField('chan6_raw', 'chan6Raw', 14, false, 2, 'uint16_t', 'us'),
    new MavLinkPacketField('chan7_raw', 'chan7Raw', 16, false, 2, 'uint16_t', 'us'),
    new MavLinkPacketField('chan8_raw', 'chan8Raw', 18, false, 2, 'uint16_t', 'us'),
    new MavLinkPacketField('port', 'port', 20, false, 1, 'uint8_t', ''),
    new MavLinkPacketField('rssi', 'rssi', 21, false, 1, 'uint8_t', ''),
  ]

  constructor() {
    super()
    this.timeBootMs = 0
    this.port = 0
    this.chan1Raw = 0
    this.chan2Raw = 0
    this.chan3Raw = 0
    this.chan4Raw = 0
    this.chan5Raw = 0
    this.chan6Raw = 0
    this.chan7Raw = 0
    this.chan8Raw = 0
    this.rssi = 0
  }

  /**
   * Timestamp (time since system boot).
   * Units: ms
   */
  timeBootMs: uint32_t

  /**
   * Servo output port (set of 8 outputs = 1 port). Flight stacks running on Pixhawk should use: 0 =
   * MAIN, 1 = AUX.
   */
  port: uint8_t

  /**
   * RC channel 1 value.
   * Units: us
   */
  chan1Raw: uint16_t

  /**
   * RC channel 2 value.
   * Units: us
   */
  chan2Raw: uint16_t

  /**
   * RC channel 3 value.
   * Units: us
   */
  chan3Raw: uint16_t

  /**
   * RC channel 4 value.
   * Units: us
   */
  chan4Raw: uint16_t

  /**
   * RC channel 5 value.
   * Units: us
   */
  chan5Raw: uint16_t

  /**
   * RC channel 6 value.
   * Units: us
   */
  chan6Raw: uint16_t

  /**
   * RC channel 7 value.
   * Units: us
   */
  chan7Raw: uint16_t

  /**
   * RC channel 8 value.
   * Units: us
   */
  chan8Raw: uint16_t

  /**
   * Receive signal strength indicator in device-dependent units/scale. Values: [0-254], UINT8_MAX:
   * invalid/unknown.
   */
  rssi: uint8_t
}

/**
 * Superseded by ACTUATOR_OUTPUT_STATUS. The RAW values of the servo outputs (for RC input from the
 * remote, use the RC_CHANNELS messages). The standard PPM modulation is as follows: 1000 microseconds:
 * 0%, 2000 microseconds: 100%.
 */
export class ServoOutputRaw extends MavLinkData {
  static MSG_ID = 36
  static MSG_NAME = 'SERVO_OUTPUT_RAW'
  static PAYLOAD_LENGTH = 37
  static MAGIC_NUMBER = 222

  static FIELDS = [
    new MavLinkPacketField('time_usec', 'timeUsec', 0, false, 4, 'uint32_t', 'us'),
    new MavLinkPacketField('servo1_raw', 'servo1Raw', 4, false, 2, 'uint16_t', 'us'),
    new MavLinkPacketField('servo2_raw', 'servo2Raw', 6, false, 2, 'uint16_t', 'us'),
    new MavLinkPacketField('servo3_raw', 'servo3Raw', 8, false, 2, 'uint16_t', 'us'),
    new MavLinkPacketField('servo4_raw', 'servo4Raw', 10, false, 2, 'uint16_t', 'us'),
    new MavLinkPacketField('servo5_raw', 'servo5Raw', 12, false, 2, 'uint16_t', 'us'),
    new MavLinkPacketField('servo6_raw', 'servo6Raw', 14, false, 2, 'uint16_t', 'us'),
    new MavLinkPacketField('servo7_raw', 'servo7Raw', 16, false, 2, 'uint16_t', 'us'),
    new MavLinkPacketField('servo8_raw', 'servo8Raw', 18, false, 2, 'uint16_t', 'us'),
    new MavLinkPacketField('port', 'port', 20, false, 1, 'uint8_t', ''),
    new MavLinkPacketField('servo9_raw', 'servo9Raw', 21, true, 2, 'uint16_t', 'us'),
    new MavLinkPacketField('servo10_raw', 'servo10Raw', 23, true, 2, 'uint16_t', 'us'),
    new MavLinkPacketField('servo11_raw', 'servo11Raw', 25, true, 2, 'uint16_t', 'us'),
    new MavLinkPacketField('servo12_raw', 'servo12Raw', 27, true, 2, 'uint16_t', 'us'),
    new MavLinkPacketField('servo13_raw', 'servo13Raw', 29, true, 2, 'uint16_t', 'us'),
    new MavLinkPacketField('servo14_raw', 'servo14Raw', 31, true, 2, 'uint16_t', 'us'),
    new MavLinkPacketField('servo15_raw', 'servo15Raw', 33, true, 2, 'uint16_t', 'us'),
    new MavLinkPacketField('servo16_raw', 'servo16Raw', 35, true, 2, 'uint16_t', 'us'),
  ]

  constructor() {
    super()
    this.timeUsec = 0
    this.port = 0
    this.servo1Raw = 0
    this.servo2Raw = 0
    this.servo3Raw = 0
    this.servo4Raw = 0
    this.servo5Raw = 0
    this.servo6Raw = 0
    this.servo7Raw = 0
    this.servo8Raw = 0
    this.servo9Raw = 0
    this.servo10Raw = 0
    this.servo11Raw = 0
    this.servo12Raw = 0
    this.servo13Raw = 0
    this.servo14Raw = 0
    this.servo15Raw = 0
    this.servo16Raw = 0
  }

  /**
   * Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format
   * (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
   * Units: us
   */
  timeUsec: uint32_t

  /**
   * Servo output port (set of 8 outputs = 1 port). Flight stacks running on Pixhawk should use: 0 =
   * MAIN, 1 = AUX.
   */
  port: uint8_t

  /**
   * Servo output 1 value
   * Units: us
   */
  servo1Raw: uint16_t

  /**
   * Servo output 2 value
   * Units: us
   */
  servo2Raw: uint16_t

  /**
   * Servo output 3 value
   * Units: us
   */
  servo3Raw: uint16_t

  /**
   * Servo output 4 value
   * Units: us
   */
  servo4Raw: uint16_t

  /**
   * Servo output 5 value
   * Units: us
   */
  servo5Raw: uint16_t

  /**
   * Servo output 6 value
   * Units: us
   */
  servo6Raw: uint16_t

  /**
   * Servo output 7 value
   * Units: us
   */
  servo7Raw: uint16_t

  /**
   * Servo output 8 value
   * Units: us
   */
  servo8Raw: uint16_t

  /**
   * Servo output 9 value
   * Units: us
   */
  servo9Raw: uint16_t

  /**
   * Servo output 10 value
   * Units: us
   */
  servo10Raw: uint16_t

  /**
   * Servo output 11 value
   * Units: us
   */
  servo11Raw: uint16_t

  /**
   * Servo output 12 value
   * Units: us
   */
  servo12Raw: uint16_t

  /**
   * Servo output 13 value
   * Units: us
   */
  servo13Raw: uint16_t

  /**
   * Servo output 14 value
   * Units: us
   */
  servo14Raw: uint16_t

  /**
   * Servo output 15 value
   * Units: us
   */
  servo15Raw: uint16_t

  /**
   * Servo output 16 value
   * Units: us
   */
  servo16Raw: uint16_t
}

/**
 * Request a partial list of mission items from the system/component.
 * https://mavlink.io/en/services/mission.html. If start and end index are the same, just send one
 * waypoint.
 */
export class MissionRequestPartialList extends MavLinkData {
  static MSG_ID = 37
  static MSG_NAME = 'MISSION_REQUEST_PARTIAL_LIST'
  static PAYLOAD_LENGTH = 7
  static MAGIC_NUMBER = 212

  static FIELDS = [
    new MavLinkPacketField('start_index', 'startIndex', 0, false, 2, 'int16_t', ''),
    new MavLinkPacketField('end_index', 'endIndex', 2, false, 2, 'int16_t', ''),
    new MavLinkPacketField('target_system', 'targetSystem', 4, false, 1, 'uint8_t', ''),
    new MavLinkPacketField('target_component', 'targetComponent', 5, false, 1, 'uint8_t', ''),
    new MavLinkPacketField('mission_type', 'missionType', 6, true, 1, 'uint8_t', ''),
  ]

  constructor() {
    super()
    this.targetSystem = 0
    this.targetComponent = 0
    this.startIndex = 0
    this.endIndex = 0
    this.missionType = MavMissionType[Object.keys(MavMissionType)[0]]
  }

  /**
   * System ID
   */
  targetSystem: uint8_t

  /**
   * Component ID
   */
  targetComponent: uint8_t

  /**
   * Start index
   */
  startIndex: int16_t

  /**
   * End index, -1 by default (-1: send list to end). Else a valid index of the list
   */
  endIndex: int16_t

  /**
   * Mission type.
   */
  missionType: MavMissionType
}

/**
 * This message is sent to the MAV to write a partial list. If start index == end index, only one item
 * will be transmitted / updated. If the start index is NOT 0 and above the current list size, this
 * request should be REJECTED!
 */
export class MissionWritePartialList extends MavLinkData {
  static MSG_ID = 38
  static MSG_NAME = 'MISSION_WRITE_PARTIAL_LIST'
  static PAYLOAD_LENGTH = 7
  static MAGIC_NUMBER = 9

  static FIELDS = [
    new MavLinkPacketField('start_index', 'startIndex', 0, false, 2, 'int16_t', ''),
    new MavLinkPacketField('end_index', 'endIndex', 2, false, 2, 'int16_t', ''),
    new MavLinkPacketField('target_system', 'targetSystem', 4, false, 1, 'uint8_t', ''),
    new MavLinkPacketField('target_component', 'targetComponent', 5, false, 1, 'uint8_t', ''),
    new MavLinkPacketField('mission_type', 'missionType', 6, true, 1, 'uint8_t', ''),
  ]

  constructor() {
    super()
    this.targetSystem = 0
    this.targetComponent = 0
    this.startIndex = 0
    this.endIndex = 0
    this.missionType = MavMissionType[Object.keys(MavMissionType)[0]]
  }

  /**
   * System ID
   */
  targetSystem: uint8_t

  /**
   * Component ID
   */
  targetComponent: uint8_t

  /**
   * Start index. Must be smaller / equal to the largest index of the current onboard list.
   */
  startIndex: int16_t

  /**
   * End index, equal or greater than start index.
   */
  endIndex: int16_t

  /**
   * Mission type.
   */
  missionType: MavMissionType
}

/**
 * Message encoding a mission item. This message is emitted to announce
 the presence of a mission item
 * and to set a mission item on the system. The mission item can be either in x, y, z meters (type:
 * LOCAL) or x:lat, y:lon, z:altitude. Local frame is Z-down, right handed (NED), global frame is Z-up,
 * right handed (ENU). NaN may be used to indicate an optional/default value (e.g. to use the system's
 * current latitude or yaw rather than a specific value). See also
 * https://mavlink.io/en/services/mission.html.
 *
 * @deprecated since 2020-06, replaced by MISSION_ITEM_INT
 */
export class MissionItem extends MavLinkData {
  static MSG_ID = 39
  static MSG_NAME = 'MISSION_ITEM'
  static PAYLOAD_LENGTH = 38
  static MAGIC_NUMBER = 254

  static FIELDS = [
    new MavLinkPacketField('param1', 'param1', 0, false, 4, 'float', ''),
    new MavLinkPacketField('param2', 'param2', 4, false, 4, 'float', ''),
    new MavLinkPacketField('param3', 'param3', 8, false, 4, 'float', ''),
    new MavLinkPacketField('param4', 'param4', 12, false, 4, 'float', ''),
    new MavLinkPacketField('x', 'x', 16, false, 4, 'float', ''),
    new MavLinkPacketField('y', 'y', 20, false, 4, 'float', ''),
    new MavLinkPacketField('z', 'z', 24, false, 4, 'float', ''),
    new MavLinkPacketField('seq', 'seq', 28, false, 2, 'uint16_t', ''),
    new MavLinkPacketField('command', 'command', 30, false, 2, 'uint16_t', ''),
    new MavLinkPacketField('target_system', 'targetSystem', 32, false, 1, 'uint8_t', ''),
    new MavLinkPacketField('target_component', 'targetComponent', 33, false, 1, 'uint8_t', ''),
    new MavLinkPacketField('frame', 'frame', 34, false, 1, 'uint8_t', ''),
    new MavLinkPacketField('current', 'current', 35, false, 1, 'uint8_t', ''),
    new MavLinkPacketField('autocontinue', 'autocontinue', 36, false, 1, 'uint8_t', ''),
    new MavLinkPacketField('mission_type', 'missionType', 37, true, 1, 'uint8_t', ''),
  ]

  constructor() {
    super()
    this.targetSystem = 0
    this.targetComponent = 0
    this.seq = 0
    this.frame = MavFrame[Object.keys(MavFrame)[0]]
    this.command = MavCmd[Object.keys(MavCmd)[0]]
    this.current = 0
    this.autocontinue = 0
    this.param1 = 0
    this.param2 = 0
    this.param3 = 0
    this.param4 = 0
    this.x = 0
    this.y = 0
    this.z = 0
    this.missionType = MavMissionType[Object.keys(MavMissionType)[0]]
  }

  /**
   * System ID
   */
  targetSystem: uint8_t

  /**
   * Component ID
   */
  targetComponent: uint8_t

  /**
   * Sequence
   */
  seq: uint16_t

  /**
   * The coordinate system of the waypoint.
   */
  frame: MavFrame

  /**
   * The scheduled action for the waypoint.
   */
  command: MavCmd

  /**
   * false:0, true:1
   */
  current: uint8_t

  /**
   * Autocontinue to next waypoint
   */
  autocontinue: uint8_t

  /**
   * PARAM1, see MAV_CMD enum
   */
  param1: float

  /**
   * PARAM2, see MAV_CMD enum
   */
  param2: float

  /**
   * PARAM3, see MAV_CMD enum
   */
  param3: float

  /**
   * PARAM4, see MAV_CMD enum
   */
  param4: float

  /**
   * PARAM5 / local: X coordinate, global: latitude
   */
  x: float

  /**
   * PARAM6 / local: Y coordinate, global: longitude
   */
  y: float

  /**
   * PARAM7 / local: Z coordinate, global: altitude (relative or absolute, depending on frame).
   */
  z: float

  /**
   * Mission type.
   */
  missionType: MavMissionType
}

/**
 * Request the information of the mission item with the sequence number seq. The response of the system
 * to this message should be a MISSION_ITEM message. https://mavlink.io/en/services/mission.html
 *
 * @deprecated since 2020-06, replaced by MISSION_REQUEST_INT; A system that gets this request should respond with MISSION_ITEM_INT (as though MISSION_REQUEST_INT was received).
 */
export class MissionRequest extends MavLinkData {
  static MSG_ID = 40
  static MSG_NAME = 'MISSION_REQUEST'
  static PAYLOAD_LENGTH = 5
  static MAGIC_NUMBER = 230

  static FIELDS = [
    new MavLinkPacketField('seq', 'seq', 0, false, 2, 'uint16_t', ''),
    new MavLinkPacketField('target_system', 'targetSystem', 2, false, 1, 'uint8_t', ''),
    new MavLinkPacketField('target_component', 'targetComponent', 3, false, 1, 'uint8_t', ''),
    new MavLinkPacketField('mission_type', 'missionType', 4, true, 1, 'uint8_t', ''),
  ]

  constructor() {
    super()
    this.targetSystem = 0
    this.targetComponent = 0
    this.seq = 0
    this.missionType = MavMissionType[Object.keys(MavMissionType)[0]]
  }

  /**
   * System ID
   */
  targetSystem: uint8_t

  /**
   * Component ID
   */
  targetComponent: uint8_t

  /**
   * Sequence
   */
  seq: uint16_t

  /**
   * Mission type.
   */
  missionType: MavMissionType
}

/**
 * Set the mission item with sequence number seq as current item. This means that the MAV will continue
 * to this mission item on the shortest path (not following the mission items in-between).
 */
export class MissionSetCurrent extends MavLinkData {
  static MSG_ID = 41
  static MSG_NAME = 'MISSION_SET_CURRENT'
  static PAYLOAD_LENGTH = 4
  static MAGIC_NUMBER = 28

  static FIELDS = [
    new MavLinkPacketField('seq', 'seq', 0, false, 2, 'uint16_t', ''),
    new MavLinkPacketField('target_system', 'targetSystem', 2, false, 1, 'uint8_t', ''),
    new MavLinkPacketField('target_component', 'targetComponent', 3, false, 1, 'uint8_t', ''),
  ]

  constructor() {
    super()
    this.targetSystem = 0
    this.targetComponent = 0
    this.seq = 0
  }

  /**
   * System ID
   */
  targetSystem: uint8_t

  /**
   * Component ID
   */
  targetComponent: uint8_t

  /**
   * Sequence
   */
  seq: uint16_t
}

/**
 * Message that announces the sequence number of the current active mission item. The MAV will fly
 * towards this mission item.
 */
export class MissionCurrent extends MavLinkData {
  static MSG_ID = 42
  static MSG_NAME = 'MISSION_CURRENT'
  static PAYLOAD_LENGTH = 38
  static MAGIC_NUMBER = 28

  static FIELDS = [
    new MavLinkPacketField('seq', 'seq', 0, false, 2, 'uint16_t', ''),
    new MavLinkPacketField('resumeSeq', 'resumeSeq', 2, true, 2, 'uint16_t', ''),
    new MavLinkPacketField('flag', 'flag', 4, true, 2, 'uint16_t', ''),
    new MavLinkPacketField('heading', 'heading', 6, true, 4, 'float', 'deg'),
    new MavLinkPacketField('vector_x', 'vectorX', 10, true, 4, 'float', 'm'),
    new MavLinkPacketField('vector_y', 'vectorY', 14, true, 4, 'float', 'm'),
    new MavLinkPacketField('vector_z', 'vectorZ', 18, true, 4, 'float', 'm'),
    new MavLinkPacketField('distance', 'distance', 22, true, 4, 'float', 'm'),
    new MavLinkPacketField('trigger_interval', 'triggerInterval', 26, true, 4, 'float', 'm'),
    new MavLinkPacketField('speed', 'speed', 30, true, 4, 'float', 'm/s'),
    new MavLinkPacketField('radius', 'radius', 34, true, 4, 'float', 'm'),
  ]

  constructor() {
    super()
    this.seq = 0
    this.resumeSeq = 0
    this.flag = MavMissionCurrentFlag[Object.keys(MavMissionCurrentFlag)[0]]
    this.heading = 0
    this.vectorX = 0
    this.vectorY = 0
    this.vectorZ = 0
    this.distance = 0
    this.triggerInterval = 0
    this.speed = 0
    this.radius = 0
  }

  /**
   * Sequence
   */
  seq: uint16_t

  /**
   * resume waypoint Sequence
   */
  resumeSeq: uint16_t

  /**
   * flag
   */
  flag: MavMissionCurrentFlag

  /**
   * Heading angle(0-360),north is 0 deq
   * Units: deg
   */
  heading: float

  /**
   * waypoint to resume point.
   * Units: m
   */
  vectorX: float

  /**
   * waypoint to resume point.
   * Units: m
   */
  vectorY: float

  /**
   * waypoint to resume point.
   * Units: m
   */
  vectorZ: float

  /**
   * waypoint to resume point distance.
   * Units: m
   */
  distance: float

  /**
   * trigger interval.
   * Units: m
   */
  triggerInterval: float

  /**
   * speed.
   * Units: m/s
   */
  speed: float

  /**
   * radius.
   * Units: m
   */
  radius: float
}

/**
 * Request the overall list of mission items from the system/component.
 */
export class MissionRequestList extends MavLinkData {
  static MSG_ID = 43
  static MSG_NAME = 'MISSION_REQUEST_LIST'
  static PAYLOAD_LENGTH = 3
  static MAGIC_NUMBER = 132

  static FIELDS = [
    new MavLinkPacketField('target_system', 'targetSystem', 0, false, 1, 'uint8_t', ''),
    new MavLinkPacketField('target_component', 'targetComponent', 1, false, 1, 'uint8_t', ''),
    new MavLinkPacketField('mission_type', 'missionType', 2, true, 1, 'uint8_t', ''),
  ]

  constructor() {
    super()
    this.targetSystem = 0
    this.targetComponent = 0
    this.missionType = MavMissionType[Object.keys(MavMissionType)[0]]
  }

  /**
   * System ID
   */
  targetSystem: uint8_t

  /**
   * Component ID
   */
  targetComponent: uint8_t

  /**
   * Mission type.
   */
  missionType: MavMissionType
}

/**
 * This message is emitted as response to MISSION_REQUEST_LIST by the MAV and to initiate a write
 * transaction. The GCS can then request the individual mission item based on the knowledge of the
 * total number of waypoints.
 */
export class MissionCount extends MavLinkData {
  static MSG_ID = 44
  static MSG_NAME = 'MISSION_COUNT'
  static PAYLOAD_LENGTH = 5
  static MAGIC_NUMBER = 221

  static FIELDS = [
    new MavLinkPacketField('count', 'count', 0, false, 2, 'uint16_t', ''),
    new MavLinkPacketField('target_system', 'targetSystem', 2, false, 1, 'uint8_t', ''),
    new MavLinkPacketField('target_component', 'targetComponent', 3, false, 1, 'uint8_t', ''),
    new MavLinkPacketField('mission_type', 'missionType', 4, true, 1, 'uint8_t', ''),
  ]

  constructor() {
    super()
    this.targetSystem = 0
    this.targetComponent = 0
    this.count = 0
    this.missionType = MavMissionType[Object.keys(MavMissionType)[0]]
  }

  /**
   * System ID
   */
  targetSystem: uint8_t

  /**
   * Component ID
   */
  targetComponent: uint8_t

  /**
   * Number of mission items in the sequence
   */
  count: uint16_t

  /**
   * Mission type.
   */
  missionType: MavMissionType
}

/**
 * Delete all mission items at once.
 */
export class MissionClearAll extends MavLinkData {
  static MSG_ID = 45
  static MSG_NAME = 'MISSION_CLEAR_ALL'
  static PAYLOAD_LENGTH = 3
  static MAGIC_NUMBER = 232

  static FIELDS = [
    new MavLinkPacketField('target_system', 'targetSystem', 0, false, 1, 'uint8_t', ''),
    new MavLinkPacketField('target_component', 'targetComponent', 1, false, 1, 'uint8_t', ''),
    new MavLinkPacketField('mission_type', 'missionType', 2, true, 1, 'uint8_t', ''),
  ]

  constructor() {
    super()
    this.targetSystem = 0
    this.targetComponent = 0
    this.missionType = MavMissionType[Object.keys(MavMissionType)[0]]
  }

  /**
   * System ID
   */
  targetSystem: uint8_t

  /**
   * Component ID
   */
  targetComponent: uint8_t

  /**
   * Mission type.
   */
  missionType: MavMissionType
}

/**
 * A certain mission item has been reached. The system will either hold this position (or circle on the
 * orbit) or (if the autocontinue on the WP was set) continue to the next waypoint.
 */
export class MissionItemReached extends MavLinkData {
  static MSG_ID = 46
  static MSG_NAME = 'MISSION_ITEM_REACHED'
  static PAYLOAD_LENGTH = 2
  static MAGIC_NUMBER = 11

  static FIELDS = [
    new MavLinkPacketField('seq', 'seq', 0, false, 2, 'uint16_t', ''),
  ]

  constructor() {
    super()
    this.seq = 0
  }

  /**
   * Sequence
   */
  seq: uint16_t
}

/**
 * Acknowledgment message during waypoint handling. The type field states if this message is a positive
 * ack (type=0) or if an error happened (type=non-zero).
 */
export class MissionAck extends MavLinkData {
  static MSG_ID = 47
  static MSG_NAME = 'MISSION_ACK'
  static PAYLOAD_LENGTH = 4
  static MAGIC_NUMBER = 153

  static FIELDS = [
    new MavLinkPacketField('target_system', 'targetSystem', 0, false, 1, 'uint8_t', ''),
    new MavLinkPacketField('target_component', 'targetComponent', 1, false, 1, 'uint8_t', ''),
    new MavLinkPacketField('type', 'type', 2, false, 1, 'uint8_t', ''),
    new MavLinkPacketField('mission_type', 'missionType', 3, true, 1, 'uint8_t', ''),
  ]

  constructor() {
    super()
    this.targetSystem = 0
    this.targetComponent = 0
    this.type = MavMissionResult[Object.keys(MavMissionResult)[0]]
    this.missionType = MavMissionType[Object.keys(MavMissionType)[0]]
  }

  /**
   * System ID
   */
  targetSystem: uint8_t

  /**
   * Component ID
   */
  targetComponent: uint8_t

  /**
   * Mission result.
   */
  type: MavMissionResult

  /**
   * Mission type.
   */
  missionType: MavMissionType
}

/**
 * Sets the GPS co-ordinates of the vehicle local origin (0,0,0) position. Vehicle should emit
 * GPS_GLOBAL_ORIGIN irrespective of whether the origin is changed. This enables transform between the
 * local coordinate frame and the global (GPS) coordinate frame, which may be necessary when (for
 * example) indoor and outdoor settings are connected and the MAV should move from in- to outdoor.
 */
export class SetGpsGlobalOrigin extends MavLinkData {
  static MSG_ID = 48
  static MSG_NAME = 'SET_GPS_GLOBAL_ORIGIN'
  static PAYLOAD_LENGTH = 21
  static MAGIC_NUMBER = 41

  static FIELDS = [
    new MavLinkPacketField('latitude', 'latitude', 0, false, 4, 'int32_t', 'degE7'),
    new MavLinkPacketField('longitude', 'longitude', 4, false, 4, 'int32_t', 'degE7'),
    new MavLinkPacketField('altitude', 'altitude', 8, false, 4, 'int32_t', 'mm'),
    new MavLinkPacketField('target_system', 'targetSystem', 12, false, 1, 'uint8_t', ''),
    new MavLinkPacketField('time_usec', 'timeUsec', 13, true, 8, 'uint64_t', 'us'),
  ]

  constructor() {
    super()
    this.targetSystem = 0
    this.latitude = 0
    this.longitude = 0
    this.altitude = 0
    this.timeUsec = BigInt(0)
  }

  /**
   * System ID
   */
  targetSystem: uint8_t

  /**
   * Latitude (WGS84)
   * Units: degE7
   */
  latitude: int32_t

  /**
   * Longitude (WGS84)
   * Units: degE7
   */
  longitude: int32_t

  /**
   * Altitude (MSL). Positive for up.
   * Units: mm
   */
  altitude: int32_t

  /**
   * Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format
   * (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
   * Units: us
   */
  timeUsec: uint64_t
}

/**
 * Publishes the GPS co-ordinates of the vehicle local origin (0,0,0) position. Emitted whenever a new
 * GPS-Local position mapping is requested or set - e.g. following SET_GPS_GLOBAL_ORIGIN message.
 */
export class GpsGlobalOrigin extends MavLinkData {
  static MSG_ID = 49
  static MSG_NAME = 'GPS_GLOBAL_ORIGIN'
  static PAYLOAD_LENGTH = 20
  static MAGIC_NUMBER = 39

  static FIELDS = [
    new MavLinkPacketField('latitude', 'latitude', 0, false, 4, 'int32_t', 'degE7'),
    new MavLinkPacketField('longitude', 'longitude', 4, false, 4, 'int32_t', 'degE7'),
    new MavLinkPacketField('altitude', 'altitude', 8, false, 4, 'int32_t', 'mm'),
    new MavLinkPacketField('time_usec', 'timeUsec', 12, true, 8, 'uint64_t', 'us'),
  ]

  constructor() {
    super()
    this.latitude = 0
    this.longitude = 0
    this.altitude = 0
    this.timeUsec = BigInt(0)
  }

  /**
   * Latitude (WGS84)
   * Units: degE7
   */
  latitude: int32_t

  /**
   * Longitude (WGS84)
   * Units: degE7
   */
  longitude: int32_t

  /**
   * Altitude (MSL). Positive for up.
   * Units: mm
   */
  altitude: int32_t

  /**
   * Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format
   * (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
   * Units: us
   */
  timeUsec: uint64_t
}

/**
 * Bind a RC channel to a parameter. The parameter should change according to the RC channel value.
 */
export class ParamMapRc extends MavLinkData {
  static MSG_ID = 50
  static MSG_NAME = 'PARAM_MAP_RC'
  static PAYLOAD_LENGTH = 37
  static MAGIC_NUMBER = 78

  static FIELDS = [
    new MavLinkPacketField('param_value0', 'paramValue0', 0, false, 4, 'float', ''),
    new MavLinkPacketField('scale', 'scale', 4, false, 4, 'float', ''),
    new MavLinkPacketField('param_value_min', 'paramValueMin', 8, false, 4, 'float', ''),
    new MavLinkPacketField('param_value_max', 'paramValueMax', 12, false, 4, 'float', ''),
    new MavLinkPacketField('param_index', 'paramIndex', 16, false, 2, 'int16_t', ''),
    new MavLinkPacketField('target_system', 'targetSystem', 18, false, 1, 'uint8_t', ''),
    new MavLinkPacketField('target_component', 'targetComponent', 19, false, 1, 'uint8_t', ''),
    new MavLinkPacketField('param_id', 'paramId', 20, false, 1, 'char[]', '', 16),
    new MavLinkPacketField('parameter_rc_channel_index', 'parameterRcChannelIndex', 36, false, 1, 'uint8_t', ''),
  ]

  constructor() {
    super()
    this.targetSystem = 0
    this.targetComponent = 0
    this.paramId = ''
    this.paramIndex = 0
    this.parameterRcChannelIndex = 0
    this.paramValue0 = 0
    this.scale = 0
    this.paramValueMin = 0
    this.paramValueMax = 0
  }

  /**
   * System ID
   */
  targetSystem: uint8_t

  /**
   * Component ID
   */
  targetComponent: uint8_t

  /**
   * Onboard parameter id, terminated by NULL if the length is less than 16 human-readable chars and
   * WITHOUT null termination (NULL) byte if the length is exactly 16 chars - applications have to
   * provide 16+1 bytes storage if the ID is stored as string
   */
  paramId: string

  /**
   * Parameter index. Send -1 to use the param ID field as identifier (else the param id will be
   * ignored), send -2 to disable any existing map for this rc_channel_index.
   */
  paramIndex: int16_t

  /**
   * Index of parameter RC channel. Not equal to the RC channel id. Typically corresponds to a
   * potentiometer-knob on the RC.
   */
  parameterRcChannelIndex: uint8_t

  /**
   * Initial parameter value
   */
  paramValue0: float

  /**
   * Scale, maps the RC range [-1, 1] to a parameter value
   */
  scale: float

  /**
   * Minimum param value. The protocol does not define if this overwrites an onboard minimum value.
   * (Depends on implementation)
   */
  paramValueMin: float

  /**
   * Maximum param value. The protocol does not define if this overwrites an onboard maximum value.
   * (Depends on implementation)
   */
  paramValueMax: float
}

/**
 * Request the information of the mission item with the sequence number seq. The response of the system
 * to this message should be a MISSION_ITEM_INT message. https://mavlink.io/en/services/mission.html
 */
export class MissionRequestInt extends MavLinkData {
  static MSG_ID = 51
  static MSG_NAME = 'MISSION_REQUEST_INT'
  static PAYLOAD_LENGTH = 5
  static MAGIC_NUMBER = 196

  static FIELDS = [
    new MavLinkPacketField('seq', 'seq', 0, false, 2, 'uint16_t', ''),
    new MavLinkPacketField('target_system', 'targetSystem', 2, false, 1, 'uint8_t', ''),
    new MavLinkPacketField('target_component', 'targetComponent', 3, false, 1, 'uint8_t', ''),
    new MavLinkPacketField('mission_type', 'missionType', 4, true, 1, 'uint8_t', ''),
  ]

  constructor() {
    super()
    this.targetSystem = 0
    this.targetComponent = 0
    this.seq = 0
    this.missionType = MavMissionType[Object.keys(MavMissionType)[0]]
  }

  /**
   * System ID
   */
  targetSystem: uint8_t

  /**
   * Component ID
   */
  targetComponent: uint8_t

  /**
   * Sequence
   */
  seq: uint16_t

  /**
   * Mission type.
   */
  missionType: MavMissionType
}

/**
 * Set a safety zone (volume), which is defined by two corners of a cube. This message can be used to
 * tell the MAV which setpoints/waypoints to accept and which to reject. Safety areas are often
 * enforced by national or competition regulations.
 */
export class SafetySetAllowedArea extends MavLinkData {
  static MSG_ID = 54
  static MSG_NAME = 'SAFETY_SET_ALLOWED_AREA'
  static PAYLOAD_LENGTH = 27
  static MAGIC_NUMBER = 15

  static FIELDS = [
    new MavLinkPacketField('p1x', 'p1x', 0, false, 4, 'float', 'm'),
    new MavLinkPacketField('p1y', 'p1y', 4, false, 4, 'float', 'm'),
    new MavLinkPacketField('p1z', 'p1z', 8, false, 4, 'float', 'm'),
    new MavLinkPacketField('p2x', 'p2x', 12, false, 4, 'float', 'm'),
    new MavLinkPacketField('p2y', 'p2y', 16, false, 4, 'float', 'm'),
    new MavLinkPacketField('p2z', 'p2z', 20, false, 4, 'float', 'm'),
    new MavLinkPacketField('target_system', 'targetSystem', 24, false, 1, 'uint8_t', ''),
    new MavLinkPacketField('target_component', 'targetComponent', 25, false, 1, 'uint8_t', ''),
    new MavLinkPacketField('frame', 'frame', 26, false, 1, 'uint8_t', ''),
  ]

  constructor() {
    super()
    this.targetSystem = 0
    this.targetComponent = 0
    this.frame = MavFrame[Object.keys(MavFrame)[0]]
    this.p1x = 0
    this.p1y = 0
    this.p1z = 0
    this.p2x = 0
    this.p2y = 0
    this.p2z = 0
  }

  /**
   * System ID
   */
  targetSystem: uint8_t

  /**
   * Component ID
   */
  targetComponent: uint8_t

  /**
   * Coordinate frame. Can be either global, GPS, right-handed with Z axis up or local, right handed, Z
   * axis down.
   */
  frame: MavFrame

  /**
   * x position 1 / Latitude 1
   * Units: m
   */
  p1x: float

  /**
   * y position 1 / Longitude 1
   * Units: m
   */
  p1y: float

  /**
   * z position 1 / Altitude 1
   * Units: m
   */
  p1z: float

  /**
   * x position 2 / Latitude 2
   * Units: m
   */
  p2x: float

  /**
   * y position 2 / Longitude 2
   * Units: m
   */
  p2y: float

  /**
   * z position 2 / Altitude 2
   * Units: m
   */
  p2z: float
}

/**
 * Read out the safety zone the MAV currently assumes.
 */
export class SafetyAllowedArea extends MavLinkData {
  static MSG_ID = 55
  static MSG_NAME = 'SAFETY_ALLOWED_AREA'
  static PAYLOAD_LENGTH = 25
  static MAGIC_NUMBER = 3

  static FIELDS = [
    new MavLinkPacketField('p1x', 'p1x', 0, false, 4, 'float', 'm'),
    new MavLinkPacketField('p1y', 'p1y', 4, false, 4, 'float', 'm'),
    new MavLinkPacketField('p1z', 'p1z', 8, false, 4, 'float', 'm'),
    new MavLinkPacketField('p2x', 'p2x', 12, false, 4, 'float', 'm'),
    new MavLinkPacketField('p2y', 'p2y', 16, false, 4, 'float', 'm'),
    new MavLinkPacketField('p2z', 'p2z', 20, false, 4, 'float', 'm'),
    new MavLinkPacketField('frame', 'frame', 24, false, 1, 'uint8_t', ''),
  ]

  constructor() {
    super()
    this.frame = MavFrame[Object.keys(MavFrame)[0]]
    this.p1x = 0
    this.p1y = 0
    this.p1z = 0
    this.p2x = 0
    this.p2y = 0
    this.p2z = 0
  }

  /**
   * Coordinate frame. Can be either global, GPS, right-handed with Z axis up or local, right handed, Z
   * axis down.
   */
  frame: MavFrame

  /**
   * x position 1 / Latitude 1
   * Units: m
   */
  p1x: float

  /**
   * y position 1 / Longitude 1
   * Units: m
   */
  p1y: float

  /**
   * z position 1 / Altitude 1
   * Units: m
   */
  p1z: float

  /**
   * x position 2 / Latitude 2
   * Units: m
   */
  p2x: float

  /**
   * y position 2 / Longitude 2
   * Units: m
   */
  p2y: float

  /**
   * z position 2 / Altitude 2
   * Units: m
   */
  p2z: float
}

/**
 * The attitude in the aeronautical frame (right-handed, Z-down, X-front, Y-right), expressed as
 * quaternion. Quaternion order is w, x, y, z and a zero rotation would be expressed as (1 0 0 0).
 */
export class AttitudeQuaternionCov extends MavLinkData {
  static MSG_ID = 61
  static MSG_NAME = 'ATTITUDE_QUATERNION_COV'
  static PAYLOAD_LENGTH = 72
  static MAGIC_NUMBER = 167

  static FIELDS = [
    new MavLinkPacketField('time_usec', 'timeUsec', 0, false, 8, 'uint64_t', 'us'),
    new MavLinkPacketField('q', 'q', 8, false, 4, 'float[]', '', 4),
    new MavLinkPacketField('rollspeed', 'rollspeed', 24, false, 4, 'float', 'rad/s'),
    new MavLinkPacketField('pitchspeed', 'pitchspeed', 28, false, 4, 'float', 'rad/s'),
    new MavLinkPacketField('yawspeed', 'yawspeed', 32, false, 4, 'float', 'rad/s'),
    new MavLinkPacketField('covariance', 'covariance', 36, false, 4, 'float[]', '', 9),
  ]

  constructor() {
    super()
    this.timeUsec = BigInt(0)
    this.q = []
    this.rollspeed = 0
    this.pitchspeed = 0
    this.yawspeed = 0
    this.covariance = []
  }

  /**
   * Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format
   * (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
   * Units: us
   */
  timeUsec: uint64_t

  /**
   * Quaternion components, w, x, y, z (1 0 0 0 is the null-rotation)
   */
  q: float[]

  /**
   * Roll angular speed
   * Units: rad/s
   */
  rollspeed: float

  /**
   * Pitch angular speed
   * Units: rad/s
   */
  pitchspeed: float

  /**
   * Yaw angular speed
   * Units: rad/s
   */
  yawspeed: float

  /**
   * Row-major representation of a 3x3 attitude covariance matrix (states: roll, pitch, yaw; first three
   * entries are the first ROW, next three entries are the second row, etc.). If unknown, assign NaN
   * value to first element in the array.
   */
  covariance: float[]
}

/**
 * The state of the navigation and position controller.
 */
export class NavControllerOutput extends MavLinkData {
  static MSG_ID = 62
  static MSG_NAME = 'NAV_CONTROLLER_OUTPUT'
  static PAYLOAD_LENGTH = 26
  static MAGIC_NUMBER = 183

  static FIELDS = [
    new MavLinkPacketField('nav_roll', 'navRoll', 0, false, 4, 'float', 'deg'),
    new MavLinkPacketField('nav_pitch', 'navPitch', 4, false, 4, 'float', 'deg'),
    new MavLinkPacketField('alt_error', 'altError', 8, false, 4, 'float', 'm'),
    new MavLinkPacketField('aspd_error', 'aspdError', 12, false, 4, 'float', 'm/s'),
    new MavLinkPacketField('xtrack_error', 'xtrackError', 16, false, 4, 'float', 'm'),
    new MavLinkPacketField('nav_bearing', 'navBearing', 20, false, 2, 'int16_t', 'deg'),
    new MavLinkPacketField('target_bearing', 'targetBearing', 22, false, 2, 'int16_t', 'deg'),
    new MavLinkPacketField('wp_dist', 'wpDist', 24, false, 2, 'uint16_t', 'm'),
  ]

  constructor() {
    super()
    this.navRoll = 0
    this.navPitch = 0
    this.navBearing = 0
    this.targetBearing = 0
    this.wpDist = 0
    this.altError = 0
    this.aspdError = 0
    this.xtrackError = 0
  }

  /**
   * Current desired roll
   * Units: deg
   */
  navRoll: float

  /**
   * Current desired pitch
   * Units: deg
   */
  navPitch: float

  /**
   * Current desired heading
   * Units: deg
   */
  navBearing: int16_t

  /**
   * Bearing to current waypoint/target
   * Units: deg
   */
  targetBearing: int16_t

  /**
   * Distance to active waypoint
   * Units: m
   */
  wpDist: uint16_t

  /**
   * Current altitude error
   * Units: m
   */
  altError: float

  /**
   * Current airspeed error
   * Units: m/s
   */
  aspdError: float

  /**
   * Current crosstrack error on x-y plane
   * Units: m
   */
  xtrackError: float
}

/**
 * The filtered global position (e.g. fused GPS and accelerometers). The position is in GPS-frame
 * (right-handed, Z-up). It is designed as scaled integer message since the resolution of float is not
 * sufficient. NOTE: This message is intended for onboard networks / companion computers and
 * higher-bandwidth links and optimized for accuracy and completeness. Please use the
 * GLOBAL_POSITION_INT message for a minimal subset.
 */
export class GlobalPositionIntCov extends MavLinkData {
  static MSG_ID = 63
  static MSG_NAME = 'GLOBAL_POSITION_INT_COV'
  static PAYLOAD_LENGTH = 181
  static MAGIC_NUMBER = 119

  static FIELDS = [
    new MavLinkPacketField('time_usec', 'timeUsec', 0, false, 8, 'uint64_t', 'us'),
    new MavLinkPacketField('lat', 'lat', 8, false, 4, 'int32_t', 'degE7'),
    new MavLinkPacketField('lon', 'lon', 12, false, 4, 'int32_t', 'degE7'),
    new MavLinkPacketField('alt', 'alt', 16, false, 4, 'int32_t', 'mm'),
    new MavLinkPacketField('relative_alt', 'relativeAlt', 20, false, 4, 'int32_t', 'mm'),
    new MavLinkPacketField('vx', 'vx', 24, false, 4, 'float', 'm/s'),
    new MavLinkPacketField('vy', 'vy', 28, false, 4, 'float', 'm/s'),
    new MavLinkPacketField('vz', 'vz', 32, false, 4, 'float', 'm/s'),
    new MavLinkPacketField('covariance', 'covariance', 36, false, 4, 'float[]', '', 36),
    new MavLinkPacketField('estimator_type', 'estimatorType', 180, false, 1, 'uint8_t', ''),
  ]

  constructor() {
    super()
    this.timeUsec = BigInt(0)
    this.estimatorType = MavEstimatorType[Object.keys(MavEstimatorType)[0]]
    this.lat = 0
    this.lon = 0
    this.alt = 0
    this.relativeAlt = 0
    this.vx = 0
    this.vy = 0
    this.vz = 0
    this.covariance = []
  }

  /**
   * Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format
   * (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
   * Units: us
   */
  timeUsec: uint64_t

  /**
   * Class id of the estimator this estimate originated from.
   */
  estimatorType: MavEstimatorType

  /**
   * Latitude
   * Units: degE7
   */
  lat: int32_t

  /**
   * Longitude
   * Units: degE7
   */
  lon: int32_t

  /**
   * Altitude in meters above MSL
   * Units: mm
   */
  alt: int32_t

  /**
   * Altitude above ground
   * Units: mm
   */
  relativeAlt: int32_t

  /**
   * Ground X Speed (Latitude)
   * Units: m/s
   */
  vx: float

  /**
   * Ground Y Speed (Longitude)
   * Units: m/s
   */
  vy: float

  /**
   * Ground Z Speed (Altitude)
   * Units: m/s
   */
  vz: float

  /**
   * Row-major representation of a 6x6 position and velocity 6x6 cross-covariance matrix (states: lat,
   * lon, alt, vx, vy, vz; first six entries are the first ROW, next six entries are the second row,
   * etc.). If unknown, assign NaN value to first element in the array.
   */
  covariance: float[]
}

/**
 * The filtered local position (e.g. fused computer vision and accelerometers). Coordinate frame is
 * right-handed, Z-axis down (aeronautical frame, NED / north-east-down convention)
 */
export class LocalPositionNedCov extends MavLinkData {
  static MSG_ID = 64
  static MSG_NAME = 'LOCAL_POSITION_NED_COV'
  static PAYLOAD_LENGTH = 225
  static MAGIC_NUMBER = 191

  static FIELDS = [
    new MavLinkPacketField('time_usec', 'timeUsec', 0, false, 8, 'uint64_t', 'us'),
    new MavLinkPacketField('x', 'x', 8, false, 4, 'float', 'm'),
    new MavLinkPacketField('y', 'y', 12, false, 4, 'float', 'm'),
    new MavLinkPacketField('z', 'z', 16, false, 4, 'float', 'm'),
    new MavLinkPacketField('vx', 'vx', 20, false, 4, 'float', 'm/s'),
    new MavLinkPacketField('vy', 'vy', 24, false, 4, 'float', 'm/s'),
    new MavLinkPacketField('vz', 'vz', 28, false, 4, 'float', 'm/s'),
    new MavLinkPacketField('ax', 'ax', 32, false, 4, 'float', 'm/s/s'),
    new MavLinkPacketField('ay', 'ay', 36, false, 4, 'float', 'm/s/s'),
    new MavLinkPacketField('az', 'az', 40, false, 4, 'float', 'm/s/s'),
    new MavLinkPacketField('covariance', 'covariance', 44, false, 4, 'float[]', '', 45),
    new MavLinkPacketField('estimator_type', 'estimatorType', 224, false, 1, 'uint8_t', ''),
  ]

  constructor() {
    super()
    this.timeUsec = BigInt(0)
    this.estimatorType = MavEstimatorType[Object.keys(MavEstimatorType)[0]]
    this.x = 0
    this.y = 0
    this.z = 0
    this.vx = 0
    this.vy = 0
    this.vz = 0
    this.ax = 0
    this.ay = 0
    this.az = 0
    this.covariance = []
  }

  /**
   * Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format
   * (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
   * Units: us
   */
  timeUsec: uint64_t

  /**
   * Class id of the estimator this estimate originated from.
   */
  estimatorType: MavEstimatorType

  /**
   * X Position
   * Units: m
   */
  x: float

  /**
   * Y Position
   * Units: m
   */
  y: float

  /**
   * Z Position
   * Units: m
   */
  z: float

  /**
   * X Speed
   * Units: m/s
   */
  vx: float

  /**
   * Y Speed
   * Units: m/s
   */
  vy: float

  /**
   * Z Speed
   * Units: m/s
   */
  vz: float

  /**
   * X Acceleration
   * Units: m/s/s
   */
  ax: float

  /**
   * Y Acceleration
   * Units: m/s/s
   */
  ay: float

  /**
   * Z Acceleration
   * Units: m/s/s
   */
  az: float

  /**
   * Row-major representation of position, velocity and acceleration 9x9 cross-covariance matrix upper
   * right triangle (states: x, y, z, vx, vy, vz, ax, ay, az; first nine entries are the first ROW, next
   * eight entries are the second row, etc.). If unknown, assign NaN value to first element in the array.
   */
  covariance: float[]
}

/**
 * The PPM values of the RC channels received. The standard PPM modulation is as follows: 1000
 * microseconds: 0%, 2000 microseconds: 100%. A value of UINT16_MAX implies the channel is unused.
 * Individual receivers/transmitters might violate this specification.
 */
export class RcChannels extends MavLinkData {
  static MSG_ID = 65
  static MSG_NAME = 'RC_CHANNELS'
  static PAYLOAD_LENGTH = 42
  static MAGIC_NUMBER = 118

  static FIELDS = [
    new MavLinkPacketField('time_boot_ms', 'timeBootMs', 0, false, 4, 'uint32_t', 'ms'),
    new MavLinkPacketField('chan1_raw', 'chan1Raw', 4, false, 2, 'uint16_t', 'us'),
    new MavLinkPacketField('chan2_raw', 'chan2Raw', 6, false, 2, 'uint16_t', 'us'),
    new MavLinkPacketField('chan3_raw', 'chan3Raw', 8, false, 2, 'uint16_t', 'us'),
    new MavLinkPacketField('chan4_raw', 'chan4Raw', 10, false, 2, 'uint16_t', 'us'),
    new MavLinkPacketField('chan5_raw', 'chan5Raw', 12, false, 2, 'uint16_t', 'us'),
    new MavLinkPacketField('chan6_raw', 'chan6Raw', 14, false, 2, 'uint16_t', 'us'),
    new MavLinkPacketField('chan7_raw', 'chan7Raw', 16, false, 2, 'uint16_t', 'us'),
    new MavLinkPacketField('chan8_raw', 'chan8Raw', 18, false, 2, 'uint16_t', 'us'),
    new MavLinkPacketField('chan9_raw', 'chan9Raw', 20, false, 2, 'uint16_t', 'us'),
    new MavLinkPacketField('chan10_raw', 'chan10Raw', 22, false, 2, 'uint16_t', 'us'),
    new MavLinkPacketField('chan11_raw', 'chan11Raw', 24, false, 2, 'uint16_t', 'us'),
    new MavLinkPacketField('chan12_raw', 'chan12Raw', 26, false, 2, 'uint16_t', 'us'),
    new MavLinkPacketField('chan13_raw', 'chan13Raw', 28, false, 2, 'uint16_t', 'us'),
    new MavLinkPacketField('chan14_raw', 'chan14Raw', 30, false, 2, 'uint16_t', 'us'),
    new MavLinkPacketField('chan15_raw', 'chan15Raw', 32, false, 2, 'uint16_t', 'us'),
    new MavLinkPacketField('chan16_raw', 'chan16Raw', 34, false, 2, 'uint16_t', 'us'),
    new MavLinkPacketField('chan17_raw', 'chan17Raw', 36, false, 2, 'uint16_t', 'us'),
    new MavLinkPacketField('chan18_raw', 'chan18Raw', 38, false, 2, 'uint16_t', 'us'),
    new MavLinkPacketField('chancount', 'chancount', 40, false, 1, 'uint8_t', ''),
    new MavLinkPacketField('rssi', 'rssi', 41, false, 1, 'uint8_t', ''),
  ]

  constructor() {
    super()
    this.timeBootMs = 0
    this.chancount = 0
    this.chan1Raw = 0
    this.chan2Raw = 0
    this.chan3Raw = 0
    this.chan4Raw = 0
    this.chan5Raw = 0
    this.chan6Raw = 0
    this.chan7Raw = 0
    this.chan8Raw = 0
    this.chan9Raw = 0
    this.chan10Raw = 0
    this.chan11Raw = 0
    this.chan12Raw = 0
    this.chan13Raw = 0
    this.chan14Raw = 0
    this.chan15Raw = 0
    this.chan16Raw = 0
    this.chan17Raw = 0
    this.chan18Raw = 0
    this.rssi = 0
  }

  /**
   * Timestamp (time since system boot).
   * Units: ms
   */
  timeBootMs: uint32_t

  /**
   * Total number of RC channels being received. This can be larger than 18, indicating that more
   * channels are available but not given in this message. This value should be 0 when no RC channels are
   * available.
   */
  chancount: uint8_t

  /**
   * RC channel 1 value.
   * Units: us
   */
  chan1Raw: uint16_t

  /**
   * RC channel 2 value.
   * Units: us
   */
  chan2Raw: uint16_t

  /**
   * RC channel 3 value.
   * Units: us
   */
  chan3Raw: uint16_t

  /**
   * RC channel 4 value.
   * Units: us
   */
  chan4Raw: uint16_t

  /**
   * RC channel 5 value.
   * Units: us
   */
  chan5Raw: uint16_t

  /**
   * RC channel 6 value.
   * Units: us
   */
  chan6Raw: uint16_t

  /**
   * RC channel 7 value.
   * Units: us
   */
  chan7Raw: uint16_t

  /**
   * RC channel 8 value.
   * Units: us
   */
  chan8Raw: uint16_t

  /**
   * RC channel 9 value.
   * Units: us
   */
  chan9Raw: uint16_t

  /**
   * RC channel 10 value.
   * Units: us
   */
  chan10Raw: uint16_t

  /**
   * RC channel 11 value.
   * Units: us
   */
  chan11Raw: uint16_t

  /**
   * RC channel 12 value.
   * Units: us
   */
  chan12Raw: uint16_t

  /**
   * RC channel 13 value.
   * Units: us
   */
  chan13Raw: uint16_t

  /**
   * RC channel 14 value.
   * Units: us
   */
  chan14Raw: uint16_t

  /**
   * RC channel 15 value.
   * Units: us
   */
  chan15Raw: uint16_t

  /**
   * RC channel 16 value.
   * Units: us
   */
  chan16Raw: uint16_t

  /**
   * RC channel 17 value.
   * Units: us
   */
  chan17Raw: uint16_t

  /**
   * RC channel 18 value.
   * Units: us
   */
  chan18Raw: uint16_t

  /**
   * Receive signal strength indicator in device-dependent units/scale. Values: [0-254], UINT8_MAX:
   * invalid/unknown.
   */
  rssi: uint8_t
}

/**
 * Request a data stream.
 *
 * @deprecated since 2015-08, replaced by SET_MESSAGE_INTERVAL
 */
export class RequestDataStream extends MavLinkData {
  static MSG_ID = 66
  static MSG_NAME = 'REQUEST_DATA_STREAM'
  static PAYLOAD_LENGTH = 6
  static MAGIC_NUMBER = 148

  static FIELDS = [
    new MavLinkPacketField('req_message_rate', 'reqMessageRate', 0, false, 2, 'uint16_t', 'Hz'),
    new MavLinkPacketField('target_system', 'targetSystem', 2, false, 1, 'uint8_t', ''),
    new MavLinkPacketField('target_component', 'targetComponent', 3, false, 1, 'uint8_t', ''),
    new MavLinkPacketField('req_stream_id', 'reqStreamId', 4, false, 1, 'uint8_t', ''),
    new MavLinkPacketField('start_stop', 'startStop', 5, false, 1, 'uint8_t', ''),
  ]

  constructor() {
    super()
    this.targetSystem = 0
    this.targetComponent = 0
    this.reqStreamId = 0
    this.reqMessageRate = 0
    this.startStop = 0
  }

  /**
   * The target requested to send the message stream.
   */
  targetSystem: uint8_t

  /**
   * The target requested to send the message stream.
   */
  targetComponent: uint8_t

  /**
   * The ID of the requested data stream
   */
  reqStreamId: uint8_t

  /**
   * The requested message rate
   * Units: Hz
   */
  reqMessageRate: uint16_t

  /**
   * 1 to start sending, 0 to stop sending.
   */
  startStop: uint8_t
}

/**
 * Data stream status information.
 *
 * @deprecated since 2015-08, replaced by MESSAGE_INTERVAL
 */
export class DataStream extends MavLinkData {
  static MSG_ID = 67
  static MSG_NAME = 'DATA_STREAM'
  static PAYLOAD_LENGTH = 4
  static MAGIC_NUMBER = 21

  static FIELDS = [
    new MavLinkPacketField('message_rate', 'messageRate', 0, false, 2, 'uint16_t', 'Hz'),
    new MavLinkPacketField('stream_id', 'streamId', 2, false, 1, 'uint8_t', ''),
    new MavLinkPacketField('on_off', 'onOff', 3, false, 1, 'uint8_t', ''),
  ]

  constructor() {
    super()
    this.streamId = 0
    this.messageRate = 0
    this.onOff = 0
  }

  /**
   * The ID of the requested data stream
   */
  streamId: uint8_t

  /**
   * The message rate
   * Units: Hz
   */
  messageRate: uint16_t

  /**
   * 1 stream is enabled, 0 stream is stopped.
   */
  onOff: uint8_t
}

/**
 * This message provides an API for manually controlling the vehicle using standard joystick axes
 * nomenclature, along with a joystick-like input device. Unused axes can be disabled and buttons
 * states are transmitted as individual on/off bits of a bitmask
 */
export class ManualControl extends MavLinkData {
  static MSG_ID = 69
  static MSG_NAME = 'MANUAL_CONTROL'
  static PAYLOAD_LENGTH = 26
  static MAGIC_NUMBER = 243

  static FIELDS = [
    new MavLinkPacketField('x', 'x', 0, false, 2, 'int16_t', ''),
    new MavLinkPacketField('y', 'y', 2, false, 2, 'int16_t', ''),
    new MavLinkPacketField('z', 'z', 4, false, 2, 'int16_t', ''),
    new MavLinkPacketField('r', 'r', 6, false, 2, 'int16_t', ''),
    new MavLinkPacketField('buttons', 'buttons', 8, false, 2, 'uint16_t', ''),
    new MavLinkPacketField('target', 'target', 10, false, 1, 'uint8_t', ''),
    new MavLinkPacketField('buttons2', 'buttons2', 11, true, 2, 'uint16_t', ''),
    new MavLinkPacketField('enabled_extensions', 'enabledExtensions', 13, true, 1, 'uint8_t', ''),
    new MavLinkPacketField('s', 's', 14, true, 2, 'int16_t', ''),
    new MavLinkPacketField('t', 't', 16, true, 2, 'int16_t', ''),
    new MavLinkPacketField('extendedAxis1', 'extendedAxis1', 18, true, 2, 'int16_t', ''),
    new MavLinkPacketField('extendedAxis2', 'extendedAxis2', 20, true, 2, 'int16_t', ''),
    new MavLinkPacketField('extendedAxis3', 'extendedAxis3', 22, true, 2, 'int16_t', ''),
    new MavLinkPacketField('extendedAxis4', 'extendedAxis4', 24, true, 2, 'int16_t', ''),
  ]

  constructor() {
    super()
    this.target = 0
    this.x = 0
    this.y = 0
    this.z = 0
    this.r = 0
    this.buttons = 0
    this.buttons2 = 0
    this.enabledExtensions = 0
    this.s = 0
    this.t = 0
    this.extendedAxis1 = 0
    this.extendedAxis2 = 0
    this.extendedAxis3 = 0
    this.extendedAxis4 = 0
  }

  /**
   * The system to be controlled.
   */
  target: uint8_t

  /**
   * X-axis, normalized to the range [-1000,1000]. A value of INT16_MAX indicates that this axis is
   * invalid. Generally corresponds to forward(1000)-backward(-1000) movement on a joystick and the pitch
   * of a vehicle.
   */
  x: int16_t

  /**
   * Y-axis, normalized to the range [-1000,1000]. A value of INT16_MAX indicates that this axis is
   * invalid. Generally corresponds to left(-1000)-right(1000) movement on a joystick and the roll of a
   * vehicle.
   */
  y: int16_t

  /**
   * Z-axis, normalized to the range [-1000,1000]. A value of INT16_MAX indicates that this axis is
   * invalid. Generally corresponds to a separate slider movement with maximum being 1000 and minimum
   * being -1000 on a joystick and the thrust of a vehicle. Positive values are positive thrust, negative
   * values are negative thrust.
   */
  z: int16_t

  /**
   * R-axis, normalized to the range [-1000,1000]. A value of INT16_MAX indicates that this axis is
   * invalid. Generally corresponds to a twisting of the joystick, with counter-clockwise being 1000 and
   * clockwise being -1000, and the yaw of a vehicle.
   */
  r: int16_t

  /**
   * A bitfield corresponding to the joystick buttons' 0-15 current state, 1 for pressed, 0 for released.
   * The lowest bit corresponds to Button 1.
   */
  buttons: uint16_t

  /**
   * A bitfield corresponding to the joystick buttons' 16-31 current state, 1 for pressed, 0 for
   * released. The lowest bit corresponds to Button 16.
   */
  buttons2: uint16_t

  /**
   * Set bits to 1 to indicate which of the following extension fields contain valid data: bit 0: pitch,
   * bit 1: roll, bit 2: extended axis
   */
  enabledExtensions: uint8_t

  /**
   * Pitch-only-axis, normalized to the range [-1000,1000]. Generally corresponds to pitch on vehicles
   * with additional degrees of freedom. Valid if bit 0 of enabled_extensions field is set. Set to 0 if
   * invalid.
   */
  s: int16_t

  /**
   * Roll-only-axis, normalized to the range [-1000,1000]. Generally corresponds to roll on vehicles with
   * additional degrees of freedom. Valid if bit 1 of enabled_extensions field is set. Set to 0 if
   * invalid.
   */
  t: int16_t

  /**
   * extendedAxis1, normalized to the range [-1000,1000]. A value of INT16_MAX indicates that this axis
   * is invalid. Generally corresponds to forward(1000)-backward(-1000) movement on a joystick and the
   * pitch of a vehicle.
   */
  extendedAxis1: int16_t

  /**
   * extendedAxis2, normalized to the range [-1000,1000]. A value of INT16_MAX indicates that this axis
   * is invalid. Generally corresponds to left(-1000)-right(1000) movement on a joystick and the roll of
   * a vehicle.
   */
  extendedAxis2: int16_t

  /**
   * extendedAxis3, normalized to the range [-1000,1000]. A value of INT16_MAX indicates that this axis
   * is invalid. Generally corresponds to a separate slider movement with maximum being 1000 and minimum
   * being -1000 on a joystick and the thrust of a vehicle. Positive values are positive thrust, negative
   * values are negative thrust.
   */
  extendedAxis3: int16_t

  /**
   * extendedAxis4, normalized to the range [-1000,1000]. A value of INT16_MAX indicates that this axis
   * is invalid. Generally corresponds to a twisting of the joystick, with counter-clockwise being 1000
   * and clockwise being -1000, and the yaw of a vehicle.
   */
  extendedAxis4: int16_t
}

/**
 * The RAW values of the RC channels sent to the MAV to override info received from the RC radio. The
 * standard PPM modulation is as follows: 1000 microseconds: 0%, 2000 microseconds: 100%. Individual
 * receivers/transmitters might violate this specification. Note carefully the semantic differences
 * between the first 8 channels and the subsequent channels
 */
export class RcChannelsOverride extends MavLinkData {
  static MSG_ID = 70
  static MSG_NAME = 'RC_CHANNELS_OVERRIDE'
  static PAYLOAD_LENGTH = 38
  static MAGIC_NUMBER = 124

  static FIELDS = [
    new MavLinkPacketField('chan1_raw', 'chan1Raw', 0, false, 2, 'uint16_t', 'us'),
    new MavLinkPacketField('chan2_raw', 'chan2Raw', 2, false, 2, 'uint16_t', 'us'),
    new MavLinkPacketField('chan3_raw', 'chan3Raw', 4, false, 2, 'uint16_t', 'us'),
    new MavLinkPacketField('chan4_raw', 'chan4Raw', 6, false, 2, 'uint16_t', 'us'),
    new MavLinkPacketField('chan5_raw', 'chan5Raw', 8, false, 2, 'uint16_t', 'us'),
    new MavLinkPacketField('chan6_raw', 'chan6Raw', 10, false, 2, 'uint16_t', 'us'),
    new MavLinkPacketField('chan7_raw', 'chan7Raw', 12, false, 2, 'uint16_t', 'us'),
    new MavLinkPacketField('chan8_raw', 'chan8Raw', 14, false, 2, 'uint16_t', 'us'),
    new MavLinkPacketField('target_system', 'targetSystem', 16, false, 1, 'uint8_t', ''),
    new MavLinkPacketField('target_component', 'targetComponent', 17, false, 1, 'uint8_t', ''),
    new MavLinkPacketField('chan9_raw', 'chan9Raw', 18, true, 2, 'uint16_t', 'us'),
    new MavLinkPacketField('chan10_raw', 'chan10Raw', 20, true, 2, 'uint16_t', 'us'),
    new MavLinkPacketField('chan11_raw', 'chan11Raw', 22, true, 2, 'uint16_t', 'us'),
    new MavLinkPacketField('chan12_raw', 'chan12Raw', 24, true, 2, 'uint16_t', 'us'),
    new MavLinkPacketField('chan13_raw', 'chan13Raw', 26, true, 2, 'uint16_t', 'us'),
    new MavLinkPacketField('chan14_raw', 'chan14Raw', 28, true, 2, 'uint16_t', 'us'),
    new MavLinkPacketField('chan15_raw', 'chan15Raw', 30, true, 2, 'uint16_t', 'us'),
    new MavLinkPacketField('chan16_raw', 'chan16Raw', 32, true, 2, 'uint16_t', 'us'),
    new MavLinkPacketField('chan17_raw', 'chan17Raw', 34, true, 2, 'uint16_t', 'us'),
    new MavLinkPacketField('chan18_raw', 'chan18Raw', 36, true, 2, 'uint16_t', 'us'),
  ]

  constructor() {
    super()
    this.targetSystem = 0
    this.targetComponent = 0
    this.chan1Raw = 0
    this.chan2Raw = 0
    this.chan3Raw = 0
    this.chan4Raw = 0
    this.chan5Raw = 0
    this.chan6Raw = 0
    this.chan7Raw = 0
    this.chan8Raw = 0
    this.chan9Raw = 0
    this.chan10Raw = 0
    this.chan11Raw = 0
    this.chan12Raw = 0
    this.chan13Raw = 0
    this.chan14Raw = 0
    this.chan15Raw = 0
    this.chan16Raw = 0
    this.chan17Raw = 0
    this.chan18Raw = 0
  }

  /**
   * System ID
   */
  targetSystem: uint8_t

  /**
   * Component ID
   */
  targetComponent: uint8_t

  /**
   * RC channel 1 value. A value of UINT16_MAX means to ignore this field. A value of 0 means to release
   * this channel back to the RC radio.
   * Units: us
   */
  chan1Raw: uint16_t

  /**
   * RC channel 2 value. A value of UINT16_MAX means to ignore this field. A value of 0 means to release
   * this channel back to the RC radio.
   * Units: us
   */
  chan2Raw: uint16_t

  /**
   * RC channel 3 value. A value of UINT16_MAX means to ignore this field. A value of 0 means to release
   * this channel back to the RC radio.
   * Units: us
   */
  chan3Raw: uint16_t

  /**
   * RC channel 4 value. A value of UINT16_MAX means to ignore this field. A value of 0 means to release
   * this channel back to the RC radio.
   * Units: us
   */
  chan4Raw: uint16_t

  /**
   * RC channel 5 value. A value of UINT16_MAX means to ignore this field. A value of 0 means to release
   * this channel back to the RC radio.
   * Units: us
   */
  chan5Raw: uint16_t

  /**
   * RC channel 6 value. A value of UINT16_MAX means to ignore this field. A value of 0 means to release
   * this channel back to the RC radio.
   * Units: us
   */
  chan6Raw: uint16_t

  /**
   * RC channel 7 value. A value of UINT16_MAX means to ignore this field. A value of 0 means to release
   * this channel back to the RC radio.
   * Units: us
   */
  chan7Raw: uint16_t

  /**
   * RC channel 8 value. A value of UINT16_MAX means to ignore this field. A value of 0 means to release
   * this channel back to the RC radio.
   * Units: us
   */
  chan8Raw: uint16_t

  /**
   * RC channel 9 value. A value of 0 or UINT16_MAX means to ignore this field. A value of UINT16_MAX-1
   * means to release this channel back to the RC radio.
   * Units: us
   */
  chan9Raw: uint16_t

  /**
   * RC channel 10 value. A value of 0 or UINT16_MAX means to ignore this field. A value of UINT16_MAX-1
   * means to release this channel back to the RC radio.
   * Units: us
   */
  chan10Raw: uint16_t

  /**
   * RC channel 11 value. A value of 0 or UINT16_MAX means to ignore this field. A value of UINT16_MAX-1
   * means to release this channel back to the RC radio.
   * Units: us
   */
  chan11Raw: uint16_t

  /**
   * RC channel 12 value. A value of 0 or UINT16_MAX means to ignore this field. A value of UINT16_MAX-1
   * means to release this channel back to the RC radio.
   * Units: us
   */
  chan12Raw: uint16_t

  /**
   * RC channel 13 value. A value of 0 or UINT16_MAX means to ignore this field. A value of UINT16_MAX-1
   * means to release this channel back to the RC radio.
   * Units: us
   */
  chan13Raw: uint16_t

  /**
   * RC channel 14 value. A value of 0 or UINT16_MAX means to ignore this field. A value of UINT16_MAX-1
   * means to release this channel back to the RC radio.
   * Units: us
   */
  chan14Raw: uint16_t

  /**
   * RC channel 15 value. A value of 0 or UINT16_MAX means to ignore this field. A value of UINT16_MAX-1
   * means to release this channel back to the RC radio.
   * Units: us
   */
  chan15Raw: uint16_t

  /**
   * RC channel 16 value. A value of 0 or UINT16_MAX means to ignore this field. A value of UINT16_MAX-1
   * means to release this channel back to the RC radio.
   * Units: us
   */
  chan16Raw: uint16_t

  /**
   * RC channel 17 value. A value of 0 or UINT16_MAX means to ignore this field. A value of UINT16_MAX-1
   * means to release this channel back to the RC radio.
   * Units: us
   */
  chan17Raw: uint16_t

  /**
   * RC channel 18 value. A value of 0 or UINT16_MAX means to ignore this field. A value of UINT16_MAX-1
   * means to release this channel back to the RC radio.
   * Units: us
   */
  chan18Raw: uint16_t
}

/**
 * Message encoding a mission item. This message is emitted to announce
 the presence of a mission item
 * and to set a mission item on the system. The mission item can be either in x, y, z meters (type:
 * LOCAL) or x:lat, y:lon, z:altitude. Local frame is Z-down, right handed (NED), global frame is Z-up,
 * right handed (ENU). NaN or INT32_MAX may be used in float/integer params (respectively) to indicate
 * optional/default values (e.g. to use the component's current latitude, yaw rather than a specific
 * value). See also https://mavlink.io/en/services/mission.html.
 */
export class MissionItemInt extends MavLinkData {
  static MSG_ID = 73
  static MSG_NAME = 'MISSION_ITEM_INT'
  static PAYLOAD_LENGTH = 38
  static MAGIC_NUMBER = 38

  static FIELDS = [
    new MavLinkPacketField('param1', 'param1', 0, false, 4, 'float', ''),
    new MavLinkPacketField('param2', 'param2', 4, false, 4, 'float', ''),
    new MavLinkPacketField('param3', 'param3', 8, false, 4, 'float', ''),
    new MavLinkPacketField('param4', 'param4', 12, false, 4, 'float', ''),
    new MavLinkPacketField('x', 'x', 16, false, 4, 'int32_t', ''),
    new MavLinkPacketField('y', 'y', 20, false, 4, 'int32_t', ''),
    new MavLinkPacketField('z', 'z', 24, false, 4, 'float', ''),
    new MavLinkPacketField('seq', 'seq', 28, false, 2, 'uint16_t', ''),
    new MavLinkPacketField('command', 'command', 30, false, 2, 'uint16_t', ''),
    new MavLinkPacketField('target_system', 'targetSystem', 32, false, 1, 'uint8_t', ''),
    new MavLinkPacketField('target_component', 'targetComponent', 33, false, 1, 'uint8_t', ''),
    new MavLinkPacketField('frame', 'frame', 34, false, 1, 'uint8_t', ''),
    new MavLinkPacketField('current', 'current', 35, false, 1, 'uint8_t', ''),
    new MavLinkPacketField('autocontinue', 'autocontinue', 36, false, 1, 'uint8_t', ''),
    new MavLinkPacketField('mission_type', 'missionType', 37, true, 1, 'uint8_t', ''),
  ]

  constructor() {
    super()
    this.targetSystem = 0
    this.targetComponent = 0
    this.seq = 0
    this.frame = MavFrame[Object.keys(MavFrame)[0]]
    this.command = MavCmd[Object.keys(MavCmd)[0]]
    this.current = 0
    this.autocontinue = 0
    this.param1 = 0
    this.param2 = 0
    this.param3 = 0
    this.param4 = 0
    this.x = 0
    this.y = 0
    this.z = 0
    this.missionType = MavMissionType[Object.keys(MavMissionType)[0]]
  }

  /**
   * System ID
   */
  targetSystem: uint8_t

  /**
   * Component ID
   */
  targetComponent: uint8_t

  /**
   * Waypoint ID (sequence number). Starts at zero. Increases monotonically for each waypoint, no gaps in
   * the sequence (0,1,2,3,4).
   */
  seq: uint16_t

  /**
   * The coordinate system of the waypoint.
   */
  frame: MavFrame

  /**
   * The scheduled action for the waypoint.
   */
  command: MavCmd

  /**
   * false:0, true:1
   */
  current: uint8_t

  /**
   * Autocontinue to next waypoint
   */
  autocontinue: uint8_t

  /**
   * PARAM1, see MAV_CMD enum
   */
  param1: float

  /**
   * PARAM2, see MAV_CMD enum
   */
  param2: float

  /**
   * PARAM3, see MAV_CMD enum
   */
  param3: float

  /**
   * PARAM4, see MAV_CMD enum
   */
  param4: float

  /**
   * PARAM5 / local: x position in meters * 1e4, global: latitude in degrees * 10^7
   */
  x: int32_t

  /**
   * PARAM6 / y position: local: x position in meters * 1e4, global: longitude in degrees *10^7
   */
  y: int32_t

  /**
   * PARAM7 / z position: global: altitude in meters (relative or absolute, depending on frame.
   */
  z: float

  /**
   * Mission type.
   */
  missionType: MavMissionType
}

/**
 * Metrics typically displayed on a HUD for fixed wing aircraft.
 */
export class VfrHud extends MavLinkData {
  static MSG_ID = 74
  static MSG_NAME = 'VFR_HUD'
  static PAYLOAD_LENGTH = 20
  static MAGIC_NUMBER = 20

  static FIELDS = [
    new MavLinkPacketField('airspeed', 'airspeed', 0, false, 4, 'float', 'm/s'),
    new MavLinkPacketField('groundspeed', 'groundspeed', 4, false, 4, 'float', 'm/s'),
    new MavLinkPacketField('alt', 'alt', 8, false, 4, 'float', 'm'),
    new MavLinkPacketField('climb', 'climb', 12, false, 4, 'float', 'm/s'),
    new MavLinkPacketField('heading', 'heading', 16, false, 2, 'int16_t', 'deg'),
    new MavLinkPacketField('throttle', 'throttle', 18, false, 2, 'uint16_t', '%'),
  ]

  constructor() {
    super()
    this.airspeed = 0
    this.groundspeed = 0
    this.heading = 0
    this.throttle = 0
    this.alt = 0
    this.climb = 0
  }

  /**
   * Vehicle speed in form appropriate for vehicle type. For standard aircraft this is typically
   * calibrated airspeed (CAS) or indicated airspeed (IAS) - either of which can be used by a pilot to
   * estimate stall speed.
   * Units: m/s
   */
  airspeed: float

  /**
   * Current ground speed.
   * Units: m/s
   */
  groundspeed: float

  /**
   * Current heading in compass units (0-360, 0=north).
   * Units: deg
   */
  heading: int16_t

  /**
   * Current throttle setting (0 to 100).
   * Units: %
   */
  throttle: uint16_t

  /**
   * Current altitude (MSL).
   * Units: m
   */
  alt: float

  /**
   * Current climb rate.
   * Units: m/s
   */
  climb: float
}

/**
 * Message encoding a command with parameters as scaled integers. Scaling depends on the actual command
 * value. NaN or INT32_MAX may be used in float/integer params (respectively) to indicate
 * optional/default values (e.g. to use the component's current latitude, yaw rather than a specific
 * value). The command microservice is documented at https://mavlink.io/en/services/command.html
 */
export class CommandInt extends MavLinkData {
  static MSG_ID = 75
  static MSG_NAME = 'COMMAND_INT'
  static PAYLOAD_LENGTH = 35
  static MAGIC_NUMBER = 158

  static FIELDS = [
    new MavLinkPacketField('param1', '_param1', 0, false, 4, 'float', ''),
    new MavLinkPacketField('param2', '_param2', 4, false, 4, 'float', ''),
    new MavLinkPacketField('param3', '_param3', 8, false, 4, 'float', ''),
    new MavLinkPacketField('param4', '_param4', 12, false, 4, 'float', ''),
    new MavLinkPacketField('x', '_param5', 16, false, 4, 'int32_t', ''),
    new MavLinkPacketField('y', '_param6', 20, false, 4, 'int32_t', ''),
    new MavLinkPacketField('z', '_param7', 24, false, 4, 'float', ''),
    new MavLinkPacketField('command', 'command', 28, false, 2, 'uint16_t', ''),
    new MavLinkPacketField('target_system', 'targetSystem', 30, false, 1, 'uint8_t', ''),
    new MavLinkPacketField('target_component', 'targetComponent', 31, false, 1, 'uint8_t', ''),
    new MavLinkPacketField('frame', 'frame', 32, false, 1, 'uint8_t', ''),
    new MavLinkPacketField('current', 'current', 33, false, 1, 'uint8_t', ''),
    new MavLinkPacketField('autocontinue', 'autocontinue', 34, false, 1, 'uint8_t', ''),
  ]

  constructor() {
    super()
    this.targetSystem = 0
    this.targetComponent = 0
    this.frame = MavFrame[Object.keys(MavFrame)[0]]
    this.command = MavCmd[Object.keys(MavCmd)[0]]
    this.current = 0
    this.autocontinue = 0
    this._param1 = 0
    this._param2 = 0
    this._param3 = 0
    this._param4 = 0
    this._param5 = 0
    this._param6 = 0
    this._param7 = 0
  }

  /**
   * System ID
   */
  targetSystem: uint8_t

  /**
   * Component ID
   */
  targetComponent: uint8_t

  /**
   * The coordinate system of the COMMAND.
   */
  frame: MavFrame

  /**
   * The scheduled action for the mission item.
   */
  command: MavCmd

  /**
   * Not used.
   */
  current: uint8_t

  /**
   * Not used (set 0).
   */
  autocontinue: uint8_t

  /**
   * PARAM1, see MAV_CMD enum
   */
  _param1: float

  /**
   * PARAM2, see MAV_CMD enum
   */
  _param2: float

  /**
   * PARAM3, see MAV_CMD enum
   */
  _param3: float

  /**
   * PARAM4, see MAV_CMD enum
   */
  _param4: float

  /**
   * PARAM5 / local: x position in meters * 1e4, global: latitude in degrees * 10^7
   */
  _param5: int32_t

  /**
   * PARAM6 / local: y position in meters * 1e4, global: longitude in degrees * 10^7
   */
  _param6: int32_t

  /**
   * PARAM7 / z position: global: altitude in meters (relative or absolute, depending on frame).
   */
  _param7: float
}

/**
 * Send a command with up to seven parameters to the MAV. The command microservice is documented at
 * https://mavlink.io/en/services/command.html
 */
export class CommandLong extends MavLinkData {
  static MSG_ID = 76
  static MSG_NAME = 'COMMAND_LONG'
  static PAYLOAD_LENGTH = 33
  static MAGIC_NUMBER = 152

  static FIELDS = [
    new MavLinkPacketField('param1', '_param1', 0, false, 4, 'float', ''),
    new MavLinkPacketField('param2', '_param2', 4, false, 4, 'float', ''),
    new MavLinkPacketField('param3', '_param3', 8, false, 4, 'float', ''),
    new MavLinkPacketField('param4', '_param4', 12, false, 4, 'float', ''),
    new MavLinkPacketField('param5', '_param5', 16, false, 4, 'float', ''),
    new MavLinkPacketField('param6', '_param6', 20, false, 4, 'float', ''),
    new MavLinkPacketField('param7', '_param7', 24, false, 4, 'float', ''),
    new MavLinkPacketField('command', 'command', 28, false, 2, 'uint16_t', ''),
    new MavLinkPacketField('target_system', 'targetSystem', 30, false, 1, 'uint8_t', ''),
    new MavLinkPacketField('target_component', 'targetComponent', 31, false, 1, 'uint8_t', ''),
    new MavLinkPacketField('confirmation', 'confirmation', 32, false, 1, 'uint8_t', ''),
  ]

  constructor() {
    super()
    this.targetSystem = 0
    this.targetComponent = 0
    this.command = MavCmd[Object.keys(MavCmd)[0]]
    this.confirmation = 0
    this._param1 = 0
    this._param2 = 0
    this._param3 = 0
    this._param4 = 0
    this._param5 = 0
    this._param6 = 0
    this._param7 = 0
  }

  /**
   * System which should execute the command
   */
  targetSystem: uint8_t

  /**
   * Component which should execute the command, 0 for all components
   */
  targetComponent: uint8_t

  /**
   * Command ID (of command to send).
   */
  command: MavCmd

  /**
   * 0: First transmission of this command. 1-255: Confirmation transmissions (e.g. for kill command)
   */
  confirmation: uint8_t

  /**
   * Parameter 1 (for the specific command).
   */
  _param1: float

  /**
   * Parameter 2 (for the specific command).
   */
  _param2: float

  /**
   * Parameter 3 (for the specific command).
   */
  _param3: float

  /**
   * Parameter 4 (for the specific command).
   */
  _param4: float

  /**
   * Parameter 5 (for the specific command).
   */
  _param5: float

  /**
   * Parameter 6 (for the specific command).
   */
  _param6: float

  /**
   * Parameter 7 (for the specific command).
   */
  _param7: float
}

/**
 * Report status of a command. Includes feedback whether the command was executed. The command
 * microservice is documented at https://mavlink.io/en/services/command.html
 */
export class CommandAck extends MavLinkData {
  static MSG_ID = 77
  static MSG_NAME = 'COMMAND_ACK'
  static PAYLOAD_LENGTH = 10
  static MAGIC_NUMBER = 143

  static FIELDS = [
    new MavLinkPacketField('command', 'command', 0, false, 2, 'uint16_t', ''),
    new MavLinkPacketField('result', 'result', 2, false, 1, 'uint8_t', ''),
    new MavLinkPacketField('progress', 'progress', 3, true, 1, 'uint8_t', ''),
    new MavLinkPacketField('result_param2', 'resultParam2', 4, true, 4, 'int32_t', ''),
    new MavLinkPacketField('target_system', 'targetSystem', 8, true, 1, 'uint8_t', ''),
    new MavLinkPacketField('target_component', 'targetComponent', 9, true, 1, 'uint8_t', ''),
  ]

  constructor() {
    super()
    this.command = MavCmd[Object.keys(MavCmd)[0]]
    this.result = MavResult[Object.keys(MavResult)[0]]
    this.progress = 0
    this.resultParam2 = 0
    this.targetSystem = 0
    this.targetComponent = 0
  }

  /**
   * Command ID (of acknowledged command).
   */
  command: MavCmd

  /**
   * Result of command.
   */
  result: MavResult

  /**
   * Also used as result_param1, it can be set with an enum containing the errors reasons of why the
   * command was denied, or the progress percentage when result is MAV_RESULT_IN_PROGRESS (UINT8_MAX if
   * the progress is unknown).
   */
  progress: uint8_t

  /**
   * Additional parameter of the result, example: which parameter of MAV_CMD_NAV_WAYPOINT caused it to be
   * denied.
   */
  resultParam2: int32_t

  /**
   * System ID of the target recipient. This is the ID of the system that sent the command for which this
   * COMMAND_ACK is an acknowledgement.
   */
  targetSystem: uint8_t

  /**
   * Component ID of the target recipient. This is the ID of the system that sent the command for which
   * this COMMAND_ACK is an acknowledgement.
   */
  targetComponent: uint8_t
}

/**
 * Setpoint in roll, pitch, yaw and thrust from the operator
 */
export class ManualSetpoint extends MavLinkData {
  static MSG_ID = 81
  static MSG_NAME = 'MANUAL_SETPOINT'
  static PAYLOAD_LENGTH = 22
  static MAGIC_NUMBER = 106

  static FIELDS = [
    new MavLinkPacketField('time_boot_ms', 'timeBootMs', 0, false, 4, 'uint32_t', 'ms'),
    new MavLinkPacketField('roll', 'roll', 4, false, 4, 'float', 'rad/s'),
    new MavLinkPacketField('pitch', 'pitch', 8, false, 4, 'float', 'rad/s'),
    new MavLinkPacketField('yaw', 'yaw', 12, false, 4, 'float', 'rad/s'),
    new MavLinkPacketField('thrust', 'thrust', 16, false, 4, 'float', ''),
    new MavLinkPacketField('mode_switch', 'modeSwitch', 20, false, 1, 'uint8_t', ''),
    new MavLinkPacketField('manual_override_switch', 'manualOverrideSwitch', 21, false, 1, 'uint8_t', ''),
  ]

  constructor() {
    super()
    this.timeBootMs = 0
    this.roll = 0
    this.pitch = 0
    this.yaw = 0
    this.thrust = 0
    this.modeSwitch = 0
    this.manualOverrideSwitch = 0
  }

  /**
   * Timestamp (time since system boot).
   * Units: ms
   */
  timeBootMs: uint32_t

  /**
   * Desired roll rate
   * Units: rad/s
   */
  roll: float

  /**
   * Desired pitch rate
   * Units: rad/s
   */
  pitch: float

  /**
   * Desired yaw rate
   * Units: rad/s
   */
  yaw: float

  /**
   * Collective thrust, normalized to 0 .. 1
   */
  thrust: float

  /**
   * Flight mode switch position, 0.. 255
   */
  modeSwitch: uint8_t

  /**
   * Override mode switch position, 0.. 255
   */
  manualOverrideSwitch: uint8_t
}

/**
 * Sets a desired vehicle attitude. Used by an external controller to command the vehicle (manual
 * controller or other system).
 */
export class SetAttitudeTarget extends MavLinkData {
  static MSG_ID = 82
  static MSG_NAME = 'SET_ATTITUDE_TARGET'
  static PAYLOAD_LENGTH = 51
  static MAGIC_NUMBER = 49

  static FIELDS = [
    new MavLinkPacketField('time_boot_ms', 'timeBootMs', 0, false, 4, 'uint32_t', 'ms'),
    new MavLinkPacketField('q', 'q', 4, false, 4, 'float[]', '', 4),
    new MavLinkPacketField('body_roll_rate', 'bodyRollRate', 20, false, 4, 'float', 'rad/s'),
    new MavLinkPacketField('body_pitch_rate', 'bodyPitchRate', 24, false, 4, 'float', 'rad/s'),
    new MavLinkPacketField('body_yaw_rate', 'bodyYawRate', 28, false, 4, 'float', 'rad/s'),
    new MavLinkPacketField('thrust', 'thrust', 32, false, 4, 'float', ''),
    new MavLinkPacketField('target_system', 'targetSystem', 36, false, 1, 'uint8_t', ''),
    new MavLinkPacketField('target_component', 'targetComponent', 37, false, 1, 'uint8_t', ''),
    new MavLinkPacketField('type_mask', 'typeMask', 38, false, 1, 'uint8_t', ''),
    new MavLinkPacketField('thrust_body', 'thrustBody', 39, true, 4, 'float[]', '', 3),
  ]

  constructor() {
    super()
    this.timeBootMs = 0
    this.targetSystem = 0
    this.targetComponent = 0
    this.typeMask = AttitudeTargetTypemask[Object.keys(AttitudeTargetTypemask)[0]]
    this.q = []
    this.bodyRollRate = 0
    this.bodyPitchRate = 0
    this.bodyYawRate = 0
    this.thrust = 0
    this.thrustBody = []
  }

  /**
   * Timestamp (time since system boot).
   * Units: ms
   */
  timeBootMs: uint32_t

  /**
   * System ID
   */
  targetSystem: uint8_t

  /**
   * Component ID
   */
  targetComponent: uint8_t

  /**
   * Bitmap to indicate which dimensions should be ignored by the vehicle.
   */
  typeMask: AttitudeTargetTypemask

  /**
   * Attitude quaternion (w, x, y, z order, zero-rotation is 1, 0, 0, 0)
   */
  q: float[]

  /**
   * Body roll rate
   * Units: rad/s
   */
  bodyRollRate: float

  /**
   * Body pitch rate
   * Units: rad/s
   */
  bodyPitchRate: float

  /**
   * Body yaw rate
   * Units: rad/s
   */
  bodyYawRate: float

  /**
   * Collective thrust, normalized to 0 .. 1 (-1 .. 1 for vehicles capable of reverse trust)
   */
  thrust: float

  /**
   * 3D thrust setpoint in the body NED frame, normalized to -1 .. 1
   */
  thrustBody: float[]
}

/**
 * Reports the current commanded attitude of the vehicle as specified by the autopilot. This should
 * match the commands sent in a SET_ATTITUDE_TARGET message if the vehicle is being controlled this
 * way.
 */
export class AttitudeTarget extends MavLinkData {
  static MSG_ID = 83
  static MSG_NAME = 'ATTITUDE_TARGET'
  static PAYLOAD_LENGTH = 37
  static MAGIC_NUMBER = 22

  static FIELDS = [
    new MavLinkPacketField('time_boot_ms', 'timeBootMs', 0, false, 4, 'uint32_t', 'ms'),
    new MavLinkPacketField('q', 'q', 4, false, 4, 'float[]', '', 4),
    new MavLinkPacketField('body_roll_rate', 'bodyRollRate', 20, false, 4, 'float', 'rad/s'),
    new MavLinkPacketField('body_pitch_rate', 'bodyPitchRate', 24, false, 4, 'float', 'rad/s'),
    new MavLinkPacketField('body_yaw_rate', 'bodyYawRate', 28, false, 4, 'float', 'rad/s'),
    new MavLinkPacketField('thrust', 'thrust', 32, false, 4, 'float', ''),
    new MavLinkPacketField('type_mask', 'typeMask', 36, false, 1, 'uint8_t', ''),
  ]

  constructor() {
    super()
    this.timeBootMs = 0
    this.typeMask = AttitudeTargetTypemask[Object.keys(AttitudeTargetTypemask)[0]]
    this.q = []
    this.bodyRollRate = 0
    this.bodyPitchRate = 0
    this.bodyYawRate = 0
    this.thrust = 0
  }

  /**
   * Timestamp (time since system boot).
   * Units: ms
   */
  timeBootMs: uint32_t

  /**
   * Bitmap to indicate which dimensions should be ignored by the vehicle.
   */
  typeMask: AttitudeTargetTypemask

  /**
   * Attitude quaternion (w, x, y, z order, zero-rotation is 1, 0, 0, 0)
   */
  q: float[]

  /**
   * Body roll rate
   * Units: rad/s
   */
  bodyRollRate: float

  /**
   * Body pitch rate
   * Units: rad/s
   */
  bodyPitchRate: float

  /**
   * Body yaw rate
   * Units: rad/s
   */
  bodyYawRate: float

  /**
   * Collective thrust, normalized to 0 .. 1 (-1 .. 1 for vehicles capable of reverse trust)
   */
  thrust: float
}

/**
 * Sets a desired vehicle position in a local north-east-down coordinate frame. Used by an external
 * controller to command the vehicle (manual controller or other system).
 */
export class SetPositionTargetLocalNed extends MavLinkData {
  static MSG_ID = 84
  static MSG_NAME = 'SET_POSITION_TARGET_LOCAL_NED'
  static PAYLOAD_LENGTH = 53
  static MAGIC_NUMBER = 143

  static FIELDS = [
    new MavLinkPacketField('time_boot_ms', 'timeBootMs', 0, false, 4, 'uint32_t', 'ms'),
    new MavLinkPacketField('x', 'x', 4, false, 4, 'float', 'm'),
    new MavLinkPacketField('y', 'y', 8, false, 4, 'float', 'm'),
    new MavLinkPacketField('z', 'z', 12, false, 4, 'float', 'm'),
    new MavLinkPacketField('vx', 'vx', 16, false, 4, 'float', 'm/s'),
    new MavLinkPacketField('vy', 'vy', 20, false, 4, 'float', 'm/s'),
    new MavLinkPacketField('vz', 'vz', 24, false, 4, 'float', 'm/s'),
    new MavLinkPacketField('afx', 'afx', 28, false, 4, 'float', 'm/s/s'),
    new MavLinkPacketField('afy', 'afy', 32, false, 4, 'float', 'm/s/s'),
    new MavLinkPacketField('afz', 'afz', 36, false, 4, 'float', 'm/s/s'),
    new MavLinkPacketField('yaw', 'yaw', 40, false, 4, 'float', 'rad'),
    new MavLinkPacketField('yaw_rate', 'yawRate', 44, false, 4, 'float', 'rad/s'),
    new MavLinkPacketField('type_mask', 'typeMask', 48, false, 2, 'uint16_t', ''),
    new MavLinkPacketField('target_system', 'targetSystem', 50, false, 1, 'uint8_t', ''),
    new MavLinkPacketField('target_component', 'targetComponent', 51, false, 1, 'uint8_t', ''),
    new MavLinkPacketField('coordinate_frame', 'coordinateFrame', 52, false, 1, 'uint8_t', ''),
  ]

  constructor() {
    super()
    this.timeBootMs = 0
    this.targetSystem = 0
    this.targetComponent = 0
    this.coordinateFrame = MavFrame[Object.keys(MavFrame)[0]]
    this.typeMask = PositionTargetTypemask[Object.keys(PositionTargetTypemask)[0]]
    this.x = 0
    this.y = 0
    this.z = 0
    this.vx = 0
    this.vy = 0
    this.vz = 0
    this.afx = 0
    this.afy = 0
    this.afz = 0
    this.yaw = 0
    this.yawRate = 0
  }

  /**
   * Timestamp (time since system boot).
   * Units: ms
   */
  timeBootMs: uint32_t

  /**
   * System ID
   */
  targetSystem: uint8_t

  /**
   * Component ID
   */
  targetComponent: uint8_t

  /**
   * Valid options are: MAV_FRAME_LOCAL_NED = 1, MAV_FRAME_LOCAL_OFFSET_NED = 7, MAV_FRAME_BODY_NED = 8,
   * MAV_FRAME_BODY_OFFSET_NED = 9
   */
  coordinateFrame: MavFrame

  /**
   * Bitmap to indicate which dimensions should be ignored by the vehicle.
   */
  typeMask: PositionTargetTypemask

  /**
   * X Position in NED frame
   * Units: m
   */
  x: float

  /**
   * Y Position in NED frame
   * Units: m
   */
  y: float

  /**
   * Z Position in NED frame (note, altitude is negative in NED)
   * Units: m
   */
  z: float

  /**
   * X velocity in NED frame
   * Units: m/s
   */
  vx: float

  /**
   * Y velocity in NED frame
   * Units: m/s
   */
  vy: float

  /**
   * Z velocity in NED frame
   * Units: m/s
   */
  vz: float

  /**
   * X acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
   * Units: m/s/s
   */
  afx: float

  /**
   * Y acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
   * Units: m/s/s
   */
  afy: float

  /**
   * Z acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
   * Units: m/s/s
   */
  afz: float

  /**
   * yaw setpoint
   * Units: rad
   */
  yaw: float

  /**
   * yaw rate setpoint
   * Units: rad/s
   */
  yawRate: float
}

/**
 * Reports the current commanded vehicle position, velocity, and acceleration as specified by the
 * autopilot. This should match the commands sent in SET_POSITION_TARGET_LOCAL_NED if the vehicle is
 * being controlled this way.
 */
export class PositionTargetLocalNed extends MavLinkData {
  static MSG_ID = 85
  static MSG_NAME = 'POSITION_TARGET_LOCAL_NED'
  static PAYLOAD_LENGTH = 51
  static MAGIC_NUMBER = 140

  static FIELDS = [
    new MavLinkPacketField('time_boot_ms', 'timeBootMs', 0, false, 4, 'uint32_t', 'ms'),
    new MavLinkPacketField('x', 'x', 4, false, 4, 'float', 'm'),
    new MavLinkPacketField('y', 'y', 8, false, 4, 'float', 'm'),
    new MavLinkPacketField('z', 'z', 12, false, 4, 'float', 'm'),
    new MavLinkPacketField('vx', 'vx', 16, false, 4, 'float', 'm/s'),
    new MavLinkPacketField('vy', 'vy', 20, false, 4, 'float', 'm/s'),
    new MavLinkPacketField('vz', 'vz', 24, false, 4, 'float', 'm/s'),
    new MavLinkPacketField('afx', 'afx', 28, false, 4, 'float', 'm/s/s'),
    new MavLinkPacketField('afy', 'afy', 32, false, 4, 'float', 'm/s/s'),
    new MavLinkPacketField('afz', 'afz', 36, false, 4, 'float', 'm/s/s'),
    new MavLinkPacketField('yaw', 'yaw', 40, false, 4, 'float', 'rad'),
    new MavLinkPacketField('yaw_rate', 'yawRate', 44, false, 4, 'float', 'rad/s'),
    new MavLinkPacketField('type_mask', 'typeMask', 48, false, 2, 'uint16_t', ''),
    new MavLinkPacketField('coordinate_frame', 'coordinateFrame', 50, false, 1, 'uint8_t', ''),
  ]

  constructor() {
    super()
    this.timeBootMs = 0
    this.coordinateFrame = MavFrame[Object.keys(MavFrame)[0]]
    this.typeMask = PositionTargetTypemask[Object.keys(PositionTargetTypemask)[0]]
    this.x = 0
    this.y = 0
    this.z = 0
    this.vx = 0
    this.vy = 0
    this.vz = 0
    this.afx = 0
    this.afy = 0
    this.afz = 0
    this.yaw = 0
    this.yawRate = 0
  }

  /**
   * Timestamp (time since system boot).
   * Units: ms
   */
  timeBootMs: uint32_t

  /**
   * Valid options are: MAV_FRAME_LOCAL_NED = 1, MAV_FRAME_LOCAL_OFFSET_NED = 7, MAV_FRAME_BODY_NED = 8,
   * MAV_FRAME_BODY_OFFSET_NED = 9
   */
  coordinateFrame: MavFrame

  /**
   * Bitmap to indicate which dimensions should be ignored by the vehicle.
   */
  typeMask: PositionTargetTypemask

  /**
   * X Position in NED frame
   * Units: m
   */
  x: float

  /**
   * Y Position in NED frame
   * Units: m
   */
  y: float

  /**
   * Z Position in NED frame (note, altitude is negative in NED)
   * Units: m
   */
  z: float

  /**
   * X velocity in NED frame
   * Units: m/s
   */
  vx: float

  /**
   * Y velocity in NED frame
   * Units: m/s
   */
  vy: float

  /**
   * Z velocity in NED frame
   * Units: m/s
   */
  vz: float

  /**
   * X acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
   * Units: m/s/s
   */
  afx: float

  /**
   * Y acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
   * Units: m/s/s
   */
  afy: float

  /**
   * Z acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
   * Units: m/s/s
   */
  afz: float

  /**
   * yaw setpoint
   * Units: rad
   */
  yaw: float

  /**
   * yaw rate setpoint
   * Units: rad/s
   */
  yawRate: float
}

/**
 * Sets a desired vehicle position, velocity, and/or acceleration in a global coordinate system
 * (WGS84). Used by an external controller to command the vehicle (manual controller or other system).
 */
export class SetPositionTargetGlobalInt extends MavLinkData {
  static MSG_ID = 86
  static MSG_NAME = 'SET_POSITION_TARGET_GLOBAL_INT'
  static PAYLOAD_LENGTH = 53
  static MAGIC_NUMBER = 5

  static FIELDS = [
    new MavLinkPacketField('time_boot_ms', 'timeBootMs', 0, false, 4, 'uint32_t', 'ms'),
    new MavLinkPacketField('lat_int', 'latInt', 4, false, 4, 'int32_t', 'degE7'),
    new MavLinkPacketField('lon_int', 'lonInt', 8, false, 4, 'int32_t', 'degE7'),
    new MavLinkPacketField('alt', 'alt', 12, false, 4, 'float', 'm'),
    new MavLinkPacketField('vx', 'vx', 16, false, 4, 'float', 'm/s'),
    new MavLinkPacketField('vy', 'vy', 20, false, 4, 'float', 'm/s'),
    new MavLinkPacketField('vz', 'vz', 24, false, 4, 'float', 'm/s'),
    new MavLinkPacketField('afx', 'afx', 28, false, 4, 'float', 'm/s/s'),
    new MavLinkPacketField('afy', 'afy', 32, false, 4, 'float', 'm/s/s'),
    new MavLinkPacketField('afz', 'afz', 36, false, 4, 'float', 'm/s/s'),
    new MavLinkPacketField('yaw', 'yaw', 40, false, 4, 'float', 'rad'),
    new MavLinkPacketField('yaw_rate', 'yawRate', 44, false, 4, 'float', 'rad/s'),
    new MavLinkPacketField('type_mask', 'typeMask', 48, false, 2, 'uint16_t', ''),
    new MavLinkPacketField('target_system', 'targetSystem', 50, false, 1, 'uint8_t', ''),
    new MavLinkPacketField('target_component', 'targetComponent', 51, false, 1, 'uint8_t', ''),
    new MavLinkPacketField('coordinate_frame', 'coordinateFrame', 52, false, 1, 'uint8_t', ''),
  ]

  constructor() {
    super()
    this.timeBootMs = 0
    this.targetSystem = 0
    this.targetComponent = 0
    this.coordinateFrame = MavFrame[Object.keys(MavFrame)[0]]
    this.typeMask = PositionTargetTypemask[Object.keys(PositionTargetTypemask)[0]]
    this.latInt = 0
    this.lonInt = 0
    this.alt = 0
    this.vx = 0
    this.vy = 0
    this.vz = 0
    this.afx = 0
    this.afy = 0
    this.afz = 0
    this.yaw = 0
    this.yawRate = 0
  }

  /**
   * Timestamp (time since system boot). The rationale for the timestamp in the setpoint is to allow the
   * system to compensate for the transport delay of the setpoint. This allows the system to compensate
   * processing latency.
   * Units: ms
   */
  timeBootMs: uint32_t

  /**
   * System ID
   */
  targetSystem: uint8_t

  /**
   * Component ID
   */
  targetComponent: uint8_t

  /**
   * Valid options are: MAV_FRAME_GLOBAL_INT = 5, MAV_FRAME_GLOBAL_RELATIVE_ALT_INT = 6,
   * MAV_FRAME_GLOBAL_TERRAIN_ALT_INT = 11
   */
  coordinateFrame: MavFrame

  /**
   * Bitmap to indicate which dimensions should be ignored by the vehicle.
   */
  typeMask: PositionTargetTypemask

  /**
   * X Position in WGS84 frame
   * Units: degE7
   */
  latInt: int32_t

  /**
   * Y Position in WGS84 frame
   * Units: degE7
   */
  lonInt: int32_t

  /**
   * Altitude (MSL, Relative to home, or AGL - depending on frame)
   * Units: m
   */
  alt: float

  /**
   * X velocity in NED frame
   * Units: m/s
   */
  vx: float

  /**
   * Y velocity in NED frame
   * Units: m/s
   */
  vy: float

  /**
   * Z velocity in NED frame
   * Units: m/s
   */
  vz: float

  /**
   * X acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
   * Units: m/s/s
   */
  afx: float

  /**
   * Y acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
   * Units: m/s/s
   */
  afy: float

  /**
   * Z acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
   * Units: m/s/s
   */
  afz: float

  /**
   * yaw setpoint
   * Units: rad
   */
  yaw: float

  /**
   * yaw rate setpoint
   * Units: rad/s
   */
  yawRate: float
}

/**
 * Reports the current commanded vehicle position, velocity, and acceleration as specified by the
 * autopilot. This should match the commands sent in SET_POSITION_TARGET_GLOBAL_INT if the vehicle is
 * being controlled this way.
 */
export class PositionTargetGlobalInt extends MavLinkData {
  static MSG_ID = 87
  static MSG_NAME = 'POSITION_TARGET_GLOBAL_INT'
  static PAYLOAD_LENGTH = 51
  static MAGIC_NUMBER = 150

  static FIELDS = [
    new MavLinkPacketField('time_boot_ms', 'timeBootMs', 0, false, 4, 'uint32_t', 'ms'),
    new MavLinkPacketField('lat_int', 'latInt', 4, false, 4, 'int32_t', 'degE7'),
    new MavLinkPacketField('lon_int', 'lonInt', 8, false, 4, 'int32_t', 'degE7'),
    new MavLinkPacketField('alt', 'alt', 12, false, 4, 'float', 'm'),
    new MavLinkPacketField('vx', 'vx', 16, false, 4, 'float', 'm/s'),
    new MavLinkPacketField('vy', 'vy', 20, false, 4, 'float', 'm/s'),
    new MavLinkPacketField('vz', 'vz', 24, false, 4, 'float', 'm/s'),
    new MavLinkPacketField('afx', 'afx', 28, false, 4, 'float', 'm/s/s'),
    new MavLinkPacketField('afy', 'afy', 32, false, 4, 'float', 'm/s/s'),
    new MavLinkPacketField('afz', 'afz', 36, false, 4, 'float', 'm/s/s'),
    new MavLinkPacketField('yaw', 'yaw', 40, false, 4, 'float', 'rad'),
    new MavLinkPacketField('yaw_rate', 'yawRate', 44, false, 4, 'float', 'rad/s'),
    new MavLinkPacketField('type_mask', 'typeMask', 48, false, 2, 'uint16_t', ''),
    new MavLinkPacketField('coordinate_frame', 'coordinateFrame', 50, false, 1, 'uint8_t', ''),
  ]

  constructor() {
    super()
    this.timeBootMs = 0
    this.coordinateFrame = MavFrame[Object.keys(MavFrame)[0]]
    this.typeMask = PositionTargetTypemask[Object.keys(PositionTargetTypemask)[0]]
    this.latInt = 0
    this.lonInt = 0
    this.alt = 0
    this.vx = 0
    this.vy = 0
    this.vz = 0
    this.afx = 0
    this.afy = 0
    this.afz = 0
    this.yaw = 0
    this.yawRate = 0
  }

  /**
   * Timestamp (time since system boot). The rationale for the timestamp in the setpoint is to allow the
   * system to compensate for the transport delay of the setpoint. This allows the system to compensate
   * processing latency.
   * Units: ms
   */
  timeBootMs: uint32_t

  /**
   * Valid options are: MAV_FRAME_GLOBAL_INT = 5, MAV_FRAME_GLOBAL_RELATIVE_ALT_INT = 6,
   * MAV_FRAME_GLOBAL_TERRAIN_ALT_INT = 11
   */
  coordinateFrame: MavFrame

  /**
   * Bitmap to indicate which dimensions should be ignored by the vehicle.
   */
  typeMask: PositionTargetTypemask

  /**
   * X Position in WGS84 frame
   * Units: degE7
   */
  latInt: int32_t

  /**
   * Y Position in WGS84 frame
   * Units: degE7
   */
  lonInt: int32_t

  /**
   * Altitude (MSL, AGL or relative to home altitude, depending on frame)
   * Units: m
   */
  alt: float

  /**
   * X velocity in NED frame
   * Units: m/s
   */
  vx: float

  /**
   * Y velocity in NED frame
   * Units: m/s
   */
  vy: float

  /**
   * Z velocity in NED frame
   * Units: m/s
   */
  vz: float

  /**
   * X acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
   * Units: m/s/s
   */
  afx: float

  /**
   * Y acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
   * Units: m/s/s
   */
  afy: float

  /**
   * Z acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
   * Units: m/s/s
   */
  afz: float

  /**
   * yaw setpoint
   * Units: rad
   */
  yaw: float

  /**
   * yaw rate setpoint
   * Units: rad/s
   */
  yawRate: float
}

/**
 * The offset in X, Y, Z and yaw between the LOCAL_POSITION_NED messages of MAV X and the global
 * coordinate frame in NED coordinates. Coordinate frame is right-handed, Z-axis down (aeronautical
 * frame, NED / north-east-down convention)
 */
export class LocalPositionNedSystemGlobalOffset extends MavLinkData {
  static MSG_ID = 89
  static MSG_NAME = 'LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET'
  static PAYLOAD_LENGTH = 28
  static MAGIC_NUMBER = 231

  static FIELDS = [
    new MavLinkPacketField('time_boot_ms', 'timeBootMs', 0, false, 4, 'uint32_t', 'ms'),
    new MavLinkPacketField('x', 'x', 4, false, 4, 'float', 'm'),
    new MavLinkPacketField('y', 'y', 8, false, 4, 'float', 'm'),
    new MavLinkPacketField('z', 'z', 12, false, 4, 'float', 'm'),
    new MavLinkPacketField('roll', 'roll', 16, false, 4, 'float', 'rad'),
    new MavLinkPacketField('pitch', 'pitch', 20, false, 4, 'float', 'rad'),
    new MavLinkPacketField('yaw', 'yaw', 24, false, 4, 'float', 'rad'),
  ]

  constructor() {
    super()
    this.timeBootMs = 0
    this.x = 0
    this.y = 0
    this.z = 0
    this.roll = 0
    this.pitch = 0
    this.yaw = 0
  }

  /**
   * Timestamp (time since system boot).
   * Units: ms
   */
  timeBootMs: uint32_t

  /**
   * X Position
   * Units: m
   */
  x: float

  /**
   * Y Position
   * Units: m
   */
  y: float

  /**
   * Z Position
   * Units: m
   */
  z: float

  /**
   * Roll
   * Units: rad
   */
  roll: float

  /**
   * Pitch
   * Units: rad
   */
  pitch: float

  /**
   * Yaw
   * Units: rad
   */
  yaw: float
}

/**
 * Sent from simulation to autopilot. This packet is useful for high throughput applications such as
 * hardware in the loop simulations.
 *
 * @deprecated since 2013-07, replaced by HIL_STATE_QUATERNION; Suffers from missing airspeed fields and singularities due to Euler angles
 */
export class HilState extends MavLinkData {
  static MSG_ID = 90
  static MSG_NAME = 'HIL_STATE'
  static PAYLOAD_LENGTH = 56
  static MAGIC_NUMBER = 183

  static FIELDS = [
    new MavLinkPacketField('time_usec', 'timeUsec', 0, false, 8, 'uint64_t', 'us'),
    new MavLinkPacketField('roll', 'roll', 8, false, 4, 'float', 'rad'),
    new MavLinkPacketField('pitch', 'pitch', 12, false, 4, 'float', 'rad'),
    new MavLinkPacketField('yaw', 'yaw', 16, false, 4, 'float', 'rad'),
    new MavLinkPacketField('rollspeed', 'rollspeed', 20, false, 4, 'float', 'rad/s'),
    new MavLinkPacketField('pitchspeed', 'pitchspeed', 24, false, 4, 'float', 'rad/s'),
    new MavLinkPacketField('yawspeed', 'yawspeed', 28, false, 4, 'float', 'rad/s'),
    new MavLinkPacketField('lat', 'lat', 32, false, 4, 'int32_t', 'degE7'),
    new MavLinkPacketField('lon', 'lon', 36, false, 4, 'int32_t', 'degE7'),
    new MavLinkPacketField('alt', 'alt', 40, false, 4, 'int32_t', 'mm'),
    new MavLinkPacketField('vx', 'vx', 44, false, 2, 'int16_t', 'cm/s'),
    new MavLinkPacketField('vy', 'vy', 46, false, 2, 'int16_t', 'cm/s'),
    new MavLinkPacketField('vz', 'vz', 48, false, 2, 'int16_t', 'cm/s'),
    new MavLinkPacketField('xacc', 'xacc', 50, false, 2, 'int16_t', 'mG'),
    new MavLinkPacketField('yacc', 'yacc', 52, false, 2, 'int16_t', 'mG'),
    new MavLinkPacketField('zacc', 'zacc', 54, false, 2, 'int16_t', 'mG'),
  ]

  constructor() {
    super()
    this.timeUsec = BigInt(0)
    this.roll = 0
    this.pitch = 0
    this.yaw = 0
    this.rollspeed = 0
    this.pitchspeed = 0
    this.yawspeed = 0
    this.lat = 0
    this.lon = 0
    this.alt = 0
    this.vx = 0
    this.vy = 0
    this.vz = 0
    this.xacc = 0
    this.yacc = 0
    this.zacc = 0
  }

  /**
   * Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format
   * (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
   * Units: us
   */
  timeUsec: uint64_t

  /**
   * Roll angle
   * Units: rad
   */
  roll: float

  /**
   * Pitch angle
   * Units: rad
   */
  pitch: float

  /**
   * Yaw angle
   * Units: rad
   */
  yaw: float

  /**
   * Body frame roll / phi angular speed
   * Units: rad/s
   */
  rollspeed: float

  /**
   * Body frame pitch / theta angular speed
   * Units: rad/s
   */
  pitchspeed: float

  /**
   * Body frame yaw / psi angular speed
   * Units: rad/s
   */
  yawspeed: float

  /**
   * Latitude
   * Units: degE7
   */
  lat: int32_t

  /**
   * Longitude
   * Units: degE7
   */
  lon: int32_t

  /**
   * Altitude
   * Units: mm
   */
  alt: int32_t

  /**
   * Ground X Speed (Latitude)
   * Units: cm/s
   */
  vx: int16_t

  /**
   * Ground Y Speed (Longitude)
   * Units: cm/s
   */
  vy: int16_t

  /**
   * Ground Z Speed (Altitude)
   * Units: cm/s
   */
  vz: int16_t

  /**
   * X acceleration
   * Units: mG
   */
  xacc: int16_t

  /**
   * Y acceleration
   * Units: mG
   */
  yacc: int16_t

  /**
   * Z acceleration
   * Units: mG
   */
  zacc: int16_t
}

/**
 * Sent from autopilot to simulation. Hardware in the loop control outputs
 */
export class HilControls extends MavLinkData {
  static MSG_ID = 91
  static MSG_NAME = 'HIL_CONTROLS'
  static PAYLOAD_LENGTH = 42
  static MAGIC_NUMBER = 63

  static FIELDS = [
    new MavLinkPacketField('time_usec', 'timeUsec', 0, false, 8, 'uint64_t', 'us'),
    new MavLinkPacketField('roll_ailerons', 'rollAilerons', 8, false, 4, 'float', ''),
    new MavLinkPacketField('pitch_elevator', 'pitchElevator', 12, false, 4, 'float', ''),
    new MavLinkPacketField('yaw_rudder', 'yawRudder', 16, false, 4, 'float', ''),
    new MavLinkPacketField('throttle', 'throttle', 20, false, 4, 'float', ''),
    new MavLinkPacketField('aux1', 'aux1', 24, false, 4, 'float', ''),
    new MavLinkPacketField('aux2', 'aux2', 28, false, 4, 'float', ''),
    new MavLinkPacketField('aux3', 'aux3', 32, false, 4, 'float', ''),
    new MavLinkPacketField('aux4', 'aux4', 36, false, 4, 'float', ''),
    new MavLinkPacketField('mode', 'mode', 40, false, 1, 'uint8_t', ''),
    new MavLinkPacketField('nav_mode', 'navMode', 41, false, 1, 'uint8_t', ''),
  ]

  constructor() {
    super()
    this.timeUsec = BigInt(0)
    this.rollAilerons = 0
    this.pitchElevator = 0
    this.yawRudder = 0
    this.throttle = 0
    this.aux1 = 0
    this.aux2 = 0
    this.aux3 = 0
    this.aux4 = 0
    this.mode = MavMode[Object.keys(MavMode)[0]]
    this.navMode = 0
  }

  /**
   * Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format
   * (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
   * Units: us
   */
  timeUsec: uint64_t

  /**
   * Control output -1 .. 1
   */
  rollAilerons: float

  /**
   * Control output -1 .. 1
   */
  pitchElevator: float

  /**
   * Control output -1 .. 1
   */
  yawRudder: float

  /**
   * Throttle 0 .. 1
   */
  throttle: float

  /**
   * Aux 1, -1 .. 1
   */
  aux1: float

  /**
   * Aux 2, -1 .. 1
   */
  aux2: float

  /**
   * Aux 3, -1 .. 1
   */
  aux3: float

  /**
   * Aux 4, -1 .. 1
   */
  aux4: float

  /**
   * System mode.
   */
  mode: MavMode

  /**
   * Navigation mode (MAV_NAV_MODE)
   */
  navMode: uint8_t
}

/**
 * Sent from simulation to autopilot. The RAW values of the RC channels received. The standard PPM
 * modulation is as follows: 1000 microseconds: 0%, 2000 microseconds: 100%. Individual
 * receivers/transmitters might violate this specification.
 */
export class HilRcInputsRaw extends MavLinkData {
  static MSG_ID = 92
  static MSG_NAME = 'HIL_RC_INPUTS_RAW'
  static PAYLOAD_LENGTH = 33
  static MAGIC_NUMBER = 54

  static FIELDS = [
    new MavLinkPacketField('time_usec', 'timeUsec', 0, false, 8, 'uint64_t', 'us'),
    new MavLinkPacketField('chan1_raw', 'chan1Raw', 8, false, 2, 'uint16_t', 'us'),
    new MavLinkPacketField('chan2_raw', 'chan2Raw', 10, false, 2, 'uint16_t', 'us'),
    new MavLinkPacketField('chan3_raw', 'chan3Raw', 12, false, 2, 'uint16_t', 'us'),
    new MavLinkPacketField('chan4_raw', 'chan4Raw', 14, false, 2, 'uint16_t', 'us'),
    new MavLinkPacketField('chan5_raw', 'chan5Raw', 16, false, 2, 'uint16_t', 'us'),
    new MavLinkPacketField('chan6_raw', 'chan6Raw', 18, false, 2, 'uint16_t', 'us'),
    new MavLinkPacketField('chan7_raw', 'chan7Raw', 20, false, 2, 'uint16_t', 'us'),
    new MavLinkPacketField('chan8_raw', 'chan8Raw', 22, false, 2, 'uint16_t', 'us'),
    new MavLinkPacketField('chan9_raw', 'chan9Raw', 24, false, 2, 'uint16_t', 'us'),
    new MavLinkPacketField('chan10_raw', 'chan10Raw', 26, false, 2, 'uint16_t', 'us'),
    new MavLinkPacketField('chan11_raw', 'chan11Raw', 28, false, 2, 'uint16_t', 'us'),
    new MavLinkPacketField('chan12_raw', 'chan12Raw', 30, false, 2, 'uint16_t', 'us'),
    new MavLinkPacketField('rssi', 'rssi', 32, false, 1, 'uint8_t', ''),
  ]

  constructor() {
    super()
    this.timeUsec = BigInt(0)
    this.chan1Raw = 0
    this.chan2Raw = 0
    this.chan3Raw = 0
    this.chan4Raw = 0
    this.chan5Raw = 0
    this.chan6Raw = 0
    this.chan7Raw = 0
    this.chan8Raw = 0
    this.chan9Raw = 0
    this.chan10Raw = 0
    this.chan11Raw = 0
    this.chan12Raw = 0
    this.rssi = 0
  }

  /**
   * Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format
   * (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
   * Units: us
   */
  timeUsec: uint64_t

  /**
   * RC channel 1 value
   * Units: us
   */
  chan1Raw: uint16_t

  /**
   * RC channel 2 value
   * Units: us
   */
  chan2Raw: uint16_t

  /**
   * RC channel 3 value
   * Units: us
   */
  chan3Raw: uint16_t

  /**
   * RC channel 4 value
   * Units: us
   */
  chan4Raw: uint16_t

  /**
   * RC channel 5 value
   * Units: us
   */
  chan5Raw: uint16_t

  /**
   * RC channel 6 value
   * Units: us
   */
  chan6Raw: uint16_t

  /**
   * RC channel 7 value
   * Units: us
   */
  chan7Raw: uint16_t

  /**
   * RC channel 8 value
   * Units: us
   */
  chan8Raw: uint16_t

  /**
   * RC channel 9 value
   * Units: us
   */
  chan9Raw: uint16_t

  /**
   * RC channel 10 value
   * Units: us
   */
  chan10Raw: uint16_t

  /**
   * RC channel 11 value
   * Units: us
   */
  chan11Raw: uint16_t

  /**
   * RC channel 12 value
   * Units: us
   */
  chan12Raw: uint16_t

  /**
   * Receive signal strength indicator in device-dependent units/scale. Values: [0-254], UINT8_MAX:
   * invalid/unknown.
   */
  rssi: uint8_t
}

/**
 * Sent from autopilot to simulation. Hardware in the loop control outputs (replacement for
 * HIL_CONTROLS)
 */
export class HilActuatorControls extends MavLinkData {
  static MSG_ID = 93
  static MSG_NAME = 'HIL_ACTUATOR_CONTROLS'
  static PAYLOAD_LENGTH = 81
  static MAGIC_NUMBER = 47

  static FIELDS = [
    new MavLinkPacketField('time_usec', 'timeUsec', 0, false, 8, 'uint64_t', 'us'),
    new MavLinkPacketField('flags', 'flags', 8, false, 8, 'uint64_t', ''),
    new MavLinkPacketField('controls', 'controls', 16, false, 4, 'float[]', '', 16),
    new MavLinkPacketField('mode', 'mode', 80, false, 1, 'uint8_t', ''),
  ]

  constructor() {
    super()
    this.timeUsec = BigInt(0)
    this.controls = []
    this.mode = MavModeFlag[Object.keys(MavModeFlag)[0]]
    this.flags = BigInt(0)
  }

  /**
   * Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format
   * (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
   * Units: us
   */
  timeUsec: uint64_t

  /**
   * Control outputs -1 .. 1. Channel assignment depends on the simulated hardware.
   */
  controls: float[]

  /**
   * System mode. Includes arming state.
   */
  mode: MavModeFlag

  /**
   * Flags as bitfield, 1: indicate simulation using lockstep.
   */
  flags: uint64_t
}

/**
 * Optical flow from a flow sensor (e.g. optical mouse sensor)
 */
export class OpticalFlow extends MavLinkData {
  static MSG_ID = 100
  static MSG_NAME = 'OPTICAL_FLOW'
  static PAYLOAD_LENGTH = 34
  static MAGIC_NUMBER = 175

  static FIELDS = [
    new MavLinkPacketField('time_usec', 'timeUsec', 0, false, 8, 'uint64_t', 'us'),
    new MavLinkPacketField('flow_comp_m_x', 'flowCompMX', 8, false, 4, 'float', 'm/s'),
    new MavLinkPacketField('flow_comp_m_y', 'flowCompMY', 12, false, 4, 'float', 'm/s'),
    new MavLinkPacketField('ground_distance', 'groundDistance', 16, false, 4, 'float', 'm'),
    new MavLinkPacketField('flow_x', 'flowX', 20, false, 2, 'int16_t', 'dpix'),
    new MavLinkPacketField('flow_y', 'flowY', 22, false, 2, 'int16_t', 'dpix'),
    new MavLinkPacketField('sensor_id', 'sensorId', 24, false, 1, 'uint8_t', ''),
    new MavLinkPacketField('quality', 'quality', 25, false, 1, 'uint8_t', ''),
    new MavLinkPacketField('flow_rate_x', 'flowRateX', 26, true, 4, 'float', 'rad/s'),
    new MavLinkPacketField('flow_rate_y', 'flowRateY', 30, true, 4, 'float', 'rad/s'),
  ]

  constructor() {
    super()
    this.timeUsec = BigInt(0)
    this.sensorId = 0
    this.flowX = 0
    this.flowY = 0
    this.flowCompMX = 0
    this.flowCompMY = 0
    this.quality = 0
    this.groundDistance = 0
    this.flowRateX = 0
    this.flowRateY = 0
  }

  /**
   * Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format
   * (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
   * Units: us
   */
  timeUsec: uint64_t

  /**
   * Sensor ID
   */
  sensorId: uint8_t

  /**
   * Flow in x-sensor direction
   * Units: dpix
   */
  flowX: int16_t

  /**
   * Flow in y-sensor direction
   * Units: dpix
   */
  flowY: int16_t

  /**
   * Flow in x-sensor direction, angular-speed compensated
   * Units: m/s
   */
  flowCompMX: float

  /**
   * Flow in y-sensor direction, angular-speed compensated
   * Units: m/s
   */
  flowCompMY: float

  /**
   * Optical flow quality / confidence. 0: bad, 255: maximum quality
   */
  quality: uint8_t

  /**
   * Ground distance. Positive value: distance known. Negative value: Unknown distance
   * Units: m
   */
  groundDistance: float

  /**
   * Flow rate about X axis
   * Units: rad/s
   */
  flowRateX: float

  /**
   * Flow rate about Y axis
   * Units: rad/s
   */
  flowRateY: float
}

/**
 * Global position/attitude estimate from a vision source.
 */
export class GlobalVisionPositionEstimate extends MavLinkData {
  static MSG_ID = 101
  static MSG_NAME = 'GLOBAL_VISION_POSITION_ESTIMATE'
  static PAYLOAD_LENGTH = 117
  static MAGIC_NUMBER = 102

  static FIELDS = [
    new MavLinkPacketField('usec', 'usec', 0, false, 8, 'uint64_t', 'us'),
    new MavLinkPacketField('x', 'x', 8, false, 4, 'float', 'm'),
    new MavLinkPacketField('y', 'y', 12, false, 4, 'float', 'm'),
    new MavLinkPacketField('z', 'z', 16, false, 4, 'float', 'm'),
    new MavLinkPacketField('roll', 'roll', 20, false, 4, 'float', 'rad'),
    new MavLinkPacketField('pitch', 'pitch', 24, false, 4, 'float', 'rad'),
    new MavLinkPacketField('yaw', 'yaw', 28, false, 4, 'float', 'rad'),
    new MavLinkPacketField('covariance', 'covariance', 32, true, 4, 'float[]', '', 21),
    new MavLinkPacketField('reset_counter', 'resetCounter', 116, true, 1, 'uint8_t', ''),
  ]

  constructor() {
    super()
    this.usec = BigInt(0)
    this.x = 0
    this.y = 0
    this.z = 0
    this.roll = 0
    this.pitch = 0
    this.yaw = 0
    this.covariance = []
    this.resetCounter = 0
  }

  /**
   * Timestamp (UNIX time or since system boot)
   * Units: us
   */
  usec: uint64_t

  /**
   * Global X position
   * Units: m
   */
  x: float

  /**
   * Global Y position
   * Units: m
   */
  y: float

  /**
   * Global Z position
   * Units: m
   */
  z: float

  /**
   * Roll angle
   * Units: rad
   */
  roll: float

  /**
   * Pitch angle
   * Units: rad
   */
  pitch: float

  /**
   * Yaw angle
   * Units: rad
   */
  yaw: float

  /**
   * Row-major representation of pose 6x6 cross-covariance matrix upper right triangle (states: x_global,
   * y_global, z_global, roll, pitch, yaw; first six entries are the first ROW, next five entries are the
   * second ROW, etc.). If unknown, assign NaN value to first element in the array.
   */
  covariance: float[]

  /**
   * Estimate reset counter. This should be incremented when the estimate resets in any of the dimensions
   * (position, velocity, attitude, angular speed). This is designed to be used when e.g an external SLAM
   * system detects a loop-closure and the estimate jumps.
   */
  resetCounter: uint8_t
}

/**
 * Local position/attitude estimate from a vision source.
 */
export class VisionPositionEstimate extends MavLinkData {
  static MSG_ID = 102
  static MSG_NAME = 'VISION_POSITION_ESTIMATE'
  static PAYLOAD_LENGTH = 117
  static MAGIC_NUMBER = 158

  static FIELDS = [
    new MavLinkPacketField('usec', 'usec', 0, false, 8, 'uint64_t', 'us'),
    new MavLinkPacketField('x', 'x', 8, false, 4, 'float', 'm'),
    new MavLinkPacketField('y', 'y', 12, false, 4, 'float', 'm'),
    new MavLinkPacketField('z', 'z', 16, false, 4, 'float', 'm'),
    new MavLinkPacketField('roll', 'roll', 20, false, 4, 'float', 'rad'),
    new MavLinkPacketField('pitch', 'pitch', 24, false, 4, 'float', 'rad'),
    new MavLinkPacketField('yaw', 'yaw', 28, false, 4, 'float', 'rad'),
    new MavLinkPacketField('covariance', 'covariance', 32, true, 4, 'float[]', '', 21),
    new MavLinkPacketField('reset_counter', 'resetCounter', 116, true, 1, 'uint8_t', ''),
  ]

  constructor() {
    super()
    this.usec = BigInt(0)
    this.x = 0
    this.y = 0
    this.z = 0
    this.roll = 0
    this.pitch = 0
    this.yaw = 0
    this.covariance = []
    this.resetCounter = 0
  }

  /**
   * Timestamp (UNIX time or time since system boot)
   * Units: us
   */
  usec: uint64_t

  /**
   * Local X position
   * Units: m
   */
  x: float

  /**
   * Local Y position
   * Units: m
   */
  y: float

  /**
   * Local Z position
   * Units: m
   */
  z: float

  /**
   * Roll angle
   * Units: rad
   */
  roll: float

  /**
   * Pitch angle
   * Units: rad
   */
  pitch: float

  /**
   * Yaw angle
   * Units: rad
   */
  yaw: float

  /**
   * Row-major representation of pose 6x6 cross-covariance matrix upper right triangle (states: x, y, z,
   * roll, pitch, yaw; first six entries are the first ROW, next five entries are the second ROW, etc.).
   * If unknown, assign NaN value to first element in the array.
   */
  covariance: float[]

  /**
   * Estimate reset counter. This should be incremented when the estimate resets in any of the dimensions
   * (position, velocity, attitude, angular speed). This is designed to be used when e.g an external SLAM
   * system detects a loop-closure and the estimate jumps.
   */
  resetCounter: uint8_t
}

/**
 * Speed estimate from a vision source.
 */
export class VisionSpeedEstimate extends MavLinkData {
  static MSG_ID = 103
  static MSG_NAME = 'VISION_SPEED_ESTIMATE'
  static PAYLOAD_LENGTH = 57
  static MAGIC_NUMBER = 208

  static FIELDS = [
    new MavLinkPacketField('usec', 'usec', 0, false, 8, 'uint64_t', 'us'),
    new MavLinkPacketField('x', 'x', 8, false, 4, 'float', 'm/s'),
    new MavLinkPacketField('y', 'y', 12, false, 4, 'float', 'm/s'),
    new MavLinkPacketField('z', 'z', 16, false, 4, 'float', 'm/s'),
    new MavLinkPacketField('covariance', 'covariance', 20, true, 4, 'float[]', '', 9),
    new MavLinkPacketField('reset_counter', 'resetCounter', 56, true, 1, 'uint8_t', ''),
  ]

  constructor() {
    super()
    this.usec = BigInt(0)
    this.x = 0
    this.y = 0
    this.z = 0
    this.covariance = []
    this.resetCounter = 0
  }

  /**
   * Timestamp (UNIX time or time since system boot)
   * Units: us
   */
  usec: uint64_t

  /**
   * Global X speed
   * Units: m/s
   */
  x: float

  /**
   * Global Y speed
   * Units: m/s
   */
  y: float

  /**
   * Global Z speed
   * Units: m/s
   */
  z: float

  /**
   * Row-major representation of 3x3 linear velocity covariance matrix (states: vx, vy, vz; 1st three
   * entries - 1st row, etc.). If unknown, assign NaN value to first element in the array.
   */
  covariance: float[]

  /**
   * Estimate reset counter. This should be incremented when the estimate resets in any of the dimensions
   * (position, velocity, attitude, angular speed). This is designed to be used when e.g an external SLAM
   * system detects a loop-closure and the estimate jumps.
   */
  resetCounter: uint8_t
}

/**
 * Global position estimate from a Vicon motion system source.
 */
export class ViconPositionEstimate extends MavLinkData {
  static MSG_ID = 104
  static MSG_NAME = 'VICON_POSITION_ESTIMATE'
  static PAYLOAD_LENGTH = 116
  static MAGIC_NUMBER = 56

  static FIELDS = [
    new MavLinkPacketField('usec', 'usec', 0, false, 8, 'uint64_t', 'us'),
    new MavLinkPacketField('x', 'x', 8, false, 4, 'float', 'm'),
    new MavLinkPacketField('y', 'y', 12, false, 4, 'float', 'm'),
    new MavLinkPacketField('z', 'z', 16, false, 4, 'float', 'm'),
    new MavLinkPacketField('roll', 'roll', 20, false, 4, 'float', 'rad'),
    new MavLinkPacketField('pitch', 'pitch', 24, false, 4, 'float', 'rad'),
    new MavLinkPacketField('yaw', 'yaw', 28, false, 4, 'float', 'rad'),
    new MavLinkPacketField('covariance', 'covariance', 32, true, 4, 'float[]', '', 21),
  ]

  constructor() {
    super()
    this.usec = BigInt(0)
    this.x = 0
    this.y = 0
    this.z = 0
    this.roll = 0
    this.pitch = 0
    this.yaw = 0
    this.covariance = []
  }

  /**
   * Timestamp (UNIX time or time since system boot)
   * Units: us
   */
  usec: uint64_t

  /**
   * Global X position
   * Units: m
   */
  x: float

  /**
   * Global Y position
   * Units: m
   */
  y: float

  /**
   * Global Z position
   * Units: m
   */
  z: float

  /**
   * Roll angle
   * Units: rad
   */
  roll: float

  /**
   * Pitch angle
   * Units: rad
   */
  pitch: float

  /**
   * Yaw angle
   * Units: rad
   */
  yaw: float

  /**
   * Row-major representation of 6x6 pose cross-covariance matrix upper right triangle (states: x, y, z,
   * roll, pitch, yaw; first six entries are the first ROW, next five entries are the second ROW, etc.).
   * If unknown, assign NaN value to first element in the array.
   */
  covariance: float[]
}

/**
 * The IMU readings in SI units in NED body frame
 */
export class HighresImu extends MavLinkData {
  static MSG_ID = 105
  static MSG_NAME = 'HIGHRES_IMU'
  static PAYLOAD_LENGTH = 63
  static MAGIC_NUMBER = 93

  static FIELDS = [
    new MavLinkPacketField('time_usec', 'timeUsec', 0, false, 8, 'uint64_t', 'us'),
    new MavLinkPacketField('xacc', 'xacc', 8, false, 4, 'float', 'm/s/s'),
    new MavLinkPacketField('yacc', 'yacc', 12, false, 4, 'float', 'm/s/s'),
    new MavLinkPacketField('zacc', 'zacc', 16, false, 4, 'float', 'm/s/s'),
    new MavLinkPacketField('xgyro', 'xgyro', 20, false, 4, 'float', 'rad/s'),
    new MavLinkPacketField('ygyro', 'ygyro', 24, false, 4, 'float', 'rad/s'),
    new MavLinkPacketField('zgyro', 'zgyro', 28, false, 4, 'float', 'rad/s'),
    new MavLinkPacketField('xmag', 'xmag', 32, false, 4, 'float', 'gauss'),
    new MavLinkPacketField('ymag', 'ymag', 36, false, 4, 'float', 'gauss'),
    new MavLinkPacketField('zmag', 'zmag', 40, false, 4, 'float', 'gauss'),
    new MavLinkPacketField('abs_pressure', 'absPressure', 44, false, 4, 'float', 'hPa'),
    new MavLinkPacketField('diff_pressure', 'diffPressure', 48, false, 4, 'float', 'hPa'),
    new MavLinkPacketField('pressure_alt', 'pressureAlt', 52, false, 4, 'float', ''),
    new MavLinkPacketField('temperature', 'temperature', 56, false, 4, 'float', 'degC'),
    new MavLinkPacketField('fields_updated', 'fieldsUpdated', 60, false, 2, 'uint16_t', ''),
    new MavLinkPacketField('id', 'id', 62, true, 1, 'uint8_t', ''),
  ]

  constructor() {
    super()
    this.timeUsec = BigInt(0)
    this.xacc = 0
    this.yacc = 0
    this.zacc = 0
    this.xgyro = 0
    this.ygyro = 0
    this.zgyro = 0
    this.xmag = 0
    this.ymag = 0
    this.zmag = 0
    this.absPressure = 0
    this.diffPressure = 0
    this.pressureAlt = 0
    this.temperature = 0
    this.fieldsUpdated = HighresImuUpdatedFlags[Object.keys(HighresImuUpdatedFlags)[0]]
    this.id = 0
  }

  /**
   * Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format
   * (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
   * Units: us
   */
  timeUsec: uint64_t

  /**
   * X acceleration
   * Units: m/s/s
   */
  xacc: float

  /**
   * Y acceleration
   * Units: m/s/s
   */
  yacc: float

  /**
   * Z acceleration
   * Units: m/s/s
   */
  zacc: float

  /**
   * Angular speed around X axis
   * Units: rad/s
   */
  xgyro: float

  /**
   * Angular speed around Y axis
   * Units: rad/s
   */
  ygyro: float

  /**
   * Angular speed around Z axis
   * Units: rad/s
   */
  zgyro: float

  /**
   * X Magnetic field
   * Units: gauss
   */
  xmag: float

  /**
   * Y Magnetic field
   * Units: gauss
   */
  ymag: float

  /**
   * Z Magnetic field
   * Units: gauss
   */
  zmag: float

  /**
   * Absolute pressure
   * Units: hPa
   */
  absPressure: float

  /**
   * Differential pressure
   * Units: hPa
   */
  diffPressure: float

  /**
   * Altitude calculated from pressure
   */
  pressureAlt: float

  /**
   * Temperature
   * Units: degC
   */
  temperature: float

  /**
   * Bitmap for fields that have updated since last message
   */
  fieldsUpdated: HighresImuUpdatedFlags

  /**
   * Id. Ids are numbered from 0 and map to IMUs numbered from 1 (e.g. IMU1 will have a message with
   * id=0)
   */
  id: uint8_t
}

/**
 * Optical flow from an angular rate flow sensor (e.g. PX4FLOW or mouse sensor)
 */
export class OpticalFlowRad extends MavLinkData {
  static MSG_ID = 106
  static MSG_NAME = 'OPTICAL_FLOW_RAD'
  static PAYLOAD_LENGTH = 44
  static MAGIC_NUMBER = 138

  static FIELDS = [
    new MavLinkPacketField('time_usec', 'timeUsec', 0, false, 8, 'uint64_t', 'us'),
    new MavLinkPacketField('integration_time_us', 'integrationTimeUs', 8, false, 4, 'uint32_t', 'us'),
    new MavLinkPacketField('integrated_x', 'integratedX', 12, false, 4, 'float', 'rad'),
    new MavLinkPacketField('integrated_y', 'integratedY', 16, false, 4, 'float', 'rad'),
    new MavLinkPacketField('integrated_xgyro', 'integratedXgyro', 20, false, 4, 'float', 'rad'),
    new MavLinkPacketField('integrated_ygyro', 'integratedYgyro', 24, false, 4, 'float', 'rad'),
    new MavLinkPacketField('integrated_zgyro', 'integratedZgyro', 28, false, 4, 'float', 'rad'),
    new MavLinkPacketField('time_delta_distance_us', 'timeDeltaDistanceUs', 32, false, 4, 'uint32_t', 'us'),
    new MavLinkPacketField('distance', 'distance', 36, false, 4, 'float', 'm'),
    new MavLinkPacketField('temperature', 'temperature', 40, false, 2, 'int16_t', 'cdegC'),
    new MavLinkPacketField('sensor_id', 'sensorId', 42, false, 1, 'uint8_t', ''),
    new MavLinkPacketField('quality', 'quality', 43, false, 1, 'uint8_t', ''),
  ]

  constructor() {
    super()
    this.timeUsec = BigInt(0)
    this.sensorId = 0
    this.integrationTimeUs = 0
    this.integratedX = 0
    this.integratedY = 0
    this.integratedXgyro = 0
    this.integratedYgyro = 0
    this.integratedZgyro = 0
    this.temperature = 0
    this.quality = 0
    this.timeDeltaDistanceUs = 0
    this.distance = 0
  }

  /**
   * Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format
   * (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
   * Units: us
   */
  timeUsec: uint64_t

  /**
   * Sensor ID
   */
  sensorId: uint8_t

  /**
   * Integration time. Divide integrated_x and integrated_y by the integration time to obtain average
   * flow. The integration time also indicates the.
   * Units: us
   */
  integrationTimeUs: uint32_t

  /**
   * Flow around X axis (Sensor RH rotation about the X axis induces a positive flow. Sensor linear
   * motion along the positive Y axis induces a negative flow.)
   * Units: rad
   */
  integratedX: float

  /**
   * Flow around Y axis (Sensor RH rotation about the Y axis induces a positive flow. Sensor linear
   * motion along the positive X axis induces a positive flow.)
   * Units: rad
   */
  integratedY: float

  /**
   * RH rotation around X axis
   * Units: rad
   */
  integratedXgyro: float

  /**
   * RH rotation around Y axis
   * Units: rad
   */
  integratedYgyro: float

  /**
   * RH rotation around Z axis
   * Units: rad
   */
  integratedZgyro: float

  /**
   * Temperature
   * Units: cdegC
   */
  temperature: int16_t

  /**
   * Optical flow quality / confidence. 0: no valid flow, 255: maximum quality
   */
  quality: uint8_t

  /**
   * Time since the distance was sampled.
   * Units: us
   */
  timeDeltaDistanceUs: uint32_t

  /**
   * Distance to the center of the flow field. Positive value (including zero): distance known. Negative
   * value: Unknown distance.
   * Units: m
   */
  distance: float
}

/**
 * The IMU readings in SI units in NED body frame
 */
export class HilSensor extends MavLinkData {
  static MSG_ID = 107
  static MSG_NAME = 'HIL_SENSOR'
  static PAYLOAD_LENGTH = 65
  static MAGIC_NUMBER = 108

  static FIELDS = [
    new MavLinkPacketField('time_usec', 'timeUsec', 0, false, 8, 'uint64_t', 'us'),
    new MavLinkPacketField('xacc', 'xacc', 8, false, 4, 'float', 'm/s/s'),
    new MavLinkPacketField('yacc', 'yacc', 12, false, 4, 'float', 'm/s/s'),
    new MavLinkPacketField('zacc', 'zacc', 16, false, 4, 'float', 'm/s/s'),
    new MavLinkPacketField('xgyro', 'xgyro', 20, false, 4, 'float', 'rad/s'),
    new MavLinkPacketField('ygyro', 'ygyro', 24, false, 4, 'float', 'rad/s'),
    new MavLinkPacketField('zgyro', 'zgyro', 28, false, 4, 'float', 'rad/s'),
    new MavLinkPacketField('xmag', 'xmag', 32, false, 4, 'float', 'gauss'),
    new MavLinkPacketField('ymag', 'ymag', 36, false, 4, 'float', 'gauss'),
    new MavLinkPacketField('zmag', 'zmag', 40, false, 4, 'float', 'gauss'),
    new MavLinkPacketField('abs_pressure', 'absPressure', 44, false, 4, 'float', 'hPa'),
    new MavLinkPacketField('diff_pressure', 'diffPressure', 48, false, 4, 'float', 'hPa'),
    new MavLinkPacketField('pressure_alt', 'pressureAlt', 52, false, 4, 'float', ''),
    new MavLinkPacketField('temperature', 'temperature', 56, false, 4, 'float', 'degC'),
    new MavLinkPacketField('fields_updated', 'fieldsUpdated', 60, false, 4, 'uint32_t', ''),
    new MavLinkPacketField('id', 'id', 64, true, 1, 'uint8_t', ''),
  ]

  constructor() {
    super()
    this.timeUsec = BigInt(0)
    this.xacc = 0
    this.yacc = 0
    this.zacc = 0
    this.xgyro = 0
    this.ygyro = 0
    this.zgyro = 0
    this.xmag = 0
    this.ymag = 0
    this.zmag = 0
    this.absPressure = 0
    this.diffPressure = 0
    this.pressureAlt = 0
    this.temperature = 0
    this.fieldsUpdated = HilSensorUpdatedFlags[Object.keys(HilSensorUpdatedFlags)[0]]
    this.id = 0
  }

  /**
   * Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format
   * (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
   * Units: us
   */
  timeUsec: uint64_t

  /**
   * X acceleration
   * Units: m/s/s
   */
  xacc: float

  /**
   * Y acceleration
   * Units: m/s/s
   */
  yacc: float

  /**
   * Z acceleration
   * Units: m/s/s
   */
  zacc: float

  /**
   * Angular speed around X axis in body frame
   * Units: rad/s
   */
  xgyro: float

  /**
   * Angular speed around Y axis in body frame
   * Units: rad/s
   */
  ygyro: float

  /**
   * Angular speed around Z axis in body frame
   * Units: rad/s
   */
  zgyro: float

  /**
   * X Magnetic field
   * Units: gauss
   */
  xmag: float

  /**
   * Y Magnetic field
   * Units: gauss
   */
  ymag: float

  /**
   * Z Magnetic field
   * Units: gauss
   */
  zmag: float

  /**
   * Absolute pressure
   * Units: hPa
   */
  absPressure: float

  /**
   * Differential pressure (airspeed)
   * Units: hPa
   */
  diffPressure: float

  /**
   * Altitude calculated from pressure
   */
  pressureAlt: float

  /**
   * Temperature
   * Units: degC
   */
  temperature: float

  /**
   * Bitmap for fields that have updated since last message
   */
  fieldsUpdated: HilSensorUpdatedFlags

  /**
   * Sensor ID (zero indexed). Used for multiple sensor inputs
   */
  id: uint8_t
}

/**
 * Status of simulation environment, if used
 */
export class SimState extends MavLinkData {
  static MSG_ID = 108
  static MSG_NAME = 'SIM_STATE'
  static PAYLOAD_LENGTH = 84
  static MAGIC_NUMBER = 32

  static FIELDS = [
    new MavLinkPacketField('q1', 'q1', 0, false, 4, 'float', ''),
    new MavLinkPacketField('q2', 'q2', 4, false, 4, 'float', ''),
    new MavLinkPacketField('q3', 'q3', 8, false, 4, 'float', ''),
    new MavLinkPacketField('q4', 'q4', 12, false, 4, 'float', ''),
    new MavLinkPacketField('roll', 'roll', 16, false, 4, 'float', ''),
    new MavLinkPacketField('pitch', 'pitch', 20, false, 4, 'float', ''),
    new MavLinkPacketField('yaw', 'yaw', 24, false, 4, 'float', ''),
    new MavLinkPacketField('xacc', 'xacc', 28, false, 4, 'float', 'm/s/s'),
    new MavLinkPacketField('yacc', 'yacc', 32, false, 4, 'float', 'm/s/s'),
    new MavLinkPacketField('zacc', 'zacc', 36, false, 4, 'float', 'm/s/s'),
    new MavLinkPacketField('xgyro', 'xgyro', 40, false, 4, 'float', 'rad/s'),
    new MavLinkPacketField('ygyro', 'ygyro', 44, false, 4, 'float', 'rad/s'),
    new MavLinkPacketField('zgyro', 'zgyro', 48, false, 4, 'float', 'rad/s'),
    new MavLinkPacketField('lat', 'lat', 52, false, 4, 'float', 'deg'),
    new MavLinkPacketField('lon', 'lon', 56, false, 4, 'float', 'deg'),
    new MavLinkPacketField('alt', 'alt', 60, false, 4, 'float', 'm'),
    new MavLinkPacketField('std_dev_horz', 'stdDevHorz', 64, false, 4, 'float', ''),
    new MavLinkPacketField('std_dev_vert', 'stdDevVert', 68, false, 4, 'float', ''),
    new MavLinkPacketField('vn', 'vn', 72, false, 4, 'float', 'm/s'),
    new MavLinkPacketField('ve', 've', 76, false, 4, 'float', 'm/s'),
    new MavLinkPacketField('vd', 'vd', 80, false, 4, 'float', 'm/s'),
  ]

  constructor() {
    super()
    this.q1 = 0
    this.q2 = 0
    this.q3 = 0
    this.q4 = 0
    this.roll = 0
    this.pitch = 0
    this.yaw = 0
    this.xacc = 0
    this.yacc = 0
    this.zacc = 0
    this.xgyro = 0
    this.ygyro = 0
    this.zgyro = 0
    this.lat = 0
    this.lon = 0
    this.alt = 0
    this.stdDevHorz = 0
    this.stdDevVert = 0
    this.vn = 0
    this.ve = 0
    this.vd = 0
  }

  /**
   * True attitude quaternion component 1, w (1 in null-rotation)
   */
  q1: float

  /**
   * True attitude quaternion component 2, x (0 in null-rotation)
   */
  q2: float

  /**
   * True attitude quaternion component 3, y (0 in null-rotation)
   */
  q3: float

  /**
   * True attitude quaternion component 4, z (0 in null-rotation)
   */
  q4: float

  /**
   * Attitude roll expressed as Euler angles, not recommended except for human-readable outputs
   */
  roll: float

  /**
   * Attitude pitch expressed as Euler angles, not recommended except for human-readable outputs
   */
  pitch: float

  /**
   * Attitude yaw expressed as Euler angles, not recommended except for human-readable outputs
   */
  yaw: float

  /**
   * X acceleration
   * Units: m/s/s
   */
  xacc: float

  /**
   * Y acceleration
   * Units: m/s/s
   */
  yacc: float

  /**
   * Z acceleration
   * Units: m/s/s
   */
  zacc: float

  /**
   * Angular speed around X axis
   * Units: rad/s
   */
  xgyro: float

  /**
   * Angular speed around Y axis
   * Units: rad/s
   */
  ygyro: float

  /**
   * Angular speed around Z axis
   * Units: rad/s
   */
  zgyro: float

  /**
   * Latitude
   * Units: deg
   */
  lat: float

  /**
   * Longitude
   * Units: deg
   */
  lon: float

  /**
   * Altitude
   * Units: m
   */
  alt: float

  /**
   * Horizontal position standard deviation
   */
  stdDevHorz: float

  /**
   * Vertical position standard deviation
   */
  stdDevVert: float

  /**
   * True velocity in north direction in earth-fixed NED frame
   * Units: m/s
   */
  vn: float

  /**
   * True velocity in east direction in earth-fixed NED frame
   * Units: m/s
   */
  ve: float

  /**
   * True velocity in down direction in earth-fixed NED frame
   * Units: m/s
   */
  vd: float
}

/**
 * Status generated by radio and injected into MAVLink stream.
 */
export class RadioStatus extends MavLinkData {
  static MSG_ID = 109
  static MSG_NAME = 'RADIO_STATUS'
  static PAYLOAD_LENGTH = 9
  static MAGIC_NUMBER = 185

  static FIELDS = [
    new MavLinkPacketField('rxerrors', 'rxerrors', 0, false, 2, 'uint16_t', ''),
    new MavLinkPacketField('fixed', 'fixed', 2, false, 2, 'uint16_t', ''),
    new MavLinkPacketField('rssi', 'rssi', 4, false, 1, 'uint8_t', ''),
    new MavLinkPacketField('remrssi', 'remrssi', 5, false, 1, 'uint8_t', ''),
    new MavLinkPacketField('txbuf', 'txbuf', 6, false, 1, 'uint8_t', '%'),
    new MavLinkPacketField('noise', 'noise', 7, false, 1, 'uint8_t', ''),
    new MavLinkPacketField('remnoise', 'remnoise', 8, false, 1, 'uint8_t', ''),
  ]

  constructor() {
    super()
    this.rssi = 0
    this.remrssi = 0
    this.txbuf = 0
    this.noise = 0
    this.remnoise = 0
    this.rxerrors = 0
    this.fixed = 0
  }

  /**
   * Local (message sender) recieved signal strength indication in device-dependent units/scale. Values:
   * [0-254], UINT8_MAX: invalid/unknown.
   */
  rssi: uint8_t

  /**
   * Remote (message receiver) signal strength indication in device-dependent units/scale. Values:
   * [0-254], UINT8_MAX: invalid/unknown.
   */
  remrssi: uint8_t

  /**
   * Remaining free transmitter buffer space.
   * Units: %
   */
  txbuf: uint8_t

  /**
   * Local background noise level. These are device dependent RSSI values (scale as approx 2x dB on SiK
   * radios). Values: [0-254], UINT8_MAX: invalid/unknown.
   */
  noise: uint8_t

  /**
   * Remote background noise level. These are device dependent RSSI values (scale as approx 2x dB on SiK
   * radios). Values: [0-254], UINT8_MAX: invalid/unknown.
   */
  remnoise: uint8_t

  /**
   * Count of radio packet receive errors (since boot).
   */
  rxerrors: uint16_t

  /**
   * Count of error corrected radio packets (since boot).
   */
  fixed: uint16_t
}

/**
 * File transfer message
 */
export class FileTransferProtocol extends MavLinkData {
  static MSG_ID = 110
  static MSG_NAME = 'FILE_TRANSFER_PROTOCOL'
  static PAYLOAD_LENGTH = 254
  static MAGIC_NUMBER = 84

  static FIELDS = [
    new MavLinkPacketField('target_network', 'targetNetwork', 0, false, 1, 'uint8_t', ''),
    new MavLinkPacketField('target_system', 'targetSystem', 1, false, 1, 'uint8_t', ''),
    new MavLinkPacketField('target_component', 'targetComponent', 2, false, 1, 'uint8_t', ''),
    new MavLinkPacketField('payload', 'payload', 3, false, 1, 'uint8_t[]', '', 251),
  ]

  constructor() {
    super()
    this.targetNetwork = 0
    this.targetSystem = 0
    this.targetComponent = 0
    this.payload = []
  }

  /**
   * Network ID (0 for broadcast)
   */
  targetNetwork: uint8_t

  /**
   * System ID (0 for broadcast)
   */
  targetSystem: uint8_t

  /**
   * Component ID (0 for broadcast)
   */
  targetComponent: uint8_t

  /**
   * Variable length payload. The length is defined by the remaining message length when subtracting the
   * header and other fields. The entire content of this block is opaque unless you understand any the
   * encoding message_type. The particular encoding used can be extension specific and might not always
   * be documented as part of the mavlink specification.
   */
  payload: uint8_t[]
}

/**
 * Time synchronization message.
 */
export class TimeSync extends MavLinkData {
  static MSG_ID = 111
  static MSG_NAME = 'TIMESYNC'
  static PAYLOAD_LENGTH = 16
  static MAGIC_NUMBER = 34

  static FIELDS = [
    new MavLinkPacketField('tc1', 'tc1', 0, false, 8, 'int64_t', ''),
    new MavLinkPacketField('ts1', 'ts1', 8, false, 8, 'int64_t', ''),
  ]

  constructor() {
    super()
    this.tc1 = BigInt(0)
    this.ts1 = BigInt(0)
  }

  /**
   * Time sync timestamp 1
   */
  tc1: int64_t

  /**
   * Time sync timestamp 2
   */
  ts1: int64_t
}

/**
 * Camera-IMU triggering and synchronisation message.
 */
export class CameraTrigger extends MavLinkData {
  static MSG_ID = 112
  static MSG_NAME = 'CAMERA_TRIGGER'
  static PAYLOAD_LENGTH = 12
  static MAGIC_NUMBER = 174

  static FIELDS = [
    new MavLinkPacketField('time_usec', 'timeUsec', 0, false, 8, 'uint64_t', 'us'),
    new MavLinkPacketField('seq', 'seq', 8, false, 4, 'uint32_t', ''),
  ]

  constructor() {
    super()
    this.timeUsec = BigInt(0)
    this.seq = 0
  }

  /**
   * Timestamp for image frame (UNIX Epoch time or time since system boot). The receiving end can infer
   * timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
   * Units: us
   */
  timeUsec: uint64_t

  /**
   * Image frame sequence
   */
  seq: uint32_t
}

/**
 * The global position, as returned by the Global Positioning System (GPS). This is
 NOT the global
 * position estimate of the sytem, but rather a RAW sensor value. See message GLOBAL_POSITION_INT for
 * the global position estimate.
 */
export class HilGps extends MavLinkData {
  static MSG_ID = 113
  static MSG_NAME = 'HIL_GPS'
  static PAYLOAD_LENGTH = 39
  static MAGIC_NUMBER = 124

  static FIELDS = [
    new MavLinkPacketField('time_usec', 'timeUsec', 0, false, 8, 'uint64_t', 'us'),
    new MavLinkPacketField('lat', 'lat', 8, false, 4, 'int32_t', 'degE7'),
    new MavLinkPacketField('lon', 'lon', 12, false, 4, 'int32_t', 'degE7'),
    new MavLinkPacketField('alt', 'alt', 16, false, 4, 'int32_t', 'mm'),
    new MavLinkPacketField('eph', 'eph', 20, false, 2, 'uint16_t', ''),
    new MavLinkPacketField('epv', 'epv', 22, false, 2, 'uint16_t', ''),
    new MavLinkPacketField('vel', 'vel', 24, false, 2, 'uint16_t', 'cm/s'),
    new MavLinkPacketField('vn', 'vn', 26, false, 2, 'int16_t', 'cm/s'),
    new MavLinkPacketField('ve', 've', 28, false, 2, 'int16_t', 'cm/s'),
    new MavLinkPacketField('vd', 'vd', 30, false, 2, 'int16_t', 'cm/s'),
    new MavLinkPacketField('cog', 'cog', 32, false, 2, 'uint16_t', 'cdeg'),
    new MavLinkPacketField('fix_type', 'fixType', 34, false, 1, 'uint8_t', ''),
    new MavLinkPacketField('satellites_visible', 'satellitesVisible', 35, false, 1, 'uint8_t', ''),
    new MavLinkPacketField('id', 'id', 36, true, 1, 'uint8_t', ''),
    new MavLinkPacketField('yaw', 'yaw', 37, true, 2, 'uint16_t', 'cdeg'),
  ]

  constructor() {
    super()
    this.timeUsec = BigInt(0)
    this.fixType = 0
    this.lat = 0
    this.lon = 0
    this.alt = 0
    this.eph = 0
    this.epv = 0
    this.vel = 0
    this.vn = 0
    this.ve = 0
    this.vd = 0
    this.cog = 0
    this.satellitesVisible = 0
    this.id = 0
    this.yaw = 0
  }

  /**
   * Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format
   * (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
   * Units: us
   */
  timeUsec: uint64_t

  /**
   * 0-1: no fix, 2: 2D fix, 3: 3D fix. Some applications will not use the value of this field unless it
   * is at least two, so always correctly fill in the fix.
   */
  fixType: uint8_t

  /**
   * Latitude (WGS84)
   * Units: degE7
   */
  lat: int32_t

  /**
   * Longitude (WGS84)
   * Units: degE7
   */
  lon: int32_t

  /**
   * Altitude (MSL). Positive for up.
   * Units: mm
   */
  alt: int32_t

  /**
   * GPS HDOP horizontal dilution of position (unitless * 100). If unknown, set to: UINT16_MAX
   */
  eph: uint16_t

  /**
   * GPS VDOP vertical dilution of position (unitless * 100). If unknown, set to: UINT16_MAX
   */
  epv: uint16_t

  /**
   * GPS ground speed. If unknown, set to: UINT16_MAX
   * Units: cm/s
   */
  vel: uint16_t

  /**
   * GPS velocity in north direction in earth-fixed NED frame
   * Units: cm/s
   */
  vn: int16_t

  /**
   * GPS velocity in east direction in earth-fixed NED frame
   * Units: cm/s
   */
  ve: int16_t

  /**
   * GPS velocity in down direction in earth-fixed NED frame
   * Units: cm/s
   */
  vd: int16_t

  /**
   * Course over ground (NOT heading, but direction of movement), 0.0..359.99 degrees. If unknown, set
   * to: UINT16_MAX
   * Units: cdeg
   */
  cog: uint16_t

  /**
   * Number of satellites visible. If unknown, set to UINT8_MAX
   */
  satellitesVisible: uint8_t

  /**
   * GPS ID (zero indexed). Used for multiple GPS inputs
   */
  id: uint8_t

  /**
   * Yaw of vehicle relative to Earth's North, zero means not available, use 36000 for north
   * Units: cdeg
   */
  yaw: uint16_t
}

/**
 * Simulated optical flow from a flow sensor (e.g. PX4FLOW or optical mouse sensor)
 */
export class HilOpticalFlow extends MavLinkData {
  static MSG_ID = 114
  static MSG_NAME = 'HIL_OPTICAL_FLOW'
  static PAYLOAD_LENGTH = 44
  static MAGIC_NUMBER = 237

  static FIELDS = [
    new MavLinkPacketField('time_usec', 'timeUsec', 0, false, 8, 'uint64_t', 'us'),
    new MavLinkPacketField('integration_time_us', 'integrationTimeUs', 8, false, 4, 'uint32_t', 'us'),
    new MavLinkPacketField('integrated_x', 'integratedX', 12, false, 4, 'float', 'rad'),
    new MavLinkPacketField('integrated_y', 'integratedY', 16, false, 4, 'float', 'rad'),
    new MavLinkPacketField('integrated_xgyro', 'integratedXgyro', 20, false, 4, 'float', 'rad'),
    new MavLinkPacketField('integrated_ygyro', 'integratedYgyro', 24, false, 4, 'float', 'rad'),
    new MavLinkPacketField('integrated_zgyro', 'integratedZgyro', 28, false, 4, 'float', 'rad'),
    new MavLinkPacketField('time_delta_distance_us', 'timeDeltaDistanceUs', 32, false, 4, 'uint32_t', 'us'),
    new MavLinkPacketField('distance', 'distance', 36, false, 4, 'float', 'm'),
    new MavLinkPacketField('temperature', 'temperature', 40, false, 2, 'int16_t', 'cdegC'),
    new MavLinkPacketField('sensor_id', 'sensorId', 42, false, 1, 'uint8_t', ''),
    new MavLinkPacketField('quality', 'quality', 43, false, 1, 'uint8_t', ''),
  ]

  constructor() {
    super()
    this.timeUsec = BigInt(0)
    this.sensorId = 0
    this.integrationTimeUs = 0
    this.integratedX = 0
    this.integratedY = 0
    this.integratedXgyro = 0
    this.integratedYgyro = 0
    this.integratedZgyro = 0
    this.temperature = 0
    this.quality = 0
    this.timeDeltaDistanceUs = 0
    this.distance = 0
  }

  /**
   * Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format
   * (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
   * Units: us
   */
  timeUsec: uint64_t

  /**
   * Sensor ID
   */
  sensorId: uint8_t

  /**
   * Integration time. Divide integrated_x and integrated_y by the integration time to obtain average
   * flow. The integration time also indicates the.
   * Units: us
   */
  integrationTimeUs: uint32_t

  /**
   * Flow in radians around X axis (Sensor RH rotation about the X axis induces a positive flow. Sensor
   * linear motion along the positive Y axis induces a negative flow.)
   * Units: rad
   */
  integratedX: float

  /**
   * Flow in radians around Y axis (Sensor RH rotation about the Y axis induces a positive flow. Sensor
   * linear motion along the positive X axis induces a positive flow.)
   * Units: rad
   */
  integratedY: float

  /**
   * RH rotation around X axis
   * Units: rad
   */
  integratedXgyro: float

  /**
   * RH rotation around Y axis
   * Units: rad
   */
  integratedYgyro: float

  /**
   * RH rotation around Z axis
   * Units: rad
   */
  integratedZgyro: float

  /**
   * Temperature
   * Units: cdegC
   */
  temperature: int16_t

  /**
   * Optical flow quality / confidence. 0: no valid flow, 255: maximum quality
   */
  quality: uint8_t

  /**
   * Time since the distance was sampled.
   * Units: us
   */
  timeDeltaDistanceUs: uint32_t

  /**
   * Distance to the center of the flow field. Positive value (including zero): distance known. Negative
   * value: Unknown distance.
   * Units: m
   */
  distance: float
}

/**
 * Sent from simulation to autopilot, avoids in contrast to HIL_STATE singularities. This packet is
 * useful for high throughput applications such as hardware in the loop simulations.
 */
export class HilStateQuaternion extends MavLinkData {
  static MSG_ID = 115
  static MSG_NAME = 'HIL_STATE_QUATERNION'
  static PAYLOAD_LENGTH = 64
  static MAGIC_NUMBER = 4

  static FIELDS = [
    new MavLinkPacketField('time_usec', 'timeUsec', 0, false, 8, 'uint64_t', 'us'),
    new MavLinkPacketField('attitude_quaternion', 'attitudeQuaternion', 8, false, 4, 'float[]', '', 4),
    new MavLinkPacketField('rollspeed', 'rollspeed', 24, false, 4, 'float', 'rad/s'),
    new MavLinkPacketField('pitchspeed', 'pitchspeed', 28, false, 4, 'float', 'rad/s'),
    new MavLinkPacketField('yawspeed', 'yawspeed', 32, false, 4, 'float', 'rad/s'),
    new MavLinkPacketField('lat', 'lat', 36, false, 4, 'int32_t', 'degE7'),
    new MavLinkPacketField('lon', 'lon', 40, false, 4, 'int32_t', 'degE7'),
    new MavLinkPacketField('alt', 'alt', 44, false, 4, 'int32_t', 'mm'),
    new MavLinkPacketField('vx', 'vx', 48, false, 2, 'int16_t', 'cm/s'),
    new MavLinkPacketField('vy', 'vy', 50, false, 2, 'int16_t', 'cm/s'),
    new MavLinkPacketField('vz', 'vz', 52, false, 2, 'int16_t', 'cm/s'),
    new MavLinkPacketField('ind_airspeed', 'indAirspeed', 54, false, 2, 'uint16_t', 'cm/s'),
    new MavLinkPacketField('true_airspeed', 'trueAirspeed', 56, false, 2, 'uint16_t', 'cm/s'),
    new MavLinkPacketField('xacc', 'xacc', 58, false, 2, 'int16_t', 'mG'),
    new MavLinkPacketField('yacc', 'yacc', 60, false, 2, 'int16_t', 'mG'),
    new MavLinkPacketField('zacc', 'zacc', 62, false, 2, 'int16_t', 'mG'),
  ]

  constructor() {
    super()
    this.timeUsec = BigInt(0)
    this.attitudeQuaternion = []
    this.rollspeed = 0
    this.pitchspeed = 0
    this.yawspeed = 0
    this.lat = 0
    this.lon = 0
    this.alt = 0
    this.vx = 0
    this.vy = 0
    this.vz = 0
    this.indAirspeed = 0
    this.trueAirspeed = 0
    this.xacc = 0
    this.yacc = 0
    this.zacc = 0
  }

  /**
   * Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format
   * (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
   * Units: us
   */
  timeUsec: uint64_t

  /**
   * Vehicle attitude expressed as normalized quaternion in w, x, y, z order (with 1 0 0 0 being the
   * null-rotation)
   */
  attitudeQuaternion: float[]

  /**
   * Body frame roll / phi angular speed
   * Units: rad/s
   */
  rollspeed: float

  /**
   * Body frame pitch / theta angular speed
   * Units: rad/s
   */
  pitchspeed: float

  /**
   * Body frame yaw / psi angular speed
   * Units: rad/s
   */
  yawspeed: float

  /**
   * Latitude
   * Units: degE7
   */
  lat: int32_t

  /**
   * Longitude
   * Units: degE7
   */
  lon: int32_t

  /**
   * Altitude
   * Units: mm
   */
  alt: int32_t

  /**
   * Ground X Speed (Latitude)
   * Units: cm/s
   */
  vx: int16_t

  /**
   * Ground Y Speed (Longitude)
   * Units: cm/s
   */
  vy: int16_t

  /**
   * Ground Z Speed (Altitude)
   * Units: cm/s
   */
  vz: int16_t

  /**
   * Indicated airspeed
   * Units: cm/s
   */
  indAirspeed: uint16_t

  /**
   * True airspeed
   * Units: cm/s
   */
  trueAirspeed: uint16_t

  /**
   * X acceleration
   * Units: mG
   */
  xacc: int16_t

  /**
   * Y acceleration
   * Units: mG
   */
  yacc: int16_t

  /**
   * Z acceleration
   * Units: mG
   */
  zacc: int16_t
}

/**
 * The RAW IMU readings for secondary 9DOF sensor setup. This message should contain the scaled values
 * to the described units
 */
export class ScaledImu2 extends MavLinkData {
  static MSG_ID = 116
  static MSG_NAME = 'SCALED_IMU2'
  static PAYLOAD_LENGTH = 24
  static MAGIC_NUMBER = 76

  static FIELDS = [
    new MavLinkPacketField('time_boot_ms', 'timeBootMs', 0, false, 4, 'uint32_t', 'ms'),
    new MavLinkPacketField('xacc', 'xacc', 4, false, 2, 'int16_t', 'mG'),
    new MavLinkPacketField('yacc', 'yacc', 6, false, 2, 'int16_t', 'mG'),
    new MavLinkPacketField('zacc', 'zacc', 8, false, 2, 'int16_t', 'mG'),
    new MavLinkPacketField('xgyro', 'xgyro', 10, false, 2, 'int16_t', 'mrad/s'),
    new MavLinkPacketField('ygyro', 'ygyro', 12, false, 2, 'int16_t', 'mrad/s'),
    new MavLinkPacketField('zgyro', 'zgyro', 14, false, 2, 'int16_t', 'mrad/s'),
    new MavLinkPacketField('xmag', 'xmag', 16, false, 2, 'int16_t', 'mgauss'),
    new MavLinkPacketField('ymag', 'ymag', 18, false, 2, 'int16_t', 'mgauss'),
    new MavLinkPacketField('zmag', 'zmag', 20, false, 2, 'int16_t', 'mgauss'),
    new MavLinkPacketField('temperature', 'temperature', 22, true, 2, 'int16_t', 'cdegC'),
  ]

  constructor() {
    super()
    this.timeBootMs = 0
    this.xacc = 0
    this.yacc = 0
    this.zacc = 0
    this.xgyro = 0
    this.ygyro = 0
    this.zgyro = 0
    this.xmag = 0
    this.ymag = 0
    this.zmag = 0
    this.temperature = 0
  }

  /**
   * Timestamp (time since system boot).
   * Units: ms
   */
  timeBootMs: uint32_t

  /**
   * X acceleration
   * Units: mG
   */
  xacc: int16_t

  /**
   * Y acceleration
   * Units: mG
   */
  yacc: int16_t

  /**
   * Z acceleration
   * Units: mG
   */
  zacc: int16_t

  /**
   * Angular speed around X axis
   * Units: mrad/s
   */
  xgyro: int16_t

  /**
   * Angular speed around Y axis
   * Units: mrad/s
   */
  ygyro: int16_t

  /**
   * Angular speed around Z axis
   * Units: mrad/s
   */
  zgyro: int16_t

  /**
   * X Magnetic field
   * Units: mgauss
   */
  xmag: int16_t

  /**
   * Y Magnetic field
   * Units: mgauss
   */
  ymag: int16_t

  /**
   * Z Magnetic field
   * Units: mgauss
   */
  zmag: int16_t

  /**
   * Temperature, 0: IMU does not provide temperature values. If the IMU is at 0C it must send 1 (0.01C).
   * Units: cdegC
   */
  temperature: int16_t
}

/**
 * Request a list of available logs. On some systems calling this may stop on-board logging until
 * LOG_REQUEST_END is called. If there are no log files available this request shall be answered with
 * one LOG_ENTRY message with id = 0 and num_logs = 0.
 */
export class LogRequestList extends MavLinkData {
  static MSG_ID = 117
  static MSG_NAME = 'LOG_REQUEST_LIST'
  static PAYLOAD_LENGTH = 6
  static MAGIC_NUMBER = 128

  static FIELDS = [
    new MavLinkPacketField('start', 'start', 0, false, 2, 'uint16_t', ''),
    new MavLinkPacketField('end', 'end', 2, false, 2, 'uint16_t', ''),
    new MavLinkPacketField('target_system', 'targetSystem', 4, false, 1, 'uint8_t', ''),
    new MavLinkPacketField('target_component', 'targetComponent', 5, false, 1, 'uint8_t', ''),
  ]

  constructor() {
    super()
    this.targetSystem = 0
    this.targetComponent = 0
    this.start = 0
    this.end = 0
  }

  /**
   * System ID
   */
  targetSystem: uint8_t

  /**
   * Component ID
   */
  targetComponent: uint8_t

  /**
   * First log id (0 for first available)
   */
  start: uint16_t

  /**
   * Last log id (0xffff for last available)
   */
  end: uint16_t
}

/**
 * Reply to LOG_REQUEST_LIST
 */
export class LogEntry extends MavLinkData {
  static MSG_ID = 118
  static MSG_NAME = 'LOG_ENTRY'
  static PAYLOAD_LENGTH = 14
  static MAGIC_NUMBER = 56

  static FIELDS = [
    new MavLinkPacketField('time_utc', 'timeUtc', 0, false, 4, 'uint32_t', 's'),
    new MavLinkPacketField('size', 'size', 4, false, 4, 'uint32_t', 'bytes'),
    new MavLinkPacketField('id', 'id', 8, false, 2, 'uint16_t', ''),
    new MavLinkPacketField('num_logs', 'numLogs', 10, false, 2, 'uint16_t', ''),
    new MavLinkPacketField('last_log_num', 'lastLogNum', 12, false, 2, 'uint16_t', ''),
  ]

  constructor() {
    super()
    this.id = 0
    this.numLogs = 0
    this.lastLogNum = 0
    this.timeUtc = 0
    this.size = 0
  }

  /**
   * Log id
   */
  id: uint16_t

  /**
   * Total number of logs
   */
  numLogs: uint16_t

  /**
   * High log number
   */
  lastLogNum: uint16_t

  /**
   * UTC timestamp of log since 1970, or 0 if not available
   * Units: s
   */
  timeUtc: uint32_t

  /**
   * Size of the log (may be approximate)
   * Units: bytes
   */
  size: uint32_t
}

/**
 * Request a chunk of a log
 */
export class LogRequestData extends MavLinkData {
  static MSG_ID = 119
  static MSG_NAME = 'LOG_REQUEST_DATA'
  static PAYLOAD_LENGTH = 12
  static MAGIC_NUMBER = 116

  static FIELDS = [
    new MavLinkPacketField('ofs', 'ofs', 0, false, 4, 'uint32_t', ''),
    new MavLinkPacketField('count', 'count', 4, false, 4, 'uint32_t', 'bytes'),
    new MavLinkPacketField('id', 'id', 8, false, 2, 'uint16_t', ''),
    new MavLinkPacketField('target_system', 'targetSystem', 10, false, 1, 'uint8_t', ''),
    new MavLinkPacketField('target_component', 'targetComponent', 11, false, 1, 'uint8_t', ''),
  ]

  constructor() {
    super()
    this.targetSystem = 0
    this.targetComponent = 0
    this.id = 0
    this.ofs = 0
    this.count = 0
  }

  /**
   * System ID
   */
  targetSystem: uint8_t

  /**
   * Component ID
   */
  targetComponent: uint8_t

  /**
   * Log id (from LOG_ENTRY reply)
   */
  id: uint16_t

  /**
   * Offset into the log
   */
  ofs: uint32_t

  /**
   * Number of bytes
   * Units: bytes
   */
  count: uint32_t
}

/**
 * Reply to LOG_REQUEST_DATA
 */
export class LogData extends MavLinkData {
  static MSG_ID = 120
  static MSG_NAME = 'LOG_DATA'
  static PAYLOAD_LENGTH = 97
  static MAGIC_NUMBER = 134

  static FIELDS = [
    new MavLinkPacketField('ofs', 'ofs', 0, false, 4, 'uint32_t', ''),
    new MavLinkPacketField('id', 'id', 4, false, 2, 'uint16_t', ''),
    new MavLinkPacketField('count', 'count', 6, false, 1, 'uint8_t', 'bytes'),
    new MavLinkPacketField('data', 'data', 7, false, 1, 'uint8_t[]', '', 90),
  ]

  constructor() {
    super()
    this.id = 0
    this.ofs = 0
    this.count = 0
    this.data = []
  }

  /**
   * Log id (from LOG_ENTRY reply)
   */
  id: uint16_t

  /**
   * Offset into the log
   */
  ofs: uint32_t

  /**
   * Number of bytes (zero for end of log)
   * Units: bytes
   */
  count: uint8_t

  /**
   * log data
   */
  data: uint8_t[]
}

/**
 * Erase all logs
 */
export class LogErase extends MavLinkData {
  static MSG_ID = 121
  static MSG_NAME = 'LOG_ERASE'
  static PAYLOAD_LENGTH = 2
  static MAGIC_NUMBER = 237

  static FIELDS = [
    new MavLinkPacketField('target_system', 'targetSystem', 0, false, 1, 'uint8_t', ''),
    new MavLinkPacketField('target_component', 'targetComponent', 1, false, 1, 'uint8_t', ''),
  ]

  constructor() {
    super()
    this.targetSystem = 0
    this.targetComponent = 0
  }

  /**
   * System ID
   */
  targetSystem: uint8_t

  /**
   * Component ID
   */
  targetComponent: uint8_t
}

/**
 * Stop log transfer and resume normal logging
 */
export class LogRequestEnd extends MavLinkData {
  static MSG_ID = 122
  static MSG_NAME = 'LOG_REQUEST_END'
  static PAYLOAD_LENGTH = 2
  static MAGIC_NUMBER = 203

  static FIELDS = [
    new MavLinkPacketField('target_system', 'targetSystem', 0, false, 1, 'uint8_t', ''),
    new MavLinkPacketField('target_component', 'targetComponent', 1, false, 1, 'uint8_t', ''),
  ]

  constructor() {
    super()
    this.targetSystem = 0
    this.targetComponent = 0
  }

  /**
   * System ID
   */
  targetSystem: uint8_t

  /**
   * Component ID
   */
  targetComponent: uint8_t
}

/**
 * Data for injecting into the onboard GPS (used for DGPS)
 */
export class GpsInjectData extends MavLinkData {
  static MSG_ID = 123
  static MSG_NAME = 'GPS_INJECT_DATA'
  static PAYLOAD_LENGTH = 113
  static MAGIC_NUMBER = 250

  static FIELDS = [
    new MavLinkPacketField('target_system', 'targetSystem', 0, false, 1, 'uint8_t', ''),
    new MavLinkPacketField('target_component', 'targetComponent', 1, false, 1, 'uint8_t', ''),
    new MavLinkPacketField('len', 'len', 2, false, 1, 'uint8_t', 'bytes'),
    new MavLinkPacketField('data', 'data', 3, false, 1, 'uint8_t[]', '', 110),
  ]

  constructor() {
    super()
    this.targetSystem = 0
    this.targetComponent = 0
    this.len = 0
    this.data = []
  }

  /**
   * System ID
   */
  targetSystem: uint8_t

  /**
   * Component ID
   */
  targetComponent: uint8_t

  /**
   * Data length
   * Units: bytes
   */
  len: uint8_t

  /**
   * Raw data (110 is enough for 12 satellites of RTCMv2)
   */
  data: uint8_t[]
}

/**
 * Second GPS data.
 */
export class Gps2Raw extends MavLinkData {
  static MSG_ID = 124
  static MSG_NAME = 'GPS2_RAW'
  static PAYLOAD_LENGTH = 57
  static MAGIC_NUMBER = 87

  static FIELDS = [
    new MavLinkPacketField('time_usec', 'timeUsec', 0, false, 8, 'uint64_t', 'us'),
    new MavLinkPacketField('lat', 'lat', 8, false, 4, 'int32_t', 'degE7'),
    new MavLinkPacketField('lon', 'lon', 12, false, 4, 'int32_t', 'degE7'),
    new MavLinkPacketField('alt', 'alt', 16, false, 4, 'int32_t', 'mm'),
    new MavLinkPacketField('dgps_age', 'dgpsAge', 20, false, 4, 'uint32_t', 'ms'),
    new MavLinkPacketField('eph', 'eph', 24, false, 2, 'uint16_t', ''),
    new MavLinkPacketField('epv', 'epv', 26, false, 2, 'uint16_t', ''),
    new MavLinkPacketField('vel', 'vel', 28, false, 2, 'uint16_t', 'cm/s'),
    new MavLinkPacketField('cog', 'cog', 30, false, 2, 'uint16_t', 'cdeg'),
    new MavLinkPacketField('fix_type', 'fixType', 32, false, 1, 'uint8_t', ''),
    new MavLinkPacketField('satellites_visible', 'satellitesVisible', 33, false, 1, 'uint8_t', ''),
    new MavLinkPacketField('dgps_numch', 'dgpsNumch', 34, false, 1, 'uint8_t', ''),
    new MavLinkPacketField('yaw', 'yaw', 35, true, 2, 'uint16_t', 'cdeg'),
    new MavLinkPacketField('alt_ellipsoid', 'altEllipsoid', 37, true, 4, 'int32_t', 'mm'),
    new MavLinkPacketField('h_acc', 'hAcc', 41, true, 4, 'uint32_t', 'mm'),
    new MavLinkPacketField('v_acc', 'vAcc', 45, true, 4, 'uint32_t', 'mm'),
    new MavLinkPacketField('vel_acc', 'velAcc', 49, true, 4, 'uint32_t', 'mm'),
    new MavLinkPacketField('hdg_acc', 'hdgAcc', 53, true, 4, 'uint32_t', 'degE5'),
  ]

  constructor() {
    super()
    this.timeUsec = BigInt(0)
    this.fixType = GpsFixType[Object.keys(GpsFixType)[0]]
    this.lat = 0
    this.lon = 0
    this.alt = 0
    this.eph = 0
    this.epv = 0
    this.vel = 0
    this.cog = 0
    this.satellitesVisible = 0
    this.dgpsNumch = 0
    this.dgpsAge = 0
    this.yaw = 0
    this.altEllipsoid = 0
    this.hAcc = 0
    this.vAcc = 0
    this.velAcc = 0
    this.hdgAcc = 0
  }

  /**
   * Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format
   * (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
   * Units: us
   */
  timeUsec: uint64_t

  /**
   * GPS fix type.
   */
  fixType: GpsFixType

  /**
   * Latitude (WGS84)
   * Units: degE7
   */
  lat: int32_t

  /**
   * Longitude (WGS84)
   * Units: degE7
   */
  lon: int32_t

  /**
   * Altitude (MSL). Positive for up.
   * Units: mm
   */
  alt: int32_t

  /**
   * GPS HDOP horizontal dilution of position (unitless * 100). If unknown, set to: UINT16_MAX
   */
  eph: uint16_t

  /**
   * GPS VDOP vertical dilution of position (unitless * 100). If unknown, set to: UINT16_MAX
   */
  epv: uint16_t

  /**
   * GPS ground speed. If unknown, set to: UINT16_MAX
   * Units: cm/s
   */
  vel: uint16_t

  /**
   * Course over ground (NOT heading, but direction of movement): 0.0..359.99 degrees. If unknown, set
   * to: UINT16_MAX
   * Units: cdeg
   */
  cog: uint16_t

  /**
   * Number of satellites visible. If unknown, set to UINT8_MAX
   */
  satellitesVisible: uint8_t

  /**
   * Number of DGPS satellites
   */
  dgpsNumch: uint8_t

  /**
   * Age of DGPS info
   * Units: ms
   */
  dgpsAge: uint32_t

  /**
   * Yaw in earth frame from north. Use 0 if this GPS does not provide yaw. Use UINT16_MAX if this GPS is
   * configured to provide yaw and is currently unable to provide it. Use 36000 for north.
   * Units: cdeg
   */
  yaw: uint16_t

  /**
   * Altitude (above WGS84, EGM96 ellipsoid). Positive for up.
   * Units: mm
   */
  altEllipsoid: int32_t

  /**
   * Position uncertainty.
   * Units: mm
   */
  hAcc: uint32_t

  /**
   * Altitude uncertainty.
   * Units: mm
   */
  vAcc: uint32_t

  /**
   * Speed uncertainty.
   * Units: mm
   */
  velAcc: uint32_t

  /**
   * Heading / track uncertainty
   * Units: degE5
   */
  hdgAcc: uint32_t
}

/**
 * Power supply status
 */
export class PowerStatus extends MavLinkData {
  static MSG_ID = 125
  static MSG_NAME = 'POWER_STATUS'
  static PAYLOAD_LENGTH = 6
  static MAGIC_NUMBER = 203

  static FIELDS = [
    new MavLinkPacketField('Vcc', 'Vcc', 0, false, 2, 'uint16_t', 'mV'),
    new MavLinkPacketField('Vservo', 'Vservo', 2, false, 2, 'uint16_t', 'mV'),
    new MavLinkPacketField('flags', 'flags', 4, false, 2, 'uint16_t', ''),
  ]

  constructor() {
    super()
    this.Vcc = 0
    this.Vservo = 0
    this.flags = MavPowerStatus[Object.keys(MavPowerStatus)[0]]
  }

  /**
   * 5V rail voltage.
   * Units: mV
   */
  Vcc: uint16_t

  /**
   * Servo rail voltage.
   * Units: mV
   */
  Vservo: uint16_t

  /**
   * Bitmap of power supply status flags.
   */
  flags: MavPowerStatus
}

/**
 * Control a serial port. This can be used for raw access to an onboard serial peripheral such as a GPS
 * or telemetry radio. It is designed to make it possible to update the devices firmware via MAVLink
 * messages or change the devices settings. A message with zero bytes can be used to change just the
 * baudrate.
 */
export class SerialControl extends MavLinkData {
  static MSG_ID = 126
  static MSG_NAME = 'SERIAL_CONTROL'
  static PAYLOAD_LENGTH = 81
  static MAGIC_NUMBER = 220

  static FIELDS = [
    new MavLinkPacketField('baudrate', 'baudrate', 0, false, 4, 'uint32_t', 'bits/s'),
    new MavLinkPacketField('timeout', 'timeout', 4, false, 2, 'uint16_t', 'ms'),
    new MavLinkPacketField('device', 'device', 6, false, 1, 'uint8_t', ''),
    new MavLinkPacketField('flags', 'flags', 7, false, 1, 'uint8_t', ''),
    new MavLinkPacketField('count', 'count', 8, false, 1, 'uint8_t', 'bytes'),
    new MavLinkPacketField('data', 'data', 9, false, 1, 'uint8_t[]', '', 70),
    new MavLinkPacketField('target_system', 'targetSystem', 79, true, 1, 'uint8_t', ''),
    new MavLinkPacketField('target_component', 'targetComponent', 80, true, 1, 'uint8_t', ''),
  ]

  constructor() {
    super()
    this.device = SerialControlDev[Object.keys(SerialControlDev)[0]]
    this.flags = SerialControlFlag[Object.keys(SerialControlFlag)[0]]
    this.timeout = 0
    this.baudrate = 0
    this.count = 0
    this.data = []
    this.targetSystem = 0
    this.targetComponent = 0
  }

  /**
   * Serial control device type.
   */
  device: SerialControlDev

  /**
   * Bitmap of serial control flags.
   */
  flags: SerialControlFlag

  /**
   * Timeout for reply data
   * Units: ms
   */
  timeout: uint16_t

  /**
   * Baudrate of transfer. Zero means no change.
   * Units: bits/s
   */
  baudrate: uint32_t

  /**
   * how many bytes in this transfer
   * Units: bytes
   */
  count: uint8_t

  /**
   * serial data
   */
  data: uint8_t[]

  /**
   * System ID
   */
  targetSystem: uint8_t

  /**
   * Component ID
   */
  targetComponent: uint8_t
}

/**
 * RTK GPS data. Gives information on the relative baseline calculation the GPS is reporting
 */
export class GpsRtk extends MavLinkData {
  static MSG_ID = 127
  static MSG_NAME = 'GPS_RTK'
  static PAYLOAD_LENGTH = 35
  static MAGIC_NUMBER = 25

  static FIELDS = [
    new MavLinkPacketField('time_last_baseline_ms', 'timeLastBaselineMs', 0, false, 4, 'uint32_t', 'ms'),
    new MavLinkPacketField('tow', 'tow', 4, false, 4, 'uint32_t', 'ms'),
    new MavLinkPacketField('baseline_a_mm', 'baselineAMm', 8, false, 4, 'int32_t', 'mm'),
    new MavLinkPacketField('baseline_b_mm', 'baselineBMm', 12, false, 4, 'int32_t', 'mm'),
    new MavLinkPacketField('baseline_c_mm', 'baselineCMm', 16, false, 4, 'int32_t', 'mm'),
    new MavLinkPacketField('accuracy', 'accuracy', 20, false, 4, 'uint32_t', ''),
    new MavLinkPacketField('iar_num_hypotheses', 'iarNumHypotheses', 24, false, 4, 'int32_t', ''),
    new MavLinkPacketField('wn', 'wn', 28, false, 2, 'uint16_t', ''),
    new MavLinkPacketField('rtk_receiver_id', 'rtkReceiverId', 30, false, 1, 'uint8_t', ''),
    new MavLinkPacketField('rtk_health', 'rtkHealth', 31, false, 1, 'uint8_t', ''),
    new MavLinkPacketField('rtk_rate', 'rtkRate', 32, false, 1, 'uint8_t', 'Hz'),
    new MavLinkPacketField('nsats', 'nsats', 33, false, 1, 'uint8_t', ''),
    new MavLinkPacketField('baseline_coords_type', 'baselineCoordsType', 34, false, 1, 'uint8_t', ''),
  ]

  constructor() {
    super()
    this.timeLastBaselineMs = 0
    this.rtkReceiverId = 0
    this.wn = 0
    this.tow = 0
    this.rtkHealth = 0
    this.rtkRate = 0
    this.nsats = 0
    this.baselineCoordsType = RtkBaselineCoordinateSystem[Object.keys(RtkBaselineCoordinateSystem)[0]]
    this.baselineAMm = 0
    this.baselineBMm = 0
    this.baselineCMm = 0
    this.accuracy = 0
    this.iarNumHypotheses = 0
  }

  /**
   * Time since boot of last baseline message received.
   * Units: ms
   */
  timeLastBaselineMs: uint32_t

  /**
   * Identification of connected RTK receiver.
   */
  rtkReceiverId: uint8_t

  /**
   * GPS Week Number of last baseline
   */
  wn: uint16_t

  /**
   * GPS Time of Week of last baseline
   * Units: ms
   */
  tow: uint32_t

  /**
   * GPS-specific health report for RTK data.
   */
  rtkHealth: uint8_t

  /**
   * Rate of baseline messages being received by GPS
   * Units: Hz
   */
  rtkRate: uint8_t

  /**
   * Current number of sats used for RTK calculation.
   */
  nsats: uint8_t

  /**
   * Coordinate system of baseline
   */
  baselineCoordsType: RtkBaselineCoordinateSystem

  /**
   * Current baseline in ECEF x or NED north component.
   * Units: mm
   */
  baselineAMm: int32_t

  /**
   * Current baseline in ECEF y or NED east component.
   * Units: mm
   */
  baselineBMm: int32_t

  /**
   * Current baseline in ECEF z or NED down component.
   * Units: mm
   */
  baselineCMm: int32_t

  /**
   * Current estimate of baseline accuracy.
   */
  accuracy: uint32_t

  /**
   * Current number of integer ambiguity hypotheses.
   */
  iarNumHypotheses: int32_t
}

/**
 * RTK GPS data. Gives information on the relative baseline calculation the GPS is reporting
 */
export class Gps2Rtk extends MavLinkData {
  static MSG_ID = 128
  static MSG_NAME = 'GPS2_RTK'
  static PAYLOAD_LENGTH = 35
  static MAGIC_NUMBER = 226

  static FIELDS = [
    new MavLinkPacketField('time_last_baseline_ms', 'timeLastBaselineMs', 0, false, 4, 'uint32_t', 'ms'),
    new MavLinkPacketField('tow', 'tow', 4, false, 4, 'uint32_t', 'ms'),
    new MavLinkPacketField('baseline_a_mm', 'baselineAMm', 8, false, 4, 'int32_t', 'mm'),
    new MavLinkPacketField('baseline_b_mm', 'baselineBMm', 12, false, 4, 'int32_t', 'mm'),
    new MavLinkPacketField('baseline_c_mm', 'baselineCMm', 16, false, 4, 'int32_t', 'mm'),
    new MavLinkPacketField('accuracy', 'accuracy', 20, false, 4, 'uint32_t', ''),
    new MavLinkPacketField('iar_num_hypotheses', 'iarNumHypotheses', 24, false, 4, 'int32_t', ''),
    new MavLinkPacketField('wn', 'wn', 28, false, 2, 'uint16_t', ''),
    new MavLinkPacketField('rtk_receiver_id', 'rtkReceiverId', 30, false, 1, 'uint8_t', ''),
    new MavLinkPacketField('rtk_health', 'rtkHealth', 31, false, 1, 'uint8_t', ''),
    new MavLinkPacketField('rtk_rate', 'rtkRate', 32, false, 1, 'uint8_t', 'Hz'),
    new MavLinkPacketField('nsats', 'nsats', 33, false, 1, 'uint8_t', ''),
    new MavLinkPacketField('baseline_coords_type', 'baselineCoordsType', 34, false, 1, 'uint8_t', ''),
  ]

  constructor() {
    super()
    this.timeLastBaselineMs = 0
    this.rtkReceiverId = 0
    this.wn = 0
    this.tow = 0
    this.rtkHealth = 0
    this.rtkRate = 0
    this.nsats = 0
    this.baselineCoordsType = RtkBaselineCoordinateSystem[Object.keys(RtkBaselineCoordinateSystem)[0]]
    this.baselineAMm = 0
    this.baselineBMm = 0
    this.baselineCMm = 0
    this.accuracy = 0
    this.iarNumHypotheses = 0
  }

  /**
   * Time since boot of last baseline message received.
   * Units: ms
   */
  timeLastBaselineMs: uint32_t

  /**
   * Identification of connected RTK receiver.
   */
  rtkReceiverId: uint8_t

  /**
   * GPS Week Number of last baseline
   */
  wn: uint16_t

  /**
   * GPS Time of Week of last baseline
   * Units: ms
   */
  tow: uint32_t

  /**
   * GPS-specific health report for RTK data.
   */
  rtkHealth: uint8_t

  /**
   * Rate of baseline messages being received by GPS
   * Units: Hz
   */
  rtkRate: uint8_t

  /**
   * Current number of sats used for RTK calculation.
   */
  nsats: uint8_t

  /**
   * Coordinate system of baseline
   */
  baselineCoordsType: RtkBaselineCoordinateSystem

  /**
   * Current baseline in ECEF x or NED north component.
   * Units: mm
   */
  baselineAMm: int32_t

  /**
   * Current baseline in ECEF y or NED east component.
   * Units: mm
   */
  baselineBMm: int32_t

  /**
   * Current baseline in ECEF z or NED down component.
   * Units: mm
   */
  baselineCMm: int32_t

  /**
   * Current estimate of baseline accuracy.
   */
  accuracy: uint32_t

  /**
   * Current number of integer ambiguity hypotheses.
   */
  iarNumHypotheses: int32_t
}

/**
 * The RAW IMU readings for 3rd 9DOF sensor setup. This message should contain the scaled values to the
 * described units
 */
export class ScaledImu3 extends MavLinkData {
  static MSG_ID = 129
  static MSG_NAME = 'SCALED_IMU3'
  static PAYLOAD_LENGTH = 24
  static MAGIC_NUMBER = 46

  static FIELDS = [
    new MavLinkPacketField('time_boot_ms', 'timeBootMs', 0, false, 4, 'uint32_t', 'ms'),
    new MavLinkPacketField('xacc', 'xacc', 4, false, 2, 'int16_t', 'mG'),
    new MavLinkPacketField('yacc', 'yacc', 6, false, 2, 'int16_t', 'mG'),
    new MavLinkPacketField('zacc', 'zacc', 8, false, 2, 'int16_t', 'mG'),
    new MavLinkPacketField('xgyro', 'xgyro', 10, false, 2, 'int16_t', 'mrad/s'),
    new MavLinkPacketField('ygyro', 'ygyro', 12, false, 2, 'int16_t', 'mrad/s'),
    new MavLinkPacketField('zgyro', 'zgyro', 14, false, 2, 'int16_t', 'mrad/s'),
    new MavLinkPacketField('xmag', 'xmag', 16, false, 2, 'int16_t', 'mgauss'),
    new MavLinkPacketField('ymag', 'ymag', 18, false, 2, 'int16_t', 'mgauss'),
    new MavLinkPacketField('zmag', 'zmag', 20, false, 2, 'int16_t', 'mgauss'),
    new MavLinkPacketField('temperature', 'temperature', 22, true, 2, 'int16_t', 'cdegC'),
  ]

  constructor() {
    super()
    this.timeBootMs = 0
    this.xacc = 0
    this.yacc = 0
    this.zacc = 0
    this.xgyro = 0
    this.ygyro = 0
    this.zgyro = 0
    this.xmag = 0
    this.ymag = 0
    this.zmag = 0
    this.temperature = 0
  }

  /**
   * Timestamp (time since system boot).
   * Units: ms
   */
  timeBootMs: uint32_t

  /**
   * X acceleration
   * Units: mG
   */
  xacc: int16_t

  /**
   * Y acceleration
   * Units: mG
   */
  yacc: int16_t

  /**
   * Z acceleration
   * Units: mG
   */
  zacc: int16_t

  /**
   * Angular speed around X axis
   * Units: mrad/s
   */
  xgyro: int16_t

  /**
   * Angular speed around Y axis
   * Units: mrad/s
   */
  ygyro: int16_t

  /**
   * Angular speed around Z axis
   * Units: mrad/s
   */
  zgyro: int16_t

  /**
   * X Magnetic field
   * Units: mgauss
   */
  xmag: int16_t

  /**
   * Y Magnetic field
   * Units: mgauss
   */
  ymag: int16_t

  /**
   * Z Magnetic field
   * Units: mgauss
   */
  zmag: int16_t

  /**
   * Temperature, 0: IMU does not provide temperature values. If the IMU is at 0C it must send 1 (0.01C).
   * Units: cdegC
   */
  temperature: int16_t
}

/**
 * Handshake message to initiate, control and stop image streaming when using the Image Transmission
 * Protocol: https://mavlink.io/en/services/image_transmission.html.
 */
export class DataTransmissionHandshake extends MavLinkData {
  static MSG_ID = 130
  static MSG_NAME = 'DATA_TRANSMISSION_HANDSHAKE'
  static PAYLOAD_LENGTH = 13
  static MAGIC_NUMBER = 29

  static FIELDS = [
    new MavLinkPacketField('size', 'size', 0, false, 4, 'uint32_t', 'bytes'),
    new MavLinkPacketField('width', 'width', 4, false, 2, 'uint16_t', ''),
    new MavLinkPacketField('height', 'height', 6, false, 2, 'uint16_t', ''),
    new MavLinkPacketField('packets', 'packets', 8, false, 2, 'uint16_t', ''),
    new MavLinkPacketField('type', 'type', 10, false, 1, 'uint8_t', ''),
    new MavLinkPacketField('payload', 'payload', 11, false, 1, 'uint8_t', 'bytes'),
    new MavLinkPacketField('jpg_quality', 'jpgQuality', 12, false, 1, 'uint8_t', '%'),
  ]

  constructor() {
    super()
    this.type = MavlinkDataStreamType[Object.keys(MavlinkDataStreamType)[0]]
    this.size = 0
    this.width = 0
    this.height = 0
    this.packets = 0
    this.payload = 0
    this.jpgQuality = 0
  }

  /**
   * Type of requested/acknowledged data.
   */
  type: MavlinkDataStreamType

  /**
   * total data size (set on ACK only).
   * Units: bytes
   */
  size: uint32_t

  /**
   * Width of a matrix or image.
   */
  width: uint16_t

  /**
   * Height of a matrix or image.
   */
  height: uint16_t

  /**
   * Number of packets being sent (set on ACK only).
   */
  packets: uint16_t

  /**
   * Payload size per packet (normally 253 byte, see DATA field size in message ENCAPSULATED_DATA) (set
   * on ACK only).
   * Units: bytes
   */
  payload: uint8_t

  /**
   * JPEG quality. Values: [1-100].
   * Units: %
   */
  jpgQuality: uint8_t
}

/**
 * Data packet for images sent using the Image Transmission Protocol:
 * https://mavlink.io/en/services/image_transmission.html.
 */
export class EncapsulatedData extends MavLinkData {
  static MSG_ID = 131
  static MSG_NAME = 'ENCAPSULATED_DATA'
  static PAYLOAD_LENGTH = 255
  static MAGIC_NUMBER = 223

  static FIELDS = [
    new MavLinkPacketField('seqnr', 'seqnr', 0, false, 2, 'uint16_t', ''),
    new MavLinkPacketField('data', 'data', 2, false, 1, 'uint8_t[]', '', 253),
  ]

  constructor() {
    super()
    this.seqnr = 0
    this.data = []
  }

  /**
   * sequence number (starting with 0 on every transmission)
   */
  seqnr: uint16_t

  /**
   * image data bytes
   */
  data: uint8_t[]
}

/**
 * Distance sensor information for an onboard rangefinder.
 */
export class DistanceSensor extends MavLinkData {
  static MSG_ID = 132
  static MSG_NAME = 'DISTANCE_SENSOR'
  static PAYLOAD_LENGTH = 39
  static MAGIC_NUMBER = 85

  static FIELDS = [
    new MavLinkPacketField('time_boot_ms', 'timeBootMs', 0, false, 4, 'uint32_t', 'ms'),
    new MavLinkPacketField('min_distance', 'minDistance', 4, false, 2, 'uint16_t', 'cm'),
    new MavLinkPacketField('max_distance', 'maxDistance', 6, false, 2, 'uint16_t', 'cm'),
    new MavLinkPacketField('current_distance', 'currentDistance', 8, false, 2, 'uint16_t', 'cm'),
    new MavLinkPacketField('type', 'type', 10, false, 1, 'uint8_t', ''),
    new MavLinkPacketField('id', 'id', 11, false, 1, 'uint8_t', ''),
    new MavLinkPacketField('orientation', 'orientation', 12, false, 1, 'uint8_t', ''),
    new MavLinkPacketField('covariance', 'covariance', 13, false, 1, 'uint8_t', 'cm^2'),
    new MavLinkPacketField('horizontal_fov', 'horizontalFov', 14, true, 4, 'float', 'rad'),
    new MavLinkPacketField('vertical_fov', 'verticalFov', 18, true, 4, 'float', 'rad'),
    new MavLinkPacketField('quaternion', 'quaternion', 22, true, 4, 'float[]', '', 4),
    new MavLinkPacketField('signal_quality', 'signalQuality', 38, true, 1, 'uint8_t', '%'),
  ]

  constructor() {
    super()
    this.timeBootMs = 0
    this.minDistance = 0
    this.maxDistance = 0
    this.currentDistance = 0
    this.type = MavDistanceSensor[Object.keys(MavDistanceSensor)[0]]
    this.id = 0
    this.orientation = MavSensorOrientation[Object.keys(MavSensorOrientation)[0]]
    this.covariance = 0
    this.horizontalFov = 0
    this.verticalFov = 0
    this.quaternion = []
    this.signalQuality = 0
  }

  /**
   * Timestamp (time since system boot).
   * Units: ms
   */
  timeBootMs: uint32_t

  /**
   * Minimum distance the sensor can measure
   * Units: cm
   */
  minDistance: uint16_t

  /**
   * Maximum distance the sensor can measure
   * Units: cm
   */
  maxDistance: uint16_t

  /**
   * Current distance reading
   * Units: cm
   */
  currentDistance: uint16_t

  /**
   * Type of distance sensor.
   */
  type: MavDistanceSensor

  /**
   * Onboard ID of the sensor
   */
  id: uint8_t

  /**
   * Direction the sensor faces. downward-facing: ROTATION_PITCH_270, upward-facing: ROTATION_PITCH_90,
   * backward-facing: ROTATION_PITCH_180, forward-facing: ROTATION_NONE, left-facing: ROTATION_YAW_90,
   * right-facing: ROTATION_YAW_270
   */
  orientation: MavSensorOrientation

  /**
   * Measurement variance. Max standard deviation is 6cm. UINT8_MAX if unknown.
   * Units: cm^2
   */
  covariance: uint8_t

  /**
   * Horizontal Field of View (angle) where the distance measurement is valid and the field of view is
   * known. Otherwise this is set to 0.
   * Units: rad
   */
  horizontalFov: float

  /**
   * Vertical Field of View (angle) where the distance measurement is valid and the field of view is
   * known. Otherwise this is set to 0.
   * Units: rad
   */
  verticalFov: float

  /**
   * Quaternion of the sensor orientation in vehicle body frame (w, x, y, z order, zero-rotation is 1, 0,
   * 0, 0). Zero-rotation is along the vehicle body x-axis. This field is required if the orientation is
   * set to MAV_SENSOR_ROTATION_CUSTOM. Set it to 0 if invalid."
   */
  quaternion: float[]

  /**
   * Signal quality of the sensor. Specific to each sensor type, representing the relation of the signal
   * strength with the target reflectivity, distance, size or aspect, but normalised as a percentage. 0 =
   * unknown/unset signal quality, 1 = invalid signal, 100 = perfect signal.
   * Units: %
   */
  signalQuality: uint8_t
}

/**
 * Request for terrain data and terrain status. See terrain protocol docs:
 * https://mavlink.io/en/services/terrain.html
 */
export class TerrainRequest extends MavLinkData {
  static MSG_ID = 133
  static MSG_NAME = 'TERRAIN_REQUEST'
  static PAYLOAD_LENGTH = 18
  static MAGIC_NUMBER = 6

  static FIELDS = [
    new MavLinkPacketField('mask', 'mask', 0, false, 8, 'uint64_t', ''),
    new MavLinkPacketField('lat', 'lat', 8, false, 4, 'int32_t', 'degE7'),
    new MavLinkPacketField('lon', 'lon', 12, false, 4, 'int32_t', 'degE7'),
    new MavLinkPacketField('grid_spacing', 'gridSpacing', 16, false, 2, 'uint16_t', 'm'),
  ]

  constructor() {
    super()
    this.lat = 0
    this.lon = 0
    this.gridSpacing = 0
    this.mask = BigInt(0)
  }

  /**
   * Latitude of SW corner of first grid
   * Units: degE7
   */
  lat: int32_t

  /**
   * Longitude of SW corner of first grid
   * Units: degE7
   */
  lon: int32_t

  /**
   * Grid spacing
   * Units: m
   */
  gridSpacing: uint16_t

  /**
   * Bitmask of requested 4x4 grids (row major 8x7 array of grids, 56 bits)
   */
  mask: uint64_t
}

/**
 * Terrain data sent from GCS. The lat/lon and grid_spacing must be the same as a lat/lon from a
 * TERRAIN_REQUEST. See terrain protocol docs: https://mavlink.io/en/services/terrain.html
 */
export class TerrainData extends MavLinkData {
  static MSG_ID = 134
  static MSG_NAME = 'TERRAIN_DATA'
  static PAYLOAD_LENGTH = 43
  static MAGIC_NUMBER = 229

  static FIELDS = [
    new MavLinkPacketField('lat', 'lat', 0, false, 4, 'int32_t', 'degE7'),
    new MavLinkPacketField('lon', 'lon', 4, false, 4, 'int32_t', 'degE7'),
    new MavLinkPacketField('grid_spacing', 'gridSpacing', 8, false, 2, 'uint16_t', 'm'),
    new MavLinkPacketField('data', 'data', 10, false, 2, 'int16_t[]', 'm', 16),
    new MavLinkPacketField('gridbit', 'gridbit', 42, false, 1, 'uint8_t', ''),
  ]

  constructor() {
    super()
    this.lat = 0
    this.lon = 0
    this.gridSpacing = 0
    this.gridbit = 0
    this.data = []
  }

  /**
   * Latitude of SW corner of first grid
   * Units: degE7
   */
  lat: int32_t

  /**
   * Longitude of SW corner of first grid
   * Units: degE7
   */
  lon: int32_t

  /**
   * Grid spacing
   * Units: m
   */
  gridSpacing: uint16_t

  /**
   * bit within the terrain request mask
   */
  gridbit: uint8_t

  /**
   * Terrain data MSL
   * Units: m
   */
  data: int16_t[]
}

/**
 * Request that the vehicle report terrain height at the given location (expected response is a
 * TERRAIN_REPORT). Used by GCS to check if vehicle has all terrain data needed for a mission.
 */
export class TerrainCheck extends MavLinkData {
  static MSG_ID = 135
  static MSG_NAME = 'TERRAIN_CHECK'
  static PAYLOAD_LENGTH = 8
  static MAGIC_NUMBER = 203

  static FIELDS = [
    new MavLinkPacketField('lat', 'lat', 0, false, 4, 'int32_t', 'degE7'),
    new MavLinkPacketField('lon', 'lon', 4, false, 4, 'int32_t', 'degE7'),
  ]

  constructor() {
    super()
    this.lat = 0
    this.lon = 0
  }

  /**
   * Latitude
   * Units: degE7
   */
  lat: int32_t

  /**
   * Longitude
   * Units: degE7
   */
  lon: int32_t
}

/**
 * Streamed from drone to report progress of terrain map download (initiated by TERRAIN_REQUEST), or
 * sent as a response to a TERRAIN_CHECK request. See terrain protocol docs:
 * https://mavlink.io/en/services/terrain.html
 */
export class TerrainReport extends MavLinkData {
  static MSG_ID = 136
  static MSG_NAME = 'TERRAIN_REPORT'
  static PAYLOAD_LENGTH = 22
  static MAGIC_NUMBER = 1

  static FIELDS = [
    new MavLinkPacketField('lat', 'lat', 0, false, 4, 'int32_t', 'degE7'),
    new MavLinkPacketField('lon', 'lon', 4, false, 4, 'int32_t', 'degE7'),
    new MavLinkPacketField('terrain_height', 'terrainHeight', 8, false, 4, 'float', 'm'),
    new MavLinkPacketField('current_height', 'currentHeight', 12, false, 4, 'float', 'm'),
    new MavLinkPacketField('spacing', 'spacing', 16, false, 2, 'uint16_t', ''),
    new MavLinkPacketField('pending', 'pending', 18, false, 2, 'uint16_t', ''),
    new MavLinkPacketField('loaded', 'loaded', 20, false, 2, 'uint16_t', ''),
  ]

  constructor() {
    super()
    this.lat = 0
    this.lon = 0
    this.spacing = 0
    this.terrainHeight = 0
    this.currentHeight = 0
    this.pending = 0
    this.loaded = 0
  }

  /**
   * Latitude
   * Units: degE7
   */
  lat: int32_t

  /**
   * Longitude
   * Units: degE7
   */
  lon: int32_t

  /**
   * grid spacing (zero if terrain at this location unavailable)
   */
  spacing: uint16_t

  /**
   * Terrain height MSL
   * Units: m
   */
  terrainHeight: float

  /**
   * Current vehicle height above lat/lon terrain height
   * Units: m
   */
  currentHeight: float

  /**
   * Number of 4x4 terrain blocks waiting to be received or read from disk
   */
  pending: uint16_t

  /**
   * Number of 4x4 terrain blocks in memory
   */
  loaded: uint16_t
}

/**
 * Barometer readings for 2nd barometer
 */
export class ScaledPressure2 extends MavLinkData {
  static MSG_ID = 137
  static MSG_NAME = 'SCALED_PRESSURE2'
  static PAYLOAD_LENGTH = 16
  static MAGIC_NUMBER = 195

  static FIELDS = [
    new MavLinkPacketField('time_boot_ms', 'timeBootMs', 0, false, 4, 'uint32_t', 'ms'),
    new MavLinkPacketField('press_abs', 'pressAbs', 4, false, 4, 'float', 'hPa'),
    new MavLinkPacketField('press_diff', 'pressDiff', 8, false, 4, 'float', 'hPa'),
    new MavLinkPacketField('temperature', 'temperature', 12, false, 2, 'int16_t', 'cdegC'),
    new MavLinkPacketField('temperature_press_diff', 'temperaturePressDiff', 14, true, 2, 'int16_t', 'cdegC'),
  ]

  constructor() {
    super()
    this.timeBootMs = 0
    this.pressAbs = 0
    this.pressDiff = 0
    this.temperature = 0
    this.temperaturePressDiff = 0
  }

  /**
   * Timestamp (time since system boot).
   * Units: ms
   */
  timeBootMs: uint32_t

  /**
   * Absolute pressure
   * Units: hPa
   */
  pressAbs: float

  /**
   * Differential pressure
   * Units: hPa
   */
  pressDiff: float

  /**
   * Absolute pressure temperature
   * Units: cdegC
   */
  temperature: int16_t

  /**
   * Differential pressure temperature (0, if not available). Report values of 0 (or 1) as 1 cdegC.
   * Units: cdegC
   */
  temperaturePressDiff: int16_t
}

/**
 * Motion capture attitude and position
 */
export class MotionCaptureAttPos extends MavLinkData {
  static MSG_ID = 138
  static MSG_NAME = 'ATT_POS_MOCAP'
  static PAYLOAD_LENGTH = 120
  static MAGIC_NUMBER = 109

  static FIELDS = [
    new MavLinkPacketField('time_usec', 'timeUsec', 0, false, 8, 'uint64_t', 'us'),
    new MavLinkPacketField('q', 'q', 8, false, 4, 'float[]', '', 4),
    new MavLinkPacketField('x', 'x', 24, false, 4, 'float', 'm'),
    new MavLinkPacketField('y', 'y', 28, false, 4, 'float', 'm'),
    new MavLinkPacketField('z', 'z', 32, false, 4, 'float', 'm'),
    new MavLinkPacketField('covariance', 'covariance', 36, true, 4, 'float[]', '', 21),
  ]

  constructor() {
    super()
    this.timeUsec = BigInt(0)
    this.q = []
    this.x = 0
    this.y = 0
    this.z = 0
    this.covariance = []
  }

  /**
   * Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format
   * (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
   * Units: us
   */
  timeUsec: uint64_t

  /**
   * Attitude quaternion (w, x, y, z order, zero-rotation is 1, 0, 0, 0)
   */
  q: float[]

  /**
   * X position (NED)
   * Units: m
   */
  x: float

  /**
   * Y position (NED)
   * Units: m
   */
  y: float

  /**
   * Z position (NED)
   * Units: m
   */
  z: float

  /**
   * Row-major representation of a pose 6x6 cross-covariance matrix upper right triangle (states: x, y,
   * z, roll, pitch, yaw; first six entries are the first ROW, next five entries are the second ROW,
   * etc.). If unknown, assign NaN value to first element in the array.
   */
  covariance: float[]
}

/**
 * Set the vehicle attitude and body angular rates.
 */
export class SetActuatorControlTarget extends MavLinkData {
  static MSG_ID = 139
  static MSG_NAME = 'SET_ACTUATOR_CONTROL_TARGET'
  static PAYLOAD_LENGTH = 43
  static MAGIC_NUMBER = 168

  static FIELDS = [
    new MavLinkPacketField('time_usec', 'timeUsec', 0, false, 8, 'uint64_t', 'us'),
    new MavLinkPacketField('controls', 'controls', 8, false, 4, 'float[]', '', 8),
    new MavLinkPacketField('group_mlx', 'groupMlx', 40, false, 1, 'uint8_t', ''),
    new MavLinkPacketField('target_system', 'targetSystem', 41, false, 1, 'uint8_t', ''),
    new MavLinkPacketField('target_component', 'targetComponent', 42, false, 1, 'uint8_t', ''),
  ]

  constructor() {
    super()
    this.timeUsec = BigInt(0)
    this.groupMlx = 0
    this.targetSystem = 0
    this.targetComponent = 0
    this.controls = []
  }

  /**
   * Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format
   * (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
   * Units: us
   */
  timeUsec: uint64_t

  /**
   * Actuator group. The "_mlx" indicates this is a multi-instance message and a MAVLink parser should
   * use this field to difference between instances.
   */
  groupMlx: uint8_t

  /**
   * System ID
   */
  targetSystem: uint8_t

  /**
   * Component ID
   */
  targetComponent: uint8_t

  /**
   * Actuator controls. Normed to -1..+1 where 0 is neutral position. Throttle for single rotation
   * direction motors is 0..1, negative range for reverse direction. Standard mapping for attitude
   * controls (group 0): (index 0-7): roll, pitch, yaw, throttle, flaps, spoilers, airbrakes, landing
   * gear. Load a pass-through mixer to repurpose them as generic outputs.
   */
  controls: float[]
}

/**
 * Set the vehicle attitude and body angular rates.
 */
export class ActuatorControlTarget extends MavLinkData {
  static MSG_ID = 140
  static MSG_NAME = 'ACTUATOR_CONTROL_TARGET'
  static PAYLOAD_LENGTH = 41
  static MAGIC_NUMBER = 181

  static FIELDS = [
    new MavLinkPacketField('time_usec', 'timeUsec', 0, false, 8, 'uint64_t', 'us'),
    new MavLinkPacketField('controls', 'controls', 8, false, 4, 'float[]', '', 8),
    new MavLinkPacketField('group_mlx', 'groupMlx', 40, false, 1, 'uint8_t', ''),
  ]

  constructor() {
    super()
    this.timeUsec = BigInt(0)
    this.groupMlx = 0
    this.controls = []
  }

  /**
   * Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format
   * (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
   * Units: us
   */
  timeUsec: uint64_t

  /**
   * Actuator group. The "_mlx" indicates this is a multi-instance message and a MAVLink parser should
   * use this field to difference between instances.
   */
  groupMlx: uint8_t

  /**
   * Actuator controls. Normed to -1..+1 where 0 is neutral position. Throttle for single rotation
   * direction motors is 0..1, negative range for reverse direction. Standard mapping for attitude
   * controls (group 0): (index 0-7): roll, pitch, yaw, throttle, flaps, spoilers, airbrakes, landing
   * gear. Load a pass-through mixer to repurpose them as generic outputs.
   */
  controls: float[]
}

/**
 * The current system altitude.
 */
export class Altitude extends MavLinkData {
  static MSG_ID = 141
  static MSG_NAME = 'ALTITUDE'
  static PAYLOAD_LENGTH = 32
  static MAGIC_NUMBER = 47

  static FIELDS = [
    new MavLinkPacketField('time_usec', 'timeUsec', 0, false, 8, 'uint64_t', 'us'),
    new MavLinkPacketField('altitude_monotonic', 'altitudeMonotonic', 8, false, 4, 'float', 'm'),
    new MavLinkPacketField('altitude_amsl', 'altitudeAmsl', 12, false, 4, 'float', 'm'),
    new MavLinkPacketField('altitude_local', 'altitudeLocal', 16, false, 4, 'float', 'm'),
    new MavLinkPacketField('altitude_relative', 'altitudeRelative', 20, false, 4, 'float', 'm'),
    new MavLinkPacketField('altitude_terrain', 'altitudeTerrain', 24, false, 4, 'float', 'm'),
    new MavLinkPacketField('bottom_clearance', 'bottomClearance', 28, false, 4, 'float', 'm'),
  ]

  constructor() {
    super()
    this.timeUsec = BigInt(0)
    this.altitudeMonotonic = 0
    this.altitudeAmsl = 0
    this.altitudeLocal = 0
    this.altitudeRelative = 0
    this.altitudeTerrain = 0
    this.bottomClearance = 0
  }

  /**
   * Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format
   * (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
   * Units: us
   */
  timeUsec: uint64_t

  /**
   * This altitude measure is initialized on system boot and monotonic (it is never reset, but represents
   * the local altitude change). The only guarantee on this field is that it will never be reset and is
   * consistent within a flight. The recommended value for this field is the uncorrected barometric
   * altitude at boot time. This altitude will also drift and vary between flights.
   * Units: m
   */
  altitudeMonotonic: float

  /**
   * This altitude measure is strictly above mean sea level and might be non-monotonic (it might reset on
   * events like GPS lock or when a new QNH value is set). It should be the altitude to which global
   * altitude waypoints are compared to. Note that it is *not* the GPS altitude, however, most GPS
   * modules already output MSL by default and not the WGS84 altitude.
   * Units: m
   */
  altitudeAmsl: float

  /**
   * This is the local altitude in the local coordinate frame. It is not the altitude above home, but in
   * reference to the coordinate origin (0, 0, 0). It is up-positive.
   * Units: m
   */
  altitudeLocal: float

  /**
   * This is the altitude above the home position. It resets on each change of the current home position.
   * Units: m
   */
  altitudeRelative: float

  /**
   * This is the altitude above terrain. It might be fed by a terrain database or an altimeter. Values
   * smaller than -1000 should be interpreted as unknown.
   * Units: m
   */
  altitudeTerrain: float

  /**
   * This is not the altitude, but the clear space below the system according to the fused clearance
   * estimate. It generally should max out at the maximum range of e.g. the laser altimeter. It is
   * generally a moving target. A negative value indicates no measurement available.
   * Units: m
   */
  bottomClearance: float
}

/**
 * The autopilot is requesting a resource (file, binary, other type of data)
 */
export class ResourceRequest extends MavLinkData {
  static MSG_ID = 142
  static MSG_NAME = 'RESOURCE_REQUEST'
  static PAYLOAD_LENGTH = 243
  static MAGIC_NUMBER = 72

  static FIELDS = [
    new MavLinkPacketField('request_id', 'requestId', 0, false, 1, 'uint8_t', ''),
    new MavLinkPacketField('uri_type', 'uriType', 1, false, 1, 'uint8_t', ''),
    new MavLinkPacketField('uri', 'uri', 2, false, 1, 'uint8_t[]', '', 120),
    new MavLinkPacketField('transfer_type', 'transferType', 122, false, 1, 'uint8_t', ''),
    new MavLinkPacketField('storage', 'storage', 123, false, 1, 'uint8_t[]', '', 120),
  ]

  constructor() {
    super()
    this.requestId = 0
    this.uriType = 0
    this.uri = []
    this.transferType = 0
    this.storage = []
  }

  /**
   * Request ID. This ID should be re-used when sending back URI contents
   */
  requestId: uint8_t

  /**
   * The type of requested URI. 0 = a file via URL. 1 = a UAVCAN binary
   */
  uriType: uint8_t

  /**
   * The requested unique resource identifier (URI). It is not necessarily a straight domain name
   * (depends on the URI type enum)
   */
  uri: uint8_t[]

  /**
   * The way the autopilot wants to receive the URI. 0 = MAVLink FTP. 1 = binary stream.
   */
  transferType: uint8_t

  /**
   * The storage path the autopilot wants the URI to be stored in. Will only be valid if the
   * transfer_type has a storage associated (e.g. MAVLink FTP).
   */
  storage: uint8_t[]
}

/**
 * Barometer readings for 3rd barometer
 */
export class ScaledPressure3 extends MavLinkData {
  static MSG_ID = 143
  static MSG_NAME = 'SCALED_PRESSURE3'
  static PAYLOAD_LENGTH = 16
  static MAGIC_NUMBER = 131

  static FIELDS = [
    new MavLinkPacketField('time_boot_ms', 'timeBootMs', 0, false, 4, 'uint32_t', 'ms'),
    new MavLinkPacketField('press_abs', 'pressAbs', 4, false, 4, 'float', 'hPa'),
    new MavLinkPacketField('press_diff', 'pressDiff', 8, false, 4, 'float', 'hPa'),
    new MavLinkPacketField('temperature', 'temperature', 12, false, 2, 'int16_t', 'cdegC'),
    new MavLinkPacketField('temperature_press_diff', 'temperaturePressDiff', 14, true, 2, 'int16_t', 'cdegC'),
  ]

  constructor() {
    super()
    this.timeBootMs = 0
    this.pressAbs = 0
    this.pressDiff = 0
    this.temperature = 0
    this.temperaturePressDiff = 0
  }

  /**
   * Timestamp (time since system boot).
   * Units: ms
   */
  timeBootMs: uint32_t

  /**
   * Absolute pressure
   * Units: hPa
   */
  pressAbs: float

  /**
   * Differential pressure
   * Units: hPa
   */
  pressDiff: float

  /**
   * Absolute pressure temperature
   * Units: cdegC
   */
  temperature: int16_t

  /**
   * Differential pressure temperature (0, if not available). Report values of 0 (or 1) as 1 cdegC.
   * Units: cdegC
   */
  temperaturePressDiff: int16_t
}

/**
 * Current motion information from a designated system
 */
export class FollowTarget extends MavLinkData {
  static MSG_ID = 144
  static MSG_NAME = 'FOLLOW_TARGET'
  static PAYLOAD_LENGTH = 97
  static MAGIC_NUMBER = 127

  static FIELDS = [
    new MavLinkPacketField('timestamp', 'timestamp', 0, false, 8, 'uint64_t', 'ms'),
    new MavLinkPacketField('custom_state', 'customState', 8, false, 8, 'uint64_t', ''),
    new MavLinkPacketField('lat', 'lat', 16, false, 4, 'int32_t', 'degE7'),
    new MavLinkPacketField('lon', 'lon', 20, false, 4, 'int32_t', 'degE7'),
    new MavLinkPacketField('alt', 'alt', 24, false, 4, 'float', 'm'),
    new MavLinkPacketField('vel', 'vel', 28, false, 4, 'float[]', 'm/s', 3),
    new MavLinkPacketField('acc', 'acc', 40, false, 4, 'float[]', 'm/s/s', 3),
    new MavLinkPacketField('attitude_q', 'attitudeQ', 52, false, 4, 'float[]', '', 4),
    new MavLinkPacketField('rates', 'rates', 68, false, 4, 'float[]', '', 3),
    new MavLinkPacketField('position_cov', 'positionCov', 80, false, 4, 'float[]', '', 3),
    new MavLinkPacketField('est_capabilities', 'estCapabilities', 92, false, 1, 'uint8_t', ''),
    new MavLinkPacketField('yaw', 'yaw', 93, true, 4, 'float', 'cdegC'),
  ]

  constructor() {
    super()
    this.timestamp = BigInt(0)
    this.estCapabilities = 0
    this.lat = 0
    this.lon = 0
    this.alt = 0
    this.vel = []
    this.acc = []
    this.attitudeQ = []
    this.rates = []
    this.positionCov = []
    this.customState = BigInt(0)
    this.yaw = 0
  }

  /**
   * Timestamp (time since system boot).
   * Units: ms
   */
  timestamp: uint64_t

  /**
   * bit positions for tracker reporting capabilities (POS = 0, VEL = 1, ACCEL = 2, ATT + RATES = 3)
   */
  estCapabilities: uint8_t

  /**
   * Latitude (WGS84)
   * Units: degE7
   */
  lat: int32_t

  /**
   * Longitude (WGS84)
   * Units: degE7
   */
  lon: int32_t

  /**
   * Altitude (MSL)
   * Units: m
   */
  alt: float

  /**
   * target velocity (0,0,0) for unknown
   * Units: m/s
   */
  vel: float[]

  /**
   * linear target acceleration (0,0,0) for unknown
   * Units: m/s/s
   */
  acc: float[]

  /**
   * (0 0 0 0 for unknown)
   */
  attitudeQ: float[]

  /**
   * (0 0 0 for unknown)
   */
  rates: float[]

  /**
   * eph epv
   */
  positionCov: float[]

  /**
   * button states or switches of a tracker device
   */
  customState: uint64_t

  /**
   * target yaw
   * Units: cdegC
   */
  yaw: float
}

/**
 * The smoothed, monotonic system state used to feed the control loops of the system.
 */
export class ControlSystemState extends MavLinkData {
  static MSG_ID = 146
  static MSG_NAME = 'CONTROL_SYSTEM_STATE'
  static PAYLOAD_LENGTH = 100
  static MAGIC_NUMBER = 103

  static FIELDS = [
    new MavLinkPacketField('time_usec', 'timeUsec', 0, false, 8, 'uint64_t', 'us'),
    new MavLinkPacketField('x_acc', 'xAcc', 8, false, 4, 'float', 'm/s/s'),
    new MavLinkPacketField('y_acc', 'yAcc', 12, false, 4, 'float', 'm/s/s'),
    new MavLinkPacketField('z_acc', 'zAcc', 16, false, 4, 'float', 'm/s/s'),
    new MavLinkPacketField('x_vel', 'xVel', 20, false, 4, 'float', 'm/s'),
    new MavLinkPacketField('y_vel', 'yVel', 24, false, 4, 'float', 'm/s'),
    new MavLinkPacketField('z_vel', 'zVel', 28, false, 4, 'float', 'm/s'),
    new MavLinkPacketField('x_pos', 'xPos', 32, false, 4, 'float', 'm'),
    new MavLinkPacketField('y_pos', 'yPos', 36, false, 4, 'float', 'm'),
    new MavLinkPacketField('z_pos', 'zPos', 40, false, 4, 'float', 'm'),
    new MavLinkPacketField('airspeed', 'airspeed', 44, false, 4, 'float', 'm/s'),
    new MavLinkPacketField('vel_variance', 'velVariance', 48, false, 4, 'float[]', '', 3),
    new MavLinkPacketField('pos_variance', 'posVariance', 60, false, 4, 'float[]', '', 3),
    new MavLinkPacketField('q', 'q', 72, false, 4, 'float[]', '', 4),
    new MavLinkPacketField('roll_rate', 'rollRate', 88, false, 4, 'float', 'rad/s'),
    new MavLinkPacketField('pitch_rate', 'pitchRate', 92, false, 4, 'float', 'rad/s'),
    new MavLinkPacketField('yaw_rate', 'yawRate', 96, false, 4, 'float', 'rad/s'),
  ]

  constructor() {
    super()
    this.timeUsec = BigInt(0)
    this.xAcc = 0
    this.yAcc = 0
    this.zAcc = 0
    this.xVel = 0
    this.yVel = 0
    this.zVel = 0
    this.xPos = 0
    this.yPos = 0
    this.zPos = 0
    this.airspeed = 0
    this.velVariance = []
    this.posVariance = []
    this.q = []
    this.rollRate = 0
    this.pitchRate = 0
    this.yawRate = 0
  }

  /**
   * Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format
   * (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
   * Units: us
   */
  timeUsec: uint64_t

  /**
   * X acceleration in body frame
   * Units: m/s/s
   */
  xAcc: float

  /**
   * Y acceleration in body frame
   * Units: m/s/s
   */
  yAcc: float

  /**
   * Z acceleration in body frame
   * Units: m/s/s
   */
  zAcc: float

  /**
   * X velocity in body frame
   * Units: m/s
   */
  xVel: float

  /**
   * Y velocity in body frame
   * Units: m/s
   */
  yVel: float

  /**
   * Z velocity in body frame
   * Units: m/s
   */
  zVel: float

  /**
   * X position in local frame
   * Units: m
   */
  xPos: float

  /**
   * Y position in local frame
   * Units: m
   */
  yPos: float

  /**
   * Z position in local frame
   * Units: m
   */
  zPos: float

  /**
   * Airspeed, set to -1 if unknown
   * Units: m/s
   */
  airspeed: float

  /**
   * Variance of body velocity estimate
   */
  velVariance: float[]

  /**
   * Variance in local position
   */
  posVariance: float[]

  /**
   * The attitude, represented as Quaternion
   */
  q: float[]

  /**
   * Angular rate in roll axis
   * Units: rad/s
   */
  rollRate: float

  /**
   * Angular rate in pitch axis
   * Units: rad/s
   */
  pitchRate: float

  /**
   * Angular rate in yaw axis
   * Units: rad/s
   */
  yawRate: float
}

/**
 * Battery information. Updates GCS with flight controller battery status. Smart batteries also use
 * this message, but may additionally send SMART_BATTERY_INFO.
 */
export class BatteryStatus extends MavLinkData {
  static MSG_ID = 147
  static MSG_NAME = 'BATTERY_STATUS'
  static PAYLOAD_LENGTH = 54
  static MAGIC_NUMBER = 154

  static FIELDS = [
    new MavLinkPacketField('current_consumed', 'currentConsumed', 0, false, 4, 'int32_t', 'mAh'),
    new MavLinkPacketField('energy_consumed', 'energyConsumed', 4, false, 4, 'int32_t', 'hJ'),
    new MavLinkPacketField('temperature', 'temperature', 8, false, 2, 'int16_t', 'cdegC'),
    new MavLinkPacketField('voltages', 'voltages', 10, false, 2, 'uint16_t[]', 'mV', 10),
    new MavLinkPacketField('current_battery', 'currentBattery', 30, false, 2, 'int16_t', 'cA'),
    new MavLinkPacketField('id', 'id', 32, false, 1, 'uint8_t', ''),
    new MavLinkPacketField('battery_function', 'batteryFunction', 33, false, 1, 'uint8_t', ''),
    new MavLinkPacketField('type', 'type', 34, false, 1, 'uint8_t', ''),
    new MavLinkPacketField('battery_remaining', 'batteryRemaining', 35, false, 1, 'int8_t', '%'),
    new MavLinkPacketField('time_remaining', 'timeRemaining', 36, true, 4, 'int32_t', 's'),
    new MavLinkPacketField('charge_state', 'chargeState', 40, true, 1, 'uint8_t', ''),
    new MavLinkPacketField('voltages_ext', 'voltagesExt', 41, true, 2, 'uint16_t[]', 'mV', 4),
    new MavLinkPacketField('mode', 'mode', 49, true, 1, 'uint8_t', ''),
    new MavLinkPacketField('fault_bitmask', 'faultBitmask', 50, true, 4, 'uint32_t', ''),
  ]

  constructor() {
    super()
    this.id = 0
    this.batteryFunction = MavBatteryFunction[Object.keys(MavBatteryFunction)[0]]
    this.type = MavBatteryType[Object.keys(MavBatteryType)[0]]
    this.temperature = 0
    this.voltages = []
    this.currentBattery = 0
    this.currentConsumed = 0
    this.energyConsumed = 0
    this.batteryRemaining = 0
    this.timeRemaining = 0
    this.chargeState = MavBatteryChargeState[Object.keys(MavBatteryChargeState)[0]]
    this.voltagesExt = []
    this.mode = MavBatteryMode[Object.keys(MavBatteryMode)[0]]
    this.faultBitmask = MavBatteryFault[Object.keys(MavBatteryFault)[0]]
  }

  /**
   * Battery ID
   */
  id: uint8_t

  /**
   * Function of the battery
   */
  batteryFunction: MavBatteryFunction

  /**
   * Type (chemistry) of the battery
   */
  type: MavBatteryType

  /**
   * Temperature of the battery. INT16_MAX for unknown temperature.
   * Units: cdegC
   */
  temperature: int16_t

  /**
   * Battery voltage of cells 1 to 10 (see voltages_ext for cells 11-14). Cells in this field above the
   * valid cell count for this battery should have the UINT16_MAX value. If individual cell voltages are
   * unknown or not measured for this battery, then the overall battery voltage should be filled in cell
   * 0, with all others set to UINT16_MAX. If the voltage of the battery is greater than (UINT16_MAX -
   * 1), then cell 0 should be set to (UINT16_MAX - 1), and cell 1 to the remaining voltage. This can be
   * extended to multiple cells if the total voltage is greater than 2 * (UINT16_MAX - 1).
   * Units: mV
   */
  voltages: uint16_t[]

  /**
   * Battery current, -1: autopilot does not measure the current
   * Units: cA
   */
  currentBattery: int16_t

  /**
   * Consumed charge, -1: autopilot does not provide consumption estimate
   * Units: mAh
   */
  currentConsumed: int32_t

  /**
   * Consumed energy, -1: autopilot does not provide energy consumption estimate
   * Units: hJ
   */
  energyConsumed: int32_t

  /**
   * Remaining battery energy. Values: [0-100], -1: autopilot does not estimate the remaining battery.
   * Units: %
   */
  batteryRemaining: int8_t

  /**
   * Remaining battery time, 0: autopilot does not provide remaining battery time estimate
   * Units: s
   */
  timeRemaining: int32_t

  /**
   * State for extent of discharge, provided by autopilot for warning or external reactions
   */
  chargeState: MavBatteryChargeState

  /**
   * Battery voltages for cells 11 to 14. Cells above the valid cell count for this battery should have a
   * value of 0, where zero indicates not supported (note, this is different than for the voltages field
   * and allows empty byte truncation). If the measured value is 0 then 1 should be sent instead.
   * Units: mV
   */
  voltagesExt: uint16_t[]

  /**
   * Battery mode. Default (0) is that battery mode reporting is not supported or battery is in
   * normal-use mode.
   */
  mode: MavBatteryMode

  /**
   * Fault/health indications. These should be set when charge_state is MAV_BATTERY_CHARGE_STATE_FAILED
   * or MAV_BATTERY_CHARGE_STATE_UNHEALTHY (if not, fault reporting is not supported).
   */
  faultBitmask: MavBatteryFault
}

/**
 * Version and capability of autopilot software. This should be emitted in response to a request with
 * MAV_CMD_REQUEST_MESSAGE.
 */
export class AutopilotVersion extends MavLinkData {
  static MSG_ID = 148
  static MSG_NAME = 'AUTOPILOT_VERSION'
  static PAYLOAD_LENGTH = 78
  static MAGIC_NUMBER = 178

  static FIELDS = [
    new MavLinkPacketField('capabilities', 'capabilities', 0, false, 8, 'uint64_t', ''),
    new MavLinkPacketField('uid', 'uid', 8, false, 8, 'uint64_t', ''),
    new MavLinkPacketField('flight_sw_version', 'flightSwVersion', 16, false, 4, 'uint32_t', ''),
    new MavLinkPacketField('middleware_sw_version', 'middlewareSwVersion', 20, false, 4, 'uint32_t', ''),
    new MavLinkPacketField('os_sw_version', 'osSwVersion', 24, false, 4, 'uint32_t', ''),
    new MavLinkPacketField('board_version', 'boardVersion', 28, false, 4, 'uint32_t', ''),
    new MavLinkPacketField('vendor_id', 'vendorId', 32, false, 2, 'uint16_t', ''),
    new MavLinkPacketField('product_id', 'productId', 34, false, 2, 'uint16_t', ''),
    new MavLinkPacketField('flight_custom_version', 'flightCustomVersion', 36, false, 1, 'uint8_t[]', '', 8),
    new MavLinkPacketField('middleware_custom_version', 'middlewareCustomVersion', 44, false, 1, 'uint8_t[]', '', 8),
    new MavLinkPacketField('os_custom_version', 'osCustomVersion', 52, false, 1, 'uint8_t[]', '', 8),
    new MavLinkPacketField('uid2', 'uid2', 60, true, 1, 'uint8_t[]', '', 18),
  ]

  constructor() {
    super()
    this.capabilities = MavProtocolCapability[Object.keys(MavProtocolCapability)[0]]
    this.flightSwVersion = 0
    this.middlewareSwVersion = 0
    this.osSwVersion = 0
    this.boardVersion = 0
    this.flightCustomVersion = []
    this.middlewareCustomVersion = []
    this.osCustomVersion = []
    this.vendorId = 0
    this.productId = 0
    this.uid = BigInt(0)
    this.uid2 = []
  }

  /**
   * Bitmap of capabilities
   */
  capabilities: MavProtocolCapability

  /**
   * Firmware version number
   */
  flightSwVersion: uint32_t

  /**
   * Middleware version number
   */
  middlewareSwVersion: uint32_t

  /**
   * Operating system version number
   */
  osSwVersion: uint32_t

  /**
   * HW / board version (last 8 bits should be silicon ID, if any). The first 16 bits of this field
   * specify https://github.com/PX4/PX4-Bootloader/blob/master/board_types.txt
   */
  boardVersion: uint32_t

  /**
   * Custom version field, commonly the first 8 bytes of the git hash. This is not an unique identifier,
   * but should allow to identify the commit using the main version number even for very large code
   * bases.
   */
  flightCustomVersion: uint8_t[]

  /**
   * Custom version field, commonly the first 8 bytes of the git hash. This is not an unique identifier,
   * but should allow to identify the commit using the main version number even for very large code
   * bases.
   */
  middlewareCustomVersion: uint8_t[]

  /**
   * Custom version field, commonly the first 8 bytes of the git hash. This is not an unique identifier,
   * but should allow to identify the commit using the main version number even for very large code
   * bases.
   */
  osCustomVersion: uint8_t[]

  /**
   * ID of the board vendor
   */
  vendorId: uint16_t

  /**
   * ID of the product
   */
  productId: uint16_t

  /**
   * UID if provided by hardware (see uid2)
   */
  uid: uint64_t

  /**
   * UID if provided by hardware (supersedes the uid field. If this is non-zero, use this field,
   * otherwise use uid)
   */
  uid2: uint8_t[]
}

/**
 * The location of a landing target. See: https://mavlink.io/en/services/landing_target.html
 */
export class LandingTarget extends MavLinkData {
  static MSG_ID = 149
  static MSG_NAME = 'LANDING_TARGET'
  static PAYLOAD_LENGTH = 60
  static MAGIC_NUMBER = 200

  static FIELDS = [
    new MavLinkPacketField('time_usec', 'timeUsec', 0, false, 8, 'uint64_t', 'us'),
    new MavLinkPacketField('angle_x', 'angleX', 8, false, 4, 'float', 'rad'),
    new MavLinkPacketField('angle_y', 'angleY', 12, false, 4, 'float', 'rad'),
    new MavLinkPacketField('distance', 'distance', 16, false, 4, 'float', 'm'),
    new MavLinkPacketField('size_x', 'sizeX', 20, false, 4, 'float', 'rad'),
    new MavLinkPacketField('size_y', 'sizeY', 24, false, 4, 'float', 'rad'),
    new MavLinkPacketField('target_num', 'targetNum', 28, false, 1, 'uint8_t', ''),
    new MavLinkPacketField('frame', 'frame', 29, false, 1, 'uint8_t', ''),
    new MavLinkPacketField('x', 'x', 30, true, 4, 'float', 'm'),
    new MavLinkPacketField('y', 'y', 34, true, 4, 'float', 'm'),
    new MavLinkPacketField('z', 'z', 38, true, 4, 'float', 'm'),
    new MavLinkPacketField('q', 'q', 42, true, 4, 'float[]', '', 4),
    new MavLinkPacketField('type', 'type', 58, true, 1, 'uint8_t', ''),
    new MavLinkPacketField('position_valid', 'positionValid', 59, true, 1, 'uint8_t', ''),
  ]

  constructor() {
    super()
    this.timeUsec = BigInt(0)
    this.targetNum = 0
    this.frame = MavFrame[Object.keys(MavFrame)[0]]
    this.angleX = 0
    this.angleY = 0
    this.distance = 0
    this.sizeX = 0
    this.sizeY = 0
    this.x = 0
    this.y = 0
    this.z = 0
    this.q = []
    this.type = LandingTargetType[Object.keys(LandingTargetType)[0]]
    this.positionValid = 0
  }

  /**
   * Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format
   * (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
   * Units: us
   */
  timeUsec: uint64_t

  /**
   * The ID of the target if multiple targets are present
   */
  targetNum: uint8_t

  /**
   * Coordinate frame used for following fields.
   */
  frame: MavFrame

  /**
   * X-axis angular offset of the target from the center of the image
   * Units: rad
   */
  angleX: float

  /**
   * Y-axis angular offset of the target from the center of the image
   * Units: rad
   */
  angleY: float

  /**
   * Distance to the target from the vehicle
   * Units: m
   */
  distance: float

  /**
   * Size of target along x-axis
   * Units: rad
   */
  sizeX: float

  /**
   * Size of target along y-axis
   * Units: rad
   */
  sizeY: float

  /**
   * X Position of the landing target in MAV_FRAME
   * Units: m
   */
  x: float

  /**
   * Y Position of the landing target in MAV_FRAME
   * Units: m
   */
  y: float

  /**
   * Z Position of the landing target in MAV_FRAME
   * Units: m
   */
  z: float

  /**
   * Quaternion of landing target orientation (w, x, y, z order, zero-rotation is 1, 0, 0, 0)
   */
  q: float[]

  /**
   * Type of landing target
   */
  type: LandingTargetType

  /**
   * Boolean indicating whether the position fields (x, y, z, q, type) contain valid target position
   * information (valid: 1, invalid: 0). Default is 0 (invalid).
   */
  positionValid: uint8_t
}

/**
 * Status of geo-fencing. Sent in extended status stream when fencing enabled.
 */
export class FenceStatus extends MavLinkData {
  static MSG_ID = 162
  static MSG_NAME = 'FENCE_STATUS'
  static PAYLOAD_LENGTH = 9
  static MAGIC_NUMBER = 189

  static FIELDS = [
    new MavLinkPacketField('breach_time', 'breachTime', 0, false, 4, 'uint32_t', 'ms'),
    new MavLinkPacketField('breach_count', 'breachCount', 4, false, 2, 'uint16_t', ''),
    new MavLinkPacketField('breach_status', 'breachStatus', 6, false, 1, 'uint8_t', ''),
    new MavLinkPacketField('breach_type', 'breachType', 7, false, 1, 'uint8_t', ''),
    new MavLinkPacketField('breach_mitigation', 'breachMitigation', 8, true, 1, 'uint8_t', ''),
  ]

  constructor() {
    super()
    this.breachStatus = 0
    this.breachCount = 0
    this.breachType = FenceBreach[Object.keys(FenceBreach)[0]]
    this.breachTime = 0
    this.breachMitigation = FenceMitigate[Object.keys(FenceMitigate)[0]]
  }

  /**
   * Breach status (0 if currently inside fence, 1 if outside).
   */
  breachStatus: uint8_t

  /**
   * Number of fence breaches.
   */
  breachCount: uint16_t

  /**
   * Last breach type.
   */
  breachType: FenceBreach

  /**
   * Time (since boot) of last breach.
   * Units: ms
   */
  breachTime: uint32_t

  /**
   * Active action to prevent fence breach
   */
  breachMitigation: FenceMitigate
}

/**
 * Reports results of completed compass calibration. Sent until MAG_CAL_ACK received.
 */
export class MagCalReport extends MavLinkData {
  static MSG_ID = 192
  static MSG_NAME = 'MAG_CAL_REPORT'
  static PAYLOAD_LENGTH = 54
  static MAGIC_NUMBER = 36

  static FIELDS = [
    new MavLinkPacketField('fitness', 'fitness', 0, false, 4, 'float', 'mgauss'),
    new MavLinkPacketField('ofs_x', 'ofsX', 4, false, 4, 'float', ''),
    new MavLinkPacketField('ofs_y', 'ofsY', 8, false, 4, 'float', ''),
    new MavLinkPacketField('ofs_z', 'ofsZ', 12, false, 4, 'float', ''),
    new MavLinkPacketField('diag_x', 'diagX', 16, false, 4, 'float', ''),
    new MavLinkPacketField('diag_y', 'diagY', 20, false, 4, 'float', ''),
    new MavLinkPacketField('diag_z', 'diagZ', 24, false, 4, 'float', ''),
    new MavLinkPacketField('offdiag_x', 'offdiagX', 28, false, 4, 'float', ''),
    new MavLinkPacketField('offdiag_y', 'offdiagY', 32, false, 4, 'float', ''),
    new MavLinkPacketField('offdiag_z', 'offdiagZ', 36, false, 4, 'float', ''),
    new MavLinkPacketField('compass_id', 'compassId', 40, false, 1, 'uint8_t', ''),
    new MavLinkPacketField('cal_mask', 'calMask', 41, false, 1, 'uint8_t', ''),
    new MavLinkPacketField('cal_status', 'calStatus', 42, false, 1, 'uint8_t', ''),
    new MavLinkPacketField('autosaved', 'autosaved', 43, false, 1, 'uint8_t', ''),
    new MavLinkPacketField('orientation_confidence', 'orientationConfidence', 44, true, 4, 'float', ''),
    new MavLinkPacketField('old_orientation', 'oldOrientation', 48, true, 1, 'uint8_t', ''),
    new MavLinkPacketField('new_orientation', 'newOrientation', 49, true, 1, 'uint8_t', ''),
    new MavLinkPacketField('scale_factor', 'scaleFactor', 50, true, 4, 'float', ''),
  ]

  constructor() {
    super()
    this.compassId = 0
    this.calMask = 0
    this.calStatus = MagCalStatus[Object.keys(MagCalStatus)[0]]
    this.autosaved = 0
    this.fitness = 0
    this.ofsX = 0
    this.ofsY = 0
    this.ofsZ = 0
    this.diagX = 0
    this.diagY = 0
    this.diagZ = 0
    this.offdiagX = 0
    this.offdiagY = 0
    this.offdiagZ = 0
    this.orientationConfidence = 0
    this.oldOrientation = MavSensorOrientation[Object.keys(MavSensorOrientation)[0]]
    this.newOrientation = MavSensorOrientation[Object.keys(MavSensorOrientation)[0]]
    this.scaleFactor = 0
  }

  /**
   * Compass being calibrated.
   */
  compassId: uint8_t

  /**
   * Bitmask of compasses being calibrated.
   */
  calMask: uint8_t

  /**
   * Calibration Status.
   */
  calStatus: MagCalStatus

  /**
   * 0=requires a MAV_CMD_DO_ACCEPT_MAG_CAL, 1=saved to parameters.
   */
  autosaved: uint8_t

  /**
   * RMS milligauss residuals.
   * Units: mgauss
   */
  fitness: float

  /**
   * X offset.
   */
  ofsX: float

  /**
   * Y offset.
   */
  ofsY: float

  /**
   * Z offset.
   */
  ofsZ: float

  /**
   * X diagonal (matrix 11).
   */
  diagX: float

  /**
   * Y diagonal (matrix 22).
   */
  diagY: float

  /**
   * Z diagonal (matrix 33).
   */
  diagZ: float

  /**
   * X off-diagonal (matrix 12 and 21).
   */
  offdiagX: float

  /**
   * Y off-diagonal (matrix 13 and 31).
   */
  offdiagY: float

  /**
   * Z off-diagonal (matrix 32 and 23).
   */
  offdiagZ: float

  /**
   * Confidence in orientation (higher is better).
   */
  orientationConfidence: float

  /**
   * orientation before calibration.
   */
  oldOrientation: MavSensorOrientation

  /**
   * orientation after calibration.
   */
  newOrientation: MavSensorOrientation

  /**
   * field radius correction factor
   */
  scaleFactor: float
}

/**
 * EFI status output
 */
export class EfiStatus extends MavLinkData {
  static MSG_ID = 225
  static MSG_NAME = 'EFI_STATUS'
  static PAYLOAD_LENGTH = 65
  static MAGIC_NUMBER = 208

  static FIELDS = [
    new MavLinkPacketField('ecu_index', 'ecuIndex', 0, false, 4, 'float', ''),
    new MavLinkPacketField('rpm', 'rpm', 4, false, 4, 'float', ''),
    new MavLinkPacketField('fuel_consumed', 'fuelConsumed', 8, false, 4, 'float', 'cm^3'),
    new MavLinkPacketField('fuel_flow', 'fuelFlow', 12, false, 4, 'float', 'cm^3/min'),
    new MavLinkPacketField('engine_load', 'engineLoad', 16, false, 4, 'float', '%'),
    new MavLinkPacketField('throttle_position', 'throttlePosition', 20, false, 4, 'float', '%'),
    new MavLinkPacketField('spark_dwell_time', 'sparkDwellTime', 24, false, 4, 'float', 'ms'),
    new MavLinkPacketField('barometric_pressure', 'barometricPressure', 28, false, 4, 'float', 'kPa'),
    new MavLinkPacketField('intake_manifold_pressure', 'intakeManifoldPressure', 32, false, 4, 'float', 'kPa'),
    new MavLinkPacketField('intake_manifold_temperature', 'intakeManifoldTemperature', 36, false, 4, 'float', 'degC'),
    new MavLinkPacketField('cylinder_head_temperature', 'cylinderHeadTemperature', 40, false, 4, 'float', 'degC'),
    new MavLinkPacketField('ignition_timing', 'ignitionTiming', 44, false, 4, 'float', 'deg'),
    new MavLinkPacketField('injection_time', 'injectionTime', 48, false, 4, 'float', 'ms'),
    new MavLinkPacketField('exhaust_gas_temperature', 'exhaustGasTemperature', 52, false, 4, 'float', 'degC'),
    new MavLinkPacketField('throttle_out', 'throttleOut', 56, false, 4, 'float', '%'),
    new MavLinkPacketField('pt_compensation', 'ptCompensation', 60, false, 4, 'float', ''),
    new MavLinkPacketField('health', 'health', 64, false, 1, 'uint8_t', ''),
  ]

  constructor() {
    super()
    this.health = 0
    this.ecuIndex = 0
    this.rpm = 0
    this.fuelConsumed = 0
    this.fuelFlow = 0
    this.engineLoad = 0
    this.throttlePosition = 0
    this.sparkDwellTime = 0
    this.barometricPressure = 0
    this.intakeManifoldPressure = 0
    this.intakeManifoldTemperature = 0
    this.cylinderHeadTemperature = 0
    this.ignitionTiming = 0
    this.injectionTime = 0
    this.exhaustGasTemperature = 0
    this.throttleOut = 0
    this.ptCompensation = 0
  }

  /**
   * EFI health status
   */
  health: uint8_t

  /**
   * ECU index
   */
  ecuIndex: float

  /**
   * RPM
   */
  rpm: float

  /**
   * Fuel consumed
   * Units: cm^3
   */
  fuelConsumed: float

  /**
   * Fuel flow rate
   * Units: cm^3/min
   */
  fuelFlow: float

  /**
   * Engine load
   * Units: %
   */
  engineLoad: float

  /**
   * Throttle position
   * Units: %
   */
  throttlePosition: float

  /**
   * Spark dwell time
   * Units: ms
   */
  sparkDwellTime: float

  /**
   * Barometric pressure
   * Units: kPa
   */
  barometricPressure: float

  /**
   * Intake manifold pressure(
   * Units: kPa
   */
  intakeManifoldPressure: float

  /**
   * Intake manifold temperature
   * Units: degC
   */
  intakeManifoldTemperature: float

  /**
   * Cylinder head temperature
   * Units: degC
   */
  cylinderHeadTemperature: float

  /**
   * Ignition timing (Crank angle degrees)
   * Units: deg
   */
  ignitionTiming: float

  /**
   * Injection time
   * Units: ms
   */
  injectionTime: float

  /**
   * Exhaust gas temperature
   * Units: degC
   */
  exhaustGasTemperature: float

  /**
   * Output throttle
   * Units: %
   */
  throttleOut: float

  /**
   * Pressure/temperature compensation
   */
  ptCompensation: float
}

/**
 * A rally point. Used to set a point when from GCS -> MAV. Also used to return a point from MAV ->
 * GCS.
 */
export class RallyPoint extends MavLinkData {
  static MSG_ID = 175
  static MSG_NAME = 'RALLY_POINT'
  static PAYLOAD_LENGTH = 19
  static MAGIC_NUMBER = 138

  static FIELDS = [
    new MavLinkPacketField('lat', 'lat', 0, false, 4, 'int32_t', 'degE7'),
    new MavLinkPacketField('lng', 'lng', 4, false, 4, 'int32_t', 'degE7'),
    new MavLinkPacketField('alt', 'alt', 8, false, 2, 'int16_t', 'm'),
    new MavLinkPacketField('break_alt', 'breakAlt', 10, false, 2, 'int16_t', 'm'),
    new MavLinkPacketField('land_dir', 'landDir', 12, false, 2, 'uint16_t', 'cdeg'),
    new MavLinkPacketField('target_system', 'targetSystem', 14, false, 1, 'uint8_t', ''),
    new MavLinkPacketField('target_component', 'targetComponent', 15, false, 1, 'uint8_t', ''),
    new MavLinkPacketField('idx', 'idx', 16, false, 1, 'uint8_t', ''),
    new MavLinkPacketField('count', 'count', 17, false, 1, 'uint8_t', ''),
    new MavLinkPacketField('flags', 'flags', 18, false, 1, 'uint8_t', ''),
  ]

  constructor() {
    super()
    this.targetSystem = 0
    this.targetComponent = 0
    this.idx = 0
    this.count = 0
    this.lat = 0
    this.lng = 0
    this.alt = 0
    this.breakAlt = 0
    this.landDir = 0
    this.flags = RallyFlags[Object.keys(RallyFlags)[0]]
  }

  /**
   * System ID.
   */
  targetSystem: uint8_t

  /**
   * Component ID.
   */
  targetComponent: uint8_t

  /**
   * Point index (first point is 0).
   */
  idx: uint8_t

  /**
   * Total number of points (for sanity checking).
   */
  count: uint8_t

  /**
   * Latitude of point.
   * Units: degE7
   */
  lat: int32_t

  /**
   * Longitude of point.
   * Units: degE7
   */
  lng: int32_t

  /**
   * Transit / loiter altitude relative to home.
   * Units: m
   */
  alt: int16_t

  /**
   * Break altitude relative to home.
   * Units: m
   */
  breakAlt: int16_t

  /**
   * Heading to aim for when landing.
   * Units: cdeg
   */
  landDir: uint16_t

  /**
   * Configuration flags.
   */
  flags: RallyFlags
}

/**
 * Request a current rally point from MAV. MAV should respond with a RALLY_POINT message. MAV should
 * not respond if the request is invalid.
 */
export class RallyFetchPoint extends MavLinkData {
  static MSG_ID = 176
  static MSG_NAME = 'RALLY_FETCH_POINT'
  static PAYLOAD_LENGTH = 3
  static MAGIC_NUMBER = 234

  static FIELDS = [
    new MavLinkPacketField('target_system', 'targetSystem', 0, false, 1, 'uint8_t', ''),
    new MavLinkPacketField('target_component', 'targetComponent', 1, false, 1, 'uint8_t', ''),
    new MavLinkPacketField('idx', 'idx', 2, false, 1, 'uint8_t', ''),
  ]

  constructor() {
    super()
    this.targetSystem = 0
    this.targetComponent = 0
    this.idx = 0
  }

  /**
   * System ID.
   */
  targetSystem: uint8_t

  /**
   * Component ID.
   */
  targetComponent: uint8_t

  /**
   * Point index (first point is 0).
   */
  idx: uint8_t
}

/**
 * Status of compassmot calibration.
 */
export class CompassMotStatus extends MavLinkData {
  static MSG_ID = 177
  static MSG_NAME = 'COMPASSMOT_STATUS'
  static PAYLOAD_LENGTH = 20
  static MAGIC_NUMBER = 240

  static FIELDS = [
    new MavLinkPacketField('current', 'current', 0, false, 4, 'float', 'A'),
    new MavLinkPacketField('CompensationX', 'CompensationX', 4, false, 4, 'float', ''),
    new MavLinkPacketField('CompensationY', 'CompensationY', 8, false, 4, 'float', ''),
    new MavLinkPacketField('CompensationZ', 'CompensationZ', 12, false, 4, 'float', ''),
    new MavLinkPacketField('throttle', 'throttle', 16, false, 2, 'uint16_t', 'd%'),
    new MavLinkPacketField('interference', 'interference', 18, false, 2, 'uint16_t', '%'),
  ]

  constructor() {
    super()
    this.throttle = 0
    this.current = 0
    this.interference = 0
    this.CompensationX = 0
    this.CompensationY = 0
    this.CompensationZ = 0
  }

  /**
   * Throttle.
   * Units: d%
   */
  throttle: uint16_t

  /**
   * Current.
   * Units: A
   */
  current: float

  /**
   * Interference.
   * Units: %
   */
  interference: uint16_t

  /**
   * Motor Compensation X.
   */
  CompensationX: float

  /**
   * Motor Compensation Y.
   */
  CompensationY: float

  /**
   * Motor Compensation Z.
   */
  CompensationZ: float
}

/**
 * Status of secondary AHRS filter if available.
 */
export class Ahrs2 extends MavLinkData {
  static MSG_ID = 178
  static MSG_NAME = 'AHRS2'
  static PAYLOAD_LENGTH = 24
  static MAGIC_NUMBER = 47

  static FIELDS = [
    new MavLinkPacketField('roll', 'roll', 0, false, 4, 'float', 'rad'),
    new MavLinkPacketField('pitch', 'pitch', 4, false, 4, 'float', 'rad'),
    new MavLinkPacketField('yaw', 'yaw', 8, false, 4, 'float', 'rad'),
    new MavLinkPacketField('altitude', 'altitude', 12, false, 4, 'float', 'm'),
    new MavLinkPacketField('lat', 'lat', 16, false, 4, 'int32_t', 'degE7'),
    new MavLinkPacketField('lng', 'lng', 20, false, 4, 'int32_t', 'degE7'),
  ]

  constructor() {
    super()
    this.roll = 0
    this.pitch = 0
    this.yaw = 0
    this.altitude = 0
    this.lat = 0
    this.lng = 0
  }

  /**
   * Roll angle.
   * Units: rad
   */
  roll: float

  /**
   * Pitch angle.
   * Units: rad
   */
  pitch: float

  /**
   * Yaw angle.
   * Units: rad
   */
  yaw: float

  /**
   * Altitude (MSL).
   * Units: m
   */
  altitude: float

  /**
   * Latitude.
   * Units: degE7
   */
  lat: int32_t

  /**
   * Longitude.
   * Units: degE7
   */
  lng: int32_t
}

/**
 * Camera Event.
 */
export class CameraStatus extends MavLinkData {
  static MSG_ID = 179
  static MSG_NAME = 'CAMERA_STATUS'
  static PAYLOAD_LENGTH = 29
  static MAGIC_NUMBER = 189

  static FIELDS = [
    new MavLinkPacketField('time_usec', 'timeUsec', 0, false, 8, 'uint64_t', 'us'),
    new MavLinkPacketField('p1', 'p1', 8, false, 4, 'float', ''),
    new MavLinkPacketField('p2', 'p2', 12, false, 4, 'float', ''),
    new MavLinkPacketField('p3', 'p3', 16, false, 4, 'float', ''),
    new MavLinkPacketField('p4', 'p4', 20, false, 4, 'float', ''),
    new MavLinkPacketField('img_idx', 'imgIdx', 24, false, 2, 'uint16_t', ''),
    new MavLinkPacketField('target_system', 'targetSystem', 26, false, 1, 'uint8_t', ''),
    new MavLinkPacketField('cam_idx', 'camIdx', 27, false, 1, 'uint8_t', ''),
    new MavLinkPacketField('event_id', 'eventId', 28, false, 1, 'uint8_t', ''),
  ]

  constructor() {
    super()
    this.timeUsec = BigInt(0)
    this.targetSystem = 0
    this.camIdx = 0
    this.imgIdx = 0
    this.eventId = CameraStatusTypes[Object.keys(CameraStatusTypes)[0]]
    this.p1 = 0
    this.p2 = 0
    this.p3 = 0
    this.p4 = 0
  }

  /**
   * Image timestamp (since UNIX epoch, according to camera clock).
   * Units: us
   */
  timeUsec: uint64_t

  /**
   * System ID.
   */
  targetSystem: uint8_t

  /**
   * Camera ID.
   */
  camIdx: uint8_t

  /**
   * Image index.
   */
  imgIdx: uint16_t

  /**
   * Event type.
   */
  eventId: CameraStatusTypes

  /**
   * Parameter 1 (meaning depends on event_id, see CAMERA_STATUS_TYPES enum).
   */
  p1: float

  /**
   * Parameter 2 (meaning depends on event_id, see CAMERA_STATUS_TYPES enum).
   */
  p2: float

  /**
   * Parameter 3 (meaning depends on event_id, see CAMERA_STATUS_TYPES enum).
   */
  p3: float

  /**
   * Parameter 4 (meaning depends on event_id, see CAMERA_STATUS_TYPES enum).
   */
  p4: float
}

/**
 * Camera Capture Feedback.
 */
export class CameraFeedback extends MavLinkData {
  static MSG_ID = 180
  static MSG_NAME = 'CAMERA_FEEDBACK'
  static PAYLOAD_LENGTH = 47
  static MAGIC_NUMBER = 52

  static FIELDS = [
    new MavLinkPacketField('time_usec', 'timeUsec', 0, false, 8, 'uint64_t', 'us'),
    new MavLinkPacketField('lat', 'lat', 8, false, 4, 'int32_t', 'degE7'),
    new MavLinkPacketField('lng', 'lng', 12, false, 4, 'int32_t', 'degE7'),
    new MavLinkPacketField('alt_msl', 'altMsl', 16, false, 4, 'float', 'm'),
    new MavLinkPacketField('alt_rel', 'altRel', 20, false, 4, 'float', 'm'),
    new MavLinkPacketField('roll', 'roll', 24, false, 4, 'float', 'deg'),
    new MavLinkPacketField('pitch', 'pitch', 28, false, 4, 'float', 'deg'),
    new MavLinkPacketField('yaw', 'yaw', 32, false, 4, 'float', 'deg'),
    new MavLinkPacketField('foc_len', 'focLen', 36, false, 4, 'float', 'mm'),
    new MavLinkPacketField('img_idx', 'imgIdx', 40, false, 2, 'uint16_t', ''),
    new MavLinkPacketField('target_system', 'targetSystem', 42, false, 1, 'uint8_t', ''),
    new MavLinkPacketField('cam_idx', 'camIdx', 43, false, 1, 'uint8_t', ''),
    new MavLinkPacketField('flags', 'flags', 44, false, 1, 'uint8_t', ''),
    new MavLinkPacketField('completed_captures', 'completedCaptures', 45, true, 2, 'uint16_t', ''),
  ]

  constructor() {
    super()
    this.timeUsec = BigInt(0)
    this.targetSystem = 0
    this.camIdx = 0
    this.imgIdx = 0
    this.lat = 0
    this.lng = 0
    this.altMsl = 0
    this.altRel = 0
    this.roll = 0
    this.pitch = 0
    this.yaw = 0
    this.focLen = 0
    this.flags = CameraFeedbackFlags[Object.keys(CameraFeedbackFlags)[0]]
    this.completedCaptures = 0
  }

  /**
   * Image timestamp (since UNIX epoch), as passed in by CAMERA_STATUS message (or autopilot if no CCB).
   * Units: us
   */
  timeUsec: uint64_t

  /**
   * System ID.
   */
  targetSystem: uint8_t

  /**
   * Camera ID.
   */
  camIdx: uint8_t

  /**
   * Image index.
   */
  imgIdx: uint16_t

  /**
   * Latitude.
   * Units: degE7
   */
  lat: int32_t

  /**
   * Longitude.
   * Units: degE7
   */
  lng: int32_t

  /**
   * Altitude (MSL).
   * Units: m
   */
  altMsl: float

  /**
   * Altitude (Relative to HOME location).
   * Units: m
   */
  altRel: float

  /**
   * Camera Roll angle (earth frame, +-180).
   * Units: deg
   */
  roll: float

  /**
   * Camera Pitch angle (earth frame, +-180).
   * Units: deg
   */
  pitch: float

  /**
   * Camera Yaw (earth frame, 0-360, true).
   * Units: deg
   */
  yaw: float

  /**
   * Focal Length.
   * Units: mm
   */
  focLen: float

  /**
   * Feedback flags.
   */
  flags: CameraFeedbackFlags

  /**
   * Completed image captures.
   */
  completedCaptures: uint16_t
}

/**
 * 2nd Battery status
 *
 * @deprecated since 2017-04, replaced by BATTERY_STATUS
 */
export class Battery2 extends MavLinkData {
  static MSG_ID = 181
  static MSG_NAME = 'BATTERY2'
  static PAYLOAD_LENGTH = 4
  static MAGIC_NUMBER = 174

  static FIELDS = [
    new MavLinkPacketField('voltage', 'voltage', 0, false, 2, 'uint16_t', 'mV'),
    new MavLinkPacketField('current_battery', 'currentBattery', 2, false, 2, 'int16_t', 'cA'),
  ]

  constructor() {
    super()
    this.voltage = 0
    this.currentBattery = 0
  }

  /**
   * Voltage.
   * Units: mV
   */
  voltage: uint16_t

  /**
   * Battery current, -1: autopilot does not measure the current.
   * Units: cA
   */
  currentBattery: int16_t
}

/**
 * Status of third AHRS filter if available. This is for ANU research group (Ali and Sean).
 */
export class Ahrs3 extends MavLinkData {
  static MSG_ID = 182
  static MSG_NAME = 'AHRS3'
  static PAYLOAD_LENGTH = 40
  static MAGIC_NUMBER = 229

  static FIELDS = [
    new MavLinkPacketField('roll', 'roll', 0, false, 4, 'float', 'rad'),
    new MavLinkPacketField('pitch', 'pitch', 4, false, 4, 'float', 'rad'),
    new MavLinkPacketField('yaw', 'yaw', 8, false, 4, 'float', 'rad'),
    new MavLinkPacketField('altitude', 'altitude', 12, false, 4, 'float', 'm'),
    new MavLinkPacketField('lat', 'lat', 16, false, 4, 'int32_t', 'degE7'),
    new MavLinkPacketField('lng', 'lng', 20, false, 4, 'int32_t', 'degE7'),
    new MavLinkPacketField('v1', 'v1', 24, false, 4, 'float', ''),
    new MavLinkPacketField('v2', 'v2', 28, false, 4, 'float', ''),
    new MavLinkPacketField('v3', 'v3', 32, false, 4, 'float', ''),
    new MavLinkPacketField('v4', 'v4', 36, false, 4, 'float', ''),
  ]

  constructor() {
    super()
    this.roll = 0
    this.pitch = 0
    this.yaw = 0
    this.altitude = 0
    this.lat = 0
    this.lng = 0
    this.v1 = 0
    this.v2 = 0
    this.v3 = 0
    this.v4 = 0
  }

  /**
   * Roll angle.
   * Units: rad
   */
  roll: float

  /**
   * Pitch angle.
   * Units: rad
   */
  pitch: float

  /**
   * Yaw angle.
   * Units: rad
   */
  yaw: float

  /**
   * Altitude (MSL).
   * Units: m
   */
  altitude: float

  /**
   * Latitude.
   * Units: degE7
   */
  lat: int32_t

  /**
   * Longitude.
   * Units: degE7
   */
  lng: int32_t

  /**
   * Test variable1.
   */
  v1: float

  /**
   * Test variable2.
   */
  v2: float

  /**
   * Test variable3.
   */
  v3: float

  /**
   * Test variable4.
   */
  v4: float
}

/**
 * Request the autopilot version from the system/component.
 */
export class AutopilotVersionRequest extends MavLinkData {
  static MSG_ID = 183
  static MSG_NAME = 'AUTOPILOT_VERSION_REQUEST'
  static PAYLOAD_LENGTH = 2
  static MAGIC_NUMBER = 85

  static FIELDS = [
    new MavLinkPacketField('target_system', 'targetSystem', 0, false, 1, 'uint8_t', ''),
    new MavLinkPacketField('target_component', 'targetComponent', 1, false, 1, 'uint8_t', ''),
  ]

  constructor() {
    super()
    this.targetSystem = 0
    this.targetComponent = 0
  }

  /**
   * System ID.
   */
  targetSystem: uint8_t

  /**
   * Component ID.
   */
  targetComponent: uint8_t
}

/**
 * Send a block of log data to remote location.
 */
export class RemoteLogDataBlock extends MavLinkData {
  static MSG_ID = 184
  static MSG_NAME = 'REMOTE_LOG_DATA_BLOCK'
  static PAYLOAD_LENGTH = 206
  static MAGIC_NUMBER = 159

  static FIELDS = [
    new MavLinkPacketField('seqno', 'seqno', 0, false, 4, 'uint32_t', ''),
    new MavLinkPacketField('target_system', 'targetSystem', 4, false, 1, 'uint8_t', ''),
    new MavLinkPacketField('target_component', 'targetComponent', 5, false, 1, 'uint8_t', ''),
    new MavLinkPacketField('data', 'data', 6, false, 1, 'uint8_t[]', '', 200),
  ]

  constructor() {
    super()
    this.targetSystem = 0
    this.targetComponent = 0
    this.seqno = MavRemoteLogDataBlockCommands[Object.keys(MavRemoteLogDataBlockCommands)[0]]
    this.data = []
  }

  /**
   * System ID.
   */
  targetSystem: uint8_t

  /**
   * Component ID.
   */
  targetComponent: uint8_t

  /**
   * Log data block sequence number.
   */
  seqno: MavRemoteLogDataBlockCommands

  /**
   * Log data block.
   */
  data: uint8_t[]
}

/**
 * Send Status of each log block that autopilot board might have sent.
 */
export class RemoteLogBlockStatus extends MavLinkData {
  static MSG_ID = 185
  static MSG_NAME = 'REMOTE_LOG_BLOCK_STATUS'
  static PAYLOAD_LENGTH = 7
  static MAGIC_NUMBER = 186

  static FIELDS = [
    new MavLinkPacketField('seqno', 'seqno', 0, false, 4, 'uint32_t', ''),
    new MavLinkPacketField('target_system', 'targetSystem', 4, false, 1, 'uint8_t', ''),
    new MavLinkPacketField('target_component', 'targetComponent', 5, false, 1, 'uint8_t', ''),
    new MavLinkPacketField('status', 'status', 6, false, 1, 'uint8_t', ''),
  ]

  constructor() {
    super()
    this.targetSystem = 0
    this.targetComponent = 0
    this.seqno = 0
    this.status = MavRemoteLogDataBlockStatuses[Object.keys(MavRemoteLogDataBlockStatuses)[0]]
  }

  /**
   * System ID.
   */
  targetSystem: uint8_t

  /**
   * Component ID.
   */
  targetComponent: uint8_t

  /**
   * Log data block sequence number.
   */
  seqno: uint32_t

  /**
   * Log data block status.
   */
  status: MavRemoteLogDataBlockStatuses
}

/**
 * Control vehicle LEDs.
 */
export class LedControl extends MavLinkData {
  static MSG_ID = 186
  static MSG_NAME = 'LED_CONTROL'
  static PAYLOAD_LENGTH = 29
  static MAGIC_NUMBER = 72

  static FIELDS = [
    new MavLinkPacketField('target_system', 'targetSystem', 0, false, 1, 'uint8_t', ''),
    new MavLinkPacketField('target_component', 'targetComponent', 1, false, 1, 'uint8_t', ''),
    new MavLinkPacketField('instance', 'instance', 2, false, 1, 'uint8_t', ''),
    new MavLinkPacketField('pattern', 'pattern', 3, false, 1, 'uint8_t', ''),
    new MavLinkPacketField('custom_len', 'customLen', 4, false, 1, 'uint8_t', ''),
    new MavLinkPacketField('custom_bytes', 'customBytes', 5, false, 1, 'uint8_t[]', '', 24),
  ]

  constructor() {
    super()
    this.targetSystem = 0
    this.targetComponent = 0
    this.instance = 0
    this.pattern = 0
    this.customLen = 0
    this.customBytes = []
  }

  /**
   * System ID.
   */
  targetSystem: uint8_t

  /**
   * Component ID.
   */
  targetComponent: uint8_t

  /**
   * Instance (LED instance to control or 255 for all LEDs).
   */
  instance: uint8_t

  /**
   * Pattern (see LED_PATTERN_ENUM).
   */
  pattern: uint8_t

  /**
   * Custom Byte Length.
   */
  customLen: uint8_t

  /**
   * Custom Bytes.
   */
  customBytes: uint8_t[]
}

/**
 * ACFly HTL IMU Sensor Info
 */
export class AcflyhtlPwmchansFb extends MavLinkData {
  static MSG_ID = 201
  static MSG_NAME = 'ACFlyHTL_PWMCHANS_FB'
  static PAYLOAD_LENGTH = 64
  static MAGIC_NUMBER = 33

  static FIELDS = [
    new MavLinkPacketField('pwms', 'pwms', 0, false, 2, 'uint16_t[]', '', 32),
  ]

  constructor() {
    super()
    this.pwms = []
  }

  /**
   * Pwms in us.
   */
  pwms: uint16_t[]
}

/**
 * ACFly HTL IMU Sensor Info
 */
export class AcflyhtlImuRegister extends MavLinkData {
  static MSG_ID = 202
  static MSG_NAME = 'ACFlyHTL_IMU_REGISTER'
  static PAYLOAD_LENGTH = 30
  static MAGIC_NUMBER = 243

  static FIELDS = [
    new MavLinkPacketField('sensitivity', 'sensitivity', 0, false, 8, 'double', ''),
    new MavLinkPacketField('imu_type', 'imuType', 8, false, 2, 'uint16_t', ''),
    new MavLinkPacketField('freq', 'freq', 10, false, 2, 'uint16_t', ''),
    new MavLinkPacketField('target_system', 'targetSystem', 12, false, 1, 'uint8_t', ''),
    new MavLinkPacketField('target_component', 'targetComponent', 13, false, 1, 'uint8_t', ''),
    new MavLinkPacketField('sensor_name', 'sensorName', 14, false, 1, 'char[]', '', 16),
  ]

  constructor() {
    super()
    this.targetSystem = 0
    this.targetComponent = 0
    this.imuType = AcflyImuFlags[Object.keys(AcflyImuFlags)[0]]
    this.freq = 0
    this.sensorName = ''
    this.sensitivity = 0
  }

  /**
   * System ID
   */
  targetSystem: uint8_t

  /**
   * Component ID
   */
  targetComponent: uint8_t

  /**
   * Imu sensor type
   */
  imuType: AcflyImuFlags

  /**
   * Sensor frequency(Hz in HTL time).
   */
  freq: uint16_t

  /**
   * position sensor name
   */
  sensorName: string

  /**
   * Sensor sensitivity
   */
  sensitivity: double
}

/**
 * ACFly HTL IMU Sensor Info
 */
export class AcflyhtlImuUpdate extends MavLinkData {
  static MSG_ID = 203
  static MSG_NAME = 'ACFlyHTL_IMU_UPDATE'
  static PAYLOAD_LENGTH = 16
  static MAGIC_NUMBER = 170

  static FIELDS = [
    new MavLinkPacketField('x', 'x', 0, false, 4, 'int32_t', ''),
    new MavLinkPacketField('y', 'y', 4, false, 4, 'int32_t', ''),
    new MavLinkPacketField('z', 'z', 8, false, 4, 'int32_t', ''),
    new MavLinkPacketField('imu_type', 'imuType', 12, false, 2, 'uint16_t', ''),
    new MavLinkPacketField('target_system', 'targetSystem', 14, false, 1, 'uint8_t', ''),
    new MavLinkPacketField('target_component', 'targetComponent', 15, false, 1, 'uint8_t', ''),
  ]

  constructor() {
    super()
    this.targetSystem = 0
    this.targetComponent = 0
    this.imuType = AcflyImuFlags[Object.keys(AcflyImuFlags)[0]]
    this.x = 0
    this.y = 0
    this.z = 0
  }

  /**
   * System ID
   */
  targetSystem: uint8_t

  /**
   * Component ID
   */
  targetComponent: uint8_t

  /**
   * Imu sensor type
   */
  imuType: AcflyImuFlags

  /**
   * Sensor x(front axis) data
   */
  x: int32_t

  /**
   * Sensor y(left axis) data
   */
  y: int32_t

  /**
   * Sensor z(up axis) data
   */
  z: int32_t
}

/**
 * ACFly Position Sensor Info
 */
export class AcflypossensorInfo extends MavLinkData {
  static MSG_ID = 206
  static MSG_NAME = 'ACFlyPosSensor_INFO'
  static PAYLOAD_LENGTH = 120
  static MAGIC_NUMBER = 232

  static FIELDS = [
    new MavLinkPacketField('time_usec', 'timeUsec', 0, false, 8, 'uint64_t', 'us'),
    new MavLinkPacketField('posE', 'posE', 8, false, 8, 'double', ''),
    new MavLinkPacketField('posN', 'posN', 16, false, 8, 'double', ''),
    new MavLinkPacketField('posU', 'posU', 24, false, 8, 'double', ''),
    new MavLinkPacketField('velE', 'velE', 32, false, 4, 'float', ''),
    new MavLinkPacketField('velN', 'velN', 36, false, 4, 'float', ''),
    new MavLinkPacketField('velU', 'velU', 40, false, 4, 'float', ''),
    new MavLinkPacketField('Lat', 'Lat', 44, false, 4, 'int32_t', ''),
    new MavLinkPacketField('Lon', 'Lon', 48, false, 4, 'int32_t', ''),
    new MavLinkPacketField('delay', 'delay', 52, false, 4, 'float', ''),
    new MavLinkPacketField('trustXY', 'trustXY', 56, false, 4, 'float', ''),
    new MavLinkPacketField('trustZ', 'trustZ', 60, false, 4, 'float', ''),
    new MavLinkPacketField('sensor_name', 'sensorName', 64, false, 1, 'char[]', '', 16),
    new MavLinkPacketField('ind', 'ind', 80, false, 1, 'int8_t', ''),
    new MavLinkPacketField('type', 'type', 81, false, 1, 'uint8_t', ''),
    new MavLinkPacketField('DataFrame', 'DataFrame', 82, false, 1, 'uint8_t', ''),
    new MavLinkPacketField('DataType', 'DataType', 83, false, 1, 'uint8_t', ''),
    new MavLinkPacketField('additionInfo1', 'additionInfo1', 84, true, 1, 'uint8_t[]', '', 4),
    new MavLinkPacketField('additionInfo2', 'additionInfo2', 88, true, 4, 'float[]', '', 8),
  ]

  constructor() {
    super()
    this.timeUsec = BigInt(0)
    this.sensorName = ''
    this.ind = 0
    this.type = AcflyPostypeFlags[Object.keys(AcflyPostypeFlags)[0]]
    this.DataFrame = 0
    this.DataType = AcflyPosdatatypeFlags[Object.keys(AcflyPosdatatypeFlags)[0]]
    this.posE = 0
    this.posN = 0
    this.posU = 0
    this.velE = 0
    this.velN = 0
    this.velU = 0
    this.Lat = 0
    this.Lon = 0
    this.delay = 0
    this.trustXY = 0
    this.trustZ = 0
    this.additionInfo1 = []
    this.additionInfo2 = []
  }

  /**
   * Timestamp. The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by
   * checking for the magnitude of the number.
   * Units: us
   */
  timeUsec: uint64_t

  /**
   * position sensor name
   */
  sensorName: string

  /**
   * Position sensor index.
   */
  ind: int8_t

  /**
   * Position sensor type.
   */
  type: AcflyPostypeFlags

  /**
   * Velocity data frame 0:ENU 1:bodyFLU bit7:sensor available.
   */
  DataFrame: uint8_t

  /**
   * Position sensor data type.
   */
  DataType: AcflyPosdatatypeFlags

  /**
   * Position East in meters
   */
  posE: double

  /**
   * Position North in meters
   */
  posN: double

  /**
   * Position Up in meters
   */
  posU: double

  /**
   * Velocity East in m/s
   */
  velE: float

  /**
   * Velocity North in m/s
   */
  velN: float

  /**
   * Velocity Up in m/s
   */
  velU: float

  /**
   * latitude 1e7
   */
  Lat: int32_t

  /**
   * lontitude 1e7
   */
  Lon: int32_t

  /**
   * Sensor observation delay in seconds.
   */
  delay: float

  /**
   * Sensor trust in meters in XY direction
   */
  trustXY: float

  /**
   * Sensor trust in meters in Z direction
   */
  trustZ: float

  /**
   * Addition info according to sensor type.
   */
  additionInfo1: uint8_t[]

  /**
   * Addition info according to sensor type.
   */
  additionInfo2: float[]
}

/**
 * ACFly Position Sensor Register
 */
export class AcflyRegeisterpossensor extends MavLinkData {
  static MSG_ID = 208
  static MSG_NAME = 'ACFly_RegeisterPosSensor'
  static PAYLOAD_LENGTH = 42
  static MAGIC_NUMBER = 175

  static FIELDS = [
    new MavLinkPacketField('delay', 'delay', 0, false, 4, 'float', ''),
    new MavLinkPacketField('trustXY', 'trustXY', 4, false, 4, 'float', ''),
    new MavLinkPacketField('trustZ', 'trustZ', 8, false, 4, 'float', ''),
    new MavLinkPacketField('LTTrustXY', 'LTTrustXY', 12, false, 4, 'float', ''),
    new MavLinkPacketField('LTTrustZ', 'LTTrustZ', 16, false, 4, 'float', ''),
    new MavLinkPacketField('target_system', 'targetSystem', 20, false, 1, 'uint8_t', ''),
    new MavLinkPacketField('target_component', 'targetComponent', 21, false, 1, 'uint8_t', ''),
    new MavLinkPacketField('sensor_name', 'sensorName', 22, false, 1, 'char[]', '', 16),
    new MavLinkPacketField('ind', 'ind', 38, false, 1, 'int8_t', ''),
    new MavLinkPacketField('type', 'type', 39, false, 1, 'uint8_t', ''),
    new MavLinkPacketField('DataFrame', 'DataFrame', 40, false, 1, 'uint8_t', ''),
    new MavLinkPacketField('DataType', 'DataType', 41, false, 1, 'uint8_t', ''),
  ]

  constructor() {
    super()
    this.targetSystem = 0
    this.targetComponent = 0
    this.sensorName = ''
    this.ind = 0
    this.type = AcflyPostypeFlags[Object.keys(AcflyPostypeFlags)[0]]
    this.DataFrame = 0
    this.DataType = AcflyPosdatatypeFlags[Object.keys(AcflyPosdatatypeFlags)[0]]
    this.delay = 0
    this.trustXY = 0
    this.trustZ = 0
    this.LTTrustXY = 0
    this.LTTrustZ = 0
  }

  /**
   * System ID
   */
  targetSystem: uint8_t

  /**
   * Component ID
   */
  targetComponent: uint8_t

  /**
   * position sensor name
   */
  sensorName: string

  /**
   * Position sensor index.
   */
  ind: int8_t

  /**
   * Position sensor type.
   */
  type: AcflyPostypeFlags

  /**
   * 0:position in ENU velocity in ENU 1:position in ENU velocity in bodyFLU 4:position in SLAM frame
   * velocity in ENU 5:position in SLAM frame velocity in bodyFLU bit7:sensor available. bit6:1 to
   * Unregister sensor.
   */
  DataFrame: uint8_t

  /**
   * Position sensor data type.
   */
  DataType: AcflyPosdatatypeFlags

  /**
   * Sensor observation delay in seconds.
   */
  delay: float

  /**
   * Sensor trust in meters in XY direction
   */
  trustXY: float

  /**
   * Sensor trust in meters in Z direction
   */
  trustZ: float

  /**
   * Sensor long-term trust in meters in XY direction
   */
  LTTrustXY: float

  /**
   * Sensor long-term trust in meters in Z direction
   */
  LTTrustZ: float
}

/**
 * ACFly Position Sensor Register
 */
export class AcflyUpdatepossensor extends MavLinkData {
  static MSG_ID = 209
  static MSG_NAME = 'ACFly_UpdatePosSensor'
  static PAYLOAD_LENGTH = 77
  static MAGIC_NUMBER = 202

  static FIELDS = [
    new MavLinkPacketField('posX', 'posX', 0, false, 8, 'double', ''),
    new MavLinkPacketField('posY', 'posY', 8, false, 8, 'double', ''),
    new MavLinkPacketField('posZ', 'posZ', 16, false, 8, 'double', ''),
    new MavLinkPacketField('velX', 'velX', 24, false, 4, 'float', ''),
    new MavLinkPacketField('velY', 'velY', 28, false, 4, 'float', ''),
    new MavLinkPacketField('velZ', 'velZ', 32, false, 4, 'float', ''),
    new MavLinkPacketField('delay', 'delay', 36, false, 4, 'float', ''),
    new MavLinkPacketField('trustXY', 'trustXY', 40, false, 4, 'float', ''),
    new MavLinkPacketField('trustZ', 'trustZ', 44, false, 4, 'float', ''),
    new MavLinkPacketField('LTTrustXY', 'LTTrustXY', 48, false, 4, 'float', ''),
    new MavLinkPacketField('LTTrustZ', 'LTTrustZ', 52, false, 4, 'float', ''),
    new MavLinkPacketField('target_system', 'targetSystem', 56, false, 1, 'uint8_t', ''),
    new MavLinkPacketField('target_component', 'targetComponent', 57, false, 1, 'uint8_t', ''),
    new MavLinkPacketField('ind', 'ind', 58, false, 1, 'int8_t', ''),
    new MavLinkPacketField('DataType', 'DataType', 59, false, 1, 'int8_t', ''),
    new MavLinkPacketField('reset', 'reset', 60, false, 1, 'uint8_t', ''),
    new MavLinkPacketField('AttQuat', 'AttQuat', 61, true, 4, 'float[]', '', 4),
  ]

  constructor() {
    super()
    this.targetSystem = 0
    this.targetComponent = 0
    this.ind = 0
    this.DataType = AcflyPosdatatypeFlags[Object.keys(AcflyPosdatatypeFlags)[0]]
    this.posX = 0
    this.posY = 0
    this.posZ = 0
    this.velX = 0
    this.velY = 0
    this.velZ = 0
    this.delay = 0
    this.trustXY = 0
    this.trustZ = 0
    this.LTTrustXY = 0
    this.LTTrustZ = 0
    this.reset = 0
    this.AttQuat = []
  }

  /**
   * System ID
   */
  targetSystem: uint8_t

  /**
   * Component ID
   */
  targetComponent: uint8_t

  /**
   * Position sensor index.
   */
  ind: int8_t

  /**
   * Position sensor data type. Set to -1 if value not changed.
   */
  DataType: AcflyPosdatatypeFlags

  /**
   * Position X in m or in Lat
   */
  posX: double

  /**
   * Position Y in m or in Lon
   */
  posY: double

  /**
   * Position Z in m
   */
  posZ: double

  /**
   * Velocity X in m/s
   */
  velX: float

  /**
   * Velocity Y in m/s
   */
  velY: float

  /**
   * Velocity Z in m/s
   */
  velZ: float

  /**
   * Sensor observation delay in seconds. Set to -1 if value not changed.
   */
  delay: float

  /**
   * Sensor trust in m in XY direction. Set to -1 if value not changed.
   */
  trustXY: float

  /**
   * Sensor trust in m in Z direction. Set to -1 if value not changed.
   */
  trustZ: float

  /**
   * Sensor long-term trust in meters in XY direction. Set to -1 if value not changed.
   */
  LTTrustXY: float

  /**
   * Sensor long-term trust in meters in Z direction. Set to -1 if value not changed.
   */
  LTTrustZ: float

  /**
   * bit7-0=available 1=not available bit6:0-Reset counter should be increased if sensor data jumped(SLAM
   * frame changed).
   */
  reset: uint8_t

  /**
   * SLAM attitude quaternion(Map to SLAM body, Axis Z must be upwords). Must be sent if position in SLAM
   * frame.
   */
  AttQuat: float[]
}

/**
 * Estimator status message including flags, innovation test ratios and estimated accuracies. The flags
 * message is an integer bitmask containing information on which EKF outputs are valid. See the
 * ESTIMATOR_STATUS_FLAGS enum definition for further information. The innovation test ratios show the
 * magnitude of the sensor innovation divided by the innovation check threshold. Under normal operation
 * the innovation test ratios should be below 0.5 with occasional values up to 1.0. Values greater than
 * 1.0 should be rare under normal operation and indicate that a measurement has been rejected by the
 * filter. The user should be notified if an innovation test ratio greater than 1.0 is recorded.
 * Notifications for values in the range between 0.5 and 1.0 should be optional and controllable by the
 * user.
 */
export class EstimatorStatus extends MavLinkData {
  static MSG_ID = 230
  static MSG_NAME = 'ESTIMATOR_STATUS'
  static PAYLOAD_LENGTH = 42
  static MAGIC_NUMBER = 163

  static FIELDS = [
    new MavLinkPacketField('time_usec', 'timeUsec', 0, false, 8, 'uint64_t', 'us'),
    new MavLinkPacketField('vel_ratio', 'velRatio', 8, false, 4, 'float', ''),
    new MavLinkPacketField('pos_horiz_ratio', 'posHorizRatio', 12, false, 4, 'float', ''),
    new MavLinkPacketField('pos_vert_ratio', 'posVertRatio', 16, false, 4, 'float', ''),
    new MavLinkPacketField('mag_ratio', 'magRatio', 20, false, 4, 'float', ''),
    new MavLinkPacketField('hagl_ratio', 'haglRatio', 24, false, 4, 'float', ''),
    new MavLinkPacketField('tas_ratio', 'tasRatio', 28, false, 4, 'float', ''),
    new MavLinkPacketField('pos_horiz_accuracy', 'posHorizAccuracy', 32, false, 4, 'float', 'm'),
    new MavLinkPacketField('pos_vert_accuracy', 'posVertAccuracy', 36, false, 4, 'float', 'm'),
    new MavLinkPacketField('flags', 'flags', 40, false, 2, 'uint16_t', ''),
  ]

  constructor() {
    super()
    this.timeUsec = BigInt(0)
    this.flags = EstimatorStatusFlags[Object.keys(EstimatorStatusFlags)[0]]
    this.velRatio = 0
    this.posHorizRatio = 0
    this.posVertRatio = 0
    this.magRatio = 0
    this.haglRatio = 0
    this.tasRatio = 0
    this.posHorizAccuracy = 0
    this.posVertAccuracy = 0
  }

  /**
   * Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format
   * (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
   * Units: us
   */
  timeUsec: uint64_t

  /**
   * Bitmap indicating which EKF outputs are valid.
   */
  flags: EstimatorStatusFlags

  /**
   * Velocity innovation test ratio
   */
  velRatio: float

  /**
   * Horizontal position innovation test ratio
   */
  posHorizRatio: float

  /**
   * Vertical position innovation test ratio
   */
  posVertRatio: float

  /**
   * Magnetometer innovation test ratio
   */
  magRatio: float

  /**
   * Height above terrain innovation test ratio
   */
  haglRatio: float

  /**
   * True airspeed innovation test ratio
   */
  tasRatio: float

  /**
   * Horizontal position 1-STD accuracy relative to the EKF local origin
   * Units: m
   */
  posHorizAccuracy: float

  /**
   * Vertical position 1-STD accuracy relative to the EKF local origin
   * Units: m
   */
  posVertAccuracy: float
}

/**
 * Wind covariance estimate from vehicle.
 */
export class WindCov extends MavLinkData {
  static MSG_ID = 231
  static MSG_NAME = 'WIND_COV'
  static PAYLOAD_LENGTH = 40
  static MAGIC_NUMBER = 105

  static FIELDS = [
    new MavLinkPacketField('time_usec', 'timeUsec', 0, false, 8, 'uint64_t', 'us'),
    new MavLinkPacketField('wind_x', 'windX', 8, false, 4, 'float', 'm/s'),
    new MavLinkPacketField('wind_y', 'windY', 12, false, 4, 'float', 'm/s'),
    new MavLinkPacketField('wind_z', 'windZ', 16, false, 4, 'float', 'm/s'),
    new MavLinkPacketField('var_horiz', 'varHoriz', 20, false, 4, 'float', 'm/s'),
    new MavLinkPacketField('var_vert', 'varVert', 24, false, 4, 'float', 'm/s'),
    new MavLinkPacketField('wind_alt', 'windAlt', 28, false, 4, 'float', 'm'),
    new MavLinkPacketField('horiz_accuracy', 'horizAccuracy', 32, false, 4, 'float', 'm'),
    new MavLinkPacketField('vert_accuracy', 'vertAccuracy', 36, false, 4, 'float', 'm'),
  ]

  constructor() {
    super()
    this.timeUsec = BigInt(0)
    this.windX = 0
    this.windY = 0
    this.windZ = 0
    this.varHoriz = 0
    this.varVert = 0
    this.windAlt = 0
    this.horizAccuracy = 0
    this.vertAccuracy = 0
  }

  /**
   * Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format
   * (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
   * Units: us
   */
  timeUsec: uint64_t

  /**
   * Wind in X (NED) direction
   * Units: m/s
   */
  windX: float

  /**
   * Wind in Y (NED) direction
   * Units: m/s
   */
  windY: float

  /**
   * Wind in Z (NED) direction
   * Units: m/s
   */
  windZ: float

  /**
   * Variability of the wind in XY. RMS of a 1 Hz lowpassed wind estimate.
   * Units: m/s
   */
  varHoriz: float

  /**
   * Variability of the wind in Z. RMS of a 1 Hz lowpassed wind estimate.
   * Units: m/s
   */
  varVert: float

  /**
   * Altitude (MSL) that this measurement was taken at
   * Units: m
   */
  windAlt: float

  /**
   * Horizontal speed 1-STD accuracy
   * Units: m
   */
  horizAccuracy: float

  /**
   * Vertical speed 1-STD accuracy
   * Units: m
   */
  vertAccuracy: float
}

/**
 * GPS sensor input message. This is a raw sensor value sent by the GPS. This is NOT the global
 * position estimate of the system.
 */
export class GpsInput extends MavLinkData {
  static MSG_ID = 232
  static MSG_NAME = 'GPS_INPUT'
  static PAYLOAD_LENGTH = 65
  static MAGIC_NUMBER = 151

  static FIELDS = [
    new MavLinkPacketField('time_usec', 'timeUsec', 0, false, 8, 'uint64_t', 'us'),
    new MavLinkPacketField('time_week_ms', 'timeWeekMs', 8, false, 4, 'uint32_t', 'ms'),
    new MavLinkPacketField('lat', 'lat', 12, false, 4, 'int32_t', 'degE7'),
    new MavLinkPacketField('lon', 'lon', 16, false, 4, 'int32_t', 'degE7'),
    new MavLinkPacketField('alt', 'alt', 20, false, 4, 'float', 'm'),
    new MavLinkPacketField('hdop', 'hdop', 24, false, 4, 'float', ''),
    new MavLinkPacketField('vdop', 'vdop', 28, false, 4, 'float', ''),
    new MavLinkPacketField('vn', 'vn', 32, false, 4, 'float', 'm/s'),
    new MavLinkPacketField('ve', 've', 36, false, 4, 'float', 'm/s'),
    new MavLinkPacketField('vd', 'vd', 40, false, 4, 'float', 'm/s'),
    new MavLinkPacketField('speed_accuracy', 'speedAccuracy', 44, false, 4, 'float', 'm/s'),
    new MavLinkPacketField('horiz_accuracy', 'horizAccuracy', 48, false, 4, 'float', 'm'),
    new MavLinkPacketField('vert_accuracy', 'vertAccuracy', 52, false, 4, 'float', 'm'),
    new MavLinkPacketField('ignore_flags', 'ignoreFlags', 56, false, 2, 'uint16_t', ''),
    new MavLinkPacketField('time_week', 'timeWeek', 58, false, 2, 'uint16_t', ''),
    new MavLinkPacketField('gps_id', 'gpsId', 60, false, 1, 'uint8_t', ''),
    new MavLinkPacketField('fix_type', 'fixType', 61, false, 1, 'uint8_t', ''),
    new MavLinkPacketField('satellites_visible', 'satellitesVisible', 62, false, 1, 'uint8_t', ''),
    new MavLinkPacketField('yaw', 'yaw', 63, true, 2, 'uint16_t', 'cdeg'),
  ]

  constructor() {
    super()
    this.timeUsec = BigInt(0)
    this.gpsId = 0
    this.ignoreFlags = GpsInputIgnoreFlags[Object.keys(GpsInputIgnoreFlags)[0]]
    this.timeWeekMs = 0
    this.timeWeek = 0
    this.fixType = 0
    this.lat = 0
    this.lon = 0
    this.alt = 0
    this.hdop = 0
    this.vdop = 0
    this.vn = 0
    this.ve = 0
    this.vd = 0
    this.speedAccuracy = 0
    this.horizAccuracy = 0
    this.vertAccuracy = 0
    this.satellitesVisible = 0
    this.yaw = 0
  }

  /**
   * Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format
   * (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
   * Units: us
   */
  timeUsec: uint64_t

  /**
   * ID of the GPS for multiple GPS inputs
   */
  gpsId: uint8_t

  /**
   * Bitmap indicating which GPS input flags fields to ignore. All other fields must be provided.
   */
  ignoreFlags: GpsInputIgnoreFlags

  /**
   * GPS time (from start of GPS week)
   * Units: ms
   */
  timeWeekMs: uint32_t

  /**
   * GPS week number
   */
  timeWeek: uint16_t

  /**
   * 0-1: no fix, 2: 2D fix, 3: 3D fix. 4: 3D with DGPS. 5: 3D with RTK
   */
  fixType: uint8_t

  /**
   * Latitude (WGS84)
   * Units: degE7
   */
  lat: int32_t

  /**
   * Longitude (WGS84)
   * Units: degE7
   */
  lon: int32_t

  /**
   * Altitude (MSL). Positive for up.
   * Units: m
   */
  alt: float

  /**
   * GPS HDOP horizontal dilution of position (unitless). If unknown, set to: UINT16_MAX
   */
  hdop: float

  /**
   * GPS VDOP vertical dilution of position (unitless). If unknown, set to: UINT16_MAX
   */
  vdop: float

  /**
   * GPS velocity in north direction in earth-fixed NED frame
   * Units: m/s
   */
  vn: float

  /**
   * GPS velocity in east direction in earth-fixed NED frame
   * Units: m/s
   */
  ve: float

  /**
   * GPS velocity in down direction in earth-fixed NED frame
   * Units: m/s
   */
  vd: float

  /**
   * GPS speed accuracy
   * Units: m/s
   */
  speedAccuracy: float

  /**
   * GPS horizontal accuracy
   * Units: m
   */
  horizAccuracy: float

  /**
   * GPS vertical accuracy
   * Units: m
   */
  vertAccuracy: float

  /**
   * Number of satellites visible.
   */
  satellitesVisible: uint8_t

  /**
   * Yaw of vehicle relative to Earth's North, zero means not available, use 36000 for north
   * Units: cdeg
   */
  yaw: uint16_t
}

/**
 * RTCM message for injecting into the onboard GPS (used for DGPS)
 */
export class GpsRtcmData extends MavLinkData {
  static MSG_ID = 233
  static MSG_NAME = 'GPS_RTCM_DATA'
  static PAYLOAD_LENGTH = 182
  static MAGIC_NUMBER = 35

  static FIELDS = [
    new MavLinkPacketField('flags', 'flags', 0, false, 1, 'uint8_t', ''),
    new MavLinkPacketField('len', 'len', 1, false, 1, 'uint8_t', 'bytes'),
    new MavLinkPacketField('data', 'data', 2, false, 1, 'uint8_t[]', '', 180),
  ]

  constructor() {
    super()
    this.flags = 0
    this.len = 0
    this.data = []
  }

  /**
   * LSB: 1 means message is fragmented, next 2 bits are the fragment ID, the remaining 5 bits are used
   * for the sequence ID. Messages are only to be flushed to the GPS when the entire message has been
   * reconstructed on the autopilot. The fragment ID specifies which order the fragments should be
   * assembled into a buffer, while the sequence ID is used to detect a mismatch between different
   * buffers. The buffer is considered fully reconstructed when either all 4 fragments are present, or
   * all the fragments before the first fragment with a non full payload is received. This management is
   * used to ensure that normal GPS operation doesn't corrupt RTCM data, and to recover from a unreliable
   * transport delivery order.
   */
  flags: uint8_t

  /**
   * data length
   * Units: bytes
   */
  len: uint8_t

  /**
   * RTCM message (may be fragmented)
   */
  data: uint8_t[]
}

/**
 * Message appropriate for high latency connections like Iridium
 *
 * @deprecated since 2020-10, replaced by HIGH_LATENCY2
 */
export class HighLatency extends MavLinkData {
  static MSG_ID = 234
  static MSG_NAME = 'HIGH_LATENCY'
  static PAYLOAD_LENGTH = 40
  static MAGIC_NUMBER = 150

  static FIELDS = [
    new MavLinkPacketField('custom_mode', 'customMode', 0, false, 4, 'uint32_t', ''),
    new MavLinkPacketField('latitude', 'latitude', 4, false, 4, 'int32_t', 'degE7'),
    new MavLinkPacketField('longitude', 'longitude', 8, false, 4, 'int32_t', 'degE7'),
    new MavLinkPacketField('roll', 'roll', 12, false, 2, 'int16_t', 'cdeg'),
    new MavLinkPacketField('pitch', 'pitch', 14, false, 2, 'int16_t', 'cdeg'),
    new MavLinkPacketField('heading', 'heading', 16, false, 2, 'uint16_t', 'cdeg'),
    new MavLinkPacketField('heading_sp', 'headingSp', 18, false, 2, 'int16_t', 'cdeg'),
    new MavLinkPacketField('altitude_amsl', 'altitudeAmsl', 20, false, 2, 'int16_t', 'm'),
    new MavLinkPacketField('altitude_sp', 'altitudeSp', 22, false, 2, 'int16_t', 'm'),
    new MavLinkPacketField('wp_distance', 'wpDistance', 24, false, 2, 'uint16_t', 'm'),
    new MavLinkPacketField('base_mode', 'baseMode', 26, false, 1, 'uint8_t', ''),
    new MavLinkPacketField('landed_state', 'landedState', 27, false, 1, 'uint8_t', ''),
    new MavLinkPacketField('throttle', 'throttle', 28, false, 1, 'int8_t', '%'),
    new MavLinkPacketField('airspeed', 'airspeed', 29, false, 1, 'uint8_t', 'm/s'),
    new MavLinkPacketField('airspeed_sp', 'airspeedSp', 30, false, 1, 'uint8_t', 'm/s'),
    new MavLinkPacketField('groundspeed', 'groundspeed', 31, false, 1, 'uint8_t', 'm/s'),
    new MavLinkPacketField('climb_rate', 'climbRate', 32, false, 1, 'int8_t', 'm/s'),
    new MavLinkPacketField('gps_nsat', 'gpsNsat', 33, false, 1, 'uint8_t', ''),
    new MavLinkPacketField('gps_fix_type', 'gpsFixType', 34, false, 1, 'uint8_t', ''),
    new MavLinkPacketField('battery_remaining', 'batteryRemaining', 35, false, 1, 'uint8_t', '%'),
    new MavLinkPacketField('temperature', 'temperature', 36, false, 1, 'int8_t', 'degC'),
    new MavLinkPacketField('temperature_air', 'temperatureAir', 37, false, 1, 'int8_t', 'degC'),
    new MavLinkPacketField('failsafe', 'failsafe', 38, false, 1, 'uint8_t', ''),
    new MavLinkPacketField('wp_num', 'wpNum', 39, false, 1, 'uint8_t', ''),
  ]

  constructor() {
    super()
    this.baseMode = MavModeFlag[Object.keys(MavModeFlag)[0]]
    this.customMode = 0
    this.landedState = MavLandedState[Object.keys(MavLandedState)[0]]
    this.roll = 0
    this.pitch = 0
    this.heading = 0
    this.throttle = 0
    this.headingSp = 0
    this.latitude = 0
    this.longitude = 0
    this.altitudeAmsl = 0
    this.altitudeSp = 0
    this.airspeed = 0
    this.airspeedSp = 0
    this.groundspeed = 0
    this.climbRate = 0
    this.gpsNsat = 0
    this.gpsFixType = GpsFixType[Object.keys(GpsFixType)[0]]
    this.batteryRemaining = 0
    this.temperature = 0
    this.temperatureAir = 0
    this.failsafe = 0
    this.wpNum = 0
    this.wpDistance = 0
  }

  /**
   * Bitmap of enabled system modes.
   */
  baseMode: MavModeFlag

  /**
   * A bitfield for use for autopilot-specific flags.
   */
  customMode: uint32_t

  /**
   * The landed state. Is set to MAV_LANDED_STATE_UNDEFINED if landed state is unknown.
   */
  landedState: MavLandedState

  /**
   * roll
   * Units: cdeg
   */
  roll: int16_t

  /**
   * pitch
   * Units: cdeg
   */
  pitch: int16_t

  /**
   * heading
   * Units: cdeg
   */
  heading: uint16_t

  /**
   * throttle (percentage)
   * Units: %
   */
  throttle: int8_t

  /**
   * heading setpoint
   * Units: cdeg
   */
  headingSp: int16_t

  /**
   * Latitude
   * Units: degE7
   */
  latitude: int32_t

  /**
   * Longitude
   * Units: degE7
   */
  longitude: int32_t

  /**
   * Altitude above mean sea level
   * Units: m
   */
  altitudeAmsl: int16_t

  /**
   * Altitude setpoint relative to the home position
   * Units: m
   */
  altitudeSp: int16_t

  /**
   * airspeed
   * Units: m/s
   */
  airspeed: uint8_t

  /**
   * airspeed setpoint
   * Units: m/s
   */
  airspeedSp: uint8_t

  /**
   * groundspeed
   * Units: m/s
   */
  groundspeed: uint8_t

  /**
   * climb rate
   * Units: m/s
   */
  climbRate: int8_t

  /**
   * Number of satellites visible. If unknown, set to UINT8_MAX
   */
  gpsNsat: uint8_t

  /**
   * GPS Fix type.
   */
  gpsFixType: GpsFixType

  /**
   * Remaining battery (percentage)
   * Units: %
   */
  batteryRemaining: uint8_t

  /**
   * Autopilot temperature (degrees C)
   * Units: degC
   */
  temperature: int8_t

  /**
   * Air temperature (degrees C) from airspeed sensor
   * Units: degC
   */
  temperatureAir: int8_t

  /**
   * failsafe (each bit represents a failsafe where 0=ok, 1=failsafe active (bit0:RC, bit1:batt,
   * bit2:GPS, bit3:GCS, bit4:fence)
   */
  failsafe: uint8_t

  /**
   * current waypoint number
   */
  wpNum: uint8_t

  /**
   * distance to target
   * Units: m
   */
  wpDistance: uint16_t
}

/**
 * Message appropriate for high latency connections like Iridium (version 2)
 */
export class HighLatency2 extends MavLinkData {
  static MSG_ID = 235
  static MSG_NAME = 'HIGH_LATENCY2'
  static PAYLOAD_LENGTH = 42
  static MAGIC_NUMBER = 179

  static FIELDS = [
    new MavLinkPacketField('timestamp', 'timestamp', 0, false, 4, 'uint32_t', 'ms'),
    new MavLinkPacketField('latitude', 'latitude', 4, false, 4, 'int32_t', 'degE7'),
    new MavLinkPacketField('longitude', 'longitude', 8, false, 4, 'int32_t', 'degE7'),
    new MavLinkPacketField('custom_mode', 'customMode', 12, false, 2, 'uint16_t', ''),
    new MavLinkPacketField('altitude', 'altitude', 14, false, 2, 'int16_t', 'm'),
    new MavLinkPacketField('target_altitude', 'targetAltitude', 16, false, 2, 'int16_t', 'm'),
    new MavLinkPacketField('target_distance', 'targetDistance', 18, false, 2, 'uint16_t', 'dam'),
    new MavLinkPacketField('wp_num', 'wpNum', 20, false, 2, 'uint16_t', ''),
    new MavLinkPacketField('failure_flags', 'failureFlags', 22, false, 2, 'uint16_t', ''),
    new MavLinkPacketField('type', 'type', 24, false, 1, 'uint8_t', ''),
    new MavLinkPacketField('autopilot', 'autopilot', 25, false, 1, 'uint8_t', ''),
    new MavLinkPacketField('heading', 'heading', 26, false, 1, 'uint8_t', 'deg/2'),
    new MavLinkPacketField('target_heading', 'targetHeading', 27, false, 1, 'uint8_t', 'deg/2'),
    new MavLinkPacketField('throttle', 'throttle', 28, false, 1, 'uint8_t', '%'),
    new MavLinkPacketField('airspeed', 'airspeed', 29, false, 1, 'uint8_t', 'm/s*5'),
    new MavLinkPacketField('airspeed_sp', 'airspeedSp', 30, false, 1, 'uint8_t', 'm/s*5'),
    new MavLinkPacketField('groundspeed', 'groundspeed', 31, false, 1, 'uint8_t', 'm/s*5'),
    new MavLinkPacketField('windspeed', 'windspeed', 32, false, 1, 'uint8_t', 'm/s*5'),
    new MavLinkPacketField('wind_heading', 'windHeading', 33, false, 1, 'uint8_t', 'deg/2'),
    new MavLinkPacketField('eph', 'eph', 34, false, 1, 'uint8_t', 'dm'),
    new MavLinkPacketField('epv', 'epv', 35, false, 1, 'uint8_t', 'dm'),
    new MavLinkPacketField('temperature_air', 'temperatureAir', 36, false, 1, 'int8_t', 'degC'),
    new MavLinkPacketField('climb_rate', 'climbRate', 37, false, 1, 'int8_t', 'dm/s'),
    new MavLinkPacketField('battery', 'battery', 38, false, 1, 'int8_t', '%'),
    new MavLinkPacketField('custom0', 'custom0', 39, false, 1, 'int8_t', ''),
    new MavLinkPacketField('custom1', 'custom1', 40, false, 1, 'int8_t', ''),
    new MavLinkPacketField('custom2', 'custom2', 41, false, 1, 'int8_t', ''),
  ]

  constructor() {
    super()
    this.timestamp = 0
    this.type = MavType[Object.keys(MavType)[0]]
    this.autopilot = MavAutopilot[Object.keys(MavAutopilot)[0]]
    this.customMode = 0
    this.latitude = 0
    this.longitude = 0
    this.altitude = 0
    this.targetAltitude = 0
    this.heading = 0
    this.targetHeading = 0
    this.targetDistance = 0
    this.throttle = 0
    this.airspeed = 0
    this.airspeedSp = 0
    this.groundspeed = 0
    this.windspeed = 0
    this.windHeading = 0
    this.eph = 0
    this.epv = 0
    this.temperatureAir = 0
    this.climbRate = 0
    this.battery = 0
    this.wpNum = 0
    this.failureFlags = HlFailureFlag[Object.keys(HlFailureFlag)[0]]
    this.custom0 = 0
    this.custom1 = 0
    this.custom2 = 0
  }

  /**
   * Timestamp (milliseconds since boot or Unix epoch)
   * Units: ms
   */
  timestamp: uint32_t

  /**
   * Type of the MAV (quadrotor, helicopter, etc.)
   */
  type: MavType

  /**
   * Autopilot type / class. Use MAV_AUTOPILOT_INVALID for components that are not flight controllers.
   */
  autopilot: MavAutopilot

  /**
   * A bitfield for use for autopilot-specific flags (2 byte version).
   */
  customMode: uint16_t

  /**
   * Latitude
   * Units: degE7
   */
  latitude: int32_t

  /**
   * Longitude
   * Units: degE7
   */
  longitude: int32_t

  /**
   * Altitude above mean sea level
   * Units: m
   */
  altitude: int16_t

  /**
   * Altitude setpoint
   * Units: m
   */
  targetAltitude: int16_t

  /**
   * Heading
   * Units: deg/2
   */
  heading: uint8_t

  /**
   * Heading setpoint
   * Units: deg/2
   */
  targetHeading: uint8_t

  /**
   * Distance to target waypoint or position
   * Units: dam
   */
  targetDistance: uint16_t

  /**
   * Throttle
   * Units: %
   */
  throttle: uint8_t

  /**
   * Airspeed
   * Units: m/s*5
   */
  airspeed: uint8_t

  /**
   * Airspeed setpoint
   * Units: m/s*5
   */
  airspeedSp: uint8_t

  /**
   * Groundspeed
   * Units: m/s*5
   */
  groundspeed: uint8_t

  /**
   * Windspeed
   * Units: m/s*5
   */
  windspeed: uint8_t

  /**
   * Wind heading
   * Units: deg/2
   */
  windHeading: uint8_t

  /**
   * Maximum error horizontal position since last message
   * Units: dm
   */
  eph: uint8_t

  /**
   * Maximum error vertical position since last message
   * Units: dm
   */
  epv: uint8_t

  /**
   * Air temperature from airspeed sensor
   * Units: degC
   */
  temperatureAir: int8_t

  /**
   * Maximum climb rate magnitude since last message
   * Units: dm/s
   */
  climbRate: int8_t

  /**
   * Battery level (-1 if field not provided).
   * Units: %
   */
  battery: int8_t

  /**
   * Current waypoint number
   */
  wpNum: uint16_t

  /**
   * Bitmap of failure flags.
   */
  failureFlags: HlFailureFlag

  /**
   * Field for custom payload.
   */
  custom0: int8_t

  /**
   * Field for custom payload.
   */
  custom1: int8_t

  /**
   * Field for custom payload.
   */
  custom2: int8_t
}

/**
 * Vibration levels and accelerometer clipping
 */
export class Vibration extends MavLinkData {
  static MSG_ID = 241
  static MSG_NAME = 'VIBRATION'
  static PAYLOAD_LENGTH = 32
  static MAGIC_NUMBER = 90

  static FIELDS = [
    new MavLinkPacketField('time_usec', 'timeUsec', 0, false, 8, 'uint64_t', 'us'),
    new MavLinkPacketField('vibration_x', 'vibrationX', 8, false, 4, 'float', ''),
    new MavLinkPacketField('vibration_y', 'vibrationY', 12, false, 4, 'float', ''),
    new MavLinkPacketField('vibration_z', 'vibrationZ', 16, false, 4, 'float', ''),
    new MavLinkPacketField('clipping_0', 'clipping0', 20, false, 4, 'uint32_t', ''),
    new MavLinkPacketField('clipping_1', 'clipping1', 24, false, 4, 'uint32_t', ''),
    new MavLinkPacketField('clipping_2', 'clipping2', 28, false, 4, 'uint32_t', ''),
  ]

  constructor() {
    super()
    this.timeUsec = BigInt(0)
    this.vibrationX = 0
    this.vibrationY = 0
    this.vibrationZ = 0
    this.clipping0 = 0
    this.clipping1 = 0
    this.clipping2 = 0
  }

  /**
   * Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format
   * (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
   * Units: us
   */
  timeUsec: uint64_t

  /**
   * Vibration levels on X-axis
   */
  vibrationX: float

  /**
   * Vibration levels on Y-axis
   */
  vibrationY: float

  /**
   * Vibration levels on Z-axis
   */
  vibrationZ: float

  /**
   * first accelerometer clipping count
   */
  clipping0: uint32_t

  /**
   * second accelerometer clipping count
   */
  clipping1: uint32_t

  /**
   * third accelerometer clipping count
   */
  clipping2: uint32_t
}

/**
 * This message can be requested by sending the MAV_CMD_GET_HOME_POSITION command. The position the
 * system will return to and land on. The position is set automatically by the system during the
 * takeoff in case it was not explicitly set by the operator before or after. The global and local
 * positions encode the position in the respective coordinate frames, while the q parameter encodes the
 * orientation of the surface. Under normal conditions it describes the heading and terrain slope,
 * which can be used by the aircraft to adjust the approach. The approach 3D vector describes the point
 * to which the system should fly in normal flight mode and then perform a landing sequence along the
 * vector.
 */
export class HomePosition extends MavLinkData {
  static MSG_ID = 242
  static MSG_NAME = 'HOME_POSITION'
  static PAYLOAD_LENGTH = 60
  static MAGIC_NUMBER = 104

  static FIELDS = [
    new MavLinkPacketField('latitude', 'latitude', 0, false, 4, 'int32_t', 'degE7'),
    new MavLinkPacketField('longitude', 'longitude', 4, false, 4, 'int32_t', 'degE7'),
    new MavLinkPacketField('altitude', 'altitude', 8, false, 4, 'int32_t', 'mm'),
    new MavLinkPacketField('x', 'x', 12, false, 4, 'float', 'm'),
    new MavLinkPacketField('y', 'y', 16, false, 4, 'float', 'm'),
    new MavLinkPacketField('z', 'z', 20, false, 4, 'float', 'm'),
    new MavLinkPacketField('q', 'q', 24, false, 4, 'float[]', '', 4),
    new MavLinkPacketField('approach_x', 'approachX', 40, false, 4, 'float', 'm'),
    new MavLinkPacketField('approach_y', 'approachY', 44, false, 4, 'float', 'm'),
    new MavLinkPacketField('approach_z', 'approachZ', 48, false, 4, 'float', 'm'),
    new MavLinkPacketField('time_usec', 'timeUsec', 52, true, 8, 'uint64_t', 'us'),
  ]

  constructor() {
    super()
    this.latitude = 0
    this.longitude = 0
    this.altitude = 0
    this.x = 0
    this.y = 0
    this.z = 0
    this.q = []
    this.approachX = 0
    this.approachY = 0
    this.approachZ = 0
    this.timeUsec = BigInt(0)
  }

  /**
   * Latitude (WGS84)
   * Units: degE7
   */
  latitude: int32_t

  /**
   * Longitude (WGS84)
   * Units: degE7
   */
  longitude: int32_t

  /**
   * Altitude (MSL). Positive for up.
   * Units: mm
   */
  altitude: int32_t

  /**
   * Local X position of this position in the local coordinate frame
   * Units: m
   */
  x: float

  /**
   * Local Y position of this position in the local coordinate frame
   * Units: m
   */
  y: float

  /**
   * Local Z position of this position in the local coordinate frame
   * Units: m
   */
  z: float

  /**
   * World to surface normal and heading transformation of the takeoff position. Used to indicate the
   * heading and slope of the ground
   */
  q: float[]

  /**
   * Local X position of the end of the approach vector. Multicopters should set this position based on
   * their takeoff path. Grass-landing fixed wing aircraft should set it the same way as multicopters.
   * Runway-landing fixed wing aircraft should set it to the opposite direction of the takeoff, assuming
   * the takeoff happened from the threshold / touchdown zone.
   * Units: m
   */
  approachX: float

  /**
   * Local Y position of the end of the approach vector. Multicopters should set this position based on
   * their takeoff path. Grass-landing fixed wing aircraft should set it the same way as multicopters.
   * Runway-landing fixed wing aircraft should set it to the opposite direction of the takeoff, assuming
   * the takeoff happened from the threshold / touchdown zone.
   * Units: m
   */
  approachY: float

  /**
   * Local Z position of the end of the approach vector. Multicopters should set this position based on
   * their takeoff path. Grass-landing fixed wing aircraft should set it the same way as multicopters.
   * Runway-landing fixed wing aircraft should set it to the opposite direction of the takeoff, assuming
   * the takeoff happened from the threshold / touchdown zone.
   * Units: m
   */
  approachZ: float

  /**
   * Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format
   * (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
   * Units: us
   */
  timeUsec: uint64_t
}

/**
 * The position the system will return to and land on. The position is set automatically by the system
 * during the takeoff in case it was not explicitly set by the operator before or after. The global and
 * local positions encode the position in the respective coordinate frames, while the q parameter
 * encodes the orientation of the surface. Under normal conditions it describes the heading and terrain
 * slope, which can be used by the aircraft to adjust the approach. The approach 3D vector describes
 * the point to which the system should fly in normal flight mode and then perform a landing sequence
 * along the vector.
 */
export class SetHomePosition extends MavLinkData {
  static MSG_ID = 243
  static MSG_NAME = 'SET_HOME_POSITION'
  static PAYLOAD_LENGTH = 61
  static MAGIC_NUMBER = 85

  static FIELDS = [
    new MavLinkPacketField('latitude', 'latitude', 0, false, 4, 'int32_t', 'degE7'),
    new MavLinkPacketField('longitude', 'longitude', 4, false, 4, 'int32_t', 'degE7'),
    new MavLinkPacketField('altitude', 'altitude', 8, false, 4, 'int32_t', 'mm'),
    new MavLinkPacketField('x', 'x', 12, false, 4, 'float', 'm'),
    new MavLinkPacketField('y', 'y', 16, false, 4, 'float', 'm'),
    new MavLinkPacketField('z', 'z', 20, false, 4, 'float', 'm'),
    new MavLinkPacketField('q', 'q', 24, false, 4, 'float[]', '', 4),
    new MavLinkPacketField('approach_x', 'approachX', 40, false, 4, 'float', 'm'),
    new MavLinkPacketField('approach_y', 'approachY', 44, false, 4, 'float', 'm'),
    new MavLinkPacketField('approach_z', 'approachZ', 48, false, 4, 'float', 'm'),
    new MavLinkPacketField('target_system', 'targetSystem', 52, false, 1, 'uint8_t', ''),
    new MavLinkPacketField('time_usec', 'timeUsec', 53, true, 8, 'uint64_t', 'us'),
  ]

  constructor() {
    super()
    this.targetSystem = 0
    this.latitude = 0
    this.longitude = 0
    this.altitude = 0
    this.x = 0
    this.y = 0
    this.z = 0
    this.q = []
    this.approachX = 0
    this.approachY = 0
    this.approachZ = 0
    this.timeUsec = BigInt(0)
  }

  /**
   * System ID.
   */
  targetSystem: uint8_t

  /**
   * Latitude (WGS84)
   * Units: degE7
   */
  latitude: int32_t

  /**
   * Longitude (WGS84)
   * Units: degE7
   */
  longitude: int32_t

  /**
   * Altitude (MSL). Positive for up.
   * Units: mm
   */
  altitude: int32_t

  /**
   * Local X position of this position in the local coordinate frame
   * Units: m
   */
  x: float

  /**
   * Local Y position of this position in the local coordinate frame
   * Units: m
   */
  y: float

  /**
   * Local Z position of this position in the local coordinate frame
   * Units: m
   */
  z: float

  /**
   * World to surface normal and heading transformation of the takeoff position. Used to indicate the
   * heading and slope of the ground
   */
  q: float[]

  /**
   * Local X position of the end of the approach vector. Multicopters should set this position based on
   * their takeoff path. Grass-landing fixed wing aircraft should set it the same way as multicopters.
   * Runway-landing fixed wing aircraft should set it to the opposite direction of the takeoff, assuming
   * the takeoff happened from the threshold / touchdown zone.
   * Units: m
   */
  approachX: float

  /**
   * Local Y position of the end of the approach vector. Multicopters should set this position based on
   * their takeoff path. Grass-landing fixed wing aircraft should set it the same way as multicopters.
   * Runway-landing fixed wing aircraft should set it to the opposite direction of the takeoff, assuming
   * the takeoff happened from the threshold / touchdown zone.
   * Units: m
   */
  approachY: float

  /**
   * Local Z position of the end of the approach vector. Multicopters should set this position based on
   * their takeoff path. Grass-landing fixed wing aircraft should set it the same way as multicopters.
   * Runway-landing fixed wing aircraft should set it to the opposite direction of the takeoff, assuming
   * the takeoff happened from the threshold / touchdown zone.
   * Units: m
   */
  approachZ: float

  /**
   * Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format
   * (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
   * Units: us
   */
  timeUsec: uint64_t
}

/**
 * The interval between messages for a particular MAVLink message ID. This message is the response to
 * the MAV_CMD_GET_MESSAGE_INTERVAL command. This interface replaces DATA_STREAM.
 */
export class MessageInterval extends MavLinkData {
  static MSG_ID = 244
  static MSG_NAME = 'MESSAGE_INTERVAL'
  static PAYLOAD_LENGTH = 6
  static MAGIC_NUMBER = 95

  static FIELDS = [
    new MavLinkPacketField('interval_us', 'intervalUs', 0, false, 4, 'int32_t', 'us'),
    new MavLinkPacketField('message_id', 'messageId', 4, false, 2, 'uint16_t', ''),
  ]

  constructor() {
    super()
    this.messageId = 0
    this.intervalUs = 0
  }

  /**
   * The ID of the requested MAVLink message. v1.0 is limited to 254 messages.
   */
  messageId: uint16_t

  /**
   * The interval between two messages. A value of -1 indicates this stream is disabled, 0 indicates it
   * is not available, > 0 indicates the interval at which it is sent.
   * Units: us
   */
  intervalUs: int32_t
}

/**
 * Provides state for additional features
 */
export class ExtendedSysState extends MavLinkData {
  static MSG_ID = 245
  static MSG_NAME = 'EXTENDED_SYS_STATE'
  static PAYLOAD_LENGTH = 2
  static MAGIC_NUMBER = 130

  static FIELDS = [
    new MavLinkPacketField('vtol_state', 'vtolState', 0, false, 1, 'uint8_t', ''),
    new MavLinkPacketField('landed_state', 'landedState', 1, false, 1, 'uint8_t', ''),
  ]

  constructor() {
    super()
    this.vtolState = MavVtolState[Object.keys(MavVtolState)[0]]
    this.landedState = MavLandedState[Object.keys(MavLandedState)[0]]
  }

  /**
   * The VTOL state if applicable. Is set to MAV_VTOL_STATE_UNDEFINED if UAV is not in VTOL
   * configuration.
   */
  vtolState: MavVtolState

  /**
   * The landed state. Is set to MAV_LANDED_STATE_UNDEFINED if landed state is unknown.
   */
  landedState: MavLandedState
}

/**
 * The location and information of an ADSB vehicle
 */
export class AdsbVehicle extends MavLinkData {
  static MSG_ID = 246
  static MSG_NAME = 'ADSB_VEHICLE'
  static PAYLOAD_LENGTH = 38
  static MAGIC_NUMBER = 184

  static FIELDS = [
    new MavLinkPacketField('ICAO_address', 'ICAOAddress', 0, false, 4, 'uint32_t', ''),
    new MavLinkPacketField('lat', 'lat', 4, false, 4, 'int32_t', 'degE7'),
    new MavLinkPacketField('lon', 'lon', 8, false, 4, 'int32_t', 'degE7'),
    new MavLinkPacketField('altitude', 'altitude', 12, false, 4, 'int32_t', 'mm'),
    new MavLinkPacketField('heading', 'heading', 16, false, 2, 'uint16_t', 'cdeg'),
    new MavLinkPacketField('hor_velocity', 'horVelocity', 18, false, 2, 'uint16_t', 'cm/s'),
    new MavLinkPacketField('ver_velocity', 'verVelocity', 20, false, 2, 'int16_t', 'cm/s'),
    new MavLinkPacketField('flags', 'flags', 22, false, 2, 'uint16_t', ''),
    new MavLinkPacketField('squawk', 'squawk', 24, false, 2, 'uint16_t', ''),
    new MavLinkPacketField('altitude_type', 'altitudeType', 26, false, 1, 'uint8_t', ''),
    new MavLinkPacketField('callsign', 'callsign', 27, false, 1, 'char[]', '', 9),
    new MavLinkPacketField('emitter_type', 'emitterType', 36, false, 1, 'uint8_t', ''),
    new MavLinkPacketField('tslc', 'tslc', 37, false, 1, 'uint8_t', 's'),
  ]

  constructor() {
    super()
    this.ICAOAddress = 0
    this.lat = 0
    this.lon = 0
    this.altitudeType = AdsbAltitudeType[Object.keys(AdsbAltitudeType)[0]]
    this.altitude = 0
    this.heading = 0
    this.horVelocity = 0
    this.verVelocity = 0
    this.callsign = ''
    this.emitterType = AdsbEmitterType[Object.keys(AdsbEmitterType)[0]]
    this.tslc = 0
    this.flags = AdsbFlags[Object.keys(AdsbFlags)[0]]
    this.squawk = 0
  }

  /**
   * ICAO address
   */
  ICAOAddress: uint32_t

  /**
   * Latitude
   * Units: degE7
   */
  lat: int32_t

  /**
   * Longitude
   * Units: degE7
   */
  lon: int32_t

  /**
   * ADSB altitude type.
   */
  altitudeType: AdsbAltitudeType

  /**
   * Altitude(ASL)
   * Units: mm
   */
  altitude: int32_t

  /**
   * Course over ground
   * Units: cdeg
   */
  heading: uint16_t

  /**
   * The horizontal velocity
   * Units: cm/s
   */
  horVelocity: uint16_t

  /**
   * The vertical velocity. Positive is up
   * Units: cm/s
   */
  verVelocity: int16_t

  /**
   * The callsign, 8+null
   */
  callsign: string

  /**
   * ADSB emitter type.
   */
  emitterType: AdsbEmitterType

  /**
   * Time since last communication in seconds
   * Units: s
   */
  tslc: uint8_t

  /**
   * Bitmap to indicate various statuses including valid data fields
   */
  flags: AdsbFlags

  /**
   * Squawk code
   */
  squawk: uint16_t
}

/**
 * Information about a potential collision
 */
export class Collision extends MavLinkData {
  static MSG_ID = 247
  static MSG_NAME = 'COLLISION'
  static PAYLOAD_LENGTH = 19
  static MAGIC_NUMBER = 81

  static FIELDS = [
    new MavLinkPacketField('id', 'id', 0, false, 4, 'uint32_t', ''),
    new MavLinkPacketField('time_to_minimum_delta', 'timeToMinimumDelta', 4, false, 4, 'float', 's'),
    new MavLinkPacketField('altitude_minimum_delta', 'altitudeMinimumDelta', 8, false, 4, 'float', 'm'),
    new MavLinkPacketField('horizontal_minimum_delta', 'horizontalMinimumDelta', 12, false, 4, 'float', 'm'),
    new MavLinkPacketField('src', 'src', 16, false, 1, 'uint8_t', ''),
    new MavLinkPacketField('action', 'action', 17, false, 1, 'uint8_t', ''),
    new MavLinkPacketField('threat_level', 'threatLevel', 18, false, 1, 'uint8_t', ''),
  ]

  constructor() {
    super()
    this.src = MavCollisionSrc[Object.keys(MavCollisionSrc)[0]]
    this.id = 0
    this.action = MavCollisionAction[Object.keys(MavCollisionAction)[0]]
    this.threatLevel = MavCollisionThreatLevel[Object.keys(MavCollisionThreatLevel)[0]]
    this.timeToMinimumDelta = 0
    this.altitudeMinimumDelta = 0
    this.horizontalMinimumDelta = 0
  }

  /**
   * Collision data source
   */
  src: MavCollisionSrc

  /**
   * Unique identifier, domain based on src field
   */
  id: uint32_t

  /**
   * Action that is being taken to avoid this collision
   */
  action: MavCollisionAction

  /**
   * How concerned the aircraft is about this collision
   */
  threatLevel: MavCollisionThreatLevel

  /**
   * Estimated time until collision occurs
   * Units: s
   */
  timeToMinimumDelta: float

  /**
   * Closest vertical distance between vehicle and object
   * Units: m
   */
  altitudeMinimumDelta: float

  /**
   * Closest horizontal distance between vehicle and object
   * Units: m
   */
  horizontalMinimumDelta: float
}

/**
 * Message implementing parts of the V2 payload specs in V1 frames for transitional support.
 */
export class V2Extension extends MavLinkData {
  static MSG_ID = 248
  static MSG_NAME = 'V2_EXTENSION'
  static PAYLOAD_LENGTH = 254
  static MAGIC_NUMBER = 8

  static FIELDS = [
    new MavLinkPacketField('message_type', 'messageType', 0, false, 2, 'uint16_t', ''),
    new MavLinkPacketField('target_network', 'targetNetwork', 2, false, 1, 'uint8_t', ''),
    new MavLinkPacketField('target_system', 'targetSystem', 3, false, 1, 'uint8_t', ''),
    new MavLinkPacketField('target_component', 'targetComponent', 4, false, 1, 'uint8_t', ''),
    new MavLinkPacketField('payload', 'payload', 5, false, 1, 'uint8_t[]', '', 249),
  ]

  constructor() {
    super()
    this.targetNetwork = 0
    this.targetSystem = 0
    this.targetComponent = 0
    this.messageType = 0
    this.payload = []
  }

  /**
   * Network ID (0 for broadcast)
   */
  targetNetwork: uint8_t

  /**
   * System ID (0 for broadcast)
   */
  targetSystem: uint8_t

  /**
   * Component ID (0 for broadcast)
   */
  targetComponent: uint8_t

  /**
   * A code that identifies the software component that understands this message (analogous to USB device
   * classes or mime type strings). If this code is less than 32768, it is considered a 'registered'
   * protocol extension and the corresponding entry should be added to
   * https://github.com/mavlink/mavlink/definition_files/extension_message_ids.xml. Software creators can
   * register blocks of message IDs as needed (useful for GCS specific metadata, etc...). Message_types
   * greater than 32767 are considered local experiments and should not be checked in to any widely
   * distributed codebase.
   */
  messageType: uint16_t

  /**
   * Variable length payload. The length must be encoded in the payload as part of the message_type
   * protocol, e.g. by including the length as payload data, or by terminating the payload data with a
   * non-zero marker. This is required in order to reconstruct zero-terminated payloads that are (or
   * otherwise would be) trimmed by MAVLink 2 empty-byte truncation. The entire content of the payload
   * block is opaque unless you understand the encoding message_type. The particular encoding used can be
   * extension specific and might not always be documented as part of the MAVLink specification.
   */
  payload: uint8_t[]
}

/**
 * Send raw controller memory. The use of this message is discouraged for normal packets, but a quite
 * efficient way for testing new messages and getting experimental debug output.
 */
export class MemoryVect extends MavLinkData {
  static MSG_ID = 249
  static MSG_NAME = 'MEMORY_VECT'
  static PAYLOAD_LENGTH = 36
  static MAGIC_NUMBER = 204

  static FIELDS = [
    new MavLinkPacketField('address', 'address', 0, false, 2, 'uint16_t', ''),
    new MavLinkPacketField('ver', 'ver', 2, false, 1, 'uint8_t', ''),
    new MavLinkPacketField('type', 'type', 3, false, 1, 'uint8_t', ''),
    new MavLinkPacketField('value', 'value', 4, false, 1, 'int8_t[]', '', 32),
  ]

  constructor() {
    super()
    this.address = 0
    this.ver = 0
    this.type = 0
    this.value = []
  }

  /**
   * Starting address of the debug variables
   */
  address: uint16_t

  /**
   * Version code of the type variable. 0=unknown, type ignored and assumed int16_t. 1=as below
   */
  ver: uint8_t

  /**
   * Type code of the memory variables. for ver = 1: 0=16 x int16_t, 1=16 x uint16_t, 2=16 x Q15, 3=16 x
   * 1Q14
   */
  type: uint8_t

  /**
   * Memory contents at specified address
   */
  value: int8_t[]
}

/**
 * To debug something using a named 3D vector.
 */
export class DebugVect extends MavLinkData {
  static MSG_ID = 250
  static MSG_NAME = 'DEBUG_VECT'
  static PAYLOAD_LENGTH = 30
  static MAGIC_NUMBER = 49

  static FIELDS = [
    new MavLinkPacketField('time_usec', 'timeUsec', 0, false, 8, 'uint64_t', 'us'),
    new MavLinkPacketField('x', 'x', 8, false, 4, 'float', ''),
    new MavLinkPacketField('y', 'y', 12, false, 4, 'float', ''),
    new MavLinkPacketField('z', 'z', 16, false, 4, 'float', ''),
    new MavLinkPacketField('name', 'name', 20, false, 1, 'char[]', '', 10),
  ]

  constructor() {
    super()
    this.name = ''
    this.timeUsec = BigInt(0)
    this.x = 0
    this.y = 0
    this.z = 0
  }

  /**
   * Name
   */
  name: string

  /**
   * Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format
   * (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
   * Units: us
   */
  timeUsec: uint64_t

  /**
   * x
   */
  x: float

  /**
   * y
   */
  y: float

  /**
   * z
   */
  z: float
}

/**
 * Send a key-value pair as float. The use of this message is discouraged for normal packets, but a
 * quite efficient way for testing new messages and getting experimental debug output.
 */
export class NamedValueFloat extends MavLinkData {
  static MSG_ID = 251
  static MSG_NAME = 'NAMED_VALUE_FLOAT'
  static PAYLOAD_LENGTH = 18
  static MAGIC_NUMBER = 170

  static FIELDS = [
    new MavLinkPacketField('time_boot_ms', 'timeBootMs', 0, false, 4, 'uint32_t', 'ms'),
    new MavLinkPacketField('value', 'value', 4, false, 4, 'float', ''),
    new MavLinkPacketField('name', 'name', 8, false, 1, 'char[]', '', 10),
  ]

  constructor() {
    super()
    this.timeBootMs = 0
    this.name = ''
    this.value = 0
  }

  /**
   * Timestamp (time since system boot).
   * Units: ms
   */
  timeBootMs: uint32_t

  /**
   * Name of the debug variable
   */
  name: string

  /**
   * Floating point value
   */
  value: float
}

/**
 * Send a key-value pair as integer. The use of this message is discouraged for normal packets, but a
 * quite efficient way for testing new messages and getting experimental debug output.
 */
export class NamedValueInt extends MavLinkData {
  static MSG_ID = 252
  static MSG_NAME = 'NAMED_VALUE_INT'
  static PAYLOAD_LENGTH = 18
  static MAGIC_NUMBER = 44

  static FIELDS = [
    new MavLinkPacketField('time_boot_ms', 'timeBootMs', 0, false, 4, 'uint32_t', 'ms'),
    new MavLinkPacketField('value', 'value', 4, false, 4, 'int32_t', ''),
    new MavLinkPacketField('name', 'name', 8, false, 1, 'char[]', '', 10),
  ]

  constructor() {
    super()
    this.timeBootMs = 0
    this.name = ''
    this.value = 0
  }

  /**
   * Timestamp (time since system boot).
   * Units: ms
   */
  timeBootMs: uint32_t

  /**
   * Name of the debug variable
   */
  name: string

  /**
   * Signed integer value
   */
  value: int32_t
}

/**
 * Status text message. These messages are printed in yellow in the COMM console of QGroundControl.
 * WARNING: They consume quite some bandwidth, so use only for important status and error messages. If
 * implemented wisely, these messages are buffered on the MCU and sent only at a limited rate (e.g. 10
 * Hz).
 */
export class StatusText extends MavLinkData {
  static MSG_ID = 253
  static MSG_NAME = 'STATUSTEXT'
  static PAYLOAD_LENGTH = 54
  static MAGIC_NUMBER = 83

  static FIELDS = [
    new MavLinkPacketField('severity', 'severity', 0, false, 1, 'uint8_t', ''),
    new MavLinkPacketField('text', 'text', 1, false, 1, 'char[]', '', 50),
    new MavLinkPacketField('id', 'id', 51, true, 2, 'uint16_t', ''),
    new MavLinkPacketField('chunk_seq', 'chunkSeq', 53, true, 1, 'uint8_t', ''),
  ]

  constructor() {
    super()
    this.severity = MavSeverity[Object.keys(MavSeverity)[0]]
    this.text = ''
    this.id = 0
    this.chunkSeq = 0
  }

  /**
   * Severity of status. Relies on the definitions within RFC-5424.
   */
  severity: MavSeverity

  /**
   * Status text message, without null termination character
   */
  text: string

  /**
   * Unique (opaque) identifier for this statustext message. May be used to reassemble a logical
   * long-statustext message from a sequence of chunks. A value of zero indicates this is the only chunk
   * in the sequence and the message can be emitted immediately.
   */
  id: uint16_t

  /**
   * This chunk's sequence number; indexing is from zero. Any null character in the text field is taken
   * to mean this was the last chunk.
   */
  chunkSeq: uint8_t
}

/**
 * Send a debug value. The index is used to discriminate between values. These values show up in the
 * plot of QGroundControl as DEBUG N.
 */
export class Debug extends MavLinkData {
  static MSG_ID = 254
  static MSG_NAME = 'DEBUG'
  static PAYLOAD_LENGTH = 9
  static MAGIC_NUMBER = 46

  static FIELDS = [
    new MavLinkPacketField('time_boot_ms', 'timeBootMs', 0, false, 4, 'uint32_t', 'ms'),
    new MavLinkPacketField('value', 'value', 4, false, 4, 'float', ''),
    new MavLinkPacketField('ind', 'ind', 8, false, 1, 'uint8_t', ''),
  ]

  constructor() {
    super()
    this.timeBootMs = 0
    this.ind = 0
    this.value = 0
  }

  /**
   * Timestamp (time since system boot).
   * Units: ms
   */
  timeBootMs: uint32_t

  /**
   * index of debug variable
   */
  ind: uint8_t

  /**
   * DEBUG value
   */
  value: float
}

/**
 * Setup a MAVLink2 signing key. If called with secret_key of all zero and zero initial_timestamp will
 * disable signing
 */
export class SetupSigning extends MavLinkData {
  static MSG_ID = 256
  static MSG_NAME = 'SETUP_SIGNING'
  static PAYLOAD_LENGTH = 42
  static MAGIC_NUMBER = 71

  static FIELDS = [
    new MavLinkPacketField('initial_timestamp', 'initialTimestamp', 0, false, 8, 'uint64_t', ''),
    new MavLinkPacketField('target_system', 'targetSystem', 8, false, 1, 'uint8_t', ''),
    new MavLinkPacketField('target_component', 'targetComponent', 9, false, 1, 'uint8_t', ''),
    new MavLinkPacketField('secret_key', 'secretKey', 10, false, 1, 'uint8_t[]', '', 32),
  ]

  constructor() {
    super()
    this.targetSystem = 0
    this.targetComponent = 0
    this.secretKey = []
    this.initialTimestamp = BigInt(0)
  }

  /**
   * system id of the target
   */
  targetSystem: uint8_t

  /**
   * component ID of the target
   */
  targetComponent: uint8_t

  /**
   * signing key
   */
  secretKey: uint8_t[]

  /**
   * initial timestamp
   */
  initialTimestamp: uint64_t
}

/**
 * Report button state change.
 */
export class ButtonChange extends MavLinkData {
  static MSG_ID = 257
  static MSG_NAME = 'BUTTON_CHANGE'
  static PAYLOAD_LENGTH = 9
  static MAGIC_NUMBER = 131

  static FIELDS = [
    new MavLinkPacketField('time_boot_ms', 'timeBootMs', 0, false, 4, 'uint32_t', 'ms'),
    new MavLinkPacketField('last_change_ms', 'lastChangeMs', 4, false, 4, 'uint32_t', 'ms'),
    new MavLinkPacketField('state', 'state', 8, false, 1, 'uint8_t', ''),
  ]

  constructor() {
    super()
    this.timeBootMs = 0
    this.lastChangeMs = 0
    this.state = 0
  }

  /**
   * Timestamp (time since system boot).
   * Units: ms
   */
  timeBootMs: uint32_t

  /**
   * Time of last change of button state.
   * Units: ms
   */
  lastChangeMs: uint32_t

  /**
   * Bitmap for state of buttons.
   */
  state: uint8_t
}

/**
 * Control vehicle tone generation (buzzer).
 *
 * @deprecated since 2019-10, replaced by PLAY_TUNE_V2; New version explicitly defines format. More interoperable.
 */
export class PlayTune extends MavLinkData {
  static MSG_ID = 258
  static MSG_NAME = 'PLAY_TUNE'
  static PAYLOAD_LENGTH = 232
  static MAGIC_NUMBER = 187

  static FIELDS = [
    new MavLinkPacketField('target_system', 'targetSystem', 0, false, 1, 'uint8_t', ''),
    new MavLinkPacketField('target_component', 'targetComponent', 1, false, 1, 'uint8_t', ''),
    new MavLinkPacketField('tune', 'tune', 2, false, 1, 'char[]', '', 30),
    new MavLinkPacketField('tune2', 'tune2', 32, true, 1, 'char[]', '', 200),
  ]

  constructor() {
    super()
    this.targetSystem = 0
    this.targetComponent = 0
    this.tune = ''
    this.tune2 = ''
  }

  /**
   * System ID
   */
  targetSystem: uint8_t

  /**
   * Component ID
   */
  targetComponent: uint8_t

  /**
   * tune in board specific format
   */
  tune: string

  /**
   * tune extension (appended to tune)
   */
  tune2: string
}

/**
 * Information about a camera. Can be requested with a MAV_CMD_REQUEST_MESSAGE command.
 */
export class CameraInformation extends MavLinkData {
  static MSG_ID = 259
  static MSG_NAME = 'CAMERA_INFORMATION'
  static PAYLOAD_LENGTH = 235
  static MAGIC_NUMBER = 92

  static FIELDS = [
    new MavLinkPacketField('time_boot_ms', 'timeBootMs', 0, false, 4, 'uint32_t', 'ms'),
    new MavLinkPacketField('firmware_version', 'firmwareVersion', 4, false, 4, 'uint32_t', ''),
    new MavLinkPacketField('focal_length', 'focalLength', 8, false, 4, 'float', 'mm'),
    new MavLinkPacketField('sensor_size_h', 'sensorSizeH', 12, false, 4, 'float', 'mm'),
    new MavLinkPacketField('sensor_size_v', 'sensorSizeV', 16, false, 4, 'float', 'mm'),
    new MavLinkPacketField('flags', 'flags', 20, false, 4, 'uint32_t', ''),
    new MavLinkPacketField('resolution_h', 'resolutionH', 24, false, 2, 'uint16_t', 'pix'),
    new MavLinkPacketField('resolution_v', 'resolutionV', 26, false, 2, 'uint16_t', 'pix'),
    new MavLinkPacketField('cam_definition_version', 'camDefinitionVersion', 28, false, 2, 'uint16_t', ''),
    new MavLinkPacketField('vendor_name', 'vendorName', 30, false, 1, 'uint8_t[]', '', 32),
    new MavLinkPacketField('model_name', 'modelName', 62, false, 1, 'uint8_t[]', '', 32),
    new MavLinkPacketField('lens_id', 'lensId', 94, false, 1, 'uint8_t', ''),
    new MavLinkPacketField('cam_definition_uri', 'camDefinitionUri', 95, false, 1, 'char[]', '', 140),
  ]

  constructor() {
    super()
    this.timeBootMs = 0
    this.vendorName = []
    this.modelName = []
    this.firmwareVersion = 0
    this.focalLength = 0
    this.sensorSizeH = 0
    this.sensorSizeV = 0
    this.resolutionH = 0
    this.resolutionV = 0
    this.lensId = 0
    this.flags = CameraCapFlags[Object.keys(CameraCapFlags)[0]]
    this.camDefinitionVersion = 0
    this.camDefinitionUri = ''
  }

  /**
   * Timestamp (time since system boot).
   * Units: ms
   */
  timeBootMs: uint32_t

  /**
   * Name of the camera vendor
   */
  vendorName: uint8_t[]

  /**
   * Name of the camera model
   */
  modelName: uint8_t[]

  /**
   * Version of the camera firmware, encoded as: (Dev & 0xff) << 24 | (Patch & 0xff) << 16 | (Minor &
   * 0xff) << 8 | (Major & 0xff)
   */
  firmwareVersion: uint32_t

  /**
   * Focal length
   * Units: mm
   */
  focalLength: float

  /**
   * Image sensor size horizontal
   * Units: mm
   */
  sensorSizeH: float

  /**
   * Image sensor size vertical
   * Units: mm
   */
  sensorSizeV: float

  /**
   * Horizontal image resolution
   * Units: pix
   */
  resolutionH: uint16_t

  /**
   * Vertical image resolution
   * Units: pix
   */
  resolutionV: uint16_t

  /**
   * Reserved for a lens ID
   */
  lensId: uint8_t

  /**
   * Bitmap of camera capability flags.
   */
  flags: CameraCapFlags

  /**
   * Camera definition version (iteration)
   */
  camDefinitionVersion: uint16_t

  /**
   * Camera definition URI (if any, otherwise only basic functions will be available). HTTP- (http://)
   * and MAVLink FTP- (mavlinkftp://) formatted URIs are allowed (and both must be supported by any GCS
   * that implements the Camera Protocol). The definition file may be xz compressed, which will be
   * indicated by the file extension .xml.xz (a GCS that implements the protocol must support
   * decompressing the file).
   */
  camDefinitionUri: string
}

/**
 * Settings of a camera. Can be requested with a MAV_CMD_REQUEST_MESSAGE command.
 */
export class CameraSettings extends MavLinkData {
  static MSG_ID = 260
  static MSG_NAME = 'CAMERA_SETTINGS'
  static PAYLOAD_LENGTH = 13
  static MAGIC_NUMBER = 146

  static FIELDS = [
    new MavLinkPacketField('time_boot_ms', 'timeBootMs', 0, false, 4, 'uint32_t', 'ms'),
    new MavLinkPacketField('mode_id', 'modeId', 4, false, 1, 'uint8_t', ''),
    new MavLinkPacketField('zoomLevel', 'zoomLevel', 5, true, 4, 'float', ''),
    new MavLinkPacketField('focusLevel', 'focusLevel', 9, true, 4, 'float', ''),
  ]

  constructor() {
    super()
    this.timeBootMs = 0
    this.modeId = CameraMode[Object.keys(CameraMode)[0]]
    this.zoomLevel = 0
    this.focusLevel = 0
  }

  /**
   * Timestamp (time since system boot).
   * Units: ms
   */
  timeBootMs: uint32_t

  /**
   * Camera mode
   */
  modeId: CameraMode

  /**
   * Current zoom level (0.0 to 100.0, NaN if not known)
   */
  zoomLevel: float

  /**
   * Current focus level (0.0 to 100.0, NaN if not known)
   */
  focusLevel: float
}

/**
 * Information about a storage medium. This message is sent in response to a request with
 * MAV_CMD_REQUEST_MESSAGE and whenever the status of the storage changes (STORAGE_STATUS). Use
 * MAV_CMD_REQUEST_MESSAGE.param2 to indicate the index/id of requested storage: 0 for all, 1 for
 * first, 2 for second, etc.
 */
export class StorageInformation extends MavLinkData {
  static MSG_ID = 261
  static MSG_NAME = 'STORAGE_INFORMATION'
  static PAYLOAD_LENGTH = 61
  static MAGIC_NUMBER = 179

  static FIELDS = [
    new MavLinkPacketField('time_boot_ms', 'timeBootMs', 0, false, 4, 'uint32_t', 'ms'),
    new MavLinkPacketField('total_capacity', 'totalCapacity', 4, false, 4, 'float', 'MiB'),
    new MavLinkPacketField('used_capacity', 'usedCapacity', 8, false, 4, 'float', 'MiB'),
    new MavLinkPacketField('available_capacity', 'availableCapacity', 12, false, 4, 'float', 'MiB'),
    new MavLinkPacketField('read_speed', 'readSpeed', 16, false, 4, 'float', 'MiB/s'),
    new MavLinkPacketField('write_speed', 'writeSpeed', 20, false, 4, 'float', 'MiB/s'),
    new MavLinkPacketField('storage_id', 'storageId', 24, false, 1, 'uint8_t', ''),
    new MavLinkPacketField('storage_count', 'storageCount', 25, false, 1, 'uint8_t', ''),
    new MavLinkPacketField('status', 'status', 26, false, 1, 'uint8_t', ''),
    new MavLinkPacketField('type', 'type', 27, true, 1, 'uint8_t', ''),
    new MavLinkPacketField('name', 'name', 28, true, 1, 'char[]', '', 32),
    new MavLinkPacketField('storage_usage', 'storageUsage', 60, true, 1, 'uint8_t', ''),
  ]

  constructor() {
    super()
    this.timeBootMs = 0
    this.storageId = 0
    this.storageCount = 0
    this.status = StorageStatus[Object.keys(StorageStatus)[0]]
    this.totalCapacity = 0
    this.usedCapacity = 0
    this.availableCapacity = 0
    this.readSpeed = 0
    this.writeSpeed = 0
    this.type = StorageType[Object.keys(StorageType)[0]]
    this.name = ''
    this.storageUsage = StorageUsageFlag[Object.keys(StorageUsageFlag)[0]]
  }

  /**
   * Timestamp (time since system boot).
   * Units: ms
   */
  timeBootMs: uint32_t

  /**
   * Storage ID (1 for first, 2 for second, etc.)
   */
  storageId: uint8_t

  /**
   * Number of storage devices
   */
  storageCount: uint8_t

  /**
   * Status of storage
   */
  status: StorageStatus

  /**
   * Total capacity. If storage is not ready (STORAGE_STATUS_READY) value will be ignored.
   * Units: MiB
   */
  totalCapacity: float

  /**
   * Used capacity. If storage is not ready (STORAGE_STATUS_READY) value will be ignored.
   * Units: MiB
   */
  usedCapacity: float

  /**
   * Available storage capacity. If storage is not ready (STORAGE_STATUS_READY) value will be ignored.
   * Units: MiB
   */
  availableCapacity: float

  /**
   * Read speed.
   * Units: MiB/s
   */
  readSpeed: float

  /**
   * Write speed.
   * Units: MiB/s
   */
  writeSpeed: float

  /**
   * Type of storage
   */
  type: StorageType

  /**
   * Textual storage name to be used in UI (microSD 1, Internal Memory, etc.) This is a NULL terminated
   * string. If it is exactly 32 characters long, add a terminating NULL. If this string is empty, the
   * generic type is shown to the user.
   */
  name: string

  /**
   * Flags indicating whether this instance is preferred storage for photos, videos, etc.
 Note:
   * Implementations should initially set the flags on the system-default storage id used for saving
   * media (if possible/supported).
 This setting can then be overridden using MAV_CMD_SET_STORAGE_USAGE.
   * If the media usage flags are not set, a GCS may assume storage ID 1 is the default storage for all
   * media types.
   */
  storageUsage: StorageUsageFlag
}

/**
 * Information about the status of a capture. Can be requested with a MAV_CMD_REQUEST_MESSAGE command.
 */
export class CameraCaptureStatus extends MavLinkData {
  static MSG_ID = 262
  static MSG_NAME = 'CAMERA_CAPTURE_STATUS'
  static PAYLOAD_LENGTH = 22
  static MAGIC_NUMBER = 12

  static FIELDS = [
    new MavLinkPacketField('time_boot_ms', 'timeBootMs', 0, false, 4, 'uint32_t', 'ms'),
    new MavLinkPacketField('image_interval', 'imageInterval', 4, false, 4, 'float', 's'),
    new MavLinkPacketField('recording_time_ms', 'recordingTimeMs', 8, false, 4, 'uint32_t', 'ms'),
    new MavLinkPacketField('available_capacity', 'availableCapacity', 12, false, 4, 'float', 'MiB'),
    new MavLinkPacketField('image_status', 'imageStatus', 16, false, 1, 'uint8_t', ''),
    new MavLinkPacketField('video_status', 'videoStatus', 17, false, 1, 'uint8_t', ''),
    new MavLinkPacketField('image_count', 'imageCount', 18, true, 4, 'int32_t', ''),
  ]

  constructor() {
    super()
    this.timeBootMs = 0
    this.imageStatus = 0
    this.videoStatus = 0
    this.imageInterval = 0
    this.recordingTimeMs = 0
    this.availableCapacity = 0
    this.imageCount = 0
  }

  /**
   * Timestamp (time since system boot).
   * Units: ms
   */
  timeBootMs: uint32_t

  /**
   * Current status of image capturing (0: idle, 1: capture in progress, 2: interval set but idle, 3:
   * interval set and capture in progress)
   */
  imageStatus: uint8_t

  /**
   * Current status of video capturing (0: idle, 1: capture in progress)
   */
  videoStatus: uint8_t

  /**
   * Image capture interval
   * Units: s
   */
  imageInterval: float

  /**
   * Elapsed time since recording started (0: Not supported/available). A GCS should compute recording
   * time and use non-zero values of this field to correct any discrepancy.
   * Units: ms
   */
  recordingTimeMs: uint32_t

  /**
   * Available storage capacity.
   * Units: MiB
   */
  availableCapacity: float

  /**
   * Total number of images captured ('forever', or until reset using MAV_CMD_STORAGE_FORMAT).
   */
  imageCount: int32_t
}

/**
 * Information about a captured image. This is emitted every time a message is captured.
 * MAV_CMD_REQUEST_MESSAGE can be used to (re)request this message for a specific sequence number or
 * range of sequence numbers:
 MAV_CMD_REQUEST_MESSAGE.param2 indicates the sequence number the first
 * image to send, or set to -1 to send the message for all sequence numbers.
 * MAV_CMD_REQUEST_MESSAGE.param3 is used to specify a range of messages to send:
 set to 0 (default)
 * to send just the the message for the sequence number in param 2,
 set to -1 to send the message for
 * the sequence number in param 2 and all the following sequence numbers,
 set to the sequence number
 * of the final message in the range.
 */
export class CameraImageCaptured extends MavLinkData {
  static MSG_ID = 263
  static MSG_NAME = 'CAMERA_IMAGE_CAPTURED'
  static PAYLOAD_LENGTH = 255
  static MAGIC_NUMBER = 133

  static FIELDS = [
    new MavLinkPacketField('time_utc', 'timeUtc', 0, false, 8, 'uint64_t', 'us'),
    new MavLinkPacketField('time_boot_ms', 'timeBootMs', 8, false, 4, 'uint32_t', 'ms'),
    new MavLinkPacketField('lat', 'lat', 12, false, 4, 'int32_t', 'degE7'),
    new MavLinkPacketField('lon', 'lon', 16, false, 4, 'int32_t', 'degE7'),
    new MavLinkPacketField('alt', 'alt', 20, false, 4, 'int32_t', 'mm'),
    new MavLinkPacketField('relative_alt', 'relativeAlt', 24, false, 4, 'int32_t', 'mm'),
    new MavLinkPacketField('q', 'q', 28, false, 4, 'float[]', '', 4),
    new MavLinkPacketField('image_index', 'imageIndex', 44, false, 4, 'int32_t', ''),
    new MavLinkPacketField('camera_id', 'cameraId', 48, false, 1, 'uint8_t', ''),
    new MavLinkPacketField('capture_result', 'captureResult', 49, false, 1, 'int8_t', ''),
    new MavLinkPacketField('file_url', 'fileUrl', 50, false, 1, 'char[]', '', 205),
  ]

  constructor() {
    super()
    this.timeBootMs = 0
    this.timeUtc = BigInt(0)
    this.cameraId = 0
    this.lat = 0
    this.lon = 0
    this.alt = 0
    this.relativeAlt = 0
    this.q = []
    this.imageIndex = 0
    this.captureResult = 0
    this.fileUrl = ''
  }

  /**
   * Timestamp (time since system boot).
   * Units: ms
   */
  timeBootMs: uint32_t

  /**
   * Timestamp (time since UNIX epoch) in UTC. 0 for unknown.
   * Units: us
   */
  timeUtc: uint64_t

  /**
   * Deprecated/unused. Component IDs are used to differentiate multiple cameras.
   */
  cameraId: uint8_t

  /**
   * Latitude where image was taken
   * Units: degE7
   */
  lat: int32_t

  /**
   * Longitude where capture was taken
   * Units: degE7
   */
  lon: int32_t

  /**
   * Altitude (MSL) where image was taken
   * Units: mm
   */
  alt: int32_t

  /**
   * Altitude above ground
   * Units: mm
   */
  relativeAlt: int32_t

  /**
   * Quaternion of camera orientation (w, x, y, z order, zero-rotation is 1, 0, 0, 0)
   */
  q: float[]

  /**
   * Zero based index of this image (i.e. a new image will have index CAMERA_CAPTURE_STATUS.image count
   * -1)
   */
  imageIndex: int32_t

  /**
   * Boolean indicating success (1) or failure (0) while capturing this image.
   */
  captureResult: int8_t

  /**
   * URL of image taken. Either local storage or http://foo.jpg if camera provides an HTTP interface.
   */
  fileUrl: string
}

/**
 * Information about flight since last arming.
 */
export class FlightInformation extends MavLinkData {
  static MSG_ID = 264
  static MSG_NAME = 'FLIGHT_INFORMATION'
  static PAYLOAD_LENGTH = 28
  static MAGIC_NUMBER = 49

  static FIELDS = [
    new MavLinkPacketField('arming_time_utc', 'armingTimeUtc', 0, false, 8, 'uint64_t', 'us'),
    new MavLinkPacketField('takeoff_time_utc', 'takeoffTimeUtc', 8, false, 8, 'uint64_t', 'us'),
    new MavLinkPacketField('flight_uuid', 'flightUuid', 16, false, 8, 'uint64_t', ''),
    new MavLinkPacketField('time_boot_ms', 'timeBootMs', 24, false, 4, 'uint32_t', 'ms'),
  ]

  constructor() {
    super()
    this.timeBootMs = 0
    this.armingTimeUtc = BigInt(0)
    this.takeoffTimeUtc = BigInt(0)
    this.flightUuid = BigInt(0)
  }

  /**
   * Timestamp (time since system boot).
   * Units: ms
   */
  timeBootMs: uint32_t

  /**
   * Timestamp at arming (time since UNIX epoch) in UTC, 0 for unknown
   * Units: us
   */
  armingTimeUtc: uint64_t

  /**
   * Timestamp at takeoff (time since UNIX epoch) in UTC, 0 for unknown
   * Units: us
   */
  takeoffTimeUtc: uint64_t

  /**
   * Universally unique identifier (UUID) of flight, should correspond to name of log files
   */
  flightUuid: uint64_t
}

/**
 * Orientation of a mount
 *
 * @deprecated since 2020-01, replaced by MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW; This message is being superseded by MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW. The message can still be used to communicate with legacy gimbals implementing it.
 */
export class MountOrientation extends MavLinkData {
  static MSG_ID = 265
  static MSG_NAME = 'MOUNT_ORIENTATION'
  static PAYLOAD_LENGTH = 20
  static MAGIC_NUMBER = 26

  static FIELDS = [
    new MavLinkPacketField('time_boot_ms', 'timeBootMs', 0, false, 4, 'uint32_t', 'ms'),
    new MavLinkPacketField('roll', 'roll', 4, false, 4, 'float', 'deg'),
    new MavLinkPacketField('pitch', 'pitch', 8, false, 4, 'float', 'deg'),
    new MavLinkPacketField('yaw', 'yaw', 12, false, 4, 'float', 'deg'),
    new MavLinkPacketField('yaw_absolute', 'yawAbsolute', 16, true, 4, 'float', 'deg'),
  ]

  constructor() {
    super()
    this.timeBootMs = 0
    this.roll = 0
    this.pitch = 0
    this.yaw = 0
    this.yawAbsolute = 0
  }

  /**
   * Timestamp (time since system boot).
   * Units: ms
   */
  timeBootMs: uint32_t

  /**
   * Roll in global frame (set to NaN for invalid).
   * Units: deg
   */
  roll: float

  /**
   * Pitch in global frame (set to NaN for invalid).
   * Units: deg
   */
  pitch: float

  /**
   * Yaw relative to vehicle (set to NaN for invalid).
   * Units: deg
   */
  yaw: float

  /**
   * Yaw in absolute frame relative to Earth's North, north is 0 (set to NaN for invalid).
   * Units: deg
   */
  yawAbsolute: float
}

/**
 * A message containing logged data (see also MAV_CMD_LOGGING_START)
 */
export class LoggingData extends MavLinkData {
  static MSG_ID = 266
  static MSG_NAME = 'LOGGING_DATA'
  static PAYLOAD_LENGTH = 255
  static MAGIC_NUMBER = 193

  static FIELDS = [
    new MavLinkPacketField('sequence', 'sequence', 0, false, 2, 'uint16_t', ''),
    new MavLinkPacketField('target_system', 'targetSystem', 2, false, 1, 'uint8_t', ''),
    new MavLinkPacketField('target_component', 'targetComponent', 3, false, 1, 'uint8_t', ''),
    new MavLinkPacketField('length', 'length', 4, false, 1, 'uint8_t', 'bytes'),
    new MavLinkPacketField('first_message_offset', 'firstMessageOffset', 5, false, 1, 'uint8_t', 'bytes'),
    new MavLinkPacketField('data', 'data', 6, false, 1, 'uint8_t[]', '', 249),
  ]

  constructor() {
    super()
    this.targetSystem = 0
    this.targetComponent = 0
    this.sequence = 0
    this.length = 0
    this.firstMessageOffset = 0
    this.data = []
  }

  /**
   * system ID of the target
   */
  targetSystem: uint8_t

  /**
   * component ID of the target
   */
  targetComponent: uint8_t

  /**
   * sequence number (can wrap)
   */
  sequence: uint16_t

  /**
   * data length
   * Units: bytes
   */
  length: uint8_t

  /**
   * offset into data where first message starts. This can be used for recovery, when a previous message
   * got lost (set to UINT8_MAX if no start exists).
   * Units: bytes
   */
  firstMessageOffset: uint8_t

  /**
   * logged data
   */
  data: uint8_t[]
}

/**
 * A message containing logged data which requires a LOGGING_ACK to be sent back
 */
export class LoggingDataAcked extends MavLinkData {
  static MSG_ID = 267
  static MSG_NAME = 'LOGGING_DATA_ACKED'
  static PAYLOAD_LENGTH = 255
  static MAGIC_NUMBER = 35

  static FIELDS = [
    new MavLinkPacketField('sequence', 'sequence', 0, false, 2, 'uint16_t', ''),
    new MavLinkPacketField('target_system', 'targetSystem', 2, false, 1, 'uint8_t', ''),
    new MavLinkPacketField('target_component', 'targetComponent', 3, false, 1, 'uint8_t', ''),
    new MavLinkPacketField('length', 'length', 4, false, 1, 'uint8_t', 'bytes'),
    new MavLinkPacketField('first_message_offset', 'firstMessageOffset', 5, false, 1, 'uint8_t', 'bytes'),
    new MavLinkPacketField('data', 'data', 6, false, 1, 'uint8_t[]', '', 249),
  ]

  constructor() {
    super()
    this.targetSystem = 0
    this.targetComponent = 0
    this.sequence = 0
    this.length = 0
    this.firstMessageOffset = 0
    this.data = []
  }

  /**
   * system ID of the target
   */
  targetSystem: uint8_t

  /**
   * component ID of the target
   */
  targetComponent: uint8_t

  /**
   * sequence number (can wrap)
   */
  sequence: uint16_t

  /**
   * data length
   * Units: bytes
   */
  length: uint8_t

  /**
   * offset into data where first message starts. This can be used for recovery, when a previous message
   * got lost (set to UINT8_MAX if no start exists).
   * Units: bytes
   */
  firstMessageOffset: uint8_t

  /**
   * logged data
   */
  data: uint8_t[]
}

/**
 * An ack for a LOGGING_DATA_ACKED message
 */
export class LoggingAck extends MavLinkData {
  static MSG_ID = 268
  static MSG_NAME = 'LOGGING_ACK'
  static PAYLOAD_LENGTH = 4
  static MAGIC_NUMBER = 14

  static FIELDS = [
    new MavLinkPacketField('sequence', 'sequence', 0, false, 2, 'uint16_t', ''),
    new MavLinkPacketField('target_system', 'targetSystem', 2, false, 1, 'uint8_t', ''),
    new MavLinkPacketField('target_component', 'targetComponent', 3, false, 1, 'uint8_t', ''),
  ]

  constructor() {
    super()
    this.targetSystem = 0
    this.targetComponent = 0
    this.sequence = 0
  }

  /**
   * system ID of the target
   */
  targetSystem: uint8_t

  /**
   * component ID of the target
   */
  targetComponent: uint8_t

  /**
   * sequence number (must match the one in LOGGING_DATA_ACKED)
   */
  sequence: uint16_t
}

/**
 * Information about video stream. It may be requested using MAV_CMD_REQUEST_MESSAGE, where param2
 * indicates the video stream id: 0 for all streams, 1 for first, 2 for second, etc.
 */
export class VideoStreamInformation extends MavLinkData {
  static MSG_ID = 269
  static MSG_NAME = 'VIDEO_STREAM_INFORMATION'
  static PAYLOAD_LENGTH = 213
  static MAGIC_NUMBER = 109

  static FIELDS = [
    new MavLinkPacketField('framerate', 'framerate', 0, false, 4, 'float', 'Hz'),
    new MavLinkPacketField('bitrate', 'bitrate', 4, false, 4, 'uint32_t', 'bits/s'),
    new MavLinkPacketField('flags', 'flags', 8, false, 2, 'uint16_t', ''),
    new MavLinkPacketField('resolution_h', 'resolutionH', 10, false, 2, 'uint16_t', 'pix'),
    new MavLinkPacketField('resolution_v', 'resolutionV', 12, false, 2, 'uint16_t', 'pix'),
    new MavLinkPacketField('rotation', 'rotation', 14, false, 2, 'uint16_t', 'deg'),
    new MavLinkPacketField('hfov', 'hfov', 16, false, 2, 'uint16_t', 'deg'),
    new MavLinkPacketField('stream_id', 'streamId', 18, false, 1, 'uint8_t', ''),
    new MavLinkPacketField('count', 'count', 19, false, 1, 'uint8_t', ''),
    new MavLinkPacketField('type', 'type', 20, false, 1, 'uint8_t', ''),
    new MavLinkPacketField('name', 'name', 21, false, 1, 'char[]', '', 32),
    new MavLinkPacketField('uri', 'uri', 53, false, 1, 'char[]', '', 160),
  ]

  constructor() {
    super()
    this.streamId = 0
    this.count = 0
    this.type = VideoStreamType[Object.keys(VideoStreamType)[0]]
    this.flags = VideoStreamStatusFlags[Object.keys(VideoStreamStatusFlags)[0]]
    this.framerate = 0
    this.resolutionH = 0
    this.resolutionV = 0
    this.bitrate = 0
    this.rotation = 0
    this.hfov = 0
    this.name = ''
    this.uri = ''
  }

  /**
   * Video Stream ID (1 for first, 2 for second, etc.)
   */
  streamId: uint8_t

  /**
   * Number of streams available.
   */
  count: uint8_t

  /**
   * Type of stream.
   */
  type: VideoStreamType

  /**
   * Bitmap of stream status flags.
   */
  flags: VideoStreamStatusFlags

  /**
   * Frame rate.
   * Units: Hz
   */
  framerate: float

  /**
   * Horizontal resolution.
   * Units: pix
   */
  resolutionH: uint16_t

  /**
   * Vertical resolution.
   * Units: pix
   */
  resolutionV: uint16_t

  /**
   * Bit rate.
   * Units: bits/s
   */
  bitrate: uint32_t

  /**
   * Video image rotation clockwise.
   * Units: deg
   */
  rotation: uint16_t

  /**
   * Horizontal Field of view.
   * Units: deg
   */
  hfov: uint16_t

  /**
   * Stream name.
   */
  name: string

  /**
   * Video stream URI (TCP or RTSP URI ground station should connect to) or port number (UDP port ground
   * station should listen to).
   */
  uri: string
}

/**
 * Information about the status of a video stream. It may be requested using MAV_CMD_REQUEST_MESSAGE.
 */
export class VideoStreamStatus extends MavLinkData {
  static MSG_ID = 270
  static MSG_NAME = 'VIDEO_STREAM_STATUS'
  static PAYLOAD_LENGTH = 19
  static MAGIC_NUMBER = 59

  static FIELDS = [
    new MavLinkPacketField('framerate', 'framerate', 0, false, 4, 'float', 'Hz'),
    new MavLinkPacketField('bitrate', 'bitrate', 4, false, 4, 'uint32_t', 'bits/s'),
    new MavLinkPacketField('flags', 'flags', 8, false, 2, 'uint16_t', ''),
    new MavLinkPacketField('resolution_h', 'resolutionH', 10, false, 2, 'uint16_t', 'pix'),
    new MavLinkPacketField('resolution_v', 'resolutionV', 12, false, 2, 'uint16_t', 'pix'),
    new MavLinkPacketField('rotation', 'rotation', 14, false, 2, 'uint16_t', 'deg'),
    new MavLinkPacketField('hfov', 'hfov', 16, false, 2, 'uint16_t', 'deg'),
    new MavLinkPacketField('stream_id', 'streamId', 18, false, 1, 'uint8_t', ''),
  ]

  constructor() {
    super()
    this.streamId = 0
    this.flags = VideoStreamStatusFlags[Object.keys(VideoStreamStatusFlags)[0]]
    this.framerate = 0
    this.resolutionH = 0
    this.resolutionV = 0
    this.bitrate = 0
    this.rotation = 0
    this.hfov = 0
  }

  /**
   * Video Stream ID (1 for first, 2 for second, etc.)
   */
  streamId: uint8_t

  /**
   * Bitmap of stream status flags
   */
  flags: VideoStreamStatusFlags

  /**
   * Frame rate
   * Units: Hz
   */
  framerate: float

  /**
   * Horizontal resolution
   * Units: pix
   */
  resolutionH: uint16_t

  /**
   * Vertical resolution
   * Units: pix
   */
  resolutionV: uint16_t

  /**
   * Bit rate
   * Units: bits/s
   */
  bitrate: uint32_t

  /**
   * Video image rotation clockwise
   * Units: deg
   */
  rotation: uint16_t

  /**
   * Horizontal Field of view
   * Units: deg
   */
  hfov: uint16_t
}

/**
 * Configure WiFi AP SSID, password, and mode. This message is re-emitted as an acknowledgement by the
 * AP. The message may also be explicitly requested using MAV_CMD_REQUEST_MESSAGE
 */
export class WifiConfigAp extends MavLinkData {
  static MSG_ID = 299
  static MSG_NAME = 'WIFI_CONFIG_AP'
  static PAYLOAD_LENGTH = 98
  static MAGIC_NUMBER = 19

  static FIELDS = [
    new MavLinkPacketField('ssid', 'ssid', 0, false, 1, 'char[]', '', 32),
    new MavLinkPacketField('password', 'password', 32, false, 1, 'char[]', '', 64),
    new MavLinkPacketField('mode', 'mode', 96, true, 1, 'int8_t', ''),
    new MavLinkPacketField('response', 'response', 97, true, 1, 'int8_t', ''),
  ]

  constructor() {
    super()
    this.ssid = ''
    this.password = ''
    this.mode = WifiConfigApMode[Object.keys(WifiConfigApMode)[0]]
    this.response = WifiConfigApResponse[Object.keys(WifiConfigApResponse)[0]]
  }

  /**
   * Name of Wi-Fi network (SSID). Blank to leave it unchanged when setting. Current SSID when sent back
   * as a response.
   */
  ssid: string

  /**
   * Password. Blank for an open AP. MD5 hash when message is sent back as a response.
   */
  password: string

  /**
   * WiFi Mode.
   */
  mode: WifiConfigApMode

  /**
   * Message acceptance response (sent back to GS).
   */
  response: WifiConfigApResponse
}

/**
 * General status information of an UAVCAN node. Please refer to the definition of the UAVCAN message
 * "uavcan.protocol.NodeStatus" for the background information. The UAVCAN specification is available
 * at http://uavcan.org.
 */
export class UavcanNodeStatus extends MavLinkData {
  static MSG_ID = 310
  static MSG_NAME = 'UAVCAN_NODE_STATUS'
  static PAYLOAD_LENGTH = 17
  static MAGIC_NUMBER = 28

  static FIELDS = [
    new MavLinkPacketField('time_usec', 'timeUsec', 0, false, 8, 'uint64_t', 'us'),
    new MavLinkPacketField('uptime_sec', 'uptimeSec', 8, false, 4, 'uint32_t', 's'),
    new MavLinkPacketField('vendor_specific_status_code', 'vendorSpecificStatusCode', 12, false, 2, 'uint16_t', ''),
    new MavLinkPacketField('health', 'health', 14, false, 1, 'uint8_t', ''),
    new MavLinkPacketField('mode', 'mode', 15, false, 1, 'uint8_t', ''),
    new MavLinkPacketField('sub_mode', 'subMode', 16, false, 1, 'uint8_t', ''),
  ]

  constructor() {
    super()
    this.timeUsec = BigInt(0)
    this.uptimeSec = 0
    this.health = UavcanNodeHealth[Object.keys(UavcanNodeHealth)[0]]
    this.mode = UavcanNodeMode[Object.keys(UavcanNodeMode)[0]]
    this.subMode = 0
    this.vendorSpecificStatusCode = 0
  }

  /**
   * Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format
   * (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
   * Units: us
   */
  timeUsec: uint64_t

  /**
   * Time since the start-up of the node.
   * Units: s
   */
  uptimeSec: uint32_t

  /**
   * Generalized node health status.
   */
  health: UavcanNodeHealth

  /**
   * Generalized operating mode.
   */
  mode: UavcanNodeMode

  /**
   * Not used currently.
   */
  subMode: uint8_t

  /**
   * Vendor-specific status information.
   */
  vendorSpecificStatusCode: uint16_t
}

/**
 * General information describing a particular UAVCAN node. Please refer to the definition of the
 * UAVCAN service "uavcan.protocol.GetNodeInfo" for the background information. This message should be
 * emitted by the system whenever a new node appears online, or an existing node reboots. Additionally,
 * it can be emitted upon request from the other end of the MAVLink channel (see
 * MAV_CMD_UAVCAN_GET_NODE_INFO). It is also not prohibited to emit this message unconditionally at a
 * low frequency. The UAVCAN specification is available at http://uavcan.org.
 */
export class UavcanNodeInfo extends MavLinkData {
  static MSG_ID = 311
  static MSG_NAME = 'UAVCAN_NODE_INFO'
  static PAYLOAD_LENGTH = 116
  static MAGIC_NUMBER = 95

  static FIELDS = [
    new MavLinkPacketField('time_usec', 'timeUsec', 0, false, 8, 'uint64_t', 'us'),
    new MavLinkPacketField('uptime_sec', 'uptimeSec', 8, false, 4, 'uint32_t', 's'),
    new MavLinkPacketField('sw_vcs_commit', 'swVcsCommit', 12, false, 4, 'uint32_t', ''),
    new MavLinkPacketField('name', 'name', 16, false, 1, 'char[]', '', 80),
    new MavLinkPacketField('hw_version_major', 'hwVersionMajor', 96, false, 1, 'uint8_t', ''),
    new MavLinkPacketField('hw_version_minor', 'hwVersionMinor', 97, false, 1, 'uint8_t', ''),
    new MavLinkPacketField('hw_unique_id', 'hwUniqueId', 98, false, 1, 'uint8_t[]', '', 16),
    new MavLinkPacketField('sw_version_major', 'swVersionMajor', 114, false, 1, 'uint8_t', ''),
    new MavLinkPacketField('sw_version_minor', 'swVersionMinor', 115, false, 1, 'uint8_t', ''),
  ]

  constructor() {
    super()
    this.timeUsec = BigInt(0)
    this.uptimeSec = 0
    this.name = ''
    this.hwVersionMajor = 0
    this.hwVersionMinor = 0
    this.hwUniqueId = []
    this.swVersionMajor = 0
    this.swVersionMinor = 0
    this.swVcsCommit = 0
  }

  /**
   * Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format
   * (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
   * Units: us
   */
  timeUsec: uint64_t

  /**
   * Time since the start-up of the node.
   * Units: s
   */
  uptimeSec: uint32_t

  /**
   * Node name string. For example, "sapog.px4.io".
   */
  name: string

  /**
   * Hardware major version number.
   */
  hwVersionMajor: uint8_t

  /**
   * Hardware minor version number.
   */
  hwVersionMinor: uint8_t

  /**
   * Hardware unique 128-bit ID.
   */
  hwUniqueId: uint8_t[]

  /**
   * Software major version number.
   */
  swVersionMajor: uint8_t

  /**
   * Software minor version number.
   */
  swVersionMinor: uint8_t

  /**
   * Version control system (VCS) revision identifier (e.g. git short commit hash). 0 if unknown.
   */
  swVcsCommit: uint32_t
}

/**
 * Request to read the value of a parameter with either the param_id string id or param_index.
 * PARAM_EXT_VALUE should be emitted in response.
 */
export class ParamExtRequestRead extends MavLinkData {
  static MSG_ID = 320
  static MSG_NAME = 'PARAM_EXT_REQUEST_READ'
  static PAYLOAD_LENGTH = 20
  static MAGIC_NUMBER = 243

  static FIELDS = [
    new MavLinkPacketField('param_index', 'paramIndex', 0, false, 2, 'int16_t', ''),
    new MavLinkPacketField('target_system', 'targetSystem', 2, false, 1, 'uint8_t', ''),
    new MavLinkPacketField('target_component', 'targetComponent', 3, false, 1, 'uint8_t', ''),
    new MavLinkPacketField('param_id', 'paramId', 4, false, 1, 'char[]', '', 16),
  ]

  constructor() {
    super()
    this.targetSystem = 0
    this.targetComponent = 0
    this.paramId = ''
    this.paramIndex = 0
  }

  /**
   * System ID
   */
  targetSystem: uint8_t

  /**
   * Component ID
   */
  targetComponent: uint8_t

  /**
   * Parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT null
   * termination (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes
   * storage if the ID is stored as string
   */
  paramId: string

  /**
   * Parameter index. Set to -1 to use the Parameter ID field as identifier (else param_id will be
   * ignored)
   */
  paramIndex: int16_t
}

/**
 * Request all parameters of this component. All parameters should be emitted in response as
 * PARAM_EXT_VALUE.
 */
export class ParamExtRequestList extends MavLinkData {
  static MSG_ID = 321
  static MSG_NAME = 'PARAM_EXT_REQUEST_LIST'
  static PAYLOAD_LENGTH = 2
  static MAGIC_NUMBER = 88

  static FIELDS = [
    new MavLinkPacketField('target_system', 'targetSystem', 0, false, 1, 'uint8_t', ''),
    new MavLinkPacketField('target_component', 'targetComponent', 1, false, 1, 'uint8_t', ''),
  ]

  constructor() {
    super()
    this.targetSystem = 0
    this.targetComponent = 0
  }

  /**
   * System ID
   */
  targetSystem: uint8_t

  /**
   * Component ID
   */
  targetComponent: uint8_t
}

/**
 * Emit the value of a parameter. The inclusion of param_count and param_index in the message allows
 * the recipient to keep track of received parameters and allows them to re-request missing parameters
 * after a loss or timeout.
 */
export class ParamExtValue extends MavLinkData {
  static MSG_ID = 322
  static MSG_NAME = 'PARAM_EXT_VALUE'
  static PAYLOAD_LENGTH = 149
  static MAGIC_NUMBER = 243

  static FIELDS = [
    new MavLinkPacketField('param_count', 'paramCount', 0, false, 2, 'uint16_t', ''),
    new MavLinkPacketField('param_index', 'paramIndex', 2, false, 2, 'uint16_t', ''),
    new MavLinkPacketField('param_id', 'paramId', 4, false, 1, 'char[]', '', 16),
    new MavLinkPacketField('param_value', 'paramValue', 20, false, 1, 'char[]', '', 128),
    new MavLinkPacketField('param_type', 'paramType', 148, false, 1, 'uint8_t', ''),
  ]

  constructor() {
    super()
    this.paramId = ''
    this.paramValue = ''
    this.paramType = MavParamExtType[Object.keys(MavParamExtType)[0]]
    this.paramCount = 0
    this.paramIndex = 0
  }

  /**
   * Parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT null
   * termination (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes
   * storage if the ID is stored as string
   */
  paramId: string

  /**
   * Parameter value
   */
  paramValue: string

  /**
   * Parameter type.
   */
  paramType: MavParamExtType

  /**
   * Total number of parameters
   */
  paramCount: uint16_t

  /**
   * Index of this parameter
   */
  paramIndex: uint16_t
}

/**
 * Set a parameter value. In order to deal with message loss (and retransmission of PARAM_EXT_SET),
 * when setting a parameter value and the new value is the same as the current value, you will
 * immediately get a PARAM_ACK_ACCEPTED response. If the current state is PARAM_ACK_IN_PROGRESS, you
 * will accordingly receive a PARAM_ACK_IN_PROGRESS in response.
 */
export class ParamExtSet extends MavLinkData {
  static MSG_ID = 323
  static MSG_NAME = 'PARAM_EXT_SET'
  static PAYLOAD_LENGTH = 147
  static MAGIC_NUMBER = 78

  static FIELDS = [
    new MavLinkPacketField('target_system', 'targetSystem', 0, false, 1, 'uint8_t', ''),
    new MavLinkPacketField('target_component', 'targetComponent', 1, false, 1, 'uint8_t', ''),
    new MavLinkPacketField('param_id', 'paramId', 2, false, 1, 'char[]', '', 16),
    new MavLinkPacketField('param_value', 'paramValue', 18, false, 1, 'char[]', '', 128),
    new MavLinkPacketField('param_type', 'paramType', 146, false, 1, 'uint8_t', ''),
  ]

  constructor() {
    super()
    this.targetSystem = 0
    this.targetComponent = 0
    this.paramId = ''
    this.paramValue = ''
    this.paramType = MavParamExtType[Object.keys(MavParamExtType)[0]]
  }

  /**
   * System ID
   */
  targetSystem: uint8_t

  /**
   * Component ID
   */
  targetComponent: uint8_t

  /**
   * Parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT null
   * termination (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes
   * storage if the ID is stored as string
   */
  paramId: string

  /**
   * Parameter value
   */
  paramValue: string

  /**
   * Parameter type.
   */
  paramType: MavParamExtType
}

/**
 * Response from a PARAM_EXT_SET message.
 */
export class ParamExtAck extends MavLinkData {
  static MSG_ID = 324
  static MSG_NAME = 'PARAM_EXT_ACK'
  static PAYLOAD_LENGTH = 146
  static MAGIC_NUMBER = 132

  static FIELDS = [
    new MavLinkPacketField('param_id', 'paramId', 0, false, 1, 'char[]', '', 16),
    new MavLinkPacketField('param_value', 'paramValue', 16, false, 1, 'char[]', '', 128),
    new MavLinkPacketField('param_type', 'paramType', 144, false, 1, 'uint8_t', ''),
    new MavLinkPacketField('param_result', 'paramResult', 145, false, 1, 'uint8_t', ''),
  ]

  constructor() {
    super()
    this.paramId = ''
    this.paramValue = ''
    this.paramType = MavParamExtType[Object.keys(MavParamExtType)[0]]
    this.paramResult = ParamAck[Object.keys(ParamAck)[0]]
  }

  /**
   * Parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT null
   * termination (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes
   * storage if the ID is stored as string
   */
  paramId: string

  /**
   * Parameter value (new value if PARAM_ACK_ACCEPTED, current value otherwise)
   */
  paramValue: string

  /**
   * Parameter type.
   */
  paramType: MavParamExtType

  /**
   * Result code.
   */
  paramResult: ParamAck
}

/**
 * Obstacle distances in front of the sensor, starting from the left in increment degrees to the right
 */
export class ObstacleDistance extends MavLinkData {
  static MSG_ID = 330
  static MSG_NAME = 'OBSTACLE_DISTANCE'
  static PAYLOAD_LENGTH = 167
  static MAGIC_NUMBER = 23

  static FIELDS = [
    new MavLinkPacketField('time_usec', 'timeUsec', 0, false, 8, 'uint64_t', 'us'),
    new MavLinkPacketField('distances', 'distances', 8, false, 2, 'uint16_t[]', 'cm', 72),
    new MavLinkPacketField('min_distance', 'minDistance', 152, false, 2, 'uint16_t', 'cm'),
    new MavLinkPacketField('max_distance', 'maxDistance', 154, false, 2, 'uint16_t', 'cm'),
    new MavLinkPacketField('sensor_type', 'sensorType', 156, false, 1, 'uint8_t', ''),
    new MavLinkPacketField('increment', 'increment', 157, false, 1, 'uint8_t', 'deg'),
    new MavLinkPacketField('increment_f', 'incrementF', 158, true, 4, 'float', 'deg'),
    new MavLinkPacketField('angle_offset', 'angleOffset', 162, true, 4, 'float', 'deg'),
    new MavLinkPacketField('frame', 'frame', 166, true, 1, 'uint8_t', ''),
  ]

  constructor() {
    super()
    this.timeUsec = BigInt(0)
    this.sensorType = MavDistanceSensor[Object.keys(MavDistanceSensor)[0]]
    this.distances = []
    this.increment = 0
    this.minDistance = 0
    this.maxDistance = 0
    this.incrementF = 0
    this.angleOffset = 0
    this.frame = MavFrame[Object.keys(MavFrame)[0]]
  }

  /**
   * Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format
   * (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
   * Units: us
   */
  timeUsec: uint64_t

  /**
   * Class id of the distance sensor type.
   */
  sensorType: MavDistanceSensor

  /**
   * Distance of obstacles around the vehicle with index 0 corresponding to north + angle_offset, unless
   * otherwise specified in the frame. A value of 0 is valid and means that the obstacle is practically
   * touching the sensor. A value of max_distance +1 means no obstacle is present. A value of UINT16_MAX
   * for unknown/not used. In a array element, one unit corresponds to 1cm.
   * Units: cm
   */
  distances: uint16_t[]

  /**
   * Angular width in degrees of each array element. Increment direction is clockwise. This field is
   * ignored if increment_f is non-zero.
   * Units: deg
   */
  increment: uint8_t

  /**
   * Minimum distance the sensor can measure.
   * Units: cm
   */
  minDistance: uint16_t

  /**
   * Maximum distance the sensor can measure.
   * Units: cm
   */
  maxDistance: uint16_t

  /**
   * Angular width in degrees of each array element as a float. If non-zero then this value is used
   * instead of the uint8_t increment field. Positive is clockwise direction, negative is
   * counter-clockwise.
   * Units: deg
   */
  incrementF: float

  /**
   * Relative angle offset of the 0-index element in the distances array. Value of 0 corresponds to
   * forward. Positive is clockwise direction, negative is counter-clockwise.
   * Units: deg
   */
  angleOffset: float

  /**
   * Coordinate frame of reference for the yaw rotation and offset of the sensor data. Defaults to
   * MAV_FRAME_GLOBAL, which is north aligned. For body-mounted sensors use MAV_FRAME_BODY_FRD, which is
   * vehicle front aligned.
   */
  frame: MavFrame
}

/**
 * Odometry message to communicate odometry information with an external interface. Fits ROS REP 147
 * standard for aerial vehicles (http://www.ros.org/reps/rep-0147.html).
 */
export class Odometry extends MavLinkData {
  static MSG_ID = 331
  static MSG_NAME = 'ODOMETRY'
  static PAYLOAD_LENGTH = 232
  static MAGIC_NUMBER = 91

  static FIELDS = [
    new MavLinkPacketField('time_usec', 'timeUsec', 0, false, 8, 'uint64_t', 'us'),
    new MavLinkPacketField('x', 'x', 8, false, 4, 'float', 'm'),
    new MavLinkPacketField('y', 'y', 12, false, 4, 'float', 'm'),
    new MavLinkPacketField('z', 'z', 16, false, 4, 'float', 'm'),
    new MavLinkPacketField('q', 'q', 20, false, 4, 'float[]', '', 4),
    new MavLinkPacketField('vx', 'vx', 36, false, 4, 'float', 'm/s'),
    new MavLinkPacketField('vy', 'vy', 40, false, 4, 'float', 'm/s'),
    new MavLinkPacketField('vz', 'vz', 44, false, 4, 'float', 'm/s'),
    new MavLinkPacketField('rollspeed', 'rollspeed', 48, false, 4, 'float', 'rad/s'),
    new MavLinkPacketField('pitchspeed', 'pitchspeed', 52, false, 4, 'float', 'rad/s'),
    new MavLinkPacketField('yawspeed', 'yawspeed', 56, false, 4, 'float', 'rad/s'),
    new MavLinkPacketField('pose_covariance', 'poseCovariance', 60, false, 4, 'float[]', '', 21),
    new MavLinkPacketField('velocity_covariance', 'velocityCovariance', 144, false, 4, 'float[]', '', 21),
    new MavLinkPacketField('frame_id', 'frameId', 228, false, 1, 'uint8_t', ''),
    new MavLinkPacketField('child_frame_id', 'childFrameId', 229, false, 1, 'uint8_t', ''),
    new MavLinkPacketField('reset_counter', 'resetCounter', 230, true, 1, 'uint8_t', ''),
    new MavLinkPacketField('estimator_type', 'estimatorType', 231, true, 1, 'uint8_t', ''),
  ]

  constructor() {
    super()
    this.timeUsec = BigInt(0)
    this.frameId = MavFrame[Object.keys(MavFrame)[0]]
    this.childFrameId = MavFrame[Object.keys(MavFrame)[0]]
    this.x = 0
    this.y = 0
    this.z = 0
    this.q = []
    this.vx = 0
    this.vy = 0
    this.vz = 0
    this.rollspeed = 0
    this.pitchspeed = 0
    this.yawspeed = 0
    this.poseCovariance = []
    this.velocityCovariance = []
    this.resetCounter = 0
    this.estimatorType = MavEstimatorType[Object.keys(MavEstimatorType)[0]]
  }

  /**
   * Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format
   * (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
   * Units: us
   */
  timeUsec: uint64_t

  /**
   * Coordinate frame of reference for the pose data.
   */
  frameId: MavFrame

  /**
   * Coordinate frame of reference for the velocity in free space (twist) data.
   */
  childFrameId: MavFrame

  /**
   * X Position
   * Units: m
   */
  x: float

  /**
   * Y Position
   * Units: m
   */
  y: float

  /**
   * Z Position
   * Units: m
   */
  z: float

  /**
   * Quaternion components, w, x, y, z (1 0 0 0 is the null-rotation)
   */
  q: float[]

  /**
   * X linear speed
   * Units: m/s
   */
  vx: float

  /**
   * Y linear speed
   * Units: m/s
   */
  vy: float

  /**
   * Z linear speed
   * Units: m/s
   */
  vz: float

  /**
   * Roll angular speed
   * Units: rad/s
   */
  rollspeed: float

  /**
   * Pitch angular speed
   * Units: rad/s
   */
  pitchspeed: float

  /**
   * Yaw angular speed
   * Units: rad/s
   */
  yawspeed: float

  /**
   * Row-major representation of a 6x6 pose cross-covariance matrix upper right triangle (states: x, y,
   * z, roll, pitch, yaw; first six entries are the first ROW, next five entries are the second ROW,
   * etc.). If unknown, assign NaN value to first element in the array.
   */
  poseCovariance: float[]

  /**
   * Row-major representation of a 6x6 velocity cross-covariance matrix upper right triangle (states: vx,
   * vy, vz, rollspeed, pitchspeed, yawspeed; first six entries are the first ROW, next five entries are
   * the second ROW, etc.). If unknown, assign NaN value to first element in the array.
   */
  velocityCovariance: float[]

  /**
   * Estimate reset counter. This should be incremented when the estimate resets in any of the dimensions
   * (position, velocity, attitude, angular speed). This is designed to be used when e.g an external SLAM
   * system detects a loop-closure and the estimate jumps.
   */
  resetCounter: uint8_t

  /**
   * Type of estimator that is providing the odometry.
   */
  estimatorType: MavEstimatorType
}

/**
 * Describe a trajectory using an array of up-to 5 waypoints in the local frame (MAV_FRAME_LOCAL_NED).
 */
export class TrajectoryRepresentationWaypoints extends MavLinkData {
  static MSG_ID = 332
  static MSG_NAME = 'TRAJECTORY_REPRESENTATION_WAYPOINTS'
  static PAYLOAD_LENGTH = 239
  static MAGIC_NUMBER = 236

  static FIELDS = [
    new MavLinkPacketField('time_usec', 'timeUsec', 0, false, 8, 'uint64_t', 'us'),
    new MavLinkPacketField('pos_x', 'posX', 8, false, 4, 'float[]', 'm', 5),
    new MavLinkPacketField('pos_y', 'posY', 28, false, 4, 'float[]', 'm', 5),
    new MavLinkPacketField('pos_z', 'posZ', 48, false, 4, 'float[]', 'm', 5),
    new MavLinkPacketField('vel_x', 'velX', 68, false, 4, 'float[]', 'm/s', 5),
    new MavLinkPacketField('vel_y', 'velY', 88, false, 4, 'float[]', 'm/s', 5),
    new MavLinkPacketField('vel_z', 'velZ', 108, false, 4, 'float[]', 'm/s', 5),
    new MavLinkPacketField('acc_x', 'accX', 128, false, 4, 'float[]', 'm/s/s', 5),
    new MavLinkPacketField('acc_y', 'accY', 148, false, 4, 'float[]', 'm/s/s', 5),
    new MavLinkPacketField('acc_z', 'accZ', 168, false, 4, 'float[]', 'm/s/s', 5),
    new MavLinkPacketField('pos_yaw', 'posYaw', 188, false, 4, 'float[]', 'rad', 5),
    new MavLinkPacketField('vel_yaw', 'velYaw', 208, false, 4, 'float[]', 'rad/s', 5),
    new MavLinkPacketField('command', 'command', 228, false, 2, 'uint16_t[]', '', 5),
    new MavLinkPacketField('valid_points', 'validPoints', 238, false, 1, 'uint8_t', ''),
  ]

  constructor() {
    super()
    this.timeUsec = BigInt(0)
    this.validPoints = 0
    this.posX = []
    this.posY = []
    this.posZ = []
    this.velX = []
    this.velY = []
    this.velZ = []
    this.accX = []
    this.accY = []
    this.accZ = []
    this.posYaw = []
    this.velYaw = []
    this.command = []
  }

  /**
   * Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format
   * (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
   * Units: us
   */
  timeUsec: uint64_t

  /**
   * Number of valid points (up-to 5 waypoints are possible)
   */
  validPoints: uint8_t

  /**
   * X-coordinate of waypoint, set to NaN if not being used
   * Units: m
   */
  posX: float[]

  /**
   * Y-coordinate of waypoint, set to NaN if not being used
   * Units: m
   */
  posY: float[]

  /**
   * Z-coordinate of waypoint, set to NaN if not being used
   * Units: m
   */
  posZ: float[]

  /**
   * X-velocity of waypoint, set to NaN if not being used
   * Units: m/s
   */
  velX: float[]

  /**
   * Y-velocity of waypoint, set to NaN if not being used
   * Units: m/s
   */
  velY: float[]

  /**
   * Z-velocity of waypoint, set to NaN if not being used
   * Units: m/s
   */
  velZ: float[]

  /**
   * X-acceleration of waypoint, set to NaN if not being used
   * Units: m/s/s
   */
  accX: float[]

  /**
   * Y-acceleration of waypoint, set to NaN if not being used
   * Units: m/s/s
   */
  accY: float[]

  /**
   * Z-acceleration of waypoint, set to NaN if not being used
   * Units: m/s/s
   */
  accZ: float[]

  /**
   * Yaw angle, set to NaN if not being used
   * Units: rad
   */
  posYaw: float[]

  /**
   * Yaw rate, set to NaN if not being used
   * Units: rad/s
   */
  velYaw: float[]

  /**
   * MAV_CMD command id of waypoint, set to UINT16_MAX if not being used.
   */
  command: MavCmd[]
}

/**
 * Describe a trajectory using an array of up-to 5 bezier control points in the local frame
 * (MAV_FRAME_LOCAL_NED).
 */
export class TrajectoryRepresentationBezier extends MavLinkData {
  static MSG_ID = 333
  static MSG_NAME = 'TRAJECTORY_REPRESENTATION_BEZIER'
  static PAYLOAD_LENGTH = 109
  static MAGIC_NUMBER = 231

  static FIELDS = [
    new MavLinkPacketField('time_usec', 'timeUsec', 0, false, 8, 'uint64_t', 'us'),
    new MavLinkPacketField('pos_x', 'posX', 8, false, 4, 'float[]', 'm', 5),
    new MavLinkPacketField('pos_y', 'posY', 28, false, 4, 'float[]', 'm', 5),
    new MavLinkPacketField('pos_z', 'posZ', 48, false, 4, 'float[]', 'm', 5),
    new MavLinkPacketField('delta', 'delta', 68, false, 4, 'float[]', 's', 5),
    new MavLinkPacketField('pos_yaw', 'posYaw', 88, false, 4, 'float[]', 'rad', 5),
    new MavLinkPacketField('valid_points', 'validPoints', 108, false, 1, 'uint8_t', ''),
  ]

  constructor() {
    super()
    this.timeUsec = BigInt(0)
    this.validPoints = 0
    this.posX = []
    this.posY = []
    this.posZ = []
    this.delta = []
    this.posYaw = []
  }

  /**
   * Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format
   * (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
   * Units: us
   */
  timeUsec: uint64_t

  /**
   * Number of valid control points (up-to 5 points are possible)
   */
  validPoints: uint8_t

  /**
   * X-coordinate of bezier control points. Set to NaN if not being used
   * Units: m
   */
  posX: float[]

  /**
   * Y-coordinate of bezier control points. Set to NaN if not being used
   * Units: m
   */
  posY: float[]

  /**
   * Z-coordinate of bezier control points. Set to NaN if not being used
   * Units: m
   */
  posZ: float[]

  /**
   * Bezier time horizon. Set to NaN if velocity/acceleration should not be incorporated
   * Units: s
   */
  delta: float[]

  /**
   * Yaw. Set to NaN for unchanged
   * Units: rad
   */
  posYaw: float[]
}

/**
 * Report current used cellular network status
 */
export class CellularStatus extends MavLinkData {
  static MSG_ID = 334
  static MSG_NAME = 'CELLULAR_STATUS'
  static PAYLOAD_LENGTH = 10
  static MAGIC_NUMBER = 72

  static FIELDS = [
    new MavLinkPacketField('mcc', 'mcc', 0, false, 2, 'uint16_t', ''),
    new MavLinkPacketField('mnc', 'mnc', 2, false, 2, 'uint16_t', ''),
    new MavLinkPacketField('lac', 'lac', 4, false, 2, 'uint16_t', ''),
    new MavLinkPacketField('status', 'status', 6, false, 1, 'uint8_t', ''),
    new MavLinkPacketField('failure_reason', 'failureReason', 7, false, 1, 'uint8_t', ''),
    new MavLinkPacketField('type', 'type', 8, false, 1, 'uint8_t', ''),
    new MavLinkPacketField('quality', 'quality', 9, false, 1, 'uint8_t', ''),
  ]

  constructor() {
    super()
    this.status = CellularStatusFlag[Object.keys(CellularStatusFlag)[0]]
    this.failureReason = CellularNetworkFailedReason[Object.keys(CellularNetworkFailedReason)[0]]
    this.type = CellularNetworkRadioType[Object.keys(CellularNetworkRadioType)[0]]
    this.quality = 0
    this.mcc = 0
    this.mnc = 0
    this.lac = 0
  }

  /**
   * Cellular modem status
   */
  status: CellularStatusFlag

  /**
   * Failure reason when status in in CELLUAR_STATUS_FAILED
   */
  failureReason: CellularNetworkFailedReason

  /**
   * Cellular network radio type: gsm, cdma, lte...
   */
  type: CellularNetworkRadioType

  /**
   * Signal quality in percent. If unknown, set to UINT8_MAX
   */
  quality: uint8_t

  /**
   * Mobile country code. If unknown, set to UINT16_MAX
   */
  mcc: uint16_t

  /**
   * Mobile network code. If unknown, set to UINT16_MAX
   */
  mnc: uint16_t

  /**
   * Location area code. If unknown, set to 0
   */
  lac: uint16_t
}

/**
 * Status of the Iridium SBD link.
 */
export class IsbdLinkStatus extends MavLinkData {
  static MSG_ID = 335
  static MSG_NAME = 'ISBD_LINK_STATUS'
  static PAYLOAD_LENGTH = 24
  static MAGIC_NUMBER = 225

  static FIELDS = [
    new MavLinkPacketField('timestamp', 'timestamp', 0, false, 8, 'uint64_t', 'us'),
    new MavLinkPacketField('last_heartbeat', 'lastHeartbeat', 8, false, 8, 'uint64_t', 'us'),
    new MavLinkPacketField('failed_sessions', 'failedSessions', 16, false, 2, 'uint16_t', ''),
    new MavLinkPacketField('successful_sessions', 'successfulSessions', 18, false, 2, 'uint16_t', ''),
    new MavLinkPacketField('signal_quality', 'signalQuality', 20, false, 1, 'uint8_t', ''),
    new MavLinkPacketField('ring_pending', 'ringPending', 21, false, 1, 'uint8_t', ''),
    new MavLinkPacketField('tx_session_pending', 'txSessionPending', 22, false, 1, 'uint8_t', ''),
    new MavLinkPacketField('rx_session_pending', 'rxSessionPending', 23, false, 1, 'uint8_t', ''),
  ]

  constructor() {
    super()
    this.timestamp = BigInt(0)
    this.lastHeartbeat = BigInt(0)
    this.failedSessions = 0
    this.successfulSessions = 0
    this.signalQuality = 0
    this.ringPending = 0
    this.txSessionPending = 0
    this.rxSessionPending = 0
  }

  /**
   * Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format
   * (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
   * Units: us
   */
  timestamp: uint64_t

  /**
   * Timestamp of the last successful sbd session. The receiving end can infer timestamp format (since
   * 1.1.1970 or since system boot) by checking for the magnitude of the number.
   * Units: us
   */
  lastHeartbeat: uint64_t

  /**
   * Number of failed SBD sessions.
   */
  failedSessions: uint16_t

  /**
   * Number of successful SBD sessions.
   */
  successfulSessions: uint16_t

  /**
   * Signal quality equal to the number of bars displayed on the ISU signal strength indicator. Range is
   * 0 to 5, where 0 indicates no signal and 5 indicates maximum signal strength.
   */
  signalQuality: uint8_t

  /**
   * 1: Ring call pending, 0: No call pending.
   */
  ringPending: uint8_t

  /**
   * 1: Transmission session pending, 0: No transmission session pending.
   */
  txSessionPending: uint8_t

  /**
   * 1: Receiving session pending, 0: No receiving session pending.
   */
  rxSessionPending: uint8_t
}

/**
 * Configure cellular modems.
 This message is re-emitted as an acknowledgement by the modem.
 The
 * message may also be explicitly requested using MAV_CMD_REQUEST_MESSAGE.
 */
export class CellularConfig extends MavLinkData {
  static MSG_ID = 336
  static MSG_NAME = 'CELLULAR_CONFIG'
  static PAYLOAD_LENGTH = 84
  static MAGIC_NUMBER = 245

  static FIELDS = [
    new MavLinkPacketField('enable_lte', 'enableLte', 0, false, 1, 'uint8_t', ''),
    new MavLinkPacketField('enable_pin', 'enablePin', 1, false, 1, 'uint8_t', ''),
    new MavLinkPacketField('pin', 'pin', 2, false, 1, 'char[]', '', 16),
    new MavLinkPacketField('new_pin', 'newPin', 18, false, 1, 'char[]', '', 16),
    new MavLinkPacketField('apn', 'apn', 34, false, 1, 'char[]', '', 32),
    new MavLinkPacketField('puk', 'puk', 66, false, 1, 'char[]', '', 16),
    new MavLinkPacketField('roaming', 'roaming', 82, false, 1, 'uint8_t', ''),
    new MavLinkPacketField('response', 'response', 83, false, 1, 'uint8_t', ''),
  ]

  constructor() {
    super()
    this.enableLte = 0
    this.enablePin = 0
    this.pin = ''
    this.newPin = ''
    this.apn = ''
    this.puk = ''
    this.roaming = 0
    this.response = CellularConfigResponse[Object.keys(CellularConfigResponse)[0]]
  }

  /**
   * Enable/disable LTE. 0: setting unchanged, 1: disabled, 2: enabled. Current setting when sent back as
   * a response.
   */
  enableLte: uint8_t

  /**
   * Enable/disable PIN on the SIM card. 0: setting unchanged, 1: disabled, 2: enabled. Current setting
   * when sent back as a response.
   */
  enablePin: uint8_t

  /**
   * PIN sent to the SIM card. Blank when PIN is disabled. Empty when message is sent back as a response.
   */
  pin: string

  /**
   * New PIN when changing the PIN. Blank to leave it unchanged. Empty when message is sent back as a
   * response.
   */
  newPin: string

  /**
   * Name of the cellular APN. Blank to leave it unchanged. Current APN when sent back as a response.
   */
  apn: string

  /**
   * Required PUK code in case the user failed to authenticate 3 times with the PIN. Empty when message
   * is sent back as a response.
   */
  puk: string

  /**
   * Enable/disable roaming. 0: setting unchanged, 1: disabled, 2: enabled. Current setting when sent
   * back as a response.
   */
  roaming: uint8_t

  /**
   * Message acceptance response (sent back to GS).
   */
  response: CellularConfigResponse
}

/**
 * RPM sensor data message.
 */
export class RawRpm extends MavLinkData {
  static MSG_ID = 339
  static MSG_NAME = 'RAW_RPM'
  static PAYLOAD_LENGTH = 5
  static MAGIC_NUMBER = 199

  static FIELDS = [
    new MavLinkPacketField('frequency', 'frequency', 0, false, 4, 'float', 'rpm'),
    new MavLinkPacketField('index', 'index', 4, false, 1, 'uint8_t', ''),
  ]

  constructor() {
    super()
    this.index = 0
    this.frequency = 0
  }

  /**
   * Index of this RPM sensor (0-indexed)
   */
  index: uint8_t

  /**
   * Indicated rate
   * Units: rpm
   */
  frequency: float
}

/**
 * The global position resulting from GPS and sensor fusion.
 */
export class UtmGlobalPosition extends MavLinkData {
  static MSG_ID = 340
  static MSG_NAME = 'UTM_GLOBAL_POSITION'
  static PAYLOAD_LENGTH = 70
  static MAGIC_NUMBER = 99

  static FIELDS = [
    new MavLinkPacketField('time', 'time', 0, false, 8, 'uint64_t', 'us'),
    new MavLinkPacketField('lat', 'lat', 8, false, 4, 'int32_t', 'degE7'),
    new MavLinkPacketField('lon', 'lon', 12, false, 4, 'int32_t', 'degE7'),
    new MavLinkPacketField('alt', 'alt', 16, false, 4, 'int32_t', 'mm'),
    new MavLinkPacketField('relative_alt', 'relativeAlt', 20, false, 4, 'int32_t', 'mm'),
    new MavLinkPacketField('next_lat', 'nextLat', 24, false, 4, 'int32_t', 'degE7'),
    new MavLinkPacketField('next_lon', 'nextLon', 28, false, 4, 'int32_t', 'degE7'),
    new MavLinkPacketField('next_alt', 'nextAlt', 32, false, 4, 'int32_t', 'mm'),
    new MavLinkPacketField('vx', 'vx', 36, false, 2, 'int16_t', 'cm/s'),
    new MavLinkPacketField('vy', 'vy', 38, false, 2, 'int16_t', 'cm/s'),
    new MavLinkPacketField('vz', 'vz', 40, false, 2, 'int16_t', 'cm/s'),
    new MavLinkPacketField('h_acc', 'hAcc', 42, false, 2, 'uint16_t', 'mm'),
    new MavLinkPacketField('v_acc', 'vAcc', 44, false, 2, 'uint16_t', 'mm'),
    new MavLinkPacketField('vel_acc', 'velAcc', 46, false, 2, 'uint16_t', 'cm/s'),
    new MavLinkPacketField('update_rate', 'updateRate', 48, false, 2, 'uint16_t', 'cs'),
    new MavLinkPacketField('uas_id', 'uasId', 50, false, 1, 'uint8_t[]', '', 18),
    new MavLinkPacketField('flight_state', 'flightState', 68, false, 1, 'uint8_t', ''),
    new MavLinkPacketField('flags', 'flags', 69, false, 1, 'uint8_t', ''),
  ]

  constructor() {
    super()
    this.time = BigInt(0)
    this.uasId = []
    this.lat = 0
    this.lon = 0
    this.alt = 0
    this.relativeAlt = 0
    this.vx = 0
    this.vy = 0
    this.vz = 0
    this.hAcc = 0
    this.vAcc = 0
    this.velAcc = 0
    this.nextLat = 0
    this.nextLon = 0
    this.nextAlt = 0
    this.updateRate = 0
    this.flightState = UtmFlightState[Object.keys(UtmFlightState)[0]]
    this.flags = UtmDataAvailFlags[Object.keys(UtmDataAvailFlags)[0]]
  }

  /**
   * Time of applicability of position (microseconds since UNIX epoch).
   * Units: us
   */
  time: uint64_t

  /**
   * Unique UAS ID.
   */
  uasId: uint8_t[]

  /**
   * Latitude (WGS84)
   * Units: degE7
   */
  lat: int32_t

  /**
   * Longitude (WGS84)
   * Units: degE7
   */
  lon: int32_t

  /**
   * Altitude (WGS84)
   * Units: mm
   */
  alt: int32_t

  /**
   * Altitude above ground
   * Units: mm
   */
  relativeAlt: int32_t

  /**
   * Ground X speed (latitude, positive north)
   * Units: cm/s
   */
  vx: int16_t

  /**
   * Ground Y speed (longitude, positive east)
   * Units: cm/s
   */
  vy: int16_t

  /**
   * Ground Z speed (altitude, positive down)
   * Units: cm/s
   */
  vz: int16_t

  /**
   * Horizontal position uncertainty (standard deviation)
   * Units: mm
   */
  hAcc: uint16_t

  /**
   * Altitude uncertainty (standard deviation)
   * Units: mm
   */
  vAcc: uint16_t

  /**
   * Speed uncertainty (standard deviation)
   * Units: cm/s
   */
  velAcc: uint16_t

  /**
   * Next waypoint, latitude (WGS84)
   * Units: degE7
   */
  nextLat: int32_t

  /**
   * Next waypoint, longitude (WGS84)
   * Units: degE7
   */
  nextLon: int32_t

  /**
   * Next waypoint, altitude (WGS84)
   * Units: mm
   */
  nextAlt: int32_t

  /**
   * Time until next update. Set to 0 if unknown or in data driven mode.
   * Units: cs
   */
  updateRate: uint16_t

  /**
   * Flight state
   */
  flightState: UtmFlightState

  /**
   * Bitwise OR combination of the data available flags.
   */
  flags: UtmDataAvailFlags
}

/**
 * Large debug/prototyping array. The message uses the maximum available payload for data. The array_id
 * and name fields are used to discriminate between messages in code and in user interfaces
 * (respectively). Do not use in production code.
 */
export class DebugFloatArray extends MavLinkData {
  static MSG_ID = 350
  static MSG_NAME = 'DEBUG_FLOAT_ARRAY'
  static PAYLOAD_LENGTH = 252
  static MAGIC_NUMBER = 232

  static FIELDS = [
    new MavLinkPacketField('time_usec', 'timeUsec', 0, false, 8, 'uint64_t', 'us'),
    new MavLinkPacketField('array_id', 'arrayId', 8, false, 2, 'uint16_t', ''),
    new MavLinkPacketField('name', 'name', 10, false, 1, 'char[]', '', 10),
    new MavLinkPacketField('data', 'data', 20, true, 4, 'float[]', '', 58),
  ]

  constructor() {
    super()
    this.timeUsec = BigInt(0)
    this.name = ''
    this.arrayId = 0
    this.data = []
  }

  /**
   * Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format
   * (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
   * Units: us
   */
  timeUsec: uint64_t

  /**
   * Name, for human-friendly display in a Ground Control Station
   */
  name: string

  /**
   * Unique ID used to discriminate between arrays
   */
  arrayId: uint16_t

  /**
   * data
   */
  data: float[]
}

/**
 * Smart Battery information (static/infrequent update). Use for updates from: smart battery to flight
 * stack, flight stack to GCS. Use BATTERY_STATUS for smart battery frequent updates.
 */
export class SmartBatteryInfo extends MavLinkData {
  static MSG_ID = 370
  static MSG_NAME = 'SMART_BATTERY_INFO'
  static PAYLOAD_LENGTH = 109
  static MAGIC_NUMBER = 75

  static FIELDS = [
    new MavLinkPacketField('capacity_full_specification', 'capacityFullSpecification', 0, false, 4, 'int32_t', 'mAh'),
    new MavLinkPacketField('capacity_full', 'capacityFull', 4, false, 4, 'int32_t', 'mAh'),
    new MavLinkPacketField('cycle_count', 'cycleCount', 8, false, 2, 'uint16_t', ''),
    new MavLinkPacketField('weight', 'weight', 10, false, 2, 'uint16_t', 'g'),
    new MavLinkPacketField('discharge_minimum_voltage', 'dischargeMinimumVoltage', 12, false, 2, 'uint16_t', 'mV'),
    new MavLinkPacketField('charging_minimum_voltage', 'chargingMinimumVoltage', 14, false, 2, 'uint16_t', 'mV'),
    new MavLinkPacketField('resting_minimum_voltage', 'restingMinimumVoltage', 16, false, 2, 'uint16_t', 'mV'),
    new MavLinkPacketField('id', 'id', 18, false, 1, 'uint8_t', ''),
    new MavLinkPacketField('battery_function', 'batteryFunction', 19, false, 1, 'uint8_t', ''),
    new MavLinkPacketField('type', 'type', 20, false, 1, 'uint8_t', ''),
    new MavLinkPacketField('serial_number', 'serialNumber', 21, false, 1, 'char[]', '', 16),
    new MavLinkPacketField('device_name', 'deviceName', 37, false, 1, 'char[]', '', 50),
    new MavLinkPacketField('charging_maximum_voltage', 'chargingMaximumVoltage', 87, true, 2, 'uint16_t', 'mV'),
    new MavLinkPacketField('cells_in_series', 'cellsInSeries', 89, true, 1, 'uint8_t', ''),
    new MavLinkPacketField('discharge_maximum_current', 'dischargeMaximumCurrent', 90, true, 4, 'uint32_t', 'mA'),
    new MavLinkPacketField('discharge_maximum_burst_current', 'dischargeMaximumBurstCurrent', 94, true, 4, 'uint32_t', 'mA'),
    new MavLinkPacketField('manufacture_date', 'manufactureDate', 98, true, 1, 'char[]', '', 11),
  ]

  constructor() {
    super()
    this.id = 0
    this.batteryFunction = MavBatteryFunction[Object.keys(MavBatteryFunction)[0]]
    this.type = MavBatteryType[Object.keys(MavBatteryType)[0]]
    this.capacityFullSpecification = 0
    this.capacityFull = 0
    this.cycleCount = 0
    this.serialNumber = ''
    this.deviceName = ''
    this.weight = 0
    this.dischargeMinimumVoltage = 0
    this.chargingMinimumVoltage = 0
    this.restingMinimumVoltage = 0
    this.chargingMaximumVoltage = 0
    this.cellsInSeries = 0
    this.dischargeMaximumCurrent = 0
    this.dischargeMaximumBurstCurrent = 0
    this.manufactureDate = ''
  }

  /**
   * Battery ID
   */
  id: uint8_t

  /**
   * Function of the battery
   */
  batteryFunction: MavBatteryFunction

  /**
   * Type (chemistry) of the battery
   */
  type: MavBatteryType

  /**
   * Capacity when full according to manufacturer, -1: field not provided.
   * Units: mAh
   */
  capacityFullSpecification: int32_t

  /**
   * Capacity when full (accounting for battery degradation), -1: field not provided.
   * Units: mAh
   */
  capacityFull: int32_t

  /**
   * Charge/discharge cycle count. UINT16_MAX: field not provided.
   */
  cycleCount: uint16_t

  /**
   * Serial number in ASCII characters, 0 terminated. All 0: field not provided.
   */
  serialNumber: string

  /**
   * Static device name in ASCII characters, 0 terminated. All 0: field not provided. Encode as
   * manufacturer name then product name separated using an underscore.
   */
  deviceName: string

  /**
   * Battery weight. 0: field not provided.
   * Units: g
   */
  weight: uint16_t

  /**
   * Minimum per-cell voltage when discharging. If not supplied set to UINT16_MAX value.
   * Units: mV
   */
  dischargeMinimumVoltage: uint16_t

  /**
   * Minimum per-cell voltage when charging. If not supplied set to UINT16_MAX value.
   * Units: mV
   */
  chargingMinimumVoltage: uint16_t

  /**
   * Minimum per-cell voltage when resting. If not supplied set to UINT16_MAX value.
   * Units: mV
   */
  restingMinimumVoltage: uint16_t

  /**
   * Maximum per-cell voltage when charged. 0: field not provided.
   * Units: mV
   */
  chargingMaximumVoltage: uint16_t

  /**
   * Number of battery cells in series. 0: field not provided.
   */
  cellsInSeries: uint8_t

  /**
   * Maximum pack discharge current. 0: field not provided.
   * Units: mA
   */
  dischargeMaximumCurrent: uint32_t

  /**
   * Maximum pack discharge burst current. 0: field not provided.
   * Units: mA
   */
  dischargeMaximumBurstCurrent: uint32_t

  /**
   * Manufacture date (DD/MM/YYYY) in ASCII characters, 0 terminated. All 0: field not provided.
   */
  manufactureDate: string
}

/**
 * Telemetry of power generation system. Alternator or mechanical generator.
 */
export class GeneratorStatus extends MavLinkData {
  static MSG_ID = 373
  static MSG_NAME = 'GENERATOR_STATUS'
  static PAYLOAD_LENGTH = 42
  static MAGIC_NUMBER = 117

  static FIELDS = [
    new MavLinkPacketField('status', 'status', 0, false, 8, 'uint64_t', ''),
    new MavLinkPacketField('battery_current', 'batteryCurrent', 8, false, 4, 'float', 'A'),
    new MavLinkPacketField('load_current', 'loadCurrent', 12, false, 4, 'float', 'A'),
    new MavLinkPacketField('power_generated', 'powerGenerated', 16, false, 4, 'float', 'W'),
    new MavLinkPacketField('bus_voltage', 'busVoltage', 20, false, 4, 'float', 'V'),
    new MavLinkPacketField('bat_current_setpoint', 'batCurrentSetpoint', 24, false, 4, 'float', 'A'),
    new MavLinkPacketField('runtime', 'runtime', 28, false, 4, 'uint32_t', 's'),
    new MavLinkPacketField('time_until_maintenance', 'timeUntilMaintenance', 32, false, 4, 'int32_t', 's'),
    new MavLinkPacketField('generator_speed', 'generatorSpeed', 36, false, 2, 'uint16_t', 'rpm'),
    new MavLinkPacketField('rectifier_temperature', 'rectifierTemperature', 38, false, 2, 'int16_t', 'degC'),
    new MavLinkPacketField('generator_temperature', 'generatorTemperature', 40, false, 2, 'int16_t', 'degC'),
  ]

  constructor() {
    super()
    this.status = MavGeneratorStatusFlag[Object.keys(MavGeneratorStatusFlag)[0]]
    this.generatorSpeed = 0
    this.batteryCurrent = 0
    this.loadCurrent = 0
    this.powerGenerated = 0
    this.busVoltage = 0
    this.rectifierTemperature = 0
    this.batCurrentSetpoint = 0
    this.generatorTemperature = 0
    this.runtime = 0
    this.timeUntilMaintenance = 0
  }

  /**
   * Status flags.
   */
  status: MavGeneratorStatusFlag

  /**
   * Speed of electrical generator or alternator. UINT16_MAX: field not provided.
   * Units: rpm
   */
  generatorSpeed: uint16_t

  /**
   * Current into/out of battery. Positive for out. Negative for in. NaN: field not provided.
   * Units: A
   */
  batteryCurrent: float

  /**
   * Current going to the UAV. If battery current not available this is the DC current from the
   * generator. Positive for out. Negative for in. NaN: field not provided
   * Units: A
   */
  loadCurrent: float

  /**
   * The power being generated. NaN: field not provided
   * Units: W
   */
  powerGenerated: float

  /**
   * Voltage of the bus seen at the generator, or battery bus if battery bus is controlled by generator
   * and at a different voltage to main bus.
   * Units: V
   */
  busVoltage: float

  /**
   * The temperature of the rectifier or power converter. INT16_MAX: field not provided.
   * Units: degC
   */
  rectifierTemperature: int16_t

  /**
   * The target battery current. Positive for out. Negative for in. NaN: field not provided
   * Units: A
   */
  batCurrentSetpoint: float

  /**
   * The temperature of the mechanical motor, fuel cell core or generator. INT16_MAX: field not provided.
   * Units: degC
   */
  generatorTemperature: int16_t

  /**
   * Seconds this generator has run since it was rebooted. UINT32_MAX: field not provided.
   * Units: s
   */
  runtime: uint32_t

  /**
   * Seconds until this generator requires maintenance. A negative value indicates maintenance is
   * past-due. INT32_MAX: field not provided.
   * Units: s
   */
  timeUntilMaintenance: int32_t
}

/**
 * The raw values of the actuator outputs (e.g. on Pixhawk, from MAIN, AUX ports). This message
 * supersedes SERVO_OUTPUT_RAW.
 */
export class ActuatorOutputStatus extends MavLinkData {
  static MSG_ID = 375
  static MSG_NAME = 'ACTUATOR_OUTPUT_STATUS'
  static PAYLOAD_LENGTH = 140
  static MAGIC_NUMBER = 251

  static FIELDS = [
    new MavLinkPacketField('time_usec', 'timeUsec', 0, false, 8, 'uint64_t', 'us'),
    new MavLinkPacketField('active', 'active', 8, false, 4, 'uint32_t', ''),
    new MavLinkPacketField('actuator', 'actuator', 12, false, 4, 'float[]', '', 32),
  ]

  constructor() {
    super()
    this.timeUsec = BigInt(0)
    this.active = 0
    this.actuator = []
  }

  /**
   * Timestamp (since system boot).
   * Units: us
   */
  timeUsec: uint64_t

  /**
   * Active outputs
   */
  active: uint32_t

  /**
   * Servo / motor output array values. Zero values indicate unused channels.
   */
  actuator: float[]
}

/**
 * Message for transporting "arbitrary" variable-length data from one component to another (broadcast
 * is not forbidden, but discouraged). The encoding of the data is usually extension specific, i.e.
 * determined by the source, and is usually not documented as part of the MAVLink specification.
 */
export class Tunnel extends MavLinkData {
  static MSG_ID = 385
  static MSG_NAME = 'TUNNEL'
  static PAYLOAD_LENGTH = 133
  static MAGIC_NUMBER = 147

  static FIELDS = [
    new MavLinkPacketField('payload_type', 'payloadType', 0, false, 2, 'uint16_t', ''),
    new MavLinkPacketField('target_system', 'targetSystem', 2, false, 1, 'uint8_t', ''),
    new MavLinkPacketField('target_component', 'targetComponent', 3, false, 1, 'uint8_t', ''),
    new MavLinkPacketField('payload_length', 'payloadLength', 4, false, 1, 'uint8_t', ''),
    new MavLinkPacketField('payload', 'payload', 5, false, 1, 'uint8_t[]', '', 128),
  ]

  constructor() {
    super()
    this.targetSystem = 0
    this.targetComponent = 0
    this.payloadType = MavTunnelPayloadType[Object.keys(MavTunnelPayloadType)[0]]
    this.payloadLength = 0
    this.payload = []
  }

  /**
   * System ID (can be 0 for broadcast, but this is discouraged)
   */
  targetSystem: uint8_t

  /**
   * Component ID (can be 0 for broadcast, but this is discouraged)
   */
  targetComponent: uint8_t

  /**
   * A code that identifies the content of the payload (0 for unknown, which is the default). If this
   * code is less than 32768, it is a 'registered' payload type and the corresponding code should be
   * added to the MAV_TUNNEL_PAYLOAD_TYPE enum. Software creators can register blocks of types as needed.
   * Codes greater than 32767 are considered local experiments and should not be checked in to any widely
   * distributed codebase.
   */
  payloadType: MavTunnelPayloadType

  /**
   * Length of the data transported in payload
   */
  payloadLength: uint8_t

  /**
   * Variable length payload. The payload length is defined by payload_length. The entire content of this
   * block is opaque unless you understand the encoding specified by payload_type.
   */
  payload: uint8_t[]
}

/**
 * Play vehicle tone/tune (buzzer). Supersedes message PLAY_TUNE.
 */
export class PlayTuneV2 extends MavLinkData {
  static MSG_ID = 400
  static MSG_NAME = 'PLAY_TUNE_V2'
  static PAYLOAD_LENGTH = 254
  static MAGIC_NUMBER = 110

  static FIELDS = [
    new MavLinkPacketField('format', 'format', 0, false, 4, 'uint32_t', ''),
    new MavLinkPacketField('target_system', 'targetSystem', 4, false, 1, 'uint8_t', ''),
    new MavLinkPacketField('target_component', 'targetComponent', 5, false, 1, 'uint8_t', ''),
    new MavLinkPacketField('tune', 'tune', 6, false, 1, 'char[]', '', 248),
  ]

  constructor() {
    super()
    this.targetSystem = 0
    this.targetComponent = 0
    this.format = TuneFormat[Object.keys(TuneFormat)[0]]
    this.tune = ''
  }

  /**
   * System ID
   */
  targetSystem: uint8_t

  /**
   * Component ID
   */
  targetComponent: uint8_t

  /**
   * Tune format
   */
  format: TuneFormat

  /**
   * Tune definition as a NULL-terminated string.
   */
  tune: string
}

/**
 * Tune formats supported by vehicle. This should be emitted as response to MAV_CMD_REQUEST_MESSAGE.
 */
export class SupportedTunes extends MavLinkData {
  static MSG_ID = 401
  static MSG_NAME = 'SUPPORTED_TUNES'
  static PAYLOAD_LENGTH = 6
  static MAGIC_NUMBER = 183

  static FIELDS = [
    new MavLinkPacketField('format', 'format', 0, false, 4, 'uint32_t', ''),
    new MavLinkPacketField('target_system', 'targetSystem', 4, false, 1, 'uint8_t', ''),
    new MavLinkPacketField('target_component', 'targetComponent', 5, false, 1, 'uint8_t', ''),
  ]

  constructor() {
    super()
    this.targetSystem = 0
    this.targetComponent = 0
    this.format = TuneFormat[Object.keys(TuneFormat)[0]]
  }

  /**
   * System ID
   */
  targetSystem: uint8_t

  /**
   * Component ID
   */
  targetComponent: uint8_t

  /**
   * Bitfield of supported tune formats.
   */
  format: TuneFormat
}

/**
 * Message for specify component.
 */
export class ComponentExtension43 extends MavLinkData {
  static MSG_ID = 500
  static MSG_NAME = 'COMPONENT_EXTENSION43'
  static PAYLOAD_LENGTH = 48
  static MAGIC_NUMBER = 243

  static FIELDS = [
    new MavLinkPacketField('message_type', 'messageType', 0, false, 2, 'uint16_t', ''),
    new MavLinkPacketField('target_network', 'targetNetwork', 2, false, 1, 'uint8_t', ''),
    new MavLinkPacketField('target_system', 'targetSystem', 3, false, 1, 'uint8_t', ''),
    new MavLinkPacketField('target_component', 'targetComponent', 4, false, 1, 'uint8_t', ''),
    new MavLinkPacketField('payload', 'payload', 5, false, 1, 'uint8_t[]', '', 43),
  ]

  constructor() {
    super()
    this.targetNetwork = 0
    this.targetSystem = 0
    this.targetComponent = 0
    this.messageType = 0
    this.payload = []
  }

  /**
   * Network ID (0 for broadcast)
   */
  targetNetwork: uint8_t

  /**
   * System ID (0 for broadcast)
   */
  targetSystem: uint8_t

  /**
   * Component ID (0 for broadcast)
   */
  targetComponent: uint8_t

  /**
   * A code that identifies the software component that understands this message (analogous to USB device
   * classes or mime type strings). If this code is less than 32768, it is considered a 'registered'
   * protocol extension and the corresponding entry should be added to
   * https://github.com/mavlink/mavlink/definition_files/extension_message_ids.xml. Software creators can
   * register blocks of message IDs as needed (useful for GCS specific metadata, etc...). Message_types
   * greater than 32767 are considered local experiments and should not be checked in to any widely
   * distributed codebase.
   */
  messageType: uint16_t

  /**
   * Variable length payload. The length must be encoded in the payload as part of the message_type
   * protocol, e.g. by including the length as payload data, or by terminating the payload data with a
   * non-zero marker. This is required in order to reconstruct zero-terminated payloads that are (or
   * otherwise would be) trimmed by MAVLink 2 empty-byte truncation. The entire content of the payload
   * block is opaque unless you understand the encoding message_type. The particular encoding used can be
   * extension specific and might not always be documented as part of the MAVLink specification.
   */
  payload: uint8_t[]
}

/**
 * Battery information. Updates GCS with flight controller battery status. Smart batteries also use
 * this message, but may additionally send SMART_BATTERY_INFO.
 */
export class BatteryStatusAcfly extends MavLinkData {
  static MSG_ID = 602
  static MSG_NAME = 'BATTERY_STATUS_ACFLY'
  static PAYLOAD_LENGTH = 72
  static MAGIC_NUMBER = 159

  static FIELDS = [
    new MavLinkPacketField('voltage', 'voltage', 0, false, 4, 'uint32_t', '100 mv'),
    new MavLinkPacketField('capacity', 'capacity', 4, false, 4, 'uint32_t', 'mAh'),
    new MavLinkPacketField('sequence_num', 'sequenceNum', 8, false, 4, 'uint32_t', ''),
    new MavLinkPacketField('fault_bitmask', 'faultBitmask', 12, false, 2, 'uint16_t', ''),
    new MavLinkPacketField('temperature', 'temperature', 14, false, 2, 'int16_t', 'deg'),
    new MavLinkPacketField('cycle_count', 'cycleCount', 16, false, 2, 'uint16_t', ''),
    new MavLinkPacketField('current', 'current', 18, false, 2, 'int16_t', '100 mA'),
    new MavLinkPacketField('id', 'id', 20, false, 1, 'uint8_t', ''),
    new MavLinkPacketField('health', 'health', 21, false, 1, 'uint8_t', '%'),
    new MavLinkPacketField('remaining_percentage', 'remainingPercentage', 22, false, 1, 'uint8_t', '%'),
    new MavLinkPacketField('cell_count', 'cellCount', 23, false, 1, 'uint8_t', ''),
    new MavLinkPacketField('voltages', 'voltages', 24, false, 1, 'uint8_t[]', '100 mV', 48),
  ]

  constructor() {
    super()
    this.id = 0
    this.health = 0
    this.remainingPercentage = 0
    this.cellCount = 0
    this.faultBitmask = 0
    this.temperature = 0
    this.cycleCount = 0
    this.current = 0
    this.voltage = 0
    this.capacity = 0
    this.sequenceNum = 0
    this.voltages = []
  }

  /**
   * Battery ID
   */
  id: uint8_t

  /**
   * battery health, Values: [0-100], UINT8_MAX for unknown
   * Units: %
   */
  health: uint8_t

  /**
   * Remaining battery energy. Values: [0-100], UINT8_MAX for unknown
   * Units: %
   */
  remainingPercentage: uint8_t

  /**
   * cell_count
   */
  cellCount: uint8_t

  /**
   * Fault/health indications.
   */
  faultBitmask: uint16_t

  /**
   * Temperature1 of the battery. INT16_MAX for unknown temperature.
   * Units: deg
   */
  temperature: int16_t

  /**
   * cycle count, UINT16_MAX for unknown
   */
  cycleCount: uint16_t

  /**
   * Battery current, INT16_MAX for unknown
   * Units: 100 mA
   */
  current: int16_t

  /**
   * Battery voltage, UINT32_MAX for unknown
   * Units: 100 mv
   */
  voltage: uint32_t

  /**
   * capacity_actual, UINT32_MAX for unknown
   * Units: mAh
   */
  capacity: uint32_t

  /**
   * Battery sequence num, UINT32_MAX for unknown
   */
  sequenceNum: uint32_t

  /**
   * Battery voltage of cells 1 to 48 (see voltages_ext for cells 11-14). Cells in this field above the
   * valid cell count for this battery should have the UINT16_MAX value. If individual cell voltages are
   * unknown or not measured for this battery, then the overall battery voltage should be filled in cell
   * 0, with all others set to UINT16_MAX. If the voltage of the battery is greater than (UINT16_MAX -
   * 1), then cell 0 should be set to (UINT16_MAX - 1), and cell 1 to the remaining voltage. This can be
   * extended to multiple cells if the total voltage is greater than 2 * (UINT16_MAX - 1).
   * Units: 100 mV
   */
  voltages: uint8_t[]
}

/**
 * xinguangfei pair code request
 */
export class OneToMoreAddrRequestXinguangfei extends MavLinkData {
  static MSG_ID = 800
  static MSG_NAME = 'ONE_TO_MORE_ADDR_REQUEST_XINGUANGFEI'
  static PAYLOAD_LENGTH = 9
  static MAGIC_NUMBER = 31

  static FIELDS = [
    new MavLinkPacketField('request', 'request', 0, false, 1, 'uint8_t', ''),
    new MavLinkPacketField('reserved', 'reserved', 1, false, 1, 'uint8_t[]', '', 8),
  ]

  constructor() {
    super()
    this.request = 0
    this.reserved = []
  }

  /**
   * request
   */
  request: uint8_t

  /**
   * reserved
   */
  reserved: uint8_t[]
}

/**
 * Battery information. Updates GCS with flight controller battery status. Smart batteries also use
 * this message, but may additionally send SMART_BATTERY_INFO.
 */
export class OneToMoreAddrXinguangfei extends MavLinkData {
  static MSG_ID = 801
  static MSG_NAME = 'ONE_TO_MORE_ADDR_XINGUANGFEI'
  static PAYLOAD_LENGTH = 15
  static MAGIC_NUMBER = 82

  static FIELDS = [
    new MavLinkPacketField('mtx_address', 'mtxAddress', 0, false, 1, 'uint8_t[]', '', 5),
    new MavLinkPacketField('mrx_address_ack', 'mrxAddressAck', 5, false, 1, 'uint8_t[]', '%', 5),
    new MavLinkPacketField('mrx_address_p1', 'mrxAddressP1', 10, false, 1, 'uint8_t[]', '%', 5),
  ]

  constructor() {
    super()
    this.mtxAddress = []
    this.mrxAddressAck = []
    this.mrxAddressP1 = []
  }

  /**
   * mtx address
   */
  mtxAddress: uint8_t[]

  /**
   * mrx address ack
   * Units: %
   */
  mrxAddressAck: uint8_t[]

  /**
   * mrx address p1
   * Units: %
   */
  mrxAddressP1: uint8_t[]
}

/**
 * Cumulative distance traveled for each reported wheel.
 */
export class WheelDistance extends MavLinkData {
  static MSG_ID = 9000
  static MSG_NAME = 'WHEEL_DISTANCE'
  static PAYLOAD_LENGTH = 137
  static MAGIC_NUMBER = 113

  static FIELDS = [
    new MavLinkPacketField('time_usec', 'timeUsec', 0, false, 8, 'uint64_t', 'us'),
    new MavLinkPacketField('distance', 'distance', 8, false, 8, 'double[]', 'm', 16),
    new MavLinkPacketField('count', 'count', 136, false, 1, 'uint8_t', ''),
  ]

  constructor() {
    super()
    this.timeUsec = BigInt(0)
    this.count = 0
    this.distance = []
  }

  /**
   * Timestamp (synced to UNIX time or since system boot).
   * Units: us
   */
  timeUsec: uint64_t

  /**
   * Number of wheels reported.
   */
  count: uint8_t

  /**
   * Distance reported by individual wheel encoders. Forward rotations increase values, reverse rotations
   * decrease them. Not all wheels will necessarily have wheel encoders; the mapping of encoders to wheel
   * positions must be agreed/understood by the endpoints.
   * Units: m
   */
  distance: double[]
}

/**
 * Winch status.
 */
export class WinchStatus extends MavLinkData {
  static MSG_ID = 9005
  static MSG_NAME = 'WINCH_STATUS'
  static PAYLOAD_LENGTH = 34
  static MAGIC_NUMBER = 117

  static FIELDS = [
    new MavLinkPacketField('time_usec', 'timeUsec', 0, false, 8, 'uint64_t', 'us'),
    new MavLinkPacketField('line_length', 'lineLength', 8, false, 4, 'float', 'm'),
    new MavLinkPacketField('speed', 'speed', 12, false, 4, 'float', 'm/s'),
    new MavLinkPacketField('tension', 'tension', 16, false, 4, 'float', 'kg'),
    new MavLinkPacketField('voltage', 'voltage', 20, false, 4, 'float', 'V'),
    new MavLinkPacketField('current', 'current', 24, false, 4, 'float', 'A'),
    new MavLinkPacketField('status', 'status', 28, false, 4, 'uint32_t', ''),
    new MavLinkPacketField('temperature', 'temperature', 32, false, 2, 'int16_t', 'degC'),
  ]

  constructor() {
    super()
    this.timeUsec = BigInt(0)
    this.lineLength = 0
    this.speed = 0
    this.tension = 0
    this.voltage = 0
    this.current = 0
    this.temperature = 0
    this.status = MavWinchStatusFlag[Object.keys(MavWinchStatusFlag)[0]]
  }

  /**
   * Timestamp (synced to UNIX time or since system boot).
   * Units: us
   */
  timeUsec: uint64_t

  /**
   * Length of line released. NaN if unknown
   * Units: m
   */
  lineLength: float

  /**
   * Speed line is being released or retracted. Positive values if being released, negative values if
   * being retracted, NaN if unknown
   * Units: m/s
   */
  speed: float

  /**
   * Tension on the line. NaN if unknown
   * Units: kg
   */
  tension: float

  /**
   * Voltage of the battery supplying the winch. NaN if unknown
   * Units: V
   */
  voltage: float

  /**
   * Current draw from the winch. NaN if unknown
   * Units: A
   */
  current: float

  /**
   * Temperature of the motor. INT16_MAX if unknown
   * Units: degC
   */
  temperature: int16_t

  /**
   * Status flags
   */
  status: MavWinchStatusFlag
}

/**
 * Transmitter (remote ID system) is enabled and ready to start sending location and other required
 * information. This is streamed by transmitter. A flight controller uses it as a condition to arm.
 */
export class OpenDroneIdArmStatus extends MavLinkData {
  static MSG_ID = 12918
  static MSG_NAME = 'OPEN_DRONE_ID_ARM_STATUS'
  static PAYLOAD_LENGTH = 51
  static MAGIC_NUMBER = 139

  static FIELDS = [
    new MavLinkPacketField('status', 'status', 0, false, 1, 'uint8_t', ''),
    new MavLinkPacketField('error', 'error', 1, false, 1, 'char[]', '', 50),
  ]

  constructor() {
    super()
    this.status = MavOdidArmStatus[Object.keys(MavOdidArmStatus)[0]]
    this.error = ''
  }

  /**
   * Status level indicating if arming is allowed.
   */
  status: MavOdidArmStatus

  /**
   * Text error message, should be empty if status is good to arm. Fill with nulls in unused portion.
   */
  error: string
}

/**
 * Update the data in the OPEN_DRONE_ID_SYSTEM message with new location information. This can be sent
 * to update the location information for the operator when no other information in the SYSTEM message
 * has changed. This message allows for efficient operation on radio links which have limited uplink
 * bandwidth while meeting requirements for update frequency of the operator location.
 */
export class OpenDroneIdSystemUpdate extends MavLinkData {
  static MSG_ID = 12919
  static MSG_NAME = 'OPEN_DRONE_ID_SYSTEM_UPDATE'
  static PAYLOAD_LENGTH = 18
  static MAGIC_NUMBER = 7

  static FIELDS = [
    new MavLinkPacketField('operator_latitude', 'operatorLatitude', 0, false, 4, 'int32_t', 'degE7'),
    new MavLinkPacketField('operator_longitude', 'operatorLongitude', 4, false, 4, 'int32_t', 'degE7'),
    new MavLinkPacketField('operator_altitude_geo', 'operatorAltitudeGeo', 8, false, 4, 'float', 'm'),
    new MavLinkPacketField('timestamp', 'timestamp', 12, false, 4, 'uint32_t', 's'),
    new MavLinkPacketField('target_system', 'targetSystem', 16, false, 1, 'uint8_t', ''),
    new MavLinkPacketField('target_component', 'targetComponent', 17, false, 1, 'uint8_t', ''),
  ]

  constructor() {
    super()
    this.targetSystem = 0
    this.targetComponent = 0
    this.operatorLatitude = 0
    this.operatorLongitude = 0
    this.operatorAltitudeGeo = 0
    this.timestamp = 0
  }

  /**
   * System ID (0 for broadcast).
   */
  targetSystem: uint8_t

  /**
   * Component ID (0 for broadcast).
   */
  targetComponent: uint8_t

  /**
   * Latitude of the operator. If unknown: 0 (both Lat/Lon).
   * Units: degE7
   */
  operatorLatitude: int32_t

  /**
   * Longitude of the operator. If unknown: 0 (both Lat/Lon).
   * Units: degE7
   */
  operatorLongitude: int32_t

  /**
   * Geodetic altitude of the operator relative to WGS84. If unknown: -1000 m.
   * Units: m
   */
  operatorAltitudeGeo: float

  /**
   * 32 bit Unix Timestamp in seconds since 00:00:00 01/01/2019.
   * Units: s
   */
  timestamp: uint32_t
}

/**
 * Temperature and humidity from hygrometer.
 */
export class HygrometerSensor extends MavLinkData {
  static MSG_ID = 12920
  static MSG_NAME = 'HYGROMETER_SENSOR'
  static PAYLOAD_LENGTH = 5
  static MAGIC_NUMBER = 20

  static FIELDS = [
    new MavLinkPacketField('temperature', 'temperature', 0, false, 2, 'int16_t', 'cdegC'),
    new MavLinkPacketField('humidity', 'humidity', 2, false, 2, 'uint16_t', 'c%'),
    new MavLinkPacketField('id', 'id', 4, false, 1, 'uint8_t', ''),
  ]

  constructor() {
    super()
    this.id = 0
    this.temperature = 0
    this.humidity = 0
  }

  /**
   * Hygrometer ID
   */
  id: uint8_t

  /**
   * Temperature
   * Units: cdegC
   */
  temperature: int16_t

  /**
   * Humidity
   * Units: c%
   */
  humidity: uint16_t
}

import { CommandLong } from './common'

/**
 * request vehicle to send WGA code.
 */
export class SendWgaCommand extends CommandLong {
  constructor(targetSystem = 1, targetComponent = 1) {
    super()
    this.command = MavCmd.SEND_WGA as number
    this.targetSystem = targetSystem
    this.targetComponent = targetComponent
  }

  /**
   * (uint32_t)0-9:WGA index 10-79:Clear
   */
  get ind() {
    return this._param1
  }
  set ind(value: number) {
    this._param1 = value
  }
}

/**
 * request vehicle to write WGA password.
 */
export class WriteWgaCommand extends CommandLong {
  constructor(targetSystem = 1, targetComponent = 1) {
    super()
    this.command = MavCmd.WRITE_WGA as number
    this.targetSystem = targetSystem
    this.targetComponent = targetComponent
  }

  /**
   * password1
   */
  get password1() {
    return this._param1
  }
  set password1(value: number) {
    this._param1 = value
  }

  /**
   * password2
   */
  get password2() {
    return this._param2
  }
  set password2(value: number) {
    this._param2 = value
  }

  /**
   * password3
   */
  get password3() {
    return this._param3
  }
  set password3(value: number) {
    this._param3 = value
  }

  /**
   * password4
   */
  get password4() {
    return this._param4
  }
  set password4(value: number) {
    this._param4 = value
  }
}

/**
 * request vehicle to write WGA password.
 */
export class SetRtcCommand extends CommandLong {
  constructor(targetSystem = 1, targetComponent = 1) {
    super()
    this.command = MavCmd.SET_RTC as number
    this.targetSystem = targetSystem
    this.targetComponent = targetComponent
  }

  /**
   * bit25-31:year-1980 bit21-24:month bit16-20:date bit11-15:hour bit5-10:minute bit0-4:seconds
   */
  get rtcTime() {
    return this._param1
  }
  set rtcTime(value: number) {
    this._param1 = value
  }
}

/**
 * request vehicle to write WGA password.
 */
export class RequestAcflyPossensorInfoStreamCommand extends CommandLong {
  constructor(targetSystem = 1, targetComponent = 1) {
    super()
    this.command = MavCmd.REQUEST_ACFLY_POSSENSOR_INFO_STREAM as number
    this.targetSystem = targetSystem
    this.targetComponent = targetComponent
  }

  /**
   * possensor ind
   */
  get ind() {
    return this._param1
  }
  set ind(value: number) {
    this._param1 = value
  }

  /**
   * positive:rate divider negative:send times per second equals 0:cancel message
   */
  get rate() {
    return this._param2
  }
  set rate(value: number) {
    this._param2 = value
  }

  /**
   * 1:cancel others(this only)
   */
  get req() {
    return this._param3
  }
  set req(value: number) {
    this._param3 = value
  }
}

/**
 * request vehicle to write WGA password.
 */
export class RequestHtlCommand extends CommandLong {
  constructor(targetSystem = 1, targetComponent = 1) {
    super()
    this.command = MavCmd.REQUEST_HTL as number
    this.targetSystem = targetSystem
    this.targetComponent = targetComponent
  }

  /**
   * HTL time rate divider. Must be positive integer.
   */
  get ratedivider() {
    return this._param1
  }
  set ratedivider(value: number) {
    this._param1 = value
  }
}

/**
 * Navigate to waypoint.
 *
 * This command has location.
 * This command is destination.
 */
export class NavWaypointCommand extends CommandLong {
  constructor(targetSystem = 1, targetComponent = 1) {
    super()
    this.command = MavCmd.NAV_WAYPOINT as number
    this.targetSystem = targetSystem
    this.targetComponent = targetComponent
  }

  /**
   * Hold time. (ignored by fixed wing, time to stay at waypoint for rotary wing)
   *
   * @units s
   * @min: 0
   */
  get hold() {
    return this._param1
  }
  set hold(value: number) {
    this._param1 = value
  }

  /**
   * Acceptance radius (if the sphere with this radius is hit, the waypoint counts as reached)
   *
   * @units m
   * @min: 0
   */
  get acceptRadius() {
    return this._param2
  }
  set acceptRadius(value: number) {
    this._param2 = value
  }

  /**
   * 0 to pass through the WP, if > 0 radius to pass by WP. Positive value for clockwise orbit, negative
   * value for counter-clockwise orbit. Allows trajectory control.
   *
   * @units m
   */
  get passRadius() {
    return this._param3
  }
  set passRadius(value: number) {
    this._param3 = value
  }

  /**
   * Desired yaw angle at waypoint (rotary wing). NaN to use the current system yaw heading mode (e.g.
   * yaw towards next waypoint, yaw to home, etc.).
   *
   * @units deg
   */
  get yaw() {
    return this._param4
  }
  set yaw(value: number) {
    this._param4 = value
  }

  /**
   * Latitude
   */
  get latitude() {
    return this._param5
  }
  set latitude(value: number) {
    this._param5 = value
  }

  /**
   * Longitude
   */
  get longitude() {
    return this._param6
  }
  set longitude(value: number) {
    this._param6 = value
  }

  /**
   * Altitude
   *
   * @units m
   */
  get altitude() {
    return this._param7
  }
  set altitude(value: number) {
    this._param7 = value
  }
}

/**
 * Loiter around this waypoint an unlimited amount of time
 *
 * This command has location.
 * This command is destination.
 */
export class NavLoiterUnlimCommand extends CommandLong {
  constructor(targetSystem = 1, targetComponent = 1) {
    super()
    this.command = MavCmd.NAV_LOITER_UNLIM as number
    this.targetSystem = targetSystem
    this.targetComponent = targetComponent
  }

  /**
   * Loiter radius around waypoint for forward-only moving vehicles (not multicopters). If positive
   * loiter clockwise, else counter-clockwise
   *
   * @units m
   */
  get radius() {
    return this._param3
  }
  set radius(value: number) {
    this._param3 = value
  }

  /**
   * Desired yaw angle. NaN to use the current system yaw heading mode (e.g. yaw towards next waypoint,
   * yaw to home, etc.).
   *
   * @units deg
   */
  get yaw() {
    return this._param4
  }
  set yaw(value: number) {
    this._param4 = value
  }

  /**
   * Latitude
   */
  get latitude() {
    return this._param5
  }
  set latitude(value: number) {
    this._param5 = value
  }

  /**
   * Longitude
   */
  get longitude() {
    return this._param6
  }
  set longitude(value: number) {
    this._param6 = value
  }

  /**
   * Altitude
   *
   * @units m
   */
  get altitude() {
    return this._param7
  }
  set altitude(value: number) {
    this._param7 = value
  }
}

/**
 * Loiter around this waypoint for X turns
 *
 * This command has location.
 * This command is destination.
 */
export class NavLoiterTurnsCommand extends CommandLong {
  constructor(targetSystem = 1, targetComponent = 1) {
    super()
    this.command = MavCmd.NAV_LOITER_TURNS as number
    this.targetSystem = targetSystem
    this.targetComponent = targetComponent
  }

  /**
   * Number of turns.
   *
   * @min: 0
   */
  get turns() {
    return this._param1
  }
  set turns(value: number) {
    this._param1 = value
  }

  /**
   * Leave loiter circle only once heading towards the next waypoint (0 = False)
   *
   * @min: 0
   * @max: 1
   * @increment: 1
   */
  get headingRequired() {
    return this._param2
  }
  set headingRequired(value: number) {
    this._param2 = value
  }

  /**
   * Loiter radius around waypoint for forward-only moving vehicles (not multicopters). If positive
   * loiter clockwise, else counter-clockwise
   *
   * @units m
   */
  get radius() {
    return this._param3
  }
  set radius(value: number) {
    this._param3 = value
  }

  /**
   * Loiter circle exit location and/or path to next waypoint ("xtrack") for forward-only moving vehicles
   * (not multicopters). 0 for the vehicle to converge towards the center xtrack when it leaves the
   * loiter (the line between the centers of the current and next waypoint), 1 to converge to the direct
   * line between the location that the vehicle exits the loiter radius and the next waypoint. Otherwise
   * the angle (in degrees) between the tangent of the loiter circle and the center xtrack at which the
   * vehicle must leave the loiter (and converge to the center xtrack). NaN to use the current system
   * default xtrack behaviour.
   */
  get xtrackLocation() {
    return this._param4
  }
  set xtrackLocation(value: number) {
    this._param4 = value
  }

  /**
   * Latitude
   */
  get latitude() {
    return this._param5
  }
  set latitude(value: number) {
    this._param5 = value
  }

  /**
   * Longitude
   */
  get longitude() {
    return this._param6
  }
  set longitude(value: number) {
    this._param6 = value
  }

  /**
   * Altitude
   *
   * @units m
   */
  get altitude() {
    return this._param7
  }
  set altitude(value: number) {
    this._param7 = value
  }
}

/**
 * Loiter at the specified latitude, longitude and altitude for a certain amount of time. Multicopter
 * vehicles stop at the point (within a vehicle-specific acceptance radius). Forward-only moving
 * vehicles (e.g. fixed-wing) circle the point with the specified radius/direction. If the Heading
 * Required parameter (2) is non-zero forward moving aircraft will only leave the loiter circle once
 * heading towards the next waypoint.
 *
 * This command has location.
 * This command is destination.
 */
export class NavLoiterTimeCommand extends CommandLong {
  constructor(targetSystem = 1, targetComponent = 1) {
    super()
    this.command = MavCmd.NAV_LOITER_TIME as number
    this.targetSystem = targetSystem
    this.targetComponent = targetComponent
  }

  /**
   * Loiter time (only starts once Lat, Lon and Alt is reached).
   *
   * @units s
   * @min: 0
   */
  get time() {
    return this._param1
  }
  set time(value: number) {
    this._param1 = value
  }

  /**
   * Leave loiter circle only once heading towards the next waypoint (0 = False)
   *
   * @min: 0
   * @max: 1
   * @increment: 1
   */
  get headingRequired() {
    return this._param2
  }
  set headingRequired(value: number) {
    this._param2 = value
  }

  /**
   * Loiter radius around waypoint for forward-only moving vehicles (not multicopters). If positive
   * loiter clockwise, else counter-clockwise.
   *
   * @units m
   */
  get radius() {
    return this._param3
  }
  set radius(value: number) {
    this._param3 = value
  }

  /**
   * Loiter circle exit location and/or path to next waypoint ("xtrack") for forward-only moving vehicles
   * (not multicopters). 0 for the vehicle to converge towards the center xtrack when it leaves the
   * loiter (the line between the centers of the current and next waypoint), 1 to converge to the direct
   * line between the location that the vehicle exits the loiter radius and the next waypoint. Otherwise
   * the angle (in degrees) between the tangent of the loiter circle and the center xtrack at which the
   * vehicle must leave the loiter (and converge to the center xtrack). NaN to use the current system
   * default xtrack behaviour.
   */
  get xtrackLocation() {
    return this._param4
  }
  set xtrackLocation(value: number) {
    this._param4 = value
  }

  /**
   * Latitude
   */
  get latitude() {
    return this._param5
  }
  set latitude(value: number) {
    this._param5 = value
  }

  /**
   * Longitude
   */
  get longitude() {
    return this._param6
  }
  set longitude(value: number) {
    this._param6 = value
  }

  /**
   * Altitude
   *
   * @units m
   */
  get altitude() {
    return this._param7
  }
  set altitude(value: number) {
    this._param7 = value
  }
}

/**
 * Return to launch location
 */
export class NavReturnToLaunchCommand extends CommandLong {
  constructor(targetSystem = 1, targetComponent = 1) {
    super()
    this.command = MavCmd.NAV_RETURN_TO_LAUNCH as number
    this.targetSystem = targetSystem
    this.targetComponent = targetComponent
  }
}

/**
 * Land at location.
 *
 * This command has location.
 * This command is destination.
 */
export class NavLandCommand extends CommandLong {
  constructor(targetSystem = 1, targetComponent = 1) {
    super()
    this.command = MavCmd.NAV_LAND as number
    this.targetSystem = targetSystem
    this.targetComponent = targetComponent
  }

  /**
   * Minimum target altitude if landing is aborted (0 = undefined/use system default).
   *
   * @units m
   */
  get abortAlt() {
    return this._param1
  }
  set abortAlt(value: number) {
    this._param1 = value
  }

  /**
   * Precision land mode.
   */
  get landMode() {
    return this._param2
  }
  set landMode(value: number) {
    this._param2 = value
  }

  /**
   * Desired yaw angle. NaN to use the current system yaw heading mode (e.g. yaw towards next waypoint,
   * yaw to home, etc.).
   *
   * @units deg
   */
  get yawAngle() {
    return this._param4
  }
  set yawAngle(value: number) {
    this._param4 = value
  }

  /**
   * Latitude.
   */
  get latitude() {
    return this._param5
  }
  set latitude(value: number) {
    this._param5 = value
  }

  /**
   * Longitude.
   */
  get longitude() {
    return this._param6
  }
  set longitude(value: number) {
    this._param6 = value
  }

  /**
   * Landing altitude (ground level in current frame).
   *
   * @units m
   */
  get altitude() {
    return this._param7
  }
  set altitude(value: number) {
    this._param7 = value
  }
}

/**
 * Takeoff from ground / hand. Vehicles that support multiple takeoff modes (e.g. VTOL quadplane)
 * should take off using the currently configured mode.
 *
 * This command has location.
 * This command is destination.
 */
export class NavTakeoffCommand extends CommandLong {
  constructor(targetSystem = 1, targetComponent = 1) {
    super()
    this.command = MavCmd.NAV_TAKEOFF as number
    this.targetSystem = targetSystem
    this.targetComponent = targetComponent
  }

  /**
   * Minimum pitch (if airspeed sensor present), desired pitch without sensor
   *
   * @units deg
   */
  get pitch() {
    return this._param1
  }
  set pitch(value: number) {
    this._param1 = value
  }

  /**
   * Yaw angle (if magnetometer present), ignored without magnetometer. NaN to use the current system yaw
   * heading mode (e.g. yaw towards next waypoint, yaw to home, etc.).
   *
   * @units deg
   */
  get yaw() {
    return this._param4
  }
  set yaw(value: number) {
    this._param4 = value
  }

  /**
   * Latitude
   */
  get latitude() {
    return this._param5
  }
  set latitude(value: number) {
    this._param5 = value
  }

  /**
   * Longitude
   */
  get longitude() {
    return this._param6
  }
  set longitude(value: number) {
    this._param6 = value
  }

  /**
   * Altitude
   *
   * @units m
   */
  get altitude() {
    return this._param7
  }
  set altitude(value: number) {
    this._param7 = value
  }
}

/**
 * Land at local position (local frame only)
 *
 * This command has location.
 * This command is destination.
 */
export class NavLandLocalCommand extends CommandLong {
  constructor(targetSystem = 1, targetComponent = 1) {
    super()
    this.command = MavCmd.NAV_LAND_LOCAL as number
    this.targetSystem = targetSystem
    this.targetComponent = targetComponent
  }

  /**
   * Landing target number (if available)
   *
   * @min: 0
   * @increment: 1
   */
  get target() {
    return this._param1
  }
  set target(value: number) {
    this._param1 = value
  }

  /**
   * Maximum accepted offset from desired landing position - computed magnitude from spherical
   * coordinates: d = sqrt(x^2 + y^2 + z^2), which gives the maximum accepted distance between the
   * desired landing position and the position where the vehicle is about to land
   *
   * @units m
   * @min: 0
   */
  get offset() {
    return this._param2
  }
  set offset(value: number) {
    this._param2 = value
  }

  /**
   * Landing descend rate
   *
   * @units m/s
   */
  get descendRate() {
    return this._param3
  }
  set descendRate(value: number) {
    this._param3 = value
  }

  /**
   * Desired yaw angle
   *
   * @units rad
   */
  get yaw() {
    return this._param4
  }
  set yaw(value: number) {
    this._param4 = value
  }

  /**
   * Y-axis position
   *
   * @units m
   */
  get yPosition() {
    return this._param5
  }
  set yPosition(value: number) {
    this._param5 = value
  }

  /**
   * X-axis position
   *
   * @units m
   */
  get xPosition() {
    return this._param6
  }
  set xPosition(value: number) {
    this._param6 = value
  }

  /**
   * Z-axis / ground level position
   *
   * @units m
   */
  get zPosition() {
    return this._param7
  }
  set zPosition(value: number) {
    this._param7 = value
  }
}

/**
 * Takeoff from local position (local frame only)
 *
 * This command has location.
 * This command is destination.
 */
export class NavTakeoffLocalCommand extends CommandLong {
  constructor(targetSystem = 1, targetComponent = 1) {
    super()
    this.command = MavCmd.NAV_TAKEOFF_LOCAL as number
    this.targetSystem = targetSystem
    this.targetComponent = targetComponent
  }

  /**
   * Minimum pitch (if airspeed sensor present), desired pitch without sensor
   *
   * @units rad
   */
  get pitch() {
    return this._param1
  }
  set pitch(value: number) {
    this._param1 = value
  }

  /**
   * Takeoff ascend rate
   *
   * @units m/s
   */
  get ascendRate() {
    return this._param3
  }
  set ascendRate(value: number) {
    this._param3 = value
  }

  /**
   * Yaw angle (if magnetometer or another yaw estimation source present), ignored without one of these
   *
   * @units rad
   */
  get yaw() {
    return this._param4
  }
  set yaw(value: number) {
    this._param4 = value
  }

  /**
   * Y-axis position
   *
   * @units m
   */
  get yPosition() {
    return this._param5
  }
  set yPosition(value: number) {
    this._param5 = value
  }

  /**
   * X-axis position
   *
   * @units m
   */
  get xPosition() {
    return this._param6
  }
  set xPosition(value: number) {
    this._param6 = value
  }

  /**
   * Z-axis position
   *
   * @units m
   */
  get zPosition() {
    return this._param7
  }
  set zPosition(value: number) {
    this._param7 = value
  }
}

/**
 * Vehicle following, i.e. this waypoint represents the position of a moving vehicle
 *
 * This command has location.
 */
export class NavFollowCommand extends CommandLong {
  constructor(targetSystem = 1, targetComponent = 1) {
    super()
    this.command = MavCmd.NAV_FOLLOW as number
    this.targetSystem = targetSystem
    this.targetComponent = targetComponent
  }

  /**
   * Following logic to use (e.g. loitering or sinusoidal following) - depends on specific autopilot
   * implementation
   *
   * @increment: 1
   */
  get following() {
    return this._param1
  }
  set following(value: number) {
    this._param1 = value
  }

  /**
   * Ground speed of vehicle to be followed
   *
   * @units m/s
   */
  get groundSpeed() {
    return this._param2
  }
  set groundSpeed(value: number) {
    this._param2 = value
  }

  /**
   * Radius around waypoint. If positive loiter clockwise, else counter-clockwise
   *
   * @units m
   */
  get radius() {
    return this._param3
  }
  set radius(value: number) {
    this._param3 = value
  }

  /**
   * Desired yaw angle.
   *
   * @units deg
   */
  get yaw() {
    return this._param4
  }
  set yaw(value: number) {
    this._param4 = value
  }

  /**
   * Latitude
   */
  get latitude() {
    return this._param5
  }
  set latitude(value: number) {
    this._param5 = value
  }

  /**
   * Longitude
   */
  get longitude() {
    return this._param6
  }
  set longitude(value: number) {
    this._param6 = value
  }

  /**
   * Altitude
   *
   * @units m
   */
  get altitude() {
    return this._param7
  }
  set altitude(value: number) {
    this._param7 = value
  }
}

/**
 * Continue on the current course and climb/descend to specified altitude. When the altitude is reached
 * continue to the next command (i.e., don't proceed to the next command until the desired altitude is
 * reached.
 *
 * This command is destination.
 */
export class NavContinueAndChangeAltCommand extends CommandLong {
  constructor(targetSystem = 1, targetComponent = 1) {
    super()
    this.command = MavCmd.NAV_CONTINUE_AND_CHANGE_ALT as number
    this.targetSystem = targetSystem
    this.targetComponent = targetComponent
  }

  /**
   * Climb or Descend (0 = Neutral, command completes when within 5m of this command's altitude, 1 =
   * Climbing, command completes when at or above this command's altitude, 2 = Descending, command
   * completes when at or below this command's altitude.
   *
   * @min: 0
   * @max: 2
   * @increment: 1
   */
  get action() {
    return this._param1
  }
  set action(value: number) {
    this._param1 = value
  }

  /**
   * Desired altitude
   *
   * @units m
   */
  get altitude() {
    return this._param7
  }
  set altitude(value: number) {
    this._param7 = value
  }
}

/**
 * Begin loiter at the specified Latitude and Longitude. If Lat=Lon=0, then loiter at the current
 * position. Don't consider the navigation command complete (don't leave loiter) until the altitude has
 * been reached. Additionally, if the Heading Required parameter is non-zero the aircraft will not
 * leave the loiter until heading toward the next waypoint.
 *
 * This command has location.
 * This command is destination.
 */
export class NavLoiterToAltCommand extends CommandLong {
  constructor(targetSystem = 1, targetComponent = 1) {
    super()
    this.command = MavCmd.NAV_LOITER_TO_ALT as number
    this.targetSystem = targetSystem
    this.targetComponent = targetComponent
  }

  /**
   * Leave loiter circle only once heading towards the next waypoint (0 = False)
   *
   * @min: 0
   * @max: 1
   * @increment: 1
   */
  get headingRequired() {
    return this._param1
  }
  set headingRequired(value: number) {
    this._param1 = value
  }

  /**
   * Loiter radius around waypoint for forward-only moving vehicles (not multicopters). If positive
   * loiter clockwise, negative counter-clockwise, 0 means no change to standard loiter.
   *
   * @units m
   */
  get radius() {
    return this._param2
  }
  set radius(value: number) {
    this._param2 = value
  }

  /**
   * Loiter circle exit location and/or path to next waypoint ("xtrack") for forward-only moving vehicles
   * (not multicopters). 0 for the vehicle to converge towards the center xtrack when it leaves the
   * loiter (the line between the centers of the current and next waypoint), 1 to converge to the direct
   * line between the location that the vehicle exits the loiter radius and the next waypoint. Otherwise
   * the angle (in degrees) between the tangent of the loiter circle and the center xtrack at which the
   * vehicle must leave the loiter (and converge to the center xtrack). NaN to use the current system
   * default xtrack behaviour.
   *
   * @min: 0
   * @max: 1
   * @increment: 1
   */
  get xtrackLocation() {
    return this._param4
  }
  set xtrackLocation(value: number) {
    this._param4 = value
  }

  /**
   * Latitude
   */
  get latitude() {
    return this._param5
  }
  set latitude(value: number) {
    this._param5 = value
  }

  /**
   * Longitude
   */
  get longitude() {
    return this._param6
  }
  set longitude(value: number) {
    this._param6 = value
  }

  /**
   * Altitude
   *
   * @units m
   */
  get altitude() {
    return this._param7
  }
  set altitude(value: number) {
    this._param7 = value
  }
}

/**
 * Begin following a target
 */
export class DoFollowCommand extends CommandLong {
  constructor(targetSystem = 1, targetComponent = 1) {
    super()
    this.command = MavCmd.DO_FOLLOW as number
    this.targetSystem = targetSystem
    this.targetComponent = targetComponent
  }

  /**
   * System ID (of the FOLLOW_TARGET beacon). Send 0 to disable follow-me and return to the default
   * position hold mode.
   *
   * @min: 0
   * @max: 255
   * @increment: 1
   */
  get systemId() {
    return this._param1
  }
  set systemId(value: number) {
    this._param1 = value
  }

  /**
   * Altitude mode: 0: Keep current altitude, 1: keep altitude difference to target, 2: go to a fixed
   * altitude above home.
   *
   * @min: 0
   * @max: 2
   * @increment: 1
   */
  get altitudeMode() {
    return this._param4
  }
  set altitudeMode(value: number) {
    this._param4 = value
  }

  /**
   * Altitude above home. (used if mode=2)
   *
   * @units m
   */
  get altitude() {
    return this._param5
  }
  set altitude(value: number) {
    this._param5 = value
  }

  /**
   * Time to land in which the MAV should go to the default position hold mode after a message RX
   * timeout.
   *
   * @units s
   * @min: 0
   */
  get timeToLand() {
    return this._param7
  }
  set timeToLand(value: number) {
    this._param7 = value
  }
}

/**
 * Reposition the MAV after a follow target command has been sent
 */
export class DoFollowRepositionCommand extends CommandLong {
  constructor(targetSystem = 1, targetComponent = 1) {
    super()
    this.command = MavCmd.DO_FOLLOW_REPOSITION as number
    this.targetSystem = targetSystem
    this.targetComponent = targetComponent
  }

  /**
   * Camera q1 (where 0 is on the ray from the camera to the tracking device)
   */
  get cameraQ1() {
    return this._param1
  }
  set cameraQ1(value: number) {
    this._param1 = value
  }

  /**
   * Camera q2
   */
  get cameraQ2() {
    return this._param2
  }
  set cameraQ2(value: number) {
    this._param2 = value
  }

  /**
   * Camera q3
   */
  get cameraQ3() {
    return this._param3
  }
  set cameraQ3(value: number) {
    this._param3 = value
  }

  /**
   * Camera q4
   */
  get cameraQ4() {
    return this._param4
  }
  set cameraQ4(value: number) {
    this._param4 = value
  }

  /**
   * altitude offset from target
   *
   * @units m
   */
  get altitudeOffset() {
    return this._param5
  }
  set altitudeOffset(value: number) {
    this._param5 = value
  }

  /**
   * X offset from target
   *
   * @units m
   */
  get xOffset() {
    return this._param6
  }
  set xOffset(value: number) {
    this._param6 = value
  }

  /**
   * Y offset from target
   *
   * @units m
   */
  get yOffset() {
    return this._param7
  }
  set yOffset(value: number) {
    this._param7 = value
  }
}

/**
 * Start orbiting on the circumference of a circle defined by the parameters. Setting values to
 * NaN/INT32_MAX (as appropriate) results in using defaults.
 *
 * This command has location.
 * This command is destination.
 */
export class DoOrbitCommand extends CommandLong {
  constructor(targetSystem = 1, targetComponent = 1) {
    super()
    this.command = MavCmd.DO_ORBIT as number
    this.targetSystem = targetSystem
    this.targetComponent = targetComponent
  }

  /**
   * Radius of the circle. Positive: orbit clockwise. Negative: orbit counter-clockwise. NaN: Use vehicle
   * default radius, or current radius if already orbiting.
   *
   * @units m
   */
  get radius() {
    return this._param1
  }
  set radius(value: number) {
    this._param1 = value
  }

  /**
   * Tangential Velocity. NaN: Use vehicle default velocity, or current velocity if already orbiting.
   *
   * @units m/s
   */
  get velocity() {
    return this._param2
  }
  set velocity(value: number) {
    this._param2 = value
  }

  /**
   * Yaw behavior of the vehicle.
   */
  get yawBehavior() {
    return this._param3
  }
  set yawBehavior(value: number) {
    this._param3 = value
  }

  /**
   * Orbit around the centre point for this many radians (i.e. for a three-quarter orbit set 270*Pi/180).
   * 0: Orbit forever. NaN: Use vehicle default, or current value if already orbiting.
   *
   * @units rad
   * @min: 0
   */
  get orbits() {
    return this._param4
  }
  set orbits(value: number) {
    this._param4 = value
  }

  /**
   * Center point latitude (if no MAV_FRAME specified) / X coordinate according to MAV_FRAME. INT32_MAX
   * (or NaN if sent in COMMAND_LONG): Use current vehicle position, or current center if already
   * orbiting.
   */
  get latitude() {
    return this._param5
  }
  set latitude(value: number) {
    this._param5 = value
  }

  /**
   * Center point longitude (if no MAV_FRAME specified) / Y coordinate according to MAV_FRAME. INT32_MAX
   * (or NaN if sent in COMMAND_LONG): Use current vehicle position, or current center if already
   * orbiting.
   */
  get longitude() {
    return this._param6
  }
  set longitude(value: number) {
    this._param6 = value
  }

  /**
   * Center point altitude (MSL) (if no MAV_FRAME specified) / Z coordinate according to MAV_FRAME. NaN:
   * Use current vehicle altitude.
   */
  get altitude() {
    return this._param7
  }
  set altitude(value: number) {
    this._param7 = value
  }
}

/**
 * Sets the region of interest (ROI) for a sensor set or the vehicle itself. This can then be used by
 * the vehicle's control system to control the vehicle attitude and the attitude of various sensors
 * such as cameras.
 *
 * This command has location.
 */
export class NavRoiCommand extends CommandLong {
  constructor(targetSystem = 1, targetComponent = 1) {
    super()
    this.command = MavCmd.NAV_ROI as number
    this.targetSystem = targetSystem
    this.targetComponent = targetComponent
  }

  /**
   * Region of interest mode.
   */
  get roiMode() {
    return this._param1
  }
  set roiMode(value: number) {
    this._param1 = value
  }

  /**
   * Waypoint index/ target ID. (see MAV_ROI enum)
   *
   * @min: 0
   * @increment: 1
   */
  get wpIndex() {
    return this._param2
  }
  set wpIndex(value: number) {
    this._param2 = value
  }

  /**
   * ROI index (allows a vehicle to manage multiple ROI's)
   *
   * @min: 0
   * @increment: 1
   */
  get roiIndex() {
    return this._param3
  }
  set roiIndex(value: number) {
    this._param3 = value
  }

  /**
   * x the location of the fixed ROI (see MAV_FRAME)
   */
  get x() {
    return this._param5
  }
  set x(value: number) {
    this._param5 = value
  }

  /**
   * y
   */
  get y() {
    return this._param6
  }
  set y(value: number) {
    this._param6 = value
  }

  /**
   * z
   */
  get z() {
    return this._param7
  }
  set z(value: number) {
    this._param7 = value
  }
}

/**
 * Control autonomous path planning on the MAV.
 *
 * This command has location.
 * This command is destination.
 */
export class NavPathplanningCommand extends CommandLong {
  constructor(targetSystem = 1, targetComponent = 1) {
    super()
    this.command = MavCmd.NAV_PATHPLANNING as number
    this.targetSystem = targetSystem
    this.targetComponent = targetComponent
  }

  /**
   * 0: Disable local obstacle avoidance / local path planning (without resetting map), 1: Enable local
   * path planning, 2: Enable and reset local path planning
   *
   * @min: 0
   * @max: 2
   * @increment: 1
   */
  get localCtrl() {
    return this._param1
  }
  set localCtrl(value: number) {
    this._param1 = value
  }

  /**
   * 0: Disable full path planning (without resetting map), 1: Enable, 2: Enable and reset map/occupancy
   * grid, 3: Enable and reset planned route, but not occupancy grid
   *
   * @min: 0
   * @max: 3
   * @increment: 1
   */
  get globalCtrl() {
    return this._param2
  }
  set globalCtrl(value: number) {
    this._param2 = value
  }

  /**
   * Yaw angle at goal
   *
   * @units deg
   */
  get yaw() {
    return this._param4
  }
  set yaw(value: number) {
    this._param4 = value
  }

  /**
   * Latitude/X of goal
   */
  get latitude() {
    return this._param5
  }
  set latitude(value: number) {
    this._param5 = value
  }

  /**
   * Longitude/Y of goal
   */
  get longitude() {
    return this._param6
  }
  set longitude(value: number) {
    this._param6 = value
  }

  /**
   * Altitude/Z of goal
   */
  get altitude() {
    return this._param7
  }
  set altitude(value: number) {
    this._param7 = value
  }
}

/**
 * Navigate to waypoint using a spline path.
 *
 * This command has location.
 * This command is destination.
 */
export class NavSplineWaypointCommand extends CommandLong {
  constructor(targetSystem = 1, targetComponent = 1) {
    super()
    this.command = MavCmd.NAV_SPLINE_WAYPOINT as number
    this.targetSystem = targetSystem
    this.targetComponent = targetComponent
  }

  /**
   * Hold time. (ignored by fixed wing, time to stay at waypoint for rotary wing)
   *
   * @units s
   * @min: 0
   */
  get hold() {
    return this._param1
  }
  set hold(value: number) {
    this._param1 = value
  }

  /**
   * Latitude/X of goal
   */
  get latitude() {
    return this._param5
  }
  set latitude(value: number) {
    this._param5 = value
  }

  /**
   * Longitude/Y of goal
   */
  get longitude() {
    return this._param6
  }
  set longitude(value: number) {
    this._param6 = value
  }

  /**
   * Altitude/Z of goal
   */
  get altitude() {
    return this._param7
  }
  set altitude(value: number) {
    this._param7 = value
  }
}

/**
 * Takeoff from ground using VTOL mode, and transition to forward flight with specified heading. The
 * command should be ignored by vehicles that dont support both VTOL and fixed-wing flight
 * (multicopters, boats,etc.).
 *
 * This command has location.
 * This command is destination.
 */
export class NavVtolTakeoffCommand extends CommandLong {
  constructor(targetSystem = 1, targetComponent = 1) {
    super()
    this.command = MavCmd.NAV_VTOL_TAKEOFF as number
    this.targetSystem = targetSystem
    this.targetComponent = targetComponent
  }

  /**
   * Front transition heading.
   */
  get transitionHeading() {
    return this._param2
  }
  set transitionHeading(value: number) {
    this._param2 = value
  }

  /**
   * Yaw angle. NaN to use the current system yaw heading mode (e.g. yaw towards next waypoint, yaw to
   * home, etc.).
   *
   * @units deg
   */
  get yawAngle() {
    return this._param4
  }
  set yawAngle(value: number) {
    this._param4 = value
  }

  /**
   * Latitude
   */
  get latitude() {
    return this._param5
  }
  set latitude(value: number) {
    this._param5 = value
  }

  /**
   * Longitude
   */
  get longitude() {
    return this._param6
  }
  set longitude(value: number) {
    this._param6 = value
  }

  /**
   * Altitude
   *
   * @units m
   */
  get altitude() {
    return this._param7
  }
  set altitude(value: number) {
    this._param7 = value
  }
}

/**
 * Land using VTOL mode
 *
 * This command has location.
 * This command is destination.
 */
export class NavVtolLandCommand extends CommandLong {
  constructor(targetSystem = 1, targetComponent = 1) {
    super()
    this.command = MavCmd.NAV_VTOL_LAND as number
    this.targetSystem = targetSystem
    this.targetComponent = targetComponent
  }

  /**
   * Landing behaviour.
   */
  get landOptions() {
    return this._param1
  }
  set landOptions(value: number) {
    this._param1 = value
  }

  /**
   * Approach altitude (with the same reference as the Altitude field). NaN if unspecified.
   *
   * @units m
   */
  get approachAltitude() {
    return this._param3
  }
  set approachAltitude(value: number) {
    this._param3 = value
  }

  /**
   * Yaw angle. NaN to use the current system yaw heading mode (e.g. yaw towards next waypoint, yaw to
   * home, etc.).
   *
   * @units deg
   */
  get yaw() {
    return this._param4
  }
  set yaw(value: number) {
    this._param4 = value
  }

  /**
   * Latitude
   */
  get latitude() {
    return this._param5
  }
  set latitude(value: number) {
    this._param5 = value
  }

  /**
   * Longitude
   */
  get longitude() {
    return this._param6
  }
  set longitude(value: number) {
    this._param6 = value
  }

  /**
   * Altitude (ground level) relative to the current coordinate frame. NaN to use system default landing
   * altitude (ignore value).
   *
   * @units m
   */
  get groundAltitude() {
    return this._param7
  }
  set groundAltitude(value: number) {
    this._param7 = value
  }
}

/**
 * hand control over to an external controller
 */
export class NavGuidedEnableCommand extends CommandLong {
  constructor(targetSystem = 1, targetComponent = 1) {
    super()
    this.command = MavCmd.NAV_GUIDED_ENABLE as number
    this.targetSystem = targetSystem
    this.targetComponent = targetComponent
  }

  /**
   * On / Off (> 0.5f on)
   *
   * @min: 0
   * @max: 1
   * @increment: 1
   */
  get enable() {
    return this._param1
  }
  set enable(value: number) {
    this._param1 = value
  }
}

/**
 * Delay the next navigation command a number of seconds or until a specified time
 */
export class NavDelayCommand extends CommandLong {
  constructor(targetSystem = 1, targetComponent = 1) {
    super()
    this.command = MavCmd.NAV_DELAY as number
    this.targetSystem = targetSystem
    this.targetComponent = targetComponent
  }

  /**
   * Delay (-1 to enable time-of-day fields)
   *
   * @units s
   * @min: -1
   * @increment: 1
   */
  get delay() {
    return this._param1
  }
  set delay(value: number) {
    this._param1 = value
  }

  /**
   * hour (24h format, UTC, -1 to ignore)
   *
   * @min: -1
   * @max: 23
   * @increment: 1
   */
  get hour() {
    return this._param2
  }
  set hour(value: number) {
    this._param2 = value
  }

  /**
   * minute (24h format, UTC, -1 to ignore)
   *
   * @min: -1
   * @max: 59
   * @increment: 1
   */
  get minute() {
    return this._param3
  }
  set minute(value: number) {
    this._param3 = value
  }

  /**
   * second (24h format, UTC, -1 to ignore)
   *
   * @min: -1
   * @max: 59
   * @increment: 1
   */
  get second() {
    return this._param4
  }
  set second(value: number) {
    this._param4 = value
  }
}

/**
 * Descend and place payload. Vehicle moves to specified location, descends until it detects a hanging
 * payload has reached the ground, and then releases the payload. If ground is not detected before the
 * reaching the maximum descent value (param1), the command will complete without releasing the
 * payload.
 *
 * This command has location.
 * This command is destination.
 */
export class NavPayloadPlaceCommand extends CommandLong {
  constructor(targetSystem = 1, targetComponent = 1) {
    super()
    this.command = MavCmd.NAV_PAYLOAD_PLACE as number
    this.targetSystem = targetSystem
    this.targetComponent = targetComponent
  }

  /**
   * Maximum distance to descend.
   *
   * @units m
   * @min: 0
   */
  get maxDescent() {
    return this._param1
  }
  set maxDescent(value: number) {
    this._param1 = value
  }

  /**
   * Latitude
   */
  get latitude() {
    return this._param5
  }
  set latitude(value: number) {
    this._param5 = value
  }

  /**
   * Longitude
   */
  get longitude() {
    return this._param6
  }
  set longitude(value: number) {
    this._param6 = value
  }

  /**
   * Altitude
   *
   * @units m
   */
  get altitude() {
    return this._param7
  }
  set altitude(value: number) {
    this._param7 = value
  }
}

/**
 * NOP - This command is only used to mark the upper limit of the NAV/ACTION commands in the
 * enumeration
 */
export class NavLastCommand extends CommandLong {
  constructor(targetSystem = 1, targetComponent = 1) {
    super()
    this.command = MavCmd.NAV_LAST as number
    this.targetSystem = targetSystem
    this.targetComponent = targetComponent
  }
}

/**
 * Delay mission state machine.
 */
export class ConditionDelayCommand extends CommandLong {
  constructor(targetSystem = 1, targetComponent = 1) {
    super()
    this.command = MavCmd.CONDITION_DELAY as number
    this.targetSystem = targetSystem
    this.targetComponent = targetComponent
  }

  /**
   * Delay
   *
   * @units s
   * @min: 0
   */
  get delay() {
    return this._param1
  }
  set delay(value: number) {
    this._param1 = value
  }
}

/**
 * Ascend/descend to target altitude at specified rate. Delay mission state machine until desired
 * altitude reached.
 *
 * This command is destination.
 */
export class ConditionChangeAltCommand extends CommandLong {
  constructor(targetSystem = 1, targetComponent = 1) {
    super()
    this.command = MavCmd.CONDITION_CHANGE_ALT as number
    this.targetSystem = targetSystem
    this.targetComponent = targetComponent
  }

  /**
   * Descent / Ascend rate.
   *
   * @units m/s
   */
  get rate() {
    return this._param1
  }
  set rate(value: number) {
    this._param1 = value
  }

  /**
   * Target Altitude
   *
   * @units m
   */
  get altitude() {
    return this._param7
  }
  set altitude(value: number) {
    this._param7 = value
  }
}

/**
 * Delay mission state machine until within desired distance of next NAV point.
 */
export class ConditionDistanceCommand extends CommandLong {
  constructor(targetSystem = 1, targetComponent = 1) {
    super()
    this.command = MavCmd.CONDITION_DISTANCE as number
    this.targetSystem = targetSystem
    this.targetComponent = targetComponent
  }

  /**
   * Distance.
   *
   * @units m
   * @min: 0
   */
  get distance() {
    return this._param1
  }
  set distance(value: number) {
    this._param1 = value
  }
}

/**
 * Reach a certain target angle.
 */
export class ConditionYawCommand extends CommandLong {
  constructor(targetSystem = 1, targetComponent = 1) {
    super()
    this.command = MavCmd.CONDITION_YAW as number
    this.targetSystem = targetSystem
    this.targetComponent = targetComponent
  }

  /**
   * target angle, 0 is north
   *
   * @units deg
   */
  get angle() {
    return this._param1
  }
  set angle(value: number) {
    this._param1 = value
  }

  /**
   * angular speed
   *
   * @units deg/s
   */
  get angularSpeed() {
    return this._param2
  }
  set angularSpeed(value: number) {
    this._param2 = value
  }

  /**
   * direction: -1: counter clockwise, 1: clockwise
   *
   * @min: -1
   * @max: 1
   * @increment: 2
   */
  get direction() {
    return this._param3
  }
  set direction(value: number) {
    this._param3 = value
  }

  /**
   * 0: absolute angle, 1: relative offset
   *
   * @min: 0
   * @max: 1
   * @increment: 1
   */
  get relative() {
    return this._param4
  }
  set relative(value: number) {
    this._param4 = value
  }
}

/**
 * NOP - This command is only used to mark the upper limit of the CONDITION commands in the enumeration
 */
export class ConditionLastCommand extends CommandLong {
  constructor(targetSystem = 1, targetComponent = 1) {
    super()
    this.command = MavCmd.CONDITION_LAST as number
    this.targetSystem = targetSystem
    this.targetComponent = targetComponent
  }
}

/**
 * Set system mode.
 */
export class DoSetModeCommand extends CommandLong {
  constructor(targetSystem = 1, targetComponent = 1) {
    super()
    this.command = MavCmd.DO_SET_MODE as number
    this.targetSystem = targetSystem
    this.targetComponent = targetComponent
  }

  /**
   * Mode
   */
  get mode() {
    return this._param1
  }
  set mode(value: number) {
    this._param1 = value
  }

  /**
   * Custom mode - this is system specific, please refer to the individual autopilot specifications for
   * details.
   */
  get customMode() {
    return this._param2
  }
  set customMode(value: number) {
    this._param2 = value
  }

  /**
   * Custom sub mode - this is system specific, please refer to the individual autopilot specifications
   * for details.
   */
  get customSubmode() {
    return this._param3
  }
  set customSubmode(value: number) {
    this._param3 = value
  }
}

/**
 * Jump to the desired command in the mission list. Repeat this action only the specified number of
 * times
 */
export class DoJumpCommand extends CommandLong {
  constructor(targetSystem = 1, targetComponent = 1) {
    super()
    this.command = MavCmd.DO_JUMP as number
    this.targetSystem = targetSystem
    this.targetComponent = targetComponent
  }

  /**
   * Sequence number
   *
   * @min: 0
   * @increment: 1
   */
  get number() {
    return this._param1
  }
  set number(value: number) {
    this._param1 = value
  }

  /**
   * Repeat count
   *
   * @min: 0
   * @increment: 1
   */
  get repeat() {
    return this._param2
  }
  set repeat(value: number) {
    this._param2 = value
  }
}

/**
 * Change speed and/or throttle set points.
 */
export class DoChangeSpeedCommand extends CommandLong {
  constructor(targetSystem = 1, targetComponent = 1) {
    super()
    this.command = MavCmd.DO_CHANGE_SPEED as number
    this.targetSystem = targetSystem
    this.targetComponent = targetComponent
  }

  /**
   * Speed type (0=Airspeed, 1=Ground Speed, 2=Climb Speed, 3=Descent Speed)
   *
   * @min: 0
   * @max: 3
   * @increment: 1
   */
  get speedType() {
    return this._param1
  }
  set speedType(value: number) {
    this._param1 = value
  }

  /**
   * Speed (-1 indicates no change)
   *
   * @units m/s
   * @min: -1
   */
  get speed() {
    return this._param2
  }
  set speed(value: number) {
    this._param2 = value
  }

  /**
   * Throttle (-1 indicates no change)
   *
   * @units %
   * @min: -1
   */
  get throttle() {
    return this._param3
  }
  set throttle(value: number) {
    this._param3 = value
  }

  /**
   * 0: absolute, 1: relative
   *
   * @min: 0
   * @max: 1
   * @increment: 1
   */
  get relative() {
    return this._param4
  }
  set relative(value: number) {
    this._param4 = value
  }
}

/**
 * Changes the home location either to the current location or a specified location.
 *
 * This command has location.
 */
export class DoSetHomeCommand extends CommandLong {
  constructor(targetSystem = 1, targetComponent = 1) {
    super()
    this.command = MavCmd.DO_SET_HOME as number
    this.targetSystem = targetSystem
    this.targetComponent = targetComponent
  }

  /**
   * Use current (1=use current location, 0=use specified location)
   *
   * @min: 0
   * @max: 1
   * @increment: 1
   */
  get useCurrent() {
    return this._param1
  }
  set useCurrent(value: number) {
    this._param1 = value
  }

  /**
   * Yaw angle. NaN to use default heading
   *
   * @units deg
   */
  get yaw() {
    return this._param4
  }
  set yaw(value: number) {
    this._param4 = value
  }

  /**
   * Latitude
   */
  get latitude() {
    return this._param5
  }
  set latitude(value: number) {
    this._param5 = value
  }

  /**
   * Longitude
   */
  get longitude() {
    return this._param6
  }
  set longitude(value: number) {
    this._param6 = value
  }

  /**
   * Altitude
   *
   * @units m
   */
  get altitude() {
    return this._param7
  }
  set altitude(value: number) {
    this._param7 = value
  }
}

/**
 * Set a system parameter. Caution! Use of this command requires knowledge of the numeric enumeration
 * value of the parameter.
 */
export class DoSetParameterCommand extends CommandLong {
  constructor(targetSystem = 1, targetComponent = 1) {
    super()
    this.command = MavCmd.DO_SET_PARAMETER as number
    this.targetSystem = targetSystem
    this.targetComponent = targetComponent
  }

  /**
   * Parameter number
   *
   * @min: 0
   * @increment: 1
   */
  get number() {
    return this._param1
  }
  set number(value: number) {
    this._param1 = value
  }

  /**
   * Parameter value
   */
  get value() {
    return this._param2
  }
  set value(value: number) {
    this._param2 = value
  }
}

/**
 * Set a relay to a condition.
 */
export class DoSetRelayCommand extends CommandLong {
  constructor(targetSystem = 1, targetComponent = 1) {
    super()
    this.command = MavCmd.DO_SET_RELAY as number
    this.targetSystem = targetSystem
    this.targetComponent = targetComponent
  }

  /**
   * Relay instance number.
   *
   * @min: 0
   * @increment: 1
   */
  get instance() {
    return this._param1
  }
  set instance(value: number) {
    this._param1 = value
  }

  /**
   * Setting. (1=on, 0=off, others possible depending on system hardware)
   *
   * @min: 0
   * @increment: 1
   */
  get setting() {
    return this._param2
  }
  set setting(value: number) {
    this._param2 = value
  }
}

/**
 * Cycle a relay on and off for a desired number of cycles with a desired period.
 */
export class DoRepeatRelayCommand extends CommandLong {
  constructor(targetSystem = 1, targetComponent = 1) {
    super()
    this.command = MavCmd.DO_REPEAT_RELAY as number
    this.targetSystem = targetSystem
    this.targetComponent = targetComponent
  }

  /**
   * Relay instance number.
   *
   * @min: 0
   * @increment: 1
   */
  get instance() {
    return this._param1
  }
  set instance(value: number) {
    this._param1 = value
  }

  /**
   * Cycle count.
   *
   * @min: 1
   * @increment: 1
   */
  get count() {
    return this._param2
  }
  set count(value: number) {
    this._param2 = value
  }

  /**
   * Cycle time.
   *
   * @units s
   * @min: 0
   */
  get time() {
    return this._param3
  }
  set time(value: number) {
    this._param3 = value
  }
}

/**
 * Set a servo to a desired PWM value.
 */
export class DoSetServoCommand extends CommandLong {
  constructor(targetSystem = 1, targetComponent = 1) {
    super()
    this.command = MavCmd.DO_SET_SERVO as number
    this.targetSystem = targetSystem
    this.targetComponent = targetComponent
  }

  /**
   * Servo instance number.
   *
   * @min: 0
   * @increment: 1
   */
  get instance() {
    return this._param1
  }
  set instance(value: number) {
    this._param1 = value
  }

  /**
   * Pulse Width Modulation.
   *
   * @units us
   * @min: 0
   * @increment: 1
   */
  get pwm() {
    return this._param2
  }
  set pwm(value: number) {
    this._param2 = value
  }
}

/**
 * Cycle a between its nominal setting and a desired PWM for a desired number of cycles with a desired
 * period.
 */
export class DoRepeatServoCommand extends CommandLong {
  constructor(targetSystem = 1, targetComponent = 1) {
    super()
    this.command = MavCmd.DO_REPEAT_SERVO as number
    this.targetSystem = targetSystem
    this.targetComponent = targetComponent
  }

  /**
   * Servo instance number.
   *
   * @min: 0
   * @increment: 1
   */
  get instance() {
    return this._param1
  }
  set instance(value: number) {
    this._param1 = value
  }

  /**
   * Pulse Width Modulation.
   *
   * @units us
   * @min: 0
   * @increment: 1
   */
  get pwm() {
    return this._param2
  }
  set pwm(value: number) {
    this._param2 = value
  }

  /**
   * Cycle count.
   *
   * @min: 1
   * @increment: 1
   */
  get count() {
    return this._param3
  }
  set count(value: number) {
    this._param3 = value
  }

  /**
   * Cycle time.
   *
   * @units s
   * @min: 0
   */
  get time() {
    return this._param4
  }
  set time(value: number) {
    this._param4 = value
  }
}

/**
 * Terminate flight immediately
 */
export class DoFlightterminationCommand extends CommandLong {
  constructor(targetSystem = 1, targetComponent = 1) {
    super()
    this.command = MavCmd.DO_FLIGHTTERMINATION as number
    this.targetSystem = targetSystem
    this.targetComponent = targetComponent
  }

  /**
   * Flight termination activated if > 0.5
   *
   * @min: 0
   * @max: 1
   * @increment: 1
   */
  get terminate() {
    return this._param1
  }
  set terminate(value: number) {
    this._param1 = value
  }
}

/**
 * Change altitude set point.
 */
export class DoChangeAltitudeCommand extends CommandLong {
  constructor(targetSystem = 1, targetComponent = 1) {
    super()
    this.command = MavCmd.DO_CHANGE_ALTITUDE as number
    this.targetSystem = targetSystem
    this.targetComponent = targetComponent
  }

  /**
   * Altitude.
   *
   * @units m
   */
  get altitude() {
    return this._param1
  }
  set altitude(value: number) {
    this._param1 = value
  }

  /**
   * Frame of new altitude.
   */
  get frame() {
    return this._param2
  }
  set frame(value: number) {
    this._param2 = value
  }
}

/**
 * Sets actuators (e.g. servos) to a desired value. The actuator numbers are mapped to specific outputs
 * (e.g. on any MAIN or AUX PWM or UAVCAN) using a flight-stack specific mechanism (i.e. a parameter).
 */
export class DoSetActuatorCommand extends CommandLong {
  constructor(targetSystem = 1, targetComponent = 1) {
    super()
    this.command = MavCmd.DO_SET_ACTUATOR as number
    this.targetSystem = targetSystem
    this.targetComponent = targetComponent
  }

  /**
   * Actuator 1 value, scaled from [-1 to 1]. NaN to ignore.
   *
   * @min: -1
   * @max: 1
   */
  get actuator1() {
    return this._param1
  }
  set actuator1(value: number) {
    this._param1 = value
  }

  /**
   * Actuator 2 value, scaled from [-1 to 1]. NaN to ignore.
   *
   * @min: -1
   * @max: 1
   */
  get actuator2() {
    return this._param2
  }
  set actuator2(value: number) {
    this._param2 = value
  }

  /**
   * Actuator 3 value, scaled from [-1 to 1]. NaN to ignore.
   *
   * @min: -1
   * @max: 1
   */
  get actuator3() {
    return this._param3
  }
  set actuator3(value: number) {
    this._param3 = value
  }

  /**
   * Actuator 4 value, scaled from [-1 to 1]. NaN to ignore.
   *
   * @min: -1
   * @max: 1
   */
  get actuator4() {
    return this._param4
  }
  set actuator4(value: number) {
    this._param4 = value
  }

  /**
   * Actuator 5 value, scaled from [-1 to 1]. NaN to ignore.
   *
   * @min: -1
   * @max: 1
   */
  get actuator5() {
    return this._param5
  }
  set actuator5(value: number) {
    this._param5 = value
  }

  /**
   * Actuator 6 value, scaled from [-1 to 1]. NaN to ignore.
   *
   * @min: -1
   * @max: 1
   */
  get actuator6() {
    return this._param6
  }
  set actuator6(value: number) {
    this._param6 = value
  }

  /**
   * Index of actuator set (i.e if set to 1, Actuator 1 becomes Actuator 7)
   *
   * @min: 0
   * @increment: 1
   */
  get index() {
    return this._param7
  }
  set index(value: number) {
    this._param7 = value
  }
}

/**
 * Mission command to perform a landing. This is used as a marker in a mission to tell the autopilot
 * where a sequence of mission items that represents a landing starts. It may also be sent via a
 * COMMAND_LONG to trigger a landing, in which case the nearest (geographically) landing sequence in
 * the mission will be used. The Latitude/Longitude is optional, and may be set to 0 if not needed. If
 * specified then it will be used to help find the closest landing sequence.
 *
 * This command has location.
 */
export class DoLandStartCommand extends CommandLong {
  constructor(targetSystem = 1, targetComponent = 1) {
    super()
    this.command = MavCmd.DO_LAND_START as number
    this.targetSystem = targetSystem
    this.targetComponent = targetComponent
  }

  /**
   * Latitude
   */
  get latitude() {
    return this._param5
  }
  set latitude(value: number) {
    this._param5 = value
  }

  /**
   * Longitude
   */
  get longitude() {
    return this._param6
  }
  set longitude(value: number) {
    this._param6 = value
  }
}

/**
 * Mission command to perform a landing from a rally point.
 */
export class DoRallyLandCommand extends CommandLong {
  constructor(targetSystem = 1, targetComponent = 1) {
    super()
    this.command = MavCmd.DO_RALLY_LAND as number
    this.targetSystem = targetSystem
    this.targetComponent = targetComponent
  }

  /**
   * Break altitude
   *
   * @units m
   */
  get altitude() {
    return this._param1
  }
  set altitude(value: number) {
    this._param1 = value
  }

  /**
   * Landing speed
   *
   * @units m/s
   */
  get speed() {
    return this._param2
  }
  set speed(value: number) {
    this._param2 = value
  }
}

/**
 * Mission command to safely abort an autonomous landing.
 */
export class DoGoAroundCommand extends CommandLong {
  constructor(targetSystem = 1, targetComponent = 1) {
    super()
    this.command = MavCmd.DO_GO_AROUND as number
    this.targetSystem = targetSystem
    this.targetComponent = targetComponent
  }

  /**
   * Altitude
   *
   * @units m
   */
  get altitude() {
    return this._param1
  }
  set altitude(value: number) {
    this._param1 = value
  }
}

/**
 * Reposition the vehicle to a specific WGS84 global position.
 *
 * This command has location.
 * This command is destination.
 */
export class DoRepositionCommand extends CommandLong {
  constructor(targetSystem = 1, targetComponent = 1) {
    super()
    this.command = MavCmd.DO_REPOSITION as number
    this.targetSystem = targetSystem
    this.targetComponent = targetComponent
  }

  /**
   * Ground speed, less than 0 (-1) for default
   *
   * @units m/s
   * @min: -1
   */
  get speed() {
    return this._param1
  }
  set speed(value: number) {
    this._param1 = value
  }

  /**
   * Bitmask of option flags.
   */
  get bitmask() {
    return this._param2
  }
  set bitmask(value: number) {
    this._param2 = value
  }

  /**
   * Yaw heading. NaN to use the current system yaw heading mode (e.g. yaw towards next waypoint, yaw to
   * home, etc.). For planes indicates loiter direction (0: clockwise, 1: counter clockwise)
   *
   * @units deg
   */
  get yaw() {
    return this._param4
  }
  set yaw(value: number) {
    this._param4 = value
  }

  /**
   * Latitude
   */
  get latitude() {
    return this._param5
  }
  set latitude(value: number) {
    this._param5 = value
  }

  /**
   * Longitude
   */
  get longitude() {
    return this._param6
  }
  set longitude(value: number) {
    this._param6 = value
  }

  /**
   * Altitude
   *
   * @units m
   */
  get altitude() {
    return this._param7
  }
  set altitude(value: number) {
    this._param7 = value
  }
}

/**
 * If in a GPS controlled position mode, hold the current position or continue.
 */
export class DoPauseContinueCommand extends CommandLong {
  constructor(targetSystem = 1, targetComponent = 1) {
    super()
    this.command = MavCmd.DO_PAUSE_CONTINUE as number
    this.targetSystem = targetSystem
    this.targetComponent = targetComponent
  }

  /**
   * 0: Pause current mission or reposition command, hold current position. 1: Continue mission. A VTOL
   * capable vehicle should enter hover mode (multicopter and VTOL planes). A plane should loiter with
   * the default loiter radius.
   *
   * @min: 0
   * @max: 1
   * @increment: 1
   */
  get continue() {
    return this._param1
  }
  set continue(value: number) {
    this._param1 = value
  }
}

/**
 * Set moving direction to forward or reverse.
 */
export class DoSetReverseCommand extends CommandLong {
  constructor(targetSystem = 1, targetComponent = 1) {
    super()
    this.command = MavCmd.DO_SET_REVERSE as number
    this.targetSystem = targetSystem
    this.targetComponent = targetComponent
  }

  /**
   * Direction (0=Forward, 1=Reverse)
   *
   * @min: 0
   * @max: 1
   * @increment: 1
   */
  get reverse() {
    return this._param1
  }
  set reverse(value: number) {
    this._param1 = value
  }
}

/**
 * Sets the region of interest (ROI) to a location. This can then be used by the vehicle's control
 * system to control the vehicle attitude and the attitude of various sensors such as cameras. This
 * command can be sent to a gimbal manager but not to a gimbal device. A gimbal is not to react to this
 * message.
 *
 * This command has location.
 */
export class DoSetRoiLocationCommand extends CommandLong {
  constructor(targetSystem = 1, targetComponent = 1) {
    super()
    this.command = MavCmd.DO_SET_ROI_LOCATION as number
    this.targetSystem = targetSystem
    this.targetComponent = targetComponent
  }

  /**
   * Component ID of gimbal device to address (or 1-6 for non-MAVLink gimbal), 0 for all gimbal device
   * components. Send command multiple times for more than one gimbal (but not all gimbals).
   */
  get gimbalDeviceId() {
    return this._param1
  }
  set gimbalDeviceId(value: number) {
    this._param1 = value
  }

  /**
   * Latitude of ROI location
   *
   * @units degE7
   */
  get latitude() {
    return this._param5
  }
  set latitude(value: number) {
    this._param5 = value
  }

  /**
   * Longitude of ROI location
   *
   * @units degE7
   */
  get longitude() {
    return this._param6
  }
  set longitude(value: number) {
    this._param6 = value
  }

  /**
   * Altitude of ROI location
   *
   * @units m
   */
  get altitude() {
    return this._param7
  }
  set altitude(value: number) {
    this._param7 = value
  }
}

/**
 * Sets the region of interest (ROI) to be toward next waypoint, with optional pitch/roll/yaw offset.
 * This can then be used by the vehicle's control system to control the vehicle attitude and the
 * attitude of various sensors such as cameras. This command can be sent to a gimbal manager but not to
 * a gimbal device. A gimbal device is not to react to this message.
 */
export class DoSetRoiWpnextOffsetCommand extends CommandLong {
  constructor(targetSystem = 1, targetComponent = 1) {
    super()
    this.command = MavCmd.DO_SET_ROI_WPNEXT_OFFSET as number
    this.targetSystem = targetSystem
    this.targetComponent = targetComponent
  }

  /**
   * Component ID of gimbal device to address (or 1-6 for non-MAVLink gimbal), 0 for all gimbal device
   * components. Send command multiple times for more than one gimbal (but not all gimbals).
   */
  get gimbalDeviceId() {
    return this._param1
  }
  set gimbalDeviceId(value: number) {
    this._param1 = value
  }

  /**
   * Pitch offset from next waypoint, positive pitching up
   */
  get pitchOffset() {
    return this._param5
  }
  set pitchOffset(value: number) {
    this._param5 = value
  }

  /**
   * Roll offset from next waypoint, positive rolling to the right
   */
  get rollOffset() {
    return this._param6
  }
  set rollOffset(value: number) {
    this._param6 = value
  }

  /**
   * Yaw offset from next waypoint, positive yawing to the right
   */
  get yawOffset() {
    return this._param7
  }
  set yawOffset(value: number) {
    this._param7 = value
  }
}

/**
 * Cancels any previous ROI command returning the vehicle/sensors to default flight characteristics.
 * This can then be used by the vehicle's control system to control the vehicle attitude and the
 * attitude of various sensors such as cameras. This command can be sent to a gimbal manager but not to
 * a gimbal device. A gimbal device is not to react to this message. After this command the gimbal
 * manager should go back to manual input if available, and otherwise assume a neutral position.
 */
export class DoSetRoiNoneCommand extends CommandLong {
  constructor(targetSystem = 1, targetComponent = 1) {
    super()
    this.command = MavCmd.DO_SET_ROI_NONE as number
    this.targetSystem = targetSystem
    this.targetComponent = targetComponent
  }

  /**
   * Component ID of gimbal device to address (or 1-6 for non-MAVLink gimbal), 0 for all gimbal device
   * components. Send command multiple times for more than one gimbal (but not all gimbals).
   */
  get gimbalDeviceId() {
    return this._param1
  }
  set gimbalDeviceId(value: number) {
    this._param1 = value
  }
}

/**
 * Mount tracks system with specified system ID. Determination of target vehicle position may be done
 * with GLOBAL_POSITION_INT or any other means. This command can be sent to a gimbal manager but not to
 * a gimbal device. A gimbal device is not to react to this message.
 */
export class DoSetRoiSysidCommand extends CommandLong {
  constructor(targetSystem = 1, targetComponent = 1) {
    super()
    this.command = MavCmd.DO_SET_ROI_SYSID as number
    this.targetSystem = targetSystem
    this.targetComponent = targetComponent
  }

  /**
   * System ID
   *
   * @min: 1
   * @max: 255
   * @increment: 1
   */
  get systemId() {
    return this._param1
  }
  set systemId(value: number) {
    this._param1 = value
  }

  /**
   * Component ID of gimbal device to address (or 1-6 for non-MAVLink gimbal), 0 for all gimbal device
   * components. Send command multiple times for more than one gimbal (but not all gimbals).
   */
  get gimbalDeviceId() {
    return this._param2
  }
  set gimbalDeviceId(value: number) {
    this._param2 = value
  }
}

/**
 * Control onboard camera system.
 */
export class DoControlVideoCommand extends CommandLong {
  constructor(targetSystem = 1, targetComponent = 1) {
    super()
    this.command = MavCmd.DO_CONTROL_VIDEO as number
    this.targetSystem = targetSystem
    this.targetComponent = targetComponent
  }

  /**
   * Camera ID (-1 for all)
   *
   * @min: -1
   * @increment: 1
   */
  get id() {
    return this._param1
  }
  set id(value: number) {
    this._param1 = value
  }

  /**
   * Transmission: 0: disabled, 1: enabled compressed, 2: enabled raw
   *
   * @min: 0
   * @max: 2
   * @increment: 1
   */
  get transmission() {
    return this._param2
  }
  set transmission(value: number) {
    this._param2 = value
  }

  /**
   * Transmission mode: 0: video stream, >0: single images every n seconds
   *
   * @units s
   * @min: 0
   */
  get interval() {
    return this._param3
  }
  set interval(value: number) {
    this._param3 = value
  }

  /**
   * Recording: 0: disabled, 1: enabled compressed, 2: enabled raw
   *
   * @min: 0
   * @max: 2
   * @increment: 1
   */
  get recording() {
    return this._param4
  }
  set recording(value: number) {
    this._param4 = value
  }
}

/**
 * Sets the region of interest (ROI) for a sensor set or the vehicle itself. This can then be used by
 * the vehicle's control system to control the vehicle attitude and the attitude of various sensors
 * such as cameras.
 *
 * This command has location.
 */
export class DoSetRoiCommand extends CommandLong {
  constructor(targetSystem = 1, targetComponent = 1) {
    super()
    this.command = MavCmd.DO_SET_ROI as number
    this.targetSystem = targetSystem
    this.targetComponent = targetComponent
  }

  /**
   * Region of interest mode.
   */
  get roiMode() {
    return this._param1
  }
  set roiMode(value: number) {
    this._param1 = value
  }

  /**
   * Waypoint index/ target ID (depends on param 1).
   *
   * @min: 0
   * @increment: 1
   */
  get wpIndex() {
    return this._param2
  }
  set wpIndex(value: number) {
    this._param2 = value
  }

  /**
   * Region of interest index. (allows a vehicle to manage multiple ROI's)
   *
   * @min: 0
   * @increment: 1
   */
  get roiIndex() {
    return this._param3
  }
  set roiIndex(value: number) {
    this._param3 = value
  }
}

/**
 * Configure digital camera. This is a fallback message for systems that have not yet implemented
 * PARAM_EXT_XXX messages and camera definition files (see
 * https://mavlink.io/en/services/camera_def.html ).
 */
export class DoDigicamConfigureCommand extends CommandLong {
  constructor(targetSystem = 1, targetComponent = 1) {
    super()
    this.command = MavCmd.DO_DIGICAM_CONFIGURE as number
    this.targetSystem = targetSystem
    this.targetComponent = targetComponent
  }

  /**
   * Modes: P, TV, AV, M, Etc.
   *
   * @min: 0
   * @increment: 1
   */
  get mode() {
    return this._param1
  }
  set mode(value: number) {
    this._param1 = value
  }

  /**
   * Shutter speed: Divisor number for one second.
   *
   * @min: 0
   * @increment: 1
   */
  get shutterSpeed() {
    return this._param2
  }
  set shutterSpeed(value: number) {
    this._param2 = value
  }

  /**
   * Aperture: F stop number.
   *
   * @min: 0
   */
  get aperture() {
    return this._param3
  }
  set aperture(value: number) {
    this._param3 = value
  }

  /**
   * ISO number e.g. 80, 100, 200, Etc.
   *
   * @min: 0
   * @increment: 1
   */
  get iso() {
    return this._param4
  }
  set iso(value: number) {
    this._param4 = value
  }

  /**
   * Exposure type enumerator.
   */
  get exposure() {
    return this._param5
  }
  set exposure(value: number) {
    this._param5 = value
  }

  /**
   * Command Identity.
   */
  get cmdIdentity() {
    return this._param6
  }
  set cmdIdentity(value: number) {
    this._param6 = value
  }

  /**
   * Main engine cut-off time before camera trigger. (0 means no cut-off)
   *
   * @units ds
   * @min: 0
   * @increment: 1
   */
  get engineCutOff() {
    return this._param7
  }
  set engineCutOff(value: number) {
    this._param7 = value
  }
}

/**
 * Control digital camera. This is a fallback message for systems that have not yet implemented
 * PARAM_EXT_XXX messages and camera definition files (see
 * https://mavlink.io/en/services/camera_def.html ).
 */
export class DoDigicamControlCommand extends CommandLong {
  constructor(targetSystem = 1, targetComponent = 1) {
    super()
    this.command = MavCmd.DO_DIGICAM_CONTROL as number
    this.targetSystem = targetSystem
    this.targetComponent = targetComponent
  }

  /**
   * Session control e.g. show/hide lens
   */
  get sessionControl() {
    return this._param1
  }
  set sessionControl(value: number) {
    this._param1 = value
  }

  /**
   * Zoom's absolute position
   */
  get zoomAbsolute() {
    return this._param2
  }
  set zoomAbsolute(value: number) {
    this._param2 = value
  }

  /**
   * Zooming step value to offset zoom from the current position
   */
  get zoomRelative() {
    return this._param3
  }
  set zoomRelative(value: number) {
    this._param3 = value
  }

  /**
   * Focus Locking, Unlocking or Re-locking
   */
  get focus() {
    return this._param4
  }
  set focus(value: number) {
    this._param4 = value
  }

  /**
   * Shooting Command
   */
  get shootCommand() {
    return this._param5
  }
  set shootCommand(value: number) {
    this._param5 = value
  }

  /**
   * Command Identity
   */
  get cmdIdentity() {
    return this._param6
  }
  set cmdIdentity(value: number) {
    this._param6 = value
  }

  /**
   * Test shot identifier. If set to 1, image will only be captured, but not counted towards internal
   * frame count.
   */
  get shotId() {
    return this._param7
  }
  set shotId(value: number) {
    this._param7 = value
  }
}

/**
 * Mission command to configure a camera or antenna mount
 */
export class DoMountConfigureCommand extends CommandLong {
  constructor(targetSystem = 1, targetComponent = 1) {
    super()
    this.command = MavCmd.DO_MOUNT_CONFIGURE as number
    this.targetSystem = targetSystem
    this.targetComponent = targetComponent
  }

  /**
   * Mount operation mode
   */
  get mode() {
    return this._param1
  }
  set mode(value: number) {
    this._param1 = value
  }

  /**
   * stabilize roll? (1 = yes, 0 = no)
   *
   * @min: 0
   * @max: 1
   * @increment: 1
   */
  get stabilizeRoll() {
    return this._param2
  }
  set stabilizeRoll(value: number) {
    this._param2 = value
  }

  /**
   * stabilize pitch? (1 = yes, 0 = no)
   *
   * @min: 0
   * @max: 1
   * @increment: 1
   */
  get stabilizePitch() {
    return this._param3
  }
  set stabilizePitch(value: number) {
    this._param3 = value
  }

  /**
   * stabilize yaw? (1 = yes, 0 = no)
   *
   * @min: 0
   * @max: 1
   * @increment: 1
   */
  get stabilizeYaw() {
    return this._param4
  }
  set stabilizeYaw(value: number) {
    this._param4 = value
  }

  /**
   * roll input (0 = angle body frame, 1 = angular rate, 2 = angle absolute frame)
   */
  get rollInputMode() {
    return this._param5
  }
  set rollInputMode(value: number) {
    this._param5 = value
  }

  /**
   * pitch input (0 = angle body frame, 1 = angular rate, 2 = angle absolute frame)
   */
  get pitchInputMode() {
    return this._param6
  }
  set pitchInputMode(value: number) {
    this._param6 = value
  }

  /**
   * yaw input (0 = angle body frame, 1 = angular rate, 2 = angle absolute frame)
   */
  get yawInputMode() {
    return this._param7
  }
  set yawInputMode(value: number) {
    this._param7 = value
  }
}

/**
 * Mission command to control a camera or antenna mount
 */
export class DoMountControlCommand extends CommandLong {
  constructor(targetSystem = 1, targetComponent = 1) {
    super()
    this.command = MavCmd.DO_MOUNT_CONTROL as number
    this.targetSystem = targetSystem
    this.targetComponent = targetComponent
  }

  /**
   * pitch depending on mount mode (degrees or degrees/second depending on pitch input).
   */
  get pitch() {
    return this._param1
  }
  set pitch(value: number) {
    this._param1 = value
  }

  /**
   * roll depending on mount mode (degrees or degrees/second depending on roll input).
   */
  get roll() {
    return this._param2
  }
  set roll(value: number) {
    this._param2 = value
  }

  /**
   * yaw depending on mount mode (degrees or degrees/second depending on yaw input).
   */
  get yaw() {
    return this._param3
  }
  set yaw(value: number) {
    this._param3 = value
  }

  /**
   * altitude depending on mount mode.
   *
   * @units m
   */
  get altitude() {
    return this._param4
  }
  set altitude(value: number) {
    this._param4 = value
  }

  /**
   * latitude, set if appropriate mount mode.
   */
  get latitude() {
    return this._param5
  }
  set latitude(value: number) {
    this._param5 = value
  }

  /**
   * longitude, set if appropriate mount mode.
   */
  get longitude() {
    return this._param6
  }
  set longitude(value: number) {
    this._param6 = value
  }

  /**
   * Mount mode.
   */
  get mode() {
    return this._param7
  }
  set mode(value: number) {
    this._param7 = value
  }
}

/**
 * Mission command to set camera trigger distance for this flight. The camera is triggered each time
 * this distance is exceeded. This command can also be used to set the shutter integration time for the
 * camera.
 */
export class DoSetCamTriggDistCommand extends CommandLong {
  constructor(targetSystem = 1, targetComponent = 1) {
    super()
    this.command = MavCmd.DO_SET_CAM_TRIGG_DIST as number
    this.targetSystem = targetSystem
    this.targetComponent = targetComponent
  }

  /**
   * Camera trigger distance. 0 to stop triggering.
   *
   * @units m
   * @min: 0
   */
  get distance() {
    return this._param1
  }
  set distance(value: number) {
    this._param1 = value
  }

  /**
   * Camera shutter integration time. -1 or 0 to ignore
   *
   * @units ms
   * @min: -1
   * @increment: 1
   */
  get shutter() {
    return this._param2
  }
  set shutter(value: number) {
    this._param2 = value
  }

  /**
   * Trigger camera once immediately. (0 = no trigger, 1 = trigger)
   *
   * @min: 0
   * @max: 1
   * @increment: 1
   */
  get trigger() {
    return this._param3
  }
  set trigger(value: number) {
    this._param3 = value
  }
}

/**
 * Mission command to enable the geofence
 */
export class DoFenceEnableCommand extends CommandLong {
  constructor(targetSystem = 1, targetComponent = 1) {
    super()
    this.command = MavCmd.DO_FENCE_ENABLE as number
    this.targetSystem = targetSystem
    this.targetComponent = targetComponent
  }

  /**
   * enable? (0=disable, 1=enable, 2=disable_floor_only)
   *
   * @min: 0
   * @max: 2
   * @increment: 1
   */
  get enable() {
    return this._param1
  }
  set enable(value: number) {
    this._param1 = value
  }
}

/**
 * Mission item/command to release a parachute or enable/disable auto release.
 */
export class DoParachuteCommand extends CommandLong {
  constructor(targetSystem = 1, targetComponent = 1) {
    super()
    this.command = MavCmd.DO_PARACHUTE as number
    this.targetSystem = targetSystem
    this.targetComponent = targetComponent
  }

  /**
   * Action
   */
  get action() {
    return this._param1
  }
  set action(value: number) {
    this._param1 = value
  }
}

/**
 * Command to perform motor test.
 */
export class DoMotorTestCommand extends CommandLong {
  constructor(targetSystem = 1, targetComponent = 1) {
    super()
    this.command = MavCmd.DO_MOTOR_TEST as number
    this.targetSystem = targetSystem
    this.targetComponent = targetComponent
  }

  /**
   * Motor instance number (from 1 to max number of motors on the vehicle).
   *
   * @min: 1
   * @increment: 1
   */
  get instance() {
    return this._param1
  }
  set instance(value: number) {
    this._param1 = value
  }

  /**
   * Throttle type (whether the Throttle Value in param3 is a percentage, PWM value, etc.)
   */
  get throttleType() {
    return this._param2
  }
  set throttleType(value: number) {
    this._param2 = value
  }

  /**
   * Throttle value.
   */
  get throttle() {
    return this._param3
  }
  set throttle(value: number) {
    this._param3 = value
  }

  /**
   * Timeout between tests that are run in sequence.
   *
   * @units s
   * @min: 0
   */
  get timeout() {
    return this._param4
  }
  set timeout(value: number) {
    this._param4 = value
  }

  /**
   * Motor count. Number of motors to test in sequence: 0/1=one motor, 2= two motors, etc. The Timeout
   * (param4) is used between tests.
   *
   * @min: 0
   * @increment: 1
   */
  get motorCount() {
    return this._param5
  }
  set motorCount(value: number) {
    this._param5 = value
  }

  /**
   * Motor test order.
   */
  get testOrder() {
    return this._param6
  }
  set testOrder(value: number) {
    this._param6 = value
  }
}

/**
 * Change to/from inverted flight.
 */
export class DoInvertedFlightCommand extends CommandLong {
  constructor(targetSystem = 1, targetComponent = 1) {
    super()
    this.command = MavCmd.DO_INVERTED_FLIGHT as number
    this.targetSystem = targetSystem
    this.targetComponent = targetComponent
  }

  /**
   * Inverted flight. (0=normal, 1=inverted)
   *
   * @min: 0
   * @max: 1
   * @increment: 1
   */
  get inverted() {
    return this._param1
  }
  set inverted(value: number) {
    this._param1 = value
  }
}

/**
 * Mission command to operate a gripper.
 */
export class DoGripperCommand extends CommandLong {
  constructor(targetSystem = 1, targetComponent = 1) {
    super()
    this.command = MavCmd.DO_GRIPPER as number
    this.targetSystem = targetSystem
    this.targetComponent = targetComponent
  }

  /**
   * Gripper instance number.
   *
   * @min: 1
   * @increment: 1
   */
  get instance() {
    return this._param1
  }
  set instance(value: number) {
    this._param1 = value
  }

  /**
   * Gripper action to perform.
   */
  get action() {
    return this._param2
  }
  set action(value: number) {
    this._param2 = value
  }
}

/**
 * Enable/disable autotune.
 */
export class DoAutotuneEnableCommand extends CommandLong {
  constructor(targetSystem = 1, targetComponent = 1) {
    super()
    this.command = MavCmd.DO_AUTOTUNE_ENABLE as number
    this.targetSystem = targetSystem
    this.targetComponent = targetComponent
  }

  /**
   * Enable (1: enable, 0:disable).
   *
   * @min: 0
   * @max: 1
   * @increment: 1
   */
  get enable() {
    return this._param1
  }
  set enable(value: number) {
    this._param1 = value
  }

  /**
   * Specify which axis are autotuned. 0 indicates autopilot default settings.
   */
  get axis() {
    return this._param2
  }
  set axis(value: number) {
    this._param2 = value
  }
}

/**
 * Sets a desired vehicle turn angle and speed change.
 */
export class NavSetYawSpeedCommand extends CommandLong {
  constructor(targetSystem = 1, targetComponent = 1) {
    super()
    this.command = MavCmd.NAV_SET_YAW_SPEED as number
    this.targetSystem = targetSystem
    this.targetComponent = targetComponent
  }

  /**
   * Yaw angle to adjust steering by.
   *
   * @units deg
   */
  get yaw() {
    return this._param1
  }
  set yaw(value: number) {
    this._param1 = value
  }

  /**
   * Speed.
   *
   * @units m/s
   */
  get speed() {
    return this._param2
  }
  set speed(value: number) {
    this._param2 = value
  }

  /**
   * Final angle. (0=absolute, 1=relative)
   *
   * @min: 0
   * @max: 1
   * @increment: 1
   */
  get angle() {
    return this._param3
  }
  set angle(value: number) {
    this._param3 = value
  }
}

/**
 * Mission command to set camera trigger interval for this flight. If triggering is enabled, the camera
 * is triggered each time this interval expires. This command can also be used to set the shutter
 * integration time for the camera.
 */
export class DoSetCamTriggIntervalCommand extends CommandLong {
  constructor(targetSystem = 1, targetComponent = 1) {
    super()
    this.command = MavCmd.DO_SET_CAM_TRIGG_INTERVAL as number
    this.targetSystem = targetSystem
    this.targetComponent = targetComponent
  }

  /**
   * Camera trigger cycle time. -1 or 0 to ignore.
   *
   * @units ms
   * @min: -1
   * @increment: 1
   */
  get triggerCycle() {
    return this._param1
  }
  set triggerCycle(value: number) {
    this._param1 = value
  }

  /**
   * Camera shutter integration time. Should be less than trigger cycle time. -1 or 0 to ignore.
   *
   * @units ms
   * @min: -1
   * @increment: 1
   */
  get shutterIntegration() {
    return this._param2
  }
  set shutterIntegration(value: number) {
    this._param2 = value
  }
}

/**
 * Mission command to control a camera or antenna mount, using a quaternion as reference.
 */
export class DoMountControlQuatCommand extends CommandLong {
  constructor(targetSystem = 1, targetComponent = 1) {
    super()
    this.command = MavCmd.DO_MOUNT_CONTROL_QUAT as number
    this.targetSystem = targetSystem
    this.targetComponent = targetComponent
  }

  /**
   * quaternion param q1, w (1 in null-rotation)
   */
  get q1() {
    return this._param1
  }
  set q1(value: number) {
    this._param1 = value
  }

  /**
   * quaternion param q2, x (0 in null-rotation)
   */
  get q2() {
    return this._param2
  }
  set q2(value: number) {
    this._param2 = value
  }

  /**
   * quaternion param q3, y (0 in null-rotation)
   */
  get q3() {
    return this._param3
  }
  set q3(value: number) {
    this._param3 = value
  }

  /**
   * quaternion param q4, z (0 in null-rotation)
   */
  get q4() {
    return this._param4
  }
  set q4(value: number) {
    this._param4 = value
  }
}

/**
 * set id of master controller
 */
export class DoGuidedMasterCommand extends CommandLong {
  constructor(targetSystem = 1, targetComponent = 1) {
    super()
    this.command = MavCmd.DO_GUIDED_MASTER as number
    this.targetSystem = targetSystem
    this.targetComponent = targetComponent
  }

  /**
   * System ID
   *
   * @min: 0
   * @max: 255
   * @increment: 1
   */
  get systemId() {
    return this._param1
  }
  set systemId(value: number) {
    this._param1 = value
  }

  /**
   * Component ID
   *
   * @min: 0
   * @max: 255
   * @increment: 1
   */
  get componentId() {
    return this._param2
  }
  set componentId(value: number) {
    this._param2 = value
  }
}

/**
 * Set limits for external control
 */
export class DoGuidedLimitsCommand extends CommandLong {
  constructor(targetSystem = 1, targetComponent = 1) {
    super()
    this.command = MavCmd.DO_GUIDED_LIMITS as number
    this.targetSystem = targetSystem
    this.targetComponent = targetComponent
  }

  /**
   * Timeout - maximum time that external controller will be allowed to control vehicle. 0 means no
   * timeout.
   *
   * @units s
   * @min: 0
   */
  get timeout() {
    return this._param1
  }
  set timeout(value: number) {
    this._param1 = value
  }

  /**
   * Altitude (MSL) min - if vehicle moves below this alt, the command will be aborted and the mission
   * will continue. 0 means no lower altitude limit.
   *
   * @units m
   */
  get minAltitude() {
    return this._param2
  }
  set minAltitude(value: number) {
    this._param2 = value
  }

  /**
   * Altitude (MSL) max - if vehicle moves above this alt, the command will be aborted and the mission
   * will continue. 0 means no upper altitude limit.
   *
   * @units m
   */
  get maxAltitude() {
    return this._param3
  }
  set maxAltitude(value: number) {
    this._param3 = value
  }

  /**
   * Horizontal move limit - if vehicle moves more than this distance from its location at the moment the
   * command was executed, the command will be aborted and the mission will continue. 0 means no
   * horizontal move limit.
   *
   * @units m
   * @min: 0
   */
  get horizMoveLimit() {
    return this._param4
  }
  set horizMoveLimit(value: number) {
    this._param4 = value
  }
}

/**
 * Control vehicle engine. This is interpreted by the vehicles engine controller to change the target
 * engine state. It is intended for vehicles with internal combustion engines
 */
export class DoEngineControlCommand extends CommandLong {
  constructor(targetSystem = 1, targetComponent = 1) {
    super()
    this.command = MavCmd.DO_ENGINE_CONTROL as number
    this.targetSystem = targetSystem
    this.targetComponent = targetComponent
  }

  /**
   * 0: Stop engine, 1:Start Engine
   *
   * @min: 0
   * @max: 1
   * @increment: 1
   */
  get startEngine() {
    return this._param1
  }
  set startEngine(value: number) {
    this._param1 = value
  }

  /**
   * 0: Warm start, 1:Cold start. Controls use of choke where applicable
   *
   * @min: 0
   * @max: 1
   * @increment: 1
   */
  get coldStart() {
    return this._param2
  }
  set coldStart(value: number) {
    this._param2 = value
  }

  /**
   * Height delay. This is for commanding engine start only after the vehicle has gained the specified
   * height. Used in VTOL vehicles during takeoff to start engine after the aircraft is off the ground.
   * Zero for no delay.
   *
   * @units m
   * @min: 0
   */
  get heightDelay() {
    return this._param3
  }
  set heightDelay(value: number) {
    this._param3 = value
  }
}

/**
 * Set the mission item with sequence number seq as current item. This means that the MAV will continue
 * to this mission item on the shortest path (not following the mission items in-between).
 */
export class DoSetMissionCurrentCommand extends CommandLong {
  constructor(targetSystem = 1, targetComponent = 1) {
    super()
    this.command = MavCmd.DO_SET_MISSION_CURRENT as number
    this.targetSystem = targetSystem
    this.targetComponent = targetComponent
  }

  /**
   * Mission sequence value to set
   *
   * @min: 0
   * @increment: 1
   */
  get number() {
    return this._param1
  }
  set number(value: number) {
    this._param1 = value
  }
}

/**
 * NOP - This command is only used to mark the upper limit of the DO commands in the enumeration
 */
export class DoLastCommand extends CommandLong {
  constructor(targetSystem = 1, targetComponent = 1) {
    super()
    this.command = MavCmd.DO_LAST as number
    this.targetSystem = targetSystem
    this.targetComponent = targetComponent
  }
}

/**
 * Trigger calibration. This command will be only accepted if in pre-flight mode. Except for
 * Temperature Calibration, only one sensor should be set in a single message and all others should be
 * zero.
 */
export class PreflightCalibrationCommand extends CommandLong {
  constructor(targetSystem = 1, targetComponent = 1) {
    super()
    this.command = MavCmd.PREFLIGHT_CALIBRATION as number
    this.targetSystem = targetSystem
    this.targetComponent = targetComponent
  }

  /**
   * 1: gyro calibration, 3: gyro temperature calibration
   *
   * @min: 0
   * @max: 3
   * @increment: 1
   */
  get gyroTemperature() {
    return this._param1
  }
  set gyroTemperature(value: number) {
    this._param1 = value
  }

  /**
   * 1: magnetometer calibration
   *
   * @min: 0
   * @max: 1
   * @increment: 1
   */
  get magnetometer() {
    return this._param2
  }
  set magnetometer(value: number) {
    this._param2 = value
  }

  /**
   * 1: ground pressure calibration
   *
   * @min: 0
   * @max: 1
   * @increment: 1
   */
  get groundPressure() {
    return this._param3
  }
  set groundPressure(value: number) {
    this._param3 = value
  }

  /**
   * 1: radio RC calibration, 2: RC trim calibration
   *
   * @min: 0
   * @max: 1
   * @increment: 1
   */
  get remoteControl() {
    return this._param4
  }
  set remoteControl(value: number) {
    this._param4 = value
  }

  /**
   * 1: accelerometer calibration, 2: board level calibration, 3: accelerometer temperature calibration,
   * 4: simple accelerometer calibration
   *
   * @min: 0
   * @max: 4
   * @increment: 1
   */
  get accelerometer() {
    return this._param5
  }
  set accelerometer(value: number) {
    this._param5 = value
  }

  /**
   * 1: APM: compass/motor interference calibration (PX4: airspeed calibration, deprecated), 2: airspeed
   * calibration
   *
   * @min: 0
   * @max: 2
   * @increment: 1
   */
  get compmotOrAirspeed() {
    return this._param6
  }
  set compmotOrAirspeed(value: number) {
    this._param6 = value
  }

  /**
   * 1: ESC calibration, 3: barometer temperature calibration
   *
   * @min: 0
   * @max: 3
   * @increment: 1
   */
  get escOrBaro() {
    return this._param7
  }
  set escOrBaro(value: number) {
    this._param7 = value
  }
}

/**
 * Set sensor offsets. This command will be only accepted if in pre-flight mode.
 */
export class PreflightSetSensorOffsetsCommand extends CommandLong {
  constructor(targetSystem = 1, targetComponent = 1) {
    super()
    this.command = MavCmd.PREFLIGHT_SET_SENSOR_OFFSETS as number
    this.targetSystem = targetSystem
    this.targetComponent = targetComponent
  }

  /**
   * Sensor to adjust the offsets for: 0: gyros, 1: accelerometer, 2: magnetometer, 3: barometer, 4:
   * optical flow, 5: second magnetometer, 6: third magnetometer
   *
   * @min: 0
   * @max: 6
   * @increment: 1
   */
  get sensorType() {
    return this._param1
  }
  set sensorType(value: number) {
    this._param1 = value
  }

  /**
   * X axis offset (or generic dimension 1), in the sensor's raw units
   */
  get xOffset() {
    return this._param2
  }
  set xOffset(value: number) {
    this._param2 = value
  }

  /**
   * Y axis offset (or generic dimension 2), in the sensor's raw units
   */
  get yOffset() {
    return this._param3
  }
  set yOffset(value: number) {
    this._param3 = value
  }

  /**
   * Z axis offset (or generic dimension 3), in the sensor's raw units
   */
  get zOffset() {
    return this._param4
  }
  set zOffset(value: number) {
    this._param4 = value
  }

  /**
   * Generic dimension 4, in the sensor's raw units
   */
  get fourthDimension() {
    return this._param5
  }
  set fourthDimension(value: number) {
    this._param5 = value
  }

  /**
   * Generic dimension 5, in the sensor's raw units
   */
  get fifthDimension() {
    return this._param6
  }
  set fifthDimension(value: number) {
    this._param6 = value
  }

  /**
   * Generic dimension 6, in the sensor's raw units
   */
  get sixthDimension() {
    return this._param7
  }
  set sixthDimension(value: number) {
    this._param7 = value
  }
}

/**
 * Trigger UAVCAN configuration (actuator ID assignment and direction mapping). Note that this maps to
 * the legacy UAVCAN v0 function UAVCAN_ENUMERATE, which is intended to be executed just once during
 * initial vehicle configuration (it is not a normal pre-flight command and has been poorly named).
 */
export class PreflightUavcanCommand extends CommandLong {
  constructor(targetSystem = 1, targetComponent = 1) {
    super()
    this.command = MavCmd.PREFLIGHT_UAVCAN as number
    this.targetSystem = targetSystem
    this.targetComponent = targetComponent
  }

  /**
   * 1: Trigger actuator ID assignment and direction mapping. 0: Cancel command.
   */
  get actuatorId() {
    return this._param1
  }
  set actuatorId(value: number) {
    this._param1 = value
  }
}

/**
 * Request storage of different parameter values and logs. This command will be only accepted if in
 * pre-flight mode.
 */
export class PreflightStorageCommand extends CommandLong {
  constructor(targetSystem = 1, targetComponent = 1) {
    super()
    this.command = MavCmd.PREFLIGHT_STORAGE as number
    this.targetSystem = targetSystem
    this.targetComponent = targetComponent
  }

  /**
   * Parameter storage: 0: READ FROM FLASH/EEPROM, 1: WRITE CURRENT TO FLASH/EEPROM, 2: Reset to
   * defaults, 3: Reset sensor calibration parameter data to factory default (or firmware default if not
   * available)
   *
   * @min: 0
   * @max: 3
   * @increment: 1
   */
  get parameterStorage() {
    return this._param1
  }
  set parameterStorage(value: number) {
    this._param1 = value
  }

  /**
   * Mission storage: 0: READ FROM FLASH/EEPROM, 1: WRITE CURRENT TO FLASH/EEPROM, 2: Reset to defaults
   *
   * @min: 0
   * @max: 2
   * @increment: 1
   */
  get missionStorage() {
    return this._param2
  }
  set missionStorage(value: number) {
    this._param2 = value
  }

  /**
   * Onboard logging: 0: Ignore, 1: Start default rate logging, -1: Stop logging, > 1: logging rate (e.g.
   * set to 1000 for 1000 Hz logging)
   *
   * @units Hz
   * @min: -1
   * @increment: 1
   */
  get loggingRate() {
    return this._param3
  }
  set loggingRate(value: number) {
    this._param3 = value
  }
}

/**
 * Request the reboot or shutdown of system components.
 */
export class PreflightRebootShutdownCommand extends CommandLong {
  constructor(targetSystem = 1, targetComponent = 1) {
    super()
    this.command = MavCmd.PREFLIGHT_REBOOT_SHUTDOWN as number
    this.targetSystem = targetSystem
    this.targetComponent = targetComponent
  }

  /**
   * 0: Do nothing for autopilot, 1: Reboot autopilot, 2: Shutdown autopilot, 3: Reboot autopilot and
   * keep it in the bootloader until upgraded.
   *
   * @min: 0
   * @max: 3
   * @increment: 1
   */
  get autopilot() {
    return this._param1
  }
  set autopilot(value: number) {
    this._param1 = value
  }

  /**
   * 0: Do nothing for onboard computer, 1: Reboot onboard computer, 2: Shutdown onboard computer, 3:
   * Reboot onboard computer and keep it in the bootloader until upgraded.
   *
   * @min: 0
   * @max: 3
   * @increment: 1
   */
  get companion() {
    return this._param2
  }
  set companion(value: number) {
    this._param2 = value
  }

  /**
   * 0: Do nothing for component, 1: Reboot component, 2: Shutdown component, 3: Reboot component and
   * keep it in the bootloader until upgraded
   *
   * @min: 0
   * @max: 3
   * @increment: 1
   */
  get componentAction() {
    return this._param3
  }
  set componentAction(value: number) {
    this._param3 = value
  }

  /**
   * MAVLink Component ID targeted in param3 (0 for all components).
   *
   * @min: 0
   * @max: 255
   * @increment: 1
   */
  get componentId() {
    return this._param4
  }
  set componentId(value: number) {
    this._param4 = value
  }
}

/**
 * Override current mission with command to pause mission, pause mission and move to position,
 * continue/resume mission. When param 1 indicates that the mission is paused (MAV_GOTO_DO_HOLD), param
 * 2 defines whether it holds in place or moves to another position.
 *
 * This command has location.
 * This command is destination.
 */
export class OverrideGotoCommand extends CommandLong {
  constructor(targetSystem = 1, targetComponent = 1) {
    super()
    this.command = MavCmd.OVERRIDE_GOTO as number
    this.targetSystem = targetSystem
    this.targetComponent = targetComponent
  }

  /**
   * MAV_GOTO_DO_HOLD: pause mission and either hold or move to specified position (depending on param2),
   * MAV_GOTO_DO_CONTINUE: resume mission.
   */
  get continue() {
    return this._param1
  }
  set continue(value: number) {
    this._param1 = value
  }

  /**
   * MAV_GOTO_HOLD_AT_CURRENT_POSITION: hold at current position, MAV_GOTO_HOLD_AT_SPECIFIED_POSITION:
   * hold at specified position.
   */
  get position() {
    return this._param2
  }
  set position(value: number) {
    this._param2 = value
  }

  /**
   * Coordinate frame of hold point.
   */
  get frame() {
    return this._param3
  }
  set frame(value: number) {
    this._param3 = value
  }

  /**
   * Desired yaw angle.
   *
   * @units deg
   */
  get yaw() {
    return this._param4
  }
  set yaw(value: number) {
    this._param4 = value
  }

  /**
   * Latitude/X position.
   */
  get latitude() {
    return this._param5
  }
  set latitude(value: number) {
    this._param5 = value
  }

  /**
   * Longitude/Y position.
   */
  get longitude() {
    return this._param6
  }
  set longitude(value: number) {
    this._param6 = value
  }

  /**
   * Altitude/Z position.
   */
  get altitude() {
    return this._param7
  }
  set altitude(value: number) {
    this._param7 = value
  }
}

/**
 * Mission command to set a Camera Auto Mount Pivoting Oblique Survey (Replaces CAM_TRIGG_DIST for this
 * purpose). The camera is triggered each time this distance is exceeded, then the mount moves to the
 * next position. Params 4~6 set-up the angle limits and number of positions for oblique survey, where
 * mount-enabled vehicles automatically roll the camera between shots to emulate an oblique camera
 * setup (providing an increased HFOV). This command can also be used to set the shutter integration
 * time for the camera.
 */
export class ObliqueSurveyCommand extends CommandLong {
  constructor(targetSystem = 1, targetComponent = 1) {
    super()
    this.command = MavCmd.OBLIQUE_SURVEY as number
    this.targetSystem = targetSystem
    this.targetComponent = targetComponent
  }

  /**
   * Camera trigger distance. 0 to stop triggering.
   *
   * @units m
   * @min: 0
   */
  get distance() {
    return this._param1
  }
  set distance(value: number) {
    this._param1 = value
  }

  /**
   * Camera shutter integration time. 0 to ignore
   *
   * @units ms
   * @min: 0
   * @increment: 1
   */
  get shutter() {
    return this._param2
  }
  set shutter(value: number) {
    this._param2 = value
  }

  /**
   * The minimum interval in which the camera is capable of taking subsequent pictures repeatedly. 0 to
   * ignore.
   *
   * @units ms
   * @min: 0
   * @max: 10000
   * @increment: 1
   */
  get minInterval() {
    return this._param3
  }
  set minInterval(value: number) {
    this._param3 = value
  }

  /**
   * Total number of roll positions at which the camera will capture photos (images captures spread
   * evenly across the limits defined by param5).
   *
   * @min: 2
   * @increment: 1
   */
  get positions() {
    return this._param4
  }
  set positions(value: number) {
    this._param4 = value
  }

  /**
   * Angle limits that the camera can be rolled to left and right of center.
   *
   * @units deg
   * @min: 0
   */
  get rollAngle() {
    return this._param5
  }
  set rollAngle(value: number) {
    this._param5 = value
  }

  /**
   * Fixed pitch angle that the camera will hold in oblique mode if the mount is actuated in the pitch
   * axis.
   *
   * @units deg
   * @min: -180
   * @max: 180
   */
  get pitchAngle() {
    return this._param6
  }
  set pitchAngle(value: number) {
    this._param6 = value
  }
}

/**
 * xingangfei ext command take off
 */
export class ExtDroneTakeoffCommand extends CommandLong {
  constructor(targetSystem = 1, targetComponent = 1) {
    super()
    this.command = MavCmd.EXT_DRONE_TAKEOFF as number
    this.targetSystem = targetSystem
    this.targetComponent = targetComponent
  }

  /**
   * front distance
   *
   * @units m
   * @min: 0
   * @max: 2
   */
  get height() {
    return this._param1
  }
  set height(value: number) {
    this._param1 = value
  }
}

/**
 * xingangfei ext command take off
 */
export class ExtDroneLandCommand extends CommandLong {
  constructor(targetSystem = 1, targetComponent = 1) {
    super()
    this.command = MavCmd.EXT_DRONE_LAND as number
    this.targetSystem = targetSystem
    this.targetComponent = targetComponent
  }

  /**
   * 0:normal land 1: qrcode land
   *
   * @min: 0
   * @max: 1
   */
  get land_mode() {
    return this._param1
  }
  set land_mode(value: number) {
    this._param1 = value
  }

  /**
   * land speed
   *
   * @units cm/s
   * @min: 0
   * @max: 200
   */
  get landspeed() {
    return this._param2
  }
  set landspeed(value: number) {
    this._param2 = value
  }
}

/**
 * xingangfei ext command move
 */
export class ExtDroneMoveCommand extends CommandLong {
  constructor(targetSystem = 1, targetComponent = 1) {
    super()
    this.command = MavCmd.EXT_DRONE_MOVE as number
    this.targetSystem = targetSystem
    this.targetComponent = targetComponent
  }

  /**
   * //front:03/behind:04/left:05/right:06/up:01/down:02
   *
   * @min: 0
   * @max: 6
   */
  get direction() {
    return this._param1
  }
  set direction(value: number) {
    this._param1 = value
  }

  /**
   * distance
   *
   * @units cm
   * @min: 0
   * @max: 1000
   */
  get distance() {
    return this._param2
  }
  set distance(value: number) {
    this._param2 = value
  }

  /**
   * speed
   *
   * @units cm/s
   * @min: 0
   * @max: 200
   */
  get speed() {
    return this._param2
  }
  set speed(value: number) {
    this._param2 = value
  }
}

/**
 * xingangfei ext command circle
 */
export class ExtDroneCircleCommand extends CommandLong {
  constructor(targetSystem = 1, targetComponent = 1) {
    super()
    this.command = MavCmd.EXT_DRONE_CIRCLE as number
    this.targetSystem = targetSystem
    this.targetComponent = targetComponent
  }

  /**
   * 0 Counterclockwise 1 clockwise
   *
   * @min: 0
   * @max: 1
   */
  get direction() {
    return this._param1
  }
  set direction(value: number) {
    this._param1 = value
  }

  /**
   * degrees
   *
   * @min: 0
   * @max: 360
   */
  get degrees() {
    return this._param2
  }
  set degrees(value: number) {
    this._param2 = value
  }
}

/**
 * xinguangfei ext command waypoint
 */
export class ExtDroneWaypointCommand extends CommandLong {
  constructor(targetSystem = 1, targetComponent = 1) {
    super()
    this.command = MavCmd.EXT_DRONE_WAYPOINT as number
    this.targetSystem = targetSystem
    this.targetComponent = targetComponent
  }

  /**
   * x Distance
   *
   * @units m
   * @min: -1000
   * @max: 1000
   */
  get xDistance() {
    return this._param1
  }
  set xDistance(value: number) {
    this._param1 = value
  }

  /**
   * y Distance
   *
   * @units m
   * @min: -1000
   * @max: 1000
   */
  get yDistance() {
    return this._param2
  }
  set yDistance(value: number) {
    this._param2 = value
  }

  /**
   * z Distance
   *
   * @units m
   * @min: -200
   * @max: 200
   */
  get zDistance() {
    return this._param3
  }
  set zDistance(value: number) {
    this._param3 = value
  }
}

/**
 * xinguangfei ext change speed
 */
export class ExtDroneChangeSpeedCommand extends CommandLong {
  constructor(targetSystem = 1, targetComponent = 1) {
    super()
    this.command = MavCmd.EXT_DRONE_CHANGE_SPEED as number
    this.targetSystem = targetSystem
    this.targetComponent = targetComponent
  }

  /**
   * speed
   *
   * @units cm/s
   * @min: 0
   * @max: 200
   */
  get speed() {
    return this._param1
  }
  set speed(value: number) {
    this._param1 = value
  }
}

/**
 * xinguangfei ext change speed
 */
export class ExtDroneLightRgbCommand extends CommandLong {
  constructor(targetSystem = 1, targetComponent = 1) {
    super()
    this.command = MavCmd.EXT_DRONE_LIGHT_RGB as number
    this.targetSystem = targetSystem
    this.targetComponent = targetComponent
  }

  /**
   * R
   *
   * @min: 0
   * @max: 255
   */
  get r() {
    return this._param1
  }
  set r(value: number) {
    this._param1 = value
  }

  /**
   * G
   *
   * @min: 0
   * @max: 255
   */
  get g() {
    return this._param2
  }
  set g(value: number) {
    this._param2 = value
  }

  /**
   * b
   *
   * @min: 0
   * @max: 255
   */
  get b() {
    return this._param3
  }
  set b(value: number) {
    this._param3 = value
  }

  /**
   * Breathe
   *
   * @min: 0
   * @max: 1
   */
  get breathe() {
    return this._param4
  }
  set breathe(value: number) {
    this._param4 = value
  }

  /**
   * rainbow
   *
   * @min: 0
   * @max: 1
   */
  get rainbow() {
    return this._param5
  }
  set rainbow(value: number) {
    this._param5 = value
  }
}

/**
 * xinguangfei ext set mode
 */
export class ExtDroneSetModeCommand extends CommandLong {
  constructor(targetSystem = 1, targetComponent = 1) {
    super()
    this.command = MavCmd.EXT_DRONE_SET_MODE as number
    this.targetSystem = targetSystem
    this.targetComponent = targetComponent
  }

  /**
   * 1:normal 2:track line 3:follow
   */
  get mode() {
    return this._param1
  }
  set mode(value: number) {
    this._param1 = value
  }
}

/**
 * xinguangfei ext set mode
 */
export class ExtDroneVersionDetectModeSetCommand extends CommandLong {
  constructor(targetSystem = 1, targetComponent = 1) {
    super()
    this.command = MavCmd.EXT_DRONE_VERSION_DETECT_MODE_SET as number
    this.targetSystem = targetSystem
    this.targetComponent = targetComponent
  }

  /**
   * Min detection value of color block L channel
   */
  get l_l() {
    return this._param1
  }
  set l_l(value: number) {
    this._param1 = value
  }

  /**
   * Max detection value of color block L channel
   */
  get l_h() {
    return this._param2
  }
  set l_h(value: number) {
    this._param2 = value
  }

  /**
   * Min detection value of color block A channel
   */
  get a_l() {
    return this._param3
  }
  set a_l(value: number) {
    this._param3 = value
  }

  /**
   * Max detection value of color block A channel
   */
  get a_h() {
    return this._param4
  }
  set a_h(value: number) {
    this._param4 = value
  }

  /**
   * Min detection value of color block B channel
   */
  get b_l() {
    return this._param5
  }
  set b_l(value: number) {
    this._param5 = value
  }

  /**
   * Max detection value of color block B channel
   */
  get b_h() {
    return this._param6
  }
  set b_h(value: number) {
    this._param6 = value
  }
}

/**
 * xinguangfei ext goto target loc
 */
export class ExtDroneGotoCmdCommand extends CommandLong {
  constructor(targetSystem = 1, targetComponent = 1) {
    super()
    this.command = MavCmd.EXT_DRONE_GOTO_CMD as number
    this.targetSystem = targetSystem
    this.targetComponent = targetComponent
  }

  /**
   * 0:fllowline 1:lock apritag
   *
   * @min: 0
   * @max: 1
   */
  get target_x() {
    return this._param1
  }
  set target_x(value: number) {
    this._param1 = value
  }

  /**
   * null
   */
  get target_y() {
    return this._param2
  }
  set target_y(value: number) {
    this._param2 = value
  }

  /**
   * null
   */
  get target_z() {
    return this._param3
  }
  set target_z(value: number) {
    this._param3 = value
  }
}

/**
 * xinguangfei ext goto target loc
 */
export class ExtDroneOpemmvCmdCommand extends CommandLong {
  constructor(targetSystem = 1, targetComponent = 1) {
    super()
    this.command = MavCmd.EXT_DRONE_OPEMMV_CMD as number
    this.targetSystem = targetSystem
    this.targetComponent = targetComponent
  }

  /**
   * null
   */
  get cmd() {
    return this._param1
  }
  set cmd(value: number) {
    this._param1 = value
  }
}

/**
 * xinguangfei ext set mode
 */
export class ExtDroneTotalCommand extends CommandLong {
  constructor(targetSystem = 1, targetComponent = 1) {
    super()
    this.command = MavCmd.EXT_DRONE_TOTAL as number
    this.targetSystem = targetSystem
    this.targetComponent = targetComponent
  }
}

/**
 * Actuator testing command. This is similar to MAV_CMD_DO_MOTOR_TEST but operates on the level of
 * output functions, i.e. it is possible to test Motor1 independent from which output it is configured
 * on. Autopilots typically refuse this command while armed.
 */
export class ActuatorTestCommand extends CommandLong {
  constructor(targetSystem = 1, targetComponent = 1) {
    super()
    this.command = MavCmd.ACTUATOR_TEST as number
    this.targetSystem = targetSystem
    this.targetComponent = targetComponent
  }

  /**
   * Output value: 1 means maximum positive output, 0 to center servos or minimum motor thrust (expected
   * to spin), -1 for maximum negative (if not supported by the motors, i.e. motor is not reversible,
   * smaller than 0 maps to NaN). And NaN maps to disarmed (stop the motors).
   *
   * @min: -1
   * @max: 1
   */
  get value() {
    return this._param1
  }
  set value(value: number) {
    this._param1 = value
  }

  /**
   * Timeout after which the test command expires and the output is restored to the previous value. A
   * timeout has to be set for safety reasons. A timeout of 0 means to restore the previous value
   * immediately.
   *
   * @units s
   * @min: 0
   * @max: 3
   */
  get timeout() {
    return this._param2
  }
  set timeout(value: number) {
    this._param2 = value
  }

  /**
   * Actuator Output function
   */
  get outputFunction() {
    return this._param5
  }
  set outputFunction(value: number) {
    this._param5 = value
  }
}

/**
 * Actuator configuration command.
 */
export class ConfigureActuatorCommand extends CommandLong {
  constructor(targetSystem = 1, targetComponent = 1) {
    super()
    this.command = MavCmd.CONFIGURE_ACTUATOR as number
    this.targetSystem = targetSystem
    this.targetComponent = targetComponent
  }

  /**
   * Actuator configuration action
   */
  get configuration() {
    return this._param1
  }
  set configuration(value: number) {
    this._param1 = value
  }

  /**
   * Actuator Output function
   */
  get outputFunction() {
    return this._param5
  }
  set outputFunction(value: number) {
    this._param5 = value
  }
}

/**
 * Arms / Disarms a component
 */
export class ComponentArmDisarmCommand extends CommandLong {
  constructor(targetSystem = 1, targetComponent = 1) {
    super()
    this.command = MavCmd.COMPONENT_ARM_DISARM as number
    this.targetSystem = targetSystem
    this.targetComponent = targetComponent
  }

  /**
   * 0: disarm, 1: arm
   *
   * @min: 0
   * @max: 1
   * @increment: 1
   */
  get arm() {
    return this._param1
  }
  set arm(value: number) {
    this._param1 = value
  }

  /**
   * 0: arm-disarm unless prevented by safety checks (i.e. when landed), 21196: force arming/disarming
   * (e.g. allow arming to override preflight checks and disarming in flight)
   *
   * @min: 0
   * @max: 21196
   * @increment: 21196
   */
  get force() {
    return this._param2
  }
  set force(value: number) {
    this._param2 = value
  }
}

/**
 * Instructs system to run pre-arm checks. This command should return MAV_RESULT_TEMPORARILY_REJECTED
 * in the case the system is armed, otherwise MAV_RESULT_ACCEPTED. Note that the return value from
 * executing this command does not indicate whether the vehicle is armable or not, just whether the
 * system has successfully run/is currently running the checks. The result of the checks is reflected
 * in the SYS_STATUS message.
 */
export class RunPrearmChecksCommand extends CommandLong {
  constructor(targetSystem = 1, targetComponent = 1) {
    super()
    this.command = MavCmd.RUN_PREARM_CHECKS as number
    this.targetSystem = targetSystem
    this.targetComponent = targetComponent
  }
}

/**
 * Turns illuminators ON/OFF. An illuminator is a light source that is used for lighting up dark areas
 * external to the sytstem: e.g. a torch or searchlight (as opposed to a light source for illuminating
 * the system itself, e.g. an indicator light).
 */
export class IlluminatorOnOffCommand extends CommandLong {
  constructor(targetSystem = 1, targetComponent = 1) {
    super()
    this.command = MavCmd.ILLUMINATOR_ON_OFF as number
    this.targetSystem = targetSystem
    this.targetComponent = targetComponent
  }

  /**
   * 0: Illuminators OFF, 1: Illuminators ON
   *
   * @min: 0
   * @max: 1
   * @increment: 1
   */
  get enable() {
    return this._param1
  }
  set enable(value: number) {
    this._param1 = value
  }
}

/**
 * Request the home position from the vehicle.
 */
export class GetHomePositionCommand extends CommandLong {
  constructor(targetSystem = 1, targetComponent = 1) {
    super()
    this.command = MavCmd.GET_HOME_POSITION as number
    this.targetSystem = targetSystem
    this.targetComponent = targetComponent
  }
}

/**
 * Inject artificial failure for testing purposes. Note that autopilots should implement an additional
 * protection before accepting this command such as a specific param setting.
 */
export class InjectFailureCommand extends CommandLong {
  constructor(targetSystem = 1, targetComponent = 1) {
    super()
    this.command = MavCmd.INJECT_FAILURE as number
    this.targetSystem = targetSystem
    this.targetComponent = targetComponent
  }

  /**
   * The unit which is affected by the failure.
   */
  get failureUnit() {
    return this._param1
  }
  set failureUnit(value: number) {
    this._param1 = value
  }

  /**
   * The type how the failure manifests itself.
   */
  get failureType() {
    return this._param2
  }
  set failureType(value: number) {
    this._param2 = value
  }

  /**
   * Instance affected by failure (0 to signal all).
   */
  get instance() {
    return this._param3
  }
  set instance(value: number) {
    this._param3 = value
  }
}

/**
 * Starts receiver pairing.
 */
export class StartRxPairCommand extends CommandLong {
  constructor(targetSystem = 1, targetComponent = 1) {
    super()
    this.command = MavCmd.START_RX_PAIR as number
    this.targetSystem = targetSystem
    this.targetComponent = targetComponent
  }

  /**
   * 0:Spektrum.
   */
  get spektrum() {
    return this._param1
  }
  set spektrum(value: number) {
    this._param1 = value
  }

  /**
   * RC type.
   */
  get rcType() {
    return this._param2
  }
  set rcType(value: number) {
    this._param2 = value
  }
}

/**
 * Request the interval between messages for a particular MAVLink message ID. The receiver should ACK
 * the command and then emit its response in a MESSAGE_INTERVAL message.
 */
export class GetMessageIntervalCommand extends CommandLong {
  constructor(targetSystem = 1, targetComponent = 1) {
    super()
    this.command = MavCmd.GET_MESSAGE_INTERVAL as number
    this.targetSystem = targetSystem
    this.targetComponent = targetComponent
  }

  /**
   * The MAVLink message ID
   *
   * @min: 0
   * @max: 16777215
   * @increment: 1
   */
  get messageId() {
    return this._param1
  }
  set messageId(value: number) {
    this._param1 = value
  }
}

/**
 * Set the interval between messages for a particular MAVLink message ID. This interface replaces
 * REQUEST_DATA_STREAM.
 */
export class SetMessageIntervalCommand extends CommandLong {
  constructor(targetSystem = 1, targetComponent = 1) {
    super()
    this.command = MavCmd.SET_MESSAGE_INTERVAL as number
    this.targetSystem = targetSystem
    this.targetComponent = targetComponent
  }

  /**
   * The MAVLink message ID
   *
   * @min: 0
   * @max: 16777215
   * @increment: 1
   */
  get messageId() {
    return this._param1
  }
  set messageId(value: number) {
    this._param1 = value
  }

  /**
   * The interval between two messages. Set to -1 to disable and 0 to request default rate.
   *
   * @units us
   * @min: -1
   * @increment: 1
   */
  get interval() {
    return this._param2
  }
  set interval(value: number) {
    this._param2 = value
  }

  /**
   * Target address of message stream (if message has target address fields). 0: Flight-stack default
   * (recommended), 1: address of requestor, 2: broadcast.
   *
   * @min: 0
   * @max: 2
   * @increment: 1
   */
  get responseTarget() {
    return this._param7
  }
  set responseTarget(value: number) {
    this._param7 = value
  }
}

/**
 * Request the target system(s) emit a single instance of a specified message (i.e. a "one-shot"
 * version of MAV_CMD_SET_MESSAGE_INTERVAL).
 */
export class RequestMessageCommand extends CommandLong {
  constructor(targetSystem = 1, targetComponent = 1) {
    super()
    this.command = MavCmd.REQUEST_MESSAGE as number
    this.targetSystem = targetSystem
    this.targetComponent = targetComponent
  }

  /**
   * The MAVLink message ID of the requested message.
   *
   * @min: 0
   * @max: 16777215
   * @increment: 1
   */
  get messageId() {
    return this._param1
  }
  set messageId(value: number) {
    this._param1 = value
  }

  /**
   * Use for index ID, if required. Otherwise, the use of this parameter (if any) must be defined in the
   * requested message. By default assumed not used (0).
   */
  get reqParam1() {
    return this._param2
  }
  set reqParam1(value: number) {
    this._param2 = value
  }

  /**
   * The use of this parameter (if any), must be defined in the requested message. By default assumed not
   * used (0).
   */
  get reqParam2() {
    return this._param3
  }
  set reqParam2(value: number) {
    this._param3 = value
  }

  /**
   * The use of this parameter (if any), must be defined in the requested message. By default assumed not
   * used (0).
   */
  get reqParam3() {
    return this._param4
  }
  set reqParam3(value: number) {
    this._param4 = value
  }

  /**
   * The use of this parameter (if any), must be defined in the requested message. By default assumed not
   * used (0).
   */
  get reqParam4() {
    return this._param5
  }
  set reqParam4(value: number) {
    this._param5 = value
  }

  /**
   * The use of this parameter (if any), must be defined in the requested message. By default assumed not
   * used (0).
   */
  get reqParam5() {
    return this._param6
  }
  set reqParam5(value: number) {
    this._param6 = value
  }

  /**
   * Target address for requested message (if message has target address fields). 0: Flight-stack
   * default, 1: address of requestor, 2: broadcast.
   *
   * @min: 0
   * @max: 2
   * @increment: 1
   */
  get responseTarget() {
    return this._param7
  }
  set responseTarget(value: number) {
    this._param7 = value
  }
}

/**
 * Request MAVLink protocol version compatibility. All receivers should ACK the command and then emit
 * their capabilities in an PROTOCOL_VERSION message
 */
export class RequestProtocolVersionCommand extends CommandLong {
  constructor(targetSystem = 1, targetComponent = 1) {
    super()
    this.command = MavCmd.REQUEST_PROTOCOL_VERSION as number
    this.targetSystem = targetSystem
    this.targetComponent = targetComponent
  }

  /**
   * 1: Request supported protocol versions by all nodes on the network
   *
   * @min: 0
   * @max: 1
   * @increment: 1
   */
  get protocol() {
    return this._param1
  }
  set protocol(value: number) {
    this._param1 = value
  }
}

/**
 * Request autopilot capabilities. The receiver should ACK the command and then emit its capabilities
 * in an AUTOPILOT_VERSION message
 */
export class RequestAutopilotCapabilitiesCommand extends CommandLong {
  constructor(targetSystem = 1, targetComponent = 1) {
    super()
    this.command = MavCmd.REQUEST_AUTOPILOT_CAPABILITIES as number
    this.targetSystem = targetSystem
    this.targetComponent = targetComponent
  }

  /**
   * 1: Request autopilot version
   *
   * @min: 0
   * @max: 1
   * @increment: 1
   */
  get version() {
    return this._param1
  }
  set version(value: number) {
    this._param1 = value
  }
}

/**
 * Request camera information (CAMERA_INFORMATION).
 */
export class RequestCameraInformationCommand extends CommandLong {
  constructor(targetSystem = 1, targetComponent = 1) {
    super()
    this.command = MavCmd.REQUEST_CAMERA_INFORMATION as number
    this.targetSystem = targetSystem
    this.targetComponent = targetComponent
  }

  /**
   * 0: No action 1: Request camera capabilities
   *
   * @min: 0
   * @max: 1
   * @increment: 1
   */
  get capabilities() {
    return this._param1
  }
  set capabilities(value: number) {
    this._param1 = value
  }
}

/**
 * Request camera settings (CAMERA_SETTINGS).
 */
export class RequestCameraSettingsCommand extends CommandLong {
  constructor(targetSystem = 1, targetComponent = 1) {
    super()
    this.command = MavCmd.REQUEST_CAMERA_SETTINGS as number
    this.targetSystem = targetSystem
    this.targetComponent = targetComponent
  }

  /**
   * 0: No Action 1: Request camera settings
   *
   * @min: 0
   * @max: 1
   * @increment: 1
   */
  get settings() {
    return this._param1
  }
  set settings(value: number) {
    this._param1 = value
  }
}

/**
 * Request storage information (STORAGE_INFORMATION). Use the command's target_component to target a
 * specific component's storage.
 */
export class RequestStorageInformationCommand extends CommandLong {
  constructor(targetSystem = 1, targetComponent = 1) {
    super()
    this.command = MavCmd.REQUEST_STORAGE_INFORMATION as number
    this.targetSystem = targetSystem
    this.targetComponent = targetComponent
  }

  /**
   * Storage ID (0 for all, 1 for first, 2 for second, etc.)
   *
   * @min: 0
   * @increment: 1
   */
  get storageId() {
    return this._param1
  }
  set storageId(value: number) {
    this._param1 = value
  }

  /**
   * 0: No Action 1: Request storage information
   *
   * @min: 0
   * @max: 1
   * @increment: 1
   */
  get information() {
    return this._param2
  }
  set information(value: number) {
    this._param2 = value
  }
}

/**
 * Format a storage medium. Once format is complete, a STORAGE_INFORMATION message is sent. Use the
 * command's target_component to target a specific component's storage.
 */
export class StorageFormatCommand extends CommandLong {
  constructor(targetSystem = 1, targetComponent = 1) {
    super()
    this.command = MavCmd.STORAGE_FORMAT as number
    this.targetSystem = targetSystem
    this.targetComponent = targetComponent
  }

  /**
   * Storage ID (1 for first, 2 for second, etc.)
   *
   * @min: 0
   * @increment: 1
   */
  get storageId() {
    return this._param1
  }
  set storageId(value: number) {
    this._param1 = value
  }

  /**
   * Format storage (and reset image log). 0: No action 1: Format storage
   *
   * @min: 0
   * @max: 1
   * @increment: 1
   */
  get format() {
    return this._param2
  }
  set format(value: number) {
    this._param2 = value
  }

  /**
   * Reset Image Log (without formatting storage medium). This will reset
   * CAMERA_CAPTURE_STATUS.image_count and CAMERA_IMAGE_CAPTURED.image_index. 0: No action 1: Reset Image
   * Log
   *
   * @min: 0
   * @max: 1
   * @increment: 1
   */
  get resetImageLog() {
    return this._param3
  }
  set resetImageLog(value: number) {
    this._param3 = value
  }
}

/**
 * Request camera capture status (CAMERA_CAPTURE_STATUS)
 */
export class RequestCameraCaptureStatusCommand extends CommandLong {
  constructor(targetSystem = 1, targetComponent = 1) {
    super()
    this.command = MavCmd.REQUEST_CAMERA_CAPTURE_STATUS as number
    this.targetSystem = targetSystem
    this.targetComponent = targetComponent
  }

  /**
   * 0: No Action 1: Request camera capture status
   *
   * @min: 0
   * @max: 1
   * @increment: 1
   */
  get captureStatus() {
    return this._param1
  }
  set captureStatus(value: number) {
    this._param1 = value
  }
}

/**
 * Request flight information (FLIGHT_INFORMATION)
 */
export class RequestFlightInformationCommand extends CommandLong {
  constructor(targetSystem = 1, targetComponent = 1) {
    super()
    this.command = MavCmd.REQUEST_FLIGHT_INFORMATION as number
    this.targetSystem = targetSystem
    this.targetComponent = targetComponent
  }

  /**
   * 1: Request flight information
   *
   * @min: 0
   * @max: 1
   * @increment: 1
   */
  get flightInformation() {
    return this._param1
  }
  set flightInformation(value: number) {
    this._param1 = value
  }
}

/**
 * Reset all camera settings to Factory Default
 */
export class ResetCameraSettingsCommand extends CommandLong {
  constructor(targetSystem = 1, targetComponent = 1) {
    super()
    this.command = MavCmd.RESET_CAMERA_SETTINGS as number
    this.targetSystem = targetSystem
    this.targetComponent = targetComponent
  }

  /**
   * 0: No Action 1: Reset all settings
   *
   * @min: 0
   * @max: 1
   * @increment: 1
   */
  get reset() {
    return this._param1
  }
  set reset(value: number) {
    this._param1 = value
  }
}

/**
 * Set camera running mode. Use NaN for reserved values. GCS will send a
 * MAV_CMD_REQUEST_VIDEO_STREAM_STATUS command after a mode change if the camera supports video
 * streaming.
 */
export class SetCameraModeCommand extends CommandLong {
  constructor(targetSystem = 1, targetComponent = 1) {
    super()
    this.command = MavCmd.SET_CAMERA_MODE as number
    this.targetSystem = targetSystem
    this.targetComponent = targetComponent
  }

  /**
   * Camera mode
   */
  get cameraMode() {
    return this._param2
  }
  set cameraMode(value: number) {
    this._param2 = value
  }
}

/**
 * Set camera zoom. Camera must respond with a CAMERA_SETTINGS message (on success).
 */
export class SetCameraZoomCommand extends CommandLong {
  constructor(targetSystem = 1, targetComponent = 1) {
    super()
    this.command = MavCmd.SET_CAMERA_ZOOM as number
    this.targetSystem = targetSystem
    this.targetComponent = targetComponent
  }

  /**
   * Zoom type
   */
  get zoomType() {
    return this._param1
  }
  set zoomType(value: number) {
    this._param1 = value
  }

  /**
   * Zoom value. The range of valid values depend on the zoom type.
   */
  get zoomValue() {
    return this._param2
  }
  set zoomValue(value: number) {
    this._param2 = value
  }
}

/**
 * Set camera focus. Camera must respond with a CAMERA_SETTINGS message (on success).
 */
export class SetCameraFocusCommand extends CommandLong {
  constructor(targetSystem = 1, targetComponent = 1) {
    super()
    this.command = MavCmd.SET_CAMERA_FOCUS as number
    this.targetSystem = targetSystem
    this.targetComponent = targetComponent
  }

  /**
   * Focus type
   */
  get focusType() {
    return this._param1
  }
  set focusType(value: number) {
    this._param1 = value
  }

  /**
   * Focus value
   */
  get focusValue() {
    return this._param2
  }
  set focusValue(value: number) {
    this._param2 = value
  }
}

/**
 * Set that a particular storage is the preferred location for saving photos, videos, and/or other
 * media (e.g. to set that an SD card is used for storing videos).
 There can only be one preferred
 * save location for each particular media type: setting a media usage flag will clear/reset that same
 * flag if set on any other storage.
 If no flag is set the system should use its default storage.
 A
 * target system can choose to always use default storage, in which case it should ACK the command with
 * MAV_RESULT_UNSUPPORTED.
 A target system can choose to not allow a particular storage to be set as
 * preferred storage, in which case it should ACK the command with MAV_RESULT_DENIED.
 */
export class SetStorageUsageCommand extends CommandLong {
  constructor(targetSystem = 1, targetComponent = 1) {
    super()
    this.command = MavCmd.SET_STORAGE_USAGE as number
    this.targetSystem = targetSystem
    this.targetComponent = targetComponent
  }

  /**
   * Storage ID (1 for first, 2 for second, etc.)
   *
   * @min: 0
   * @increment: 1
   */
  get storageId() {
    return this._param1
  }
  set storageId(value: number) {
    this._param1 = value
  }

  /**
   * Usage flags
   */
  get usage() {
    return this._param2
  }
  set usage(value: number) {
    this._param2 = value
  }
}

/**
 * Tagged jump target. Can be jumped to with MAV_CMD_DO_JUMP_TAG.
 */
export class JumpTagCommand extends CommandLong {
  constructor(targetSystem = 1, targetComponent = 1) {
    super()
    this.command = MavCmd.JUMP_TAG as number
    this.targetSystem = targetSystem
    this.targetComponent = targetComponent
  }

  /**
   * Tag.
   *
   * @min: 0
   * @increment: 1
   */
  get tag() {
    return this._param1
  }
  set tag(value: number) {
    this._param1 = value
  }
}

/**
 * Jump to the matching tag in the mission list. Repeat this action for the specified number of times.
 * A mission should contain a single matching tag for each jump. If this is not the case then a jump to
 * a missing tag should complete the mission, and a jump where there are multiple matching tags should
 * always select the one with the lowest mission sequence number.
 */
export class DoJumpTagCommand extends CommandLong {
  constructor(targetSystem = 1, targetComponent = 1) {
    super()
    this.command = MavCmd.DO_JUMP_TAG as number
    this.targetSystem = targetSystem
    this.targetComponent = targetComponent
  }

  /**
   * Target tag to jump to.
   *
   * @min: 0
   * @increment: 1
   */
  get tag() {
    return this._param1
  }
  set tag(value: number) {
    this._param1 = value
  }

  /**
   * Repeat count.
   *
   * @min: 0
   * @increment: 1
   */
  get repeat() {
    return this._param2
  }
  set repeat(value: number) {
    this._param2 = value
  }
}

/**
 * High level setpoint to be sent to a gimbal manager to set a gimbal attitude. It is possible to set
 * combinations of the values below. E.g. an angle as well as a desired angular rate can be used to get
 * to this angle at a certain angular rate, or an angular rate only will result in continuous turning.
 * NaN is to be used to signal unset. Note: a gimbal is never to react to this command but only the
 * gimbal manager.
 */
export class DoGimbalManagerPitchyawCommand extends CommandLong {
  constructor(targetSystem = 1, targetComponent = 1) {
    super()
    this.command = MavCmd.DO_GIMBAL_MANAGER_PITCHYAW as number
    this.targetSystem = targetSystem
    this.targetComponent = targetComponent
  }

  /**
   * Pitch angle (positive to pitch up, relative to vehicle for FOLLOW mode, relative to world horizon
   * for LOCK mode).
   *
   * @units deg
   * @min: -180
   * @max: 180
   */
  get pitchAngle() {
    return this._param1
  }
  set pitchAngle(value: number) {
    this._param1 = value
  }

  /**
   * Yaw angle (positive to yaw to the right, relative to vehicle for FOLLOW mode, absolute to North for
   * LOCK mode).
   *
   * @units deg
   * @min: -180
   * @max: 180
   */
  get yawAngle() {
    return this._param2
  }
  set yawAngle(value: number) {
    this._param2 = value
  }

  /**
   * Pitch rate (positive to pitch up).
   *
   * @units deg/s
   */
  get pitchRate() {
    return this._param3
  }
  set pitchRate(value: number) {
    this._param3 = value
  }

  /**
   * Yaw rate (positive to yaw to the right).
   *
   * @units deg/s
   */
  get yawRate() {
    return this._param4
  }
  set yawRate(value: number) {
    this._param4 = value
  }

  /**
   * Gimbal manager flags to use.
   */
  get gimbalManagerFlags() {
    return this._param5
  }
  set gimbalManagerFlags(value: number) {
    this._param5 = value
  }

  /**
   * Component ID of gimbal device to address (or 1-6 for non-MAVLink gimbal), 0 for all gimbal device
   * components. Send command multiple times for more than one gimbal (but not all gimbals).
   */
  get gimbalDeviceId() {
    return this._param7
  }
  set gimbalDeviceId(value: number) {
    this._param7 = value
  }
}

/**
 * Gimbal configuration to set which sysid/compid is in primary and secondary control.
 */
export class DoGimbalManagerConfigureCommand extends CommandLong {
  constructor(targetSystem = 1, targetComponent = 1) {
    super()
    this.command = MavCmd.DO_GIMBAL_MANAGER_CONFIGURE as number
    this.targetSystem = targetSystem
    this.targetComponent = targetComponent
  }

  /**
   * Sysid for primary control (0: no one in control, -1: leave unchanged, -2: set itself in control (for
   * missions where the own sysid is still unknown), -3: remove control if currently in control).
   */
  get sysidPrimaryControl() {
    return this._param1
  }
  set sysidPrimaryControl(value: number) {
    this._param1 = value
  }

  /**
   * Compid for primary control (0: no one in control, -1: leave unchanged, -2: set itself in control
   * (for missions where the own sysid is still unknown), -3: remove control if currently in control).
   */
  get compidPrimaryControl() {
    return this._param2
  }
  set compidPrimaryControl(value: number) {
    this._param2 = value
  }

  /**
   * Sysid for secondary control (0: no one in control, -1: leave unchanged, -2: set itself in control
   * (for missions where the own sysid is still unknown), -3: remove control if currently in control).
   */
  get sysidSecondaryControl() {
    return this._param3
  }
  set sysidSecondaryControl(value: number) {
    this._param3 = value
  }

  /**
   * Compid for secondary control (0: no one in control, -1: leave unchanged, -2: set itself in control
   * (for missions where the own sysid is still unknown), -3: remove control if currently in control).
   */
  get compidSecondaryControl() {
    return this._param4
  }
  set compidSecondaryControl(value: number) {
    this._param4 = value
  }

  /**
   * Component ID of gimbal device to address (or 1-6 for non-MAVLink gimbal), 0 for all gimbal device
   * components. Send command multiple times for more than one gimbal (but not all gimbals).
   */
  get gimbalDeviceId() {
    return this._param7
  }
  set gimbalDeviceId(value: number) {
    this._param7 = value
  }
}

/**
 * Gimbal configuration to set which sysid/compid is in primary and secondary control.
 */
export class DoGimbalManagerLaserCommand extends CommandLong {
  constructor(targetSystem = 1, targetComponent = 1) {
    super()
    this.command = MavCmd.DO_GIMBAL_MANAGER_LASER as number
    this.targetSystem = targetSystem
    this.targetComponent = targetComponent
  }

  /**
   * Sysid for primary control (0: no one in control, -1: leave unchanged, -2: set itself in control (for
   * missions where the own sysid is still unknown), -3: remove control if currently in control).
   */
  get sysidPrimaryControl() {
    return this._param1
  }
  set sysidPrimaryControl(value: number) {
    this._param1 = value
  }

  /**
   * Compid for primary control (0: no one in control, -1: leave unchanged, -2: set itself in control
   * (for missions where the own sysid is still unknown), -3: remove control if currently in control).
   */
  get compidPrimaryControl() {
    return this._param2
  }
  set compidPrimaryControl(value: number) {
    this._param2 = value
  }

  /**
   * Sysid for secondary control (0: no one in control, -1: leave unchanged, -2: set itself in control
   * (for missions where the own sysid is still unknown), -3: remove control if currently in control).
   */
  get sysidSecondaryControl() {
    return this._param3
  }
  set sysidSecondaryControl(value: number) {
    this._param3 = value
  }

  /**
   * Compid for secondary control (0: no one in control, -1: leave unchanged, -2: set itself in control
   * (for missions where the own sysid is still unknown), -3: remove control if currently in control).
   */
  get compidSecondaryControl() {
    return this._param4
  }
  set compidSecondaryControl(value: number) {
    this._param4 = value
  }

  /**
   * Component ID of gimbal device to address (or 1-6 for non-MAVLink gimbal), 0 for all gimbal device
   * components. Send command multiple times for more than one gimbal (but not all gimbals).
   */
  get gimbalDeviceId() {
    return this._param7
  }
  set gimbalDeviceId(value: number) {
    this._param7 = value
  }
}

/**
 * Gimbal configuration to set which sysid/compid is in primary and secondary control.
 */
export class DoGimbalManagerThermalCommand extends CommandLong {
  constructor(targetSystem = 1, targetComponent = 1) {
    super()
    this.command = MavCmd.DO_GIMBAL_MANAGER_THERMAL as number
    this.targetSystem = targetSystem
    this.targetComponent = targetComponent
  }

  /**
   * Sysid for primary control (0: no one in control, -1: leave unchanged, -2: set itself in control (for
   * missions where the own sysid is still unknown), -3: remove control if currently in control).
   */
  get sysidPrimaryControl() {
    return this._param1
  }
  set sysidPrimaryControl(value: number) {
    this._param1 = value
  }

  /**
   * Compid for primary control (0: no one in control, -1: leave unchanged, -2: set itself in control
   * (for missions where the own sysid is still unknown), -3: remove control if currently in control).
   */
  get compidPrimaryControl() {
    return this._param2
  }
  set compidPrimaryControl(value: number) {
    this._param2 = value
  }

  /**
   * Sysid for secondary control (0: no one in control, -1: leave unchanged, -2: set itself in control
   * (for missions where the own sysid is still unknown), -3: remove control if currently in control).
   */
  get sysidSecondaryControl() {
    return this._param3
  }
  set sysidSecondaryControl(value: number) {
    this._param3 = value
  }

  /**
   * Compid for secondary control (0: no one in control, -1: leave unchanged, -2: set itself in control
   * (for missions where the own sysid is still unknown), -3: remove control if currently in control).
   */
  get compidSecondaryControl() {
    return this._param4
  }
  set compidSecondaryControl(value: number) {
    this._param4 = value
  }

  /**
   * Component ID of gimbal device to address (or 1-6 for non-MAVLink gimbal), 0 for all gimbal device
   * components. Send command multiple times for more than one gimbal (but not all gimbals).
   */
  get gimbalDeviceId() {
    return this._param7
  }
  set gimbalDeviceId(value: number) {
    this._param7 = value
  }
}

/**
 * Gimbal configuration to set which sysid/compid is in primary and secondary control.
 */
export class DoGimbalManagerCenterCommand extends CommandLong {
  constructor(targetSystem = 1, targetComponent = 1) {
    super()
    this.command = MavCmd.DO_GIMBAL_MANAGER_CENTER as number
    this.targetSystem = targetSystem
    this.targetComponent = targetComponent
  }

  /**
   * Sysid for primary control (0: no one in control, -1: leave unchanged, -2: set itself in control (for
   * missions where the own sysid is still unknown), -3: remove control if currently in control).
   */
  get sysidPrimaryControl() {
    return this._param1
  }
  set sysidPrimaryControl(value: number) {
    this._param1 = value
  }

  /**
   * Compid for primary control (0: no one in control, -1: leave unchanged, -2: set itself in control
   * (for missions where the own sysid is still unknown), -3: remove control if currently in control).
   */
  get compidPrimaryControl() {
    return this._param2
  }
  set compidPrimaryControl(value: number) {
    this._param2 = value
  }

  /**
   * Sysid for secondary control (0: no one in control, -1: leave unchanged, -2: set itself in control
   * (for missions where the own sysid is still unknown), -3: remove control if currently in control).
   */
  get sysidSecondaryControl() {
    return this._param3
  }
  set sysidSecondaryControl(value: number) {
    this._param3 = value
  }

  /**
   * Compid for secondary control (0: no one in control, -1: leave unchanged, -2: set itself in control
   * (for missions where the own sysid is still unknown), -3: remove control if currently in control).
   */
  get compidSecondaryControl() {
    return this._param4
  }
  set compidSecondaryControl(value: number) {
    this._param4 = value
  }

  /**
   * Component ID of gimbal device to address (or 1-6 for non-MAVLink gimbal), 0 for all gimbal device
   * components. Send command multiple times for more than one gimbal (but not all gimbals).
   */
  get gimbalDeviceId() {
    return this._param7
  }
  set gimbalDeviceId(value: number) {
    this._param7 = value
  }
}

/**
 * Gimbal configuration to set which sysid/compid is in primary and secondary control.
 */
export class DoGimbalManagerUpCommand extends CommandLong {
  constructor(targetSystem = 1, targetComponent = 1) {
    super()
    this.command = MavCmd.DO_GIMBAL_MANAGER_UP as number
    this.targetSystem = targetSystem
    this.targetComponent = targetComponent
  }

  /**
   * Sysid for primary control (0: no one in control, -1: leave unchanged, -2: set itself in control (for
   * missions where the own sysid is still unknown), -3: remove control if currently in control).
   */
  get sysidPrimaryControl() {
    return this._param1
  }
  set sysidPrimaryControl(value: number) {
    this._param1 = value
  }

  /**
   * Compid for primary control (0: no one in control, -1: leave unchanged, -2: set itself in control
   * (for missions where the own sysid is still unknown), -3: remove control if currently in control).
   */
  get compidPrimaryControl() {
    return this._param2
  }
  set compidPrimaryControl(value: number) {
    this._param2 = value
  }

  /**
   * Sysid for secondary control (0: no one in control, -1: leave unchanged, -2: set itself in control
   * (for missions where the own sysid is still unknown), -3: remove control if currently in control).
   */
  get sysidSecondaryControl() {
    return this._param3
  }
  set sysidSecondaryControl(value: number) {
    this._param3 = value
  }

  /**
   * Compid for secondary control (0: no one in control, -1: leave unchanged, -2: set itself in control
   * (for missions where the own sysid is still unknown), -3: remove control if currently in control).
   */
  get compidSecondaryControl() {
    return this._param4
  }
  set compidSecondaryControl(value: number) {
    this._param4 = value
  }

  /**
   * Component ID of gimbal device to address (or 1-6 for non-MAVLink gimbal), 0 for all gimbal device
   * components. Send command multiple times for more than one gimbal (but not all gimbals).
   */
  get gimbalDeviceId() {
    return this._param7
  }
  set gimbalDeviceId(value: number) {
    this._param7 = value
  }
}

/**
 * Gimbal configuration to set which sysid/compid is in primary and secondary control.
 */
export class DoGimbalManagerDownCommand extends CommandLong {
  constructor(targetSystem = 1, targetComponent = 1) {
    super()
    this.command = MavCmd.DO_GIMBAL_MANAGER_DOWN as number
    this.targetSystem = targetSystem
    this.targetComponent = targetComponent
  }

  /**
   * Sysid for primary control (0: no one in control, -1: leave unchanged, -2: set itself in control (for
   * missions where the own sysid is still unknown), -3: remove control if currently in control).
   */
  get sysidPrimaryControl() {
    return this._param1
  }
  set sysidPrimaryControl(value: number) {
    this._param1 = value
  }

  /**
   * Compid for primary control (0: no one in control, -1: leave unchanged, -2: set itself in control
   * (for missions where the own sysid is still unknown), -3: remove control if currently in control).
   */
  get compidPrimaryControl() {
    return this._param2
  }
  set compidPrimaryControl(value: number) {
    this._param2 = value
  }

  /**
   * Sysid for secondary control (0: no one in control, -1: leave unchanged, -2: set itself in control
   * (for missions where the own sysid is still unknown), -3: remove control if currently in control).
   */
  get sysidSecondaryControl() {
    return this._param3
  }
  set sysidSecondaryControl(value: number) {
    this._param3 = value
  }

  /**
   * Compid for secondary control (0: no one in control, -1: leave unchanged, -2: set itself in control
   * (for missions where the own sysid is still unknown), -3: remove control if currently in control).
   */
  get compidSecondaryControl() {
    return this._param4
  }
  set compidSecondaryControl(value: number) {
    this._param4 = value
  }

  /**
   * Component ID of gimbal device to address (or 1-6 for non-MAVLink gimbal), 0 for all gimbal device
   * components. Send command multiple times for more than one gimbal (but not all gimbals).
   */
  get gimbalDeviceId() {
    return this._param7
  }
  set gimbalDeviceId(value: number) {
    this._param7 = value
  }
}

/**
 * Gimbal configuration to set which sysid/compid is in primary and secondary control.
 */
export class DoGimbalStartTargetDetectionCommand extends CommandLong {
  constructor(targetSystem = 1, targetComponent = 1) {
    super()
    this.command = MavCmd.DO_GIMBAL_START_TARGET_DETECTION as number
    this.targetSystem = targetSystem
    this.targetComponent = targetComponent
  }

  /**
   * Sysid for primary control (0: no one in control, -1: leave unchanged, -2: set itself in control (for
   * missions where the own sysid is still unknown), -3: remove control if currently in control).
   */
  get sysidPrimaryControl() {
    return this._param1
  }
  set sysidPrimaryControl(value: number) {
    this._param1 = value
  }

  /**
   * Compid for primary control (0: no one in control, -1: leave unchanged, -2: set itself in control
   * (for missions where the own sysid is still unknown), -3: remove control if currently in control).
   */
  get compidPrimaryControl() {
    return this._param2
  }
  set compidPrimaryControl(value: number) {
    this._param2 = value
  }

  /**
   * Sysid for secondary control (0: no one in control, -1: leave unchanged, -2: set itself in control
   * (for missions where the own sysid is still unknown), -3: remove control if currently in control).
   */
  get sysidSecondaryControl() {
    return this._param3
  }
  set sysidSecondaryControl(value: number) {
    this._param3 = value
  }

  /**
   * Compid for secondary control (0: no one in control, -1: leave unchanged, -2: set itself in control
   * (for missions where the own sysid is still unknown), -3: remove control if currently in control).
   */
  get compidSecondaryControl() {
    return this._param4
  }
  set compidSecondaryControl(value: number) {
    this._param4 = value
  }

  /**
   * Component ID of gimbal device to address (or 1-6 for non-MAVLink gimbal), 0 for all gimbal device
   * components. Send command multiple times for more than one gimbal (but not all gimbals).
   */
  get gimbalDeviceId() {
    return this._param7
  }
  set gimbalDeviceId(value: number) {
    this._param7 = value
  }
}

/**
 * Gimbal configuration to set which sysid/compid is in primary and secondary control.
 */
export class DoGimbalStopTargetDetectionCommand extends CommandLong {
  constructor(targetSystem = 1, targetComponent = 1) {
    super()
    this.command = MavCmd.DO_GIMBAL_STOP_TARGET_DETECTION as number
    this.targetSystem = targetSystem
    this.targetComponent = targetComponent
  }

  /**
   * Sysid for primary control (0: no one in control, -1: leave unchanged, -2: set itself in control (for
   * missions where the own sysid is still unknown), -3: remove control if currently in control).
   */
  get sysidPrimaryControl() {
    return this._param1
  }
  set sysidPrimaryControl(value: number) {
    this._param1 = value
  }

  /**
   * Compid for primary control (0: no one in control, -1: leave unchanged, -2: set itself in control
   * (for missions where the own sysid is still unknown), -3: remove control if currently in control).
   */
  get compidPrimaryControl() {
    return this._param2
  }
  set compidPrimaryControl(value: number) {
    this._param2 = value
  }

  /**
   * Sysid for secondary control (0: no one in control, -1: leave unchanged, -2: set itself in control
   * (for missions where the own sysid is still unknown), -3: remove control if currently in control).
   */
  get sysidSecondaryControl() {
    return this._param3
  }
  set sysidSecondaryControl(value: number) {
    this._param3 = value
  }

  /**
   * Compid for secondary control (0: no one in control, -1: leave unchanged, -2: set itself in control
   * (for missions where the own sysid is still unknown), -3: remove control if currently in control).
   */
  get compidSecondaryControl() {
    return this._param4
  }
  set compidSecondaryControl(value: number) {
    this._param4 = value
  }

  /**
   * Component ID of gimbal device to address (or 1-6 for non-MAVLink gimbal), 0 for all gimbal device
   * components. Send command multiple times for more than one gimbal (but not all gimbals).
   */
  get gimbalDeviceId() {
    return this._param7
  }
  set gimbalDeviceId(value: number) {
    this._param7 = value
  }
}

/**
 * Gimbal configuration to set which sysid/compid is in primary and secondary control.
 */
export class DoGimbalEnableOsdCommand extends CommandLong {
  constructor(targetSystem = 1, targetComponent = 1) {
    super()
    this.command = MavCmd.DO_GIMBAL_ENABLE_OSD as number
    this.targetSystem = targetSystem
    this.targetComponent = targetComponent
  }

  /**
   * Sysid for primary control (0: no one in control, -1: leave unchanged, -2: set itself in control (for
   * missions where the own sysid is still unknown), -3: remove control if currently in control).
   */
  get sysidPrimaryControl() {
    return this._param1
  }
  set sysidPrimaryControl(value: number) {
    this._param1 = value
  }

  /**
   * Compid for primary control (0: no one in control, -1: leave unchanged, -2: set itself in control
   * (for missions where the own sysid is still unknown), -3: remove control if currently in control).
   */
  get compidPrimaryControl() {
    return this._param2
  }
  set compidPrimaryControl(value: number) {
    this._param2 = value
  }

  /**
   * Sysid for secondary control (0: no one in control, -1: leave unchanged, -2: set itself in control
   * (for missions where the own sysid is still unknown), -3: remove control if currently in control).
   */
  get sysidSecondaryControl() {
    return this._param3
  }
  set sysidSecondaryControl(value: number) {
    this._param3 = value
  }

  /**
   * Compid for secondary control (0: no one in control, -1: leave unchanged, -2: set itself in control
   * (for missions where the own sysid is still unknown), -3: remove control if currently in control).
   */
  get compidSecondaryControl() {
    return this._param4
  }
  set compidSecondaryControl(value: number) {
    this._param4 = value
  }

  /**
   * Component ID of gimbal device to address (or 1-6 for non-MAVLink gimbal), 0 for all gimbal device
   * components. Send command multiple times for more than one gimbal (but not all gimbals).
   */
  get gimbalDeviceId() {
    return this._param7
  }
  set gimbalDeviceId(value: number) {
    this._param7 = value
  }
}

/**
 * Gimbal configuration to set which sysid/compid is in primary and secondary control.
 */
export class DoGimbalDisableOsdCommand extends CommandLong {
  constructor(targetSystem = 1, targetComponent = 1) {
    super()
    this.command = MavCmd.DO_GIMBAL_DISABLE_OSD as number
    this.targetSystem = targetSystem
    this.targetComponent = targetComponent
  }

  /**
   * Sysid for primary control (0: no one in control, -1: leave unchanged, -2: set itself in control (for
   * missions where the own sysid is still unknown), -3: remove control if currently in control).
   */
  get sysidPrimaryControl() {
    return this._param1
  }
  set sysidPrimaryControl(value: number) {
    this._param1 = value
  }

  /**
   * Compid for primary control (0: no one in control, -1: leave unchanged, -2: set itself in control
   * (for missions where the own sysid is still unknown), -3: remove control if currently in control).
   */
  get compidPrimaryControl() {
    return this._param2
  }
  set compidPrimaryControl(value: number) {
    this._param2 = value
  }

  /**
   * Sysid for secondary control (0: no one in control, -1: leave unchanged, -2: set itself in control
   * (for missions where the own sysid is still unknown), -3: remove control if currently in control).
   */
  get sysidSecondaryControl() {
    return this._param3
  }
  set sysidSecondaryControl(value: number) {
    this._param3 = value
  }

  /**
   * Compid for secondary control (0: no one in control, -1: leave unchanged, -2: set itself in control
   * (for missions where the own sysid is still unknown), -3: remove control if currently in control).
   */
  get compidSecondaryControl() {
    return this._param4
  }
  set compidSecondaryControl(value: number) {
    this._param4 = value
  }

  /**
   * Component ID of gimbal device to address (or 1-6 for non-MAVLink gimbal), 0 for all gimbal device
   * components. Send command multiple times for more than one gimbal (but not all gimbals).
   */
  get gimbalDeviceId() {
    return this._param7
  }
  set gimbalDeviceId(value: number) {
    this._param7 = value
  }
}

/**
 * Start image capture sequence. Sends CAMERA_IMAGE_CAPTURED after each capture. Use NaN for reserved
 * values.
 */
export class ImageStartCaptureCommand extends CommandLong {
  constructor(targetSystem = 1, targetComponent = 1) {
    super()
    this.command = MavCmd.IMAGE_START_CAPTURE as number
    this.targetSystem = targetSystem
    this.targetComponent = targetComponent
  }

  /**
   * Desired elapsed time between two consecutive pictures (in seconds). Minimum values depend on
   * hardware (typically greater than 2 seconds).
   *
   * @units s
   * @min: 0
   */
  get interval() {
    return this._param2
  }
  set interval(value: number) {
    this._param2 = value
  }

  /**
   * Total number of images to capture. 0 to capture forever/until MAV_CMD_IMAGE_STOP_CAPTURE.
   *
   * @min: 0
   * @increment: 1
   */
  get totalImages() {
    return this._param3
  }
  set totalImages(value: number) {
    this._param3 = value
  }

  /**
   * Capture sequence number starting from 1. This is only valid for single-capture (param3 == 1),
   * otherwise set to 0. Increment the capture ID for each capture command to prevent double captures
   * when a command is re-transmitted.
   *
   * @min: 1
   * @increment: 1
   */
  get sequenceNumber() {
    return this._param4
  }
  set sequenceNumber(value: number) {
    this._param4 = value
  }
}

/**
 * Stop image capture sequence Use NaN for reserved values.
 */
export class ImageStopCaptureCommand extends CommandLong {
  constructor(targetSystem = 1, targetComponent = 1) {
    super()
    this.command = MavCmd.IMAGE_STOP_CAPTURE as number
    this.targetSystem = targetSystem
    this.targetComponent = targetComponent
  }
}

/**
 * Re-request a CAMERA_IMAGE_CAPTURED message.
 */
export class RequestCameraImageCaptureCommand extends CommandLong {
  constructor(targetSystem = 1, targetComponent = 1) {
    super()
    this.command = MavCmd.REQUEST_CAMERA_IMAGE_CAPTURE as number
    this.targetSystem = targetSystem
    this.targetComponent = targetComponent
  }

  /**
   * Sequence number for missing CAMERA_IMAGE_CAPTURED message
   *
   * @min: 0
   * @increment: 1
   */
  get number() {
    return this._param1
  }
  set number(value: number) {
    this._param1 = value
  }
}

/**
 * Enable or disable on-board camera triggering system.
 */
export class DoTriggerControlCommand extends CommandLong {
  constructor(targetSystem = 1, targetComponent = 1) {
    super()
    this.command = MavCmd.DO_TRIGGER_CONTROL as number
    this.targetSystem = targetSystem
    this.targetComponent = targetComponent
  }

  /**
   * Trigger enable/disable (0 for disable, 1 for start), -1 to ignore
   *
   * @min: -1
   * @max: 1
   * @increment: 1
   */
  get enable() {
    return this._param1
  }
  set enable(value: number) {
    this._param1 = value
  }

  /**
   * 1 to reset the trigger sequence, -1 or 0 to ignore
   *
   * @min: -1
   * @max: 1
   * @increment: 1
   */
  get reset() {
    return this._param2
  }
  set reset(value: number) {
    this._param2 = value
  }

  /**
   * 1 to pause triggering, but without switching the camera off or retracting it. -1 to ignore
   *
   * @min: -1
   * @max: 1
   * @increment: 2
   */
  get pause() {
    return this._param3
  }
  set pause(value: number) {
    this._param3 = value
  }
}

/**
 * If the camera supports point visual tracking (CAMERA_CAP_FLAGS_HAS_TRACKING_POINT is set), this
 * command allows to initiate the tracking.
 */
export class CameraTrackPointCommand extends CommandLong {
  constructor(targetSystem = 1, targetComponent = 1) {
    super()
    this.command = MavCmd.CAMERA_TRACK_POINT as number
    this.targetSystem = targetSystem
    this.targetComponent = targetComponent
  }

  /**
   * Point to track x value (normalized 0..1, 0 is left, 1 is right).
   *
   * @min: 0
   * @max: 1
   */
  get pointX() {
    return this._param1
  }
  set pointX(value: number) {
    this._param1 = value
  }

  /**
   * Point to track y value (normalized 0..1, 0 is top, 1 is bottom).
   *
   * @min: 0
   * @max: 1
   */
  get pointY() {
    return this._param2
  }
  set pointY(value: number) {
    this._param2 = value
  }

  /**
   * Point radius (normalized 0..1, 0 is image left, 1 is image right).
   *
   * @min: 0
   * @max: 1
   */
  get radius() {
    return this._param3
  }
  set radius(value: number) {
    this._param3 = value
  }
}

/**
 * If the camera supports rectangle visual tracking (CAMERA_CAP_FLAGS_HAS_TRACKING_RECTANGLE is set),
 * this command allows to initiate the tracking.
 */
export class CameraTrackRectangleCommand extends CommandLong {
  constructor(targetSystem = 1, targetComponent = 1) {
    super()
    this.command = MavCmd.CAMERA_TRACK_RECTANGLE as number
    this.targetSystem = targetSystem
    this.targetComponent = targetComponent
  }

  /**
   * Top left corner of rectangle x value (normalized 0..1, 0 is left, 1 is right).
   *
   * @min: 0
   * @max: 1
   */
  get topLeftCornerX() {
    return this._param1
  }
  set topLeftCornerX(value: number) {
    this._param1 = value
  }

  /**
   * Top left corner of rectangle y value (normalized 0..1, 0 is top, 1 is bottom).
   *
   * @min: 0
   * @max: 1
   */
  get topLeftCornerY() {
    return this._param2
  }
  set topLeftCornerY(value: number) {
    this._param2 = value
  }

  /**
   * Bottom right corner of rectangle x value (normalized 0..1, 0 is left, 1 is right).
   *
   * @min: 0
   * @max: 1
   */
  get bottomRightCornerX() {
    return this._param3
  }
  set bottomRightCornerX(value: number) {
    this._param3 = value
  }

  /**
   * Bottom right corner of rectangle y value (normalized 0..1, 0 is top, 1 is bottom).
   *
   * @min: 0
   * @max: 1
   */
  get bottomRightCornerY() {
    return this._param4
  }
  set bottomRightCornerY(value: number) {
    this._param4 = value
  }
}

/**
 * Stops ongoing tracking.
 */
export class CameraStopTrackingCommand extends CommandLong {
  constructor(targetSystem = 1, targetComponent = 1) {
    super()
    this.command = MavCmd.CAMERA_STOP_TRACKING as number
    this.targetSystem = targetSystem
    this.targetComponent = targetComponent
  }
}

/**
 * Starts video capture (recording).
 */
export class VideoStartCaptureCommand extends CommandLong {
  constructor(targetSystem = 1, targetComponent = 1) {
    super()
    this.command = MavCmd.VIDEO_START_CAPTURE as number
    this.targetSystem = targetSystem
    this.targetComponent = targetComponent
  }

  /**
   * Video Stream ID (0 for all streams)
   *
   * @min: 0
   * @increment: 1
   */
  get streamId() {
    return this._param1
  }
  set streamId(value: number) {
    this._param1 = value
  }

  /**
   * Frequency CAMERA_CAPTURE_STATUS messages should be sent while recording (0 for no messages,
   * otherwise frequency)
   *
   * @units Hz
   * @min: 0
   */
  get statusFrequency() {
    return this._param2
  }
  set statusFrequency(value: number) {
    this._param2 = value
  }
}

/**
 * Stop the current video capture (recording).
 */
export class VideoStopCaptureCommand extends CommandLong {
  constructor(targetSystem = 1, targetComponent = 1) {
    super()
    this.command = MavCmd.VIDEO_STOP_CAPTURE as number
    this.targetSystem = targetSystem
    this.targetComponent = targetComponent
  }

  /**
   * Video Stream ID (0 for all streams)
   *
   * @min: 0
   * @increment: 1
   */
  get streamId() {
    return this._param1
  }
  set streamId(value: number) {
    this._param1 = value
  }
}

/**
 * Start video streaming
 */
export class VideoStartStreamingCommand extends CommandLong {
  constructor(targetSystem = 1, targetComponent = 1) {
    super()
    this.command = MavCmd.VIDEO_START_STREAMING as number
    this.targetSystem = targetSystem
    this.targetComponent = targetComponent
  }

  /**
   * Video Stream ID (0 for all streams, 1 for first, 2 for second, etc.)
   *
   * @min: 0
   * @increment: 1
   */
  get streamId() {
    return this._param1
  }
  set streamId(value: number) {
    this._param1 = value
  }
}

/**
 * Stop the given video stream
 */
export class VideoStopStreamingCommand extends CommandLong {
  constructor(targetSystem = 1, targetComponent = 1) {
    super()
    this.command = MavCmd.VIDEO_STOP_STREAMING as number
    this.targetSystem = targetSystem
    this.targetComponent = targetComponent
  }

  /**
   * Video Stream ID (0 for all streams, 1 for first, 2 for second, etc.)
   *
   * @min: 0
   * @increment: 1
   */
  get streamId() {
    return this._param1
  }
  set streamId(value: number) {
    this._param1 = value
  }
}

/**
 * Request video stream information (VIDEO_STREAM_INFORMATION)
 */
export class RequestVideoStreamInformationCommand extends CommandLong {
  constructor(targetSystem = 1, targetComponent = 1) {
    super()
    this.command = MavCmd.REQUEST_VIDEO_STREAM_INFORMATION as number
    this.targetSystem = targetSystem
    this.targetComponent = targetComponent
  }

  /**
   * Video Stream ID (0 for all streams, 1 for first, 2 for second, etc.)
   *
   * @min: 0
   * @increment: 1
   */
  get streamId() {
    return this._param1
  }
  set streamId(value: number) {
    this._param1 = value
  }
}

/**
 * Request video stream status (VIDEO_STREAM_STATUS)
 */
export class RequestVideoStreamStatusCommand extends CommandLong {
  constructor(targetSystem = 1, targetComponent = 1) {
    super()
    this.command = MavCmd.REQUEST_VIDEO_STREAM_STATUS as number
    this.targetSystem = targetSystem
    this.targetComponent = targetComponent
  }

  /**
   * Video Stream ID (0 for all streams, 1 for first, 2 for second, etc.)
   *
   * @min: 0
   * @increment: 1
   */
  get streamId() {
    return this._param1
  }
  set streamId(value: number) {
    this._param1 = value
  }
}

/**
 * Request to start streaming logging data over MAVLink (see also LOGGING_DATA message)
 */
export class LoggingStartCommand extends CommandLong {
  constructor(targetSystem = 1, targetComponent = 1) {
    super()
    this.command = MavCmd.LOGGING_START as number
    this.targetSystem = targetSystem
    this.targetComponent = targetComponent
  }

  /**
   * Format: 0: ULog
   *
   * @min: 0
   * @increment: 1
   */
  get format() {
    return this._param1
  }
  set format(value: number) {
    this._param1 = value
  }
}

/**
 * Request to stop streaming log data over MAVLink
 */
export class LoggingStopCommand extends CommandLong {
  constructor(targetSystem = 1, targetComponent = 1) {
    super()
    this.command = MavCmd.LOGGING_STOP as number
    this.targetSystem = targetSystem
    this.targetComponent = targetComponent
  }
}

/**
 */
export class AirframeConfigurationCommand extends CommandLong {
  constructor(targetSystem = 1, targetComponent = 1) {
    super()
    this.command = MavCmd.AIRFRAME_CONFIGURATION as number
    this.targetSystem = targetSystem
    this.targetComponent = targetComponent
  }

  /**
   * Landing gear ID (default: 0, -1 for all)
   *
   * @min: -1
   * @increment: 1
   */
  get landingGearId() {
    return this._param1
  }
  set landingGearId(value: number) {
    this._param1 = value
  }

  /**
   * Landing gear position (Down: 0, Up: 1, NaN for no change)
   */
  get landingGearPosition() {
    return this._param2
  }
  set landingGearPosition(value: number) {
    this._param2 = value
  }
}

/**
 * Request to start/stop transmitting over the high latency telemetry
 */
export class ControlHighLatencyCommand extends CommandLong {
  constructor(targetSystem = 1, targetComponent = 1) {
    super()
    this.command = MavCmd.CONTROL_HIGH_LATENCY as number
    this.targetSystem = targetSystem
    this.targetComponent = targetComponent
  }

  /**
   * Control transmission over high latency telemetry (0: stop, 1: start)
   *
   * @min: 0
   * @max: 1
   * @increment: 1
   */
  get enable() {
    return this._param1
  }
  set enable(value: number) {
    this._param1 = value
  }
}

/**
 * Create a panorama at the current position
 */
export class PanoramaCreateCommand extends CommandLong {
  constructor(targetSystem = 1, targetComponent = 1) {
    super()
    this.command = MavCmd.PANORAMA_CREATE as number
    this.targetSystem = targetSystem
    this.targetComponent = targetComponent
  }

  /**
   * Viewing angle horizontal of the panorama (+- 0.5 the total angle)
   *
   * @units deg
   */
  get horizontalAngle() {
    return this._param1
  }
  set horizontalAngle(value: number) {
    this._param1 = value
  }

  /**
   * Viewing angle vertical of panorama.
   *
   * @units deg
   */
  get verticalAngle() {
    return this._param2
  }
  set verticalAngle(value: number) {
    this._param2 = value
  }

  /**
   * Speed of the horizontal rotation.
   *
   * @units deg/s
   */
  get horizontalSpeed() {
    return this._param3
  }
  set horizontalSpeed(value: number) {
    this._param3 = value
  }

  /**
   * Speed of the vertical rotation.
   *
   * @units deg/s
   */
  get verticalSpeed() {
    return this._param4
  }
  set verticalSpeed(value: number) {
    this._param4 = value
  }
}

/**
 * Request VTOL transition
 */
export class DoVtolTransitionCommand extends CommandLong {
  constructor(targetSystem = 1, targetComponent = 1) {
    super()
    this.command = MavCmd.DO_VTOL_TRANSITION as number
    this.targetSystem = targetSystem
    this.targetComponent = targetComponent
  }

  /**
   * The target VTOL state. For normal transitions, only MAV_VTOL_STATE_MC and MAV_VTOL_STATE_FW can be
   * used.
   */
  get state() {
    return this._param1
  }
  set state(value: number) {
    this._param1 = value
  }

  /**
   * Force immediate transition to the specified MAV_VTOL_STATE. 1: Force immediate, 0: normal
   * transition. Can be used, for example, to trigger an emergency "Quadchute". Caution: Can be
   * dangerous/damage vehicle, depending on autopilot implementation of this command.
   */
  get immediate() {
    return this._param2
  }
  set immediate(value: number) {
    this._param2 = value
  }
}

/**
 * Request authorization to arm the vehicle to a external entity, the arm authorizer is responsible to
 * request all data that is needs from the vehicle before authorize or deny the request. If approved
 * the progress of command_ack message should be set with period of time that this authorization is
 * valid in seconds or in case it was denied it should be set with one of the reasons in
 * ARM_AUTH_DENIED_REASON.
 */
export class ArmAuthorizationRequestCommand extends CommandLong {
  constructor(targetSystem = 1, targetComponent = 1) {
    super()
    this.command = MavCmd.ARM_AUTHORIZATION_REQUEST as number
    this.targetSystem = targetSystem
    this.targetComponent = targetComponent
  }

  /**
   * Vehicle system id, this way ground station can request arm authorization on behalf of any vehicle
   *
   * @min: 0
   * @max: 255
   * @increment: 1
   */
  get systemId() {
    return this._param1
  }
  set systemId(value: number) {
    this._param1 = value
  }
}

/**
 * This command sets the submode to standard guided when vehicle is in guided mode. The vehicle holds
 * position and altitude and the user can input the desired velocities along all three axes.
 */
export class SetGuidedSubmodeStandardCommand extends CommandLong {
  constructor(targetSystem = 1, targetComponent = 1) {
    super()
    this.command = MavCmd.SET_GUIDED_SUBMODE_STANDARD as number
    this.targetSystem = targetSystem
    this.targetComponent = targetComponent
  }
}

/**
 * This command sets submode circle when vehicle is in guided mode. Vehicle flies along a circle facing
 * the center of the circle. The user can input the velocity along the circle and change the radius. If
 * no input is given the vehicle will hold position.
 *
 * This command has location.
 */
export class SetGuidedSubmodeCircleCommand extends CommandLong {
  constructor(targetSystem = 1, targetComponent = 1) {
    super()
    this.command = MavCmd.SET_GUIDED_SUBMODE_CIRCLE as number
    this.targetSystem = targetSystem
    this.targetComponent = targetComponent
  }

  /**
   * Radius of desired circle in CIRCLE_MODE
   *
   * @units m
   */
  get radius() {
    return this._param1
  }
  set radius(value: number) {
    this._param1 = value
  }

  /**
   * Target latitude of center of circle in CIRCLE_MODE
   *
   * @units degE7
   */
  get latitude() {
    return this._param5
  }
  set latitude(value: number) {
    this._param5 = value
  }

  /**
   * Target longitude of center of circle in CIRCLE_MODE
   *
   * @units degE7
   */
  get longitude() {
    return this._param6
  }
  set longitude(value: number) {
    this._param6 = value
  }
}

/**
 * Delay mission state machine until gate has been reached.
 *
 * This command has location.
 * This command is destination.
 */
export class ConditionGateCommand extends CommandLong {
  constructor(targetSystem = 1, targetComponent = 1) {
    super()
    this.command = MavCmd.CONDITION_GATE as number
    this.targetSystem = targetSystem
    this.targetComponent = targetComponent
  }

  /**
   * Geometry: 0: orthogonal to path between previous and next waypoint.
   *
   * @min: 0
   * @increment: 1
   */
  get geometry() {
    return this._param1
  }
  set geometry(value: number) {
    this._param1 = value
  }

  /**
   * Altitude: 0: ignore altitude
   *
   * @min: 0
   * @max: 1
   * @increment: 1
   */
  get usealtitude() {
    return this._param2
  }
  set usealtitude(value: number) {
    this._param2 = value
  }

  /**
   * Latitude
   */
  get latitude() {
    return this._param5
  }
  set latitude(value: number) {
    this._param5 = value
  }

  /**
   * Longitude
   */
  get longitude() {
    return this._param6
  }
  set longitude(value: number) {
    this._param6 = value
  }

  /**
   * Altitude
   *
   * @units m
   */
  get altitude() {
    return this._param7
  }
  set altitude(value: number) {
    this._param7 = value
  }
}

/**
 * Fence return point (there can only be one such point in a geofence definition). If rally points are
 * supported they should be used instead.
 *
 * This command has location.
 * This command is destination.
 */
export class NavFenceReturnPointCommand extends CommandLong {
  constructor(targetSystem = 1, targetComponent = 1) {
    super()
    this.command = MavCmd.NAV_FENCE_RETURN_POINT as number
    this.targetSystem = targetSystem
    this.targetComponent = targetComponent
  }

  /**
   * Latitude
   */
  get latitude() {
    return this._param5
  }
  set latitude(value: number) {
    this._param5 = value
  }

  /**
   * Longitude
   */
  get longitude() {
    return this._param6
  }
  set longitude(value: number) {
    this._param6 = value
  }

  /**
   * Altitude
   *
   * @units m
   */
  get altitude() {
    return this._param7
  }
  set altitude(value: number) {
    this._param7 = value
  }
}

/**
 * Fence vertex for an inclusion polygon (the polygon must not be self-intersecting). The vehicle must
 * stay within this area. Minimum of 3 vertices required.
 *
 * This command has location.
 */
export class NavFencePolygonVertexInclusionCommand extends CommandLong {
  constructor(targetSystem = 1, targetComponent = 1) {
    super()
    this.command = MavCmd.NAV_FENCE_POLYGON_VERTEX_INCLUSION as number
    this.targetSystem = targetSystem
    this.targetComponent = targetComponent
  }

  /**
   * Polygon vertex count
   *
   * @min: 3
   * @increment: 1
   */
  get vertexCount() {
    return this._param1
  }
  set vertexCount(value: number) {
    this._param1 = value
  }

  /**
   * Vehicle must be inside ALL inclusion zones in a single group, vehicle must be inside at least one
   * group, must be the same for all points in each polygon
   *
   * @min: 0
   * @increment: 1
   */
  get inclusionGroup() {
    return this._param2
  }
  set inclusionGroup(value: number) {
    this._param2 = value
  }

  /**
   * Latitude
   */
  get latitude() {
    return this._param5
  }
  set latitude(value: number) {
    this._param5 = value
  }

  /**
   * Longitude
   */
  get longitude() {
    return this._param6
  }
  set longitude(value: number) {
    this._param6 = value
  }
}

/**
 * Fence vertex for an exclusion polygon (the polygon must not be self-intersecting). The vehicle must
 * stay outside this area. Minimum of 3 vertices required.
 *
 * This command has location.
 */
export class NavFencePolygonVertexExclusionCommand extends CommandLong {
  constructor(targetSystem = 1, targetComponent = 1) {
    super()
    this.command = MavCmd.NAV_FENCE_POLYGON_VERTEX_EXCLUSION as number
    this.targetSystem = targetSystem
    this.targetComponent = targetComponent
  }

  /**
   * Polygon vertex count
   *
   * @min: 3
   * @increment: 1
   */
  get vertexCount() {
    return this._param1
  }
  set vertexCount(value: number) {
    this._param1 = value
  }

  /**
   * Latitude
   */
  get latitude() {
    return this._param5
  }
  set latitude(value: number) {
    this._param5 = value
  }

  /**
   * Longitude
   */
  get longitude() {
    return this._param6
  }
  set longitude(value: number) {
    this._param6 = value
  }
}

/**
 * Circular fence area. The vehicle must stay inside this area.
 *
 * This command has location.
 */
export class NavFenceCircleInclusionCommand extends CommandLong {
  constructor(targetSystem = 1, targetComponent = 1) {
    super()
    this.command = MavCmd.NAV_FENCE_CIRCLE_INCLUSION as number
    this.targetSystem = targetSystem
    this.targetComponent = targetComponent
  }

  /**
   * Radius.
   *
   * @units m
   */
  get radius() {
    return this._param1
  }
  set radius(value: number) {
    this._param1 = value
  }

  /**
   * Vehicle must be inside ALL inclusion zones in a single group, vehicle must be inside at least one
   * group
   *
   * @min: 0
   * @increment: 1
   */
  get inclusionGroup() {
    return this._param2
  }
  set inclusionGroup(value: number) {
    this._param2 = value
  }

  /**
   * Latitude
   */
  get latitude() {
    return this._param5
  }
  set latitude(value: number) {
    this._param5 = value
  }

  /**
   * Longitude
   */
  get longitude() {
    return this._param6
  }
  set longitude(value: number) {
    this._param6 = value
  }
}

/**
 * Circular fence area. The vehicle must stay outside this area.
 *
 * This command has location.
 */
export class NavFenceCircleExclusionCommand extends CommandLong {
  constructor(targetSystem = 1, targetComponent = 1) {
    super()
    this.command = MavCmd.NAV_FENCE_CIRCLE_EXCLUSION as number
    this.targetSystem = targetSystem
    this.targetComponent = targetComponent
  }

  /**
   * Radius.
   *
   * @units m
   */
  get radius() {
    return this._param1
  }
  set radius(value: number) {
    this._param1 = value
  }

  /**
   * Latitude
   */
  get latitude() {
    return this._param5
  }
  set latitude(value: number) {
    this._param5 = value
  }

  /**
   * Longitude
   */
  get longitude() {
    return this._param6
  }
  set longitude(value: number) {
    this._param6 = value
  }
}

/**
 * Rally point. You can have multiple rally points defined.
 *
 * This command has location.
 */
export class NavRallyPointCommand extends CommandLong {
  constructor(targetSystem = 1, targetComponent = 1) {
    super()
    this.command = MavCmd.NAV_RALLY_POINT as number
    this.targetSystem = targetSystem
    this.targetComponent = targetComponent
  }

  /**
   * Latitude
   */
  get latitude() {
    return this._param5
  }
  set latitude(value: number) {
    this._param5 = value
  }

  /**
   * Longitude
   */
  get longitude() {
    return this._param6
  }
  set longitude(value: number) {
    this._param6 = value
  }

  /**
   * Altitude
   *
   * @units m
   */
  get altitude() {
    return this._param7
  }
  set altitude(value: number) {
    this._param7 = value
  }
}

/**
 * Commands the vehicle to respond with a sequence of messages UAVCAN_NODE_INFO, one message per every
 * UAVCAN node that is online. Note that some of the response messages can be lost, which the receiver
 * can detect easily by checking whether every received UAVCAN_NODE_STATUS has a matching message
 * UAVCAN_NODE_INFO received earlier; if not, this command should be sent again in order to request
 * re-transmission of the node information messages.
 */
export class UavcanGetNodeInfoCommand extends CommandLong {
  constructor(targetSystem = 1, targetComponent = 1) {
    super()
    this.command = MavCmd.UAVCAN_GET_NODE_INFO as number
    this.targetSystem = targetSystem
    this.targetComponent = targetComponent
  }
}

/**
 * Trigger the start of an ADSB-out IDENT. This should only be used when requested to do so by an Air
 * Traffic Controller in controlled airspace. This starts the IDENT which is then typically held for 18
 * seconds by the hardware per the Mode A, C, and S transponder spec.
 */
export class DoAdsbOutIdentCommand extends CommandLong {
  constructor(targetSystem = 1, targetComponent = 1) {
    super()
    this.command = MavCmd.DO_ADSB_OUT_IDENT as number
    this.targetSystem = targetSystem
    this.targetComponent = targetComponent
  }
}

/**
 * Deploy payload on a Lat / Lon / Alt position. This includes the navigation to reach the required
 * release position and velocity.
 *
 * This command has location.
 * This command is destination.
 */
export class PayloadPrepareDeployCommand extends CommandLong {
  constructor(targetSystem = 1, targetComponent = 1) {
    super()
    this.command = MavCmd.PAYLOAD_PREPARE_DEPLOY as number
    this.targetSystem = targetSystem
    this.targetComponent = targetComponent
  }

  /**
   * Operation mode. 0: prepare single payload deploy (overwriting previous requests), but do not execute
   * it. 1: execute payload deploy immediately (rejecting further deploy commands during execution, but
   * allowing abort). 2: add payload deploy to existing deployment list.
   *
   * @min: 0
   * @max: 2
   * @increment: 1
   */
  get operationMode() {
    return this._param1
  }
  set operationMode(value: number) {
    this._param1 = value
  }

  /**
   * Desired approach vector in compass heading. A negative value indicates the system can define the
   * approach vector at will.
   *
   * @units deg
   * @min: -1
   * @max: 360
   */
  get approachVector() {
    return this._param2
  }
  set approachVector(value: number) {
    this._param2 = value
  }

  /**
   * Desired ground speed at release time. This can be overridden by the airframe in case it needs to
   * meet minimum airspeed. A negative value indicates the system can define the ground speed at will.
   *
   * @min: -1
   */
  get groundSpeed() {
    return this._param3
  }
  set groundSpeed(value: number) {
    this._param3 = value
  }

  /**
   * Minimum altitude clearance to the release position. A negative value indicates the system can define
   * the clearance at will.
   *
   * @units m
   * @min: -1
   */
  get altitudeClearance() {
    return this._param4
  }
  set altitudeClearance(value: number) {
    this._param4 = value
  }

  /**
   * Latitude. Note, if used in MISSION_ITEM (deprecated) the units are degrees (unscaled)
   *
   * @units degE7
   */
  get latitude() {
    return this._param5
  }
  set latitude(value: number) {
    this._param5 = value
  }

  /**
   * Longitude. Note, if used in MISSION_ITEM (deprecated) the units are degrees (unscaled)
   *
   * @units degE7
   */
  get longitude() {
    return this._param6
  }
  set longitude(value: number) {
    this._param6 = value
  }

  /**
   * Altitude (MSL)
   *
   * @units m
   */
  get altitude() {
    return this._param7
  }
  set altitude(value: number) {
    this._param7 = value
  }
}

/**
 * Control the payload deployment.
 */
export class PayloadControlDeployCommand extends CommandLong {
  constructor(targetSystem = 1, targetComponent = 1) {
    super()
    this.command = MavCmd.PAYLOAD_CONTROL_DEPLOY as number
    this.targetSystem = targetSystem
    this.targetComponent = targetComponent
  }

  /**
   * Operation mode. 0: Abort deployment, continue normal mission. 1: switch to payload deployment mode.
   * 100: delete first payload deployment request. 101: delete all payload deployment requests.
   *
   * @min: 0
   * @max: 101
   * @increment: 1
   */
  get operationMode() {
    return this._param1
  }
  set operationMode(value: number) {
    this._param1 = value
  }
}

/**
 * Magnetometer calibration based on provided known yaw. This allows for fast calibration using WMM
 * field tables in the vehicle, given only the known yaw of the vehicle. If Latitude and longitude are
 * both zero then use the current vehicle location.
 *
 * This command has location.
 */
export class FixedMagCalYawCommand extends CommandLong {
  constructor(targetSystem = 1, targetComponent = 1) {
    super()
    this.command = MavCmd.FIXED_MAG_CAL_YAW as number
    this.targetSystem = targetSystem
    this.targetComponent = targetComponent
  }

  /**
   * Yaw of vehicle in earth frame.
   *
   * @units deg
   */
  get yaw() {
    return this._param1
  }
  set yaw(value: number) {
    this._param1 = value
  }

  /**
   * CompassMask, 0 for all.
   */
  get compassmask() {
    return this._param2
  }
  set compassmask(value: number) {
    this._param2 = value
  }

  /**
   * Latitude.
   *
   * @units deg
   */
  get latitude() {
    return this._param3
  }
  set latitude(value: number) {
    this._param3 = value
  }

  /**
   * Longitude.
   *
   * @units deg
   */
  get longitude() {
    return this._param4
  }
  set longitude(value: number) {
    this._param4 = value
  }
}

/**
 * Command to operate winch.
 */
export class DoWinchCommand extends CommandLong {
  constructor(targetSystem = 1, targetComponent = 1) {
    super()
    this.command = MavCmd.DO_WINCH as number
    this.targetSystem = targetSystem
    this.targetComponent = targetComponent
  }

  /**
   * Winch instance number.
   *
   * @min: 1
   * @increment: 1
   */
  get instance() {
    return this._param1
  }
  set instance(value: number) {
    this._param1 = value
  }

  /**
   * Action to perform.
   */
  get action() {
    return this._param2
  }
  set action(value: number) {
    this._param2 = value
  }

  /**
   * Length of cable to release (negative to wind).
   *
   * @units m
   */
  get length() {
    return this._param3
  }
  set length(value: number) {
    this._param3 = value
  }

  /**
   * Release rate (negative to wind).
   *
   * @units m/s
   */
  get rate() {
    return this._param4
  }
  set rate(value: number) {
    this._param4 = value
  }
}

/**
 * User defined waypoint item. Ground Station will show the Vehicle as flying through this item.
 *
 * This command has location.
 * This command is destination.
 */
export class WaypointUser1Command extends CommandLong {
  constructor(targetSystem = 1, targetComponent = 1) {
    super()
    this.command = MavCmd.WAYPOINT_USER_1 as number
    this.targetSystem = targetSystem
    this.targetComponent = targetComponent
  }

  /**
   * Latitude unscaled
   */
  get latitude() {
    return this._param5
  }
  set latitude(value: number) {
    this._param5 = value
  }

  /**
   * Longitude unscaled
   */
  get longitude() {
    return this._param6
  }
  set longitude(value: number) {
    this._param6 = value
  }

  /**
   * Altitude (MSL)
   *
   * @units m
   */
  get altitude() {
    return this._param7
  }
  set altitude(value: number) {
    this._param7 = value
  }
}

/**
 * User defined waypoint item. Ground Station will show the Vehicle as flying through this item.
 *
 * This command has location.
 * This command is destination.
 */
export class WaypointUser2Command extends CommandLong {
  constructor(targetSystem = 1, targetComponent = 1) {
    super()
    this.command = MavCmd.WAYPOINT_USER_2 as number
    this.targetSystem = targetSystem
    this.targetComponent = targetComponent
  }

  /**
   * Latitude unscaled
   */
  get latitude() {
    return this._param5
  }
  set latitude(value: number) {
    this._param5 = value
  }

  /**
   * Longitude unscaled
   */
  get longitude() {
    return this._param6
  }
  set longitude(value: number) {
    this._param6 = value
  }

  /**
   * Altitude (MSL)
   *
   * @units m
   */
  get altitude() {
    return this._param7
  }
  set altitude(value: number) {
    this._param7 = value
  }
}

/**
 * User defined waypoint item. Ground Station will show the Vehicle as flying through this item.
 *
 * This command has location.
 * This command is destination.
 */
export class WaypointUser3Command extends CommandLong {
  constructor(targetSystem = 1, targetComponent = 1) {
    super()
    this.command = MavCmd.WAYPOINT_USER_3 as number
    this.targetSystem = targetSystem
    this.targetComponent = targetComponent
  }

  /**
   * Latitude unscaled
   */
  get latitude() {
    return this._param5
  }
  set latitude(value: number) {
    this._param5 = value
  }

  /**
   * Longitude unscaled
   */
  get longitude() {
    return this._param6
  }
  set longitude(value: number) {
    this._param6 = value
  }

  /**
   * Altitude (MSL)
   *
   * @units m
   */
  get altitude() {
    return this._param7
  }
  set altitude(value: number) {
    this._param7 = value
  }
}

/**
 * User defined waypoint item. Ground Station will show the Vehicle as flying through this item.
 *
 * This command has location.
 * This command is destination.
 */
export class WaypointUser4Command extends CommandLong {
  constructor(targetSystem = 1, targetComponent = 1) {
    super()
    this.command = MavCmd.WAYPOINT_USER_4 as number
    this.targetSystem = targetSystem
    this.targetComponent = targetComponent
  }

  /**
   * Latitude unscaled
   */
  get latitude() {
    return this._param5
  }
  set latitude(value: number) {
    this._param5 = value
  }

  /**
   * Longitude unscaled
   */
  get longitude() {
    return this._param6
  }
  set longitude(value: number) {
    this._param6 = value
  }

  /**
   * Altitude (MSL)
   *
   * @units m
   */
  get altitude() {
    return this._param7
  }
  set altitude(value: number) {
    this._param7 = value
  }
}

/**
 * User defined waypoint item. Ground Station will show the Vehicle as flying through this item.
 *
 * This command has location.
 * This command is destination.
 */
export class WaypointUser5Command extends CommandLong {
  constructor(targetSystem = 1, targetComponent = 1) {
    super()
    this.command = MavCmd.WAYPOINT_USER_5 as number
    this.targetSystem = targetSystem
    this.targetComponent = targetComponent
  }

  /**
   * Latitude unscaled
   */
  get latitude() {
    return this._param5
  }
  set latitude(value: number) {
    this._param5 = value
  }

  /**
   * Longitude unscaled
   */
  get longitude() {
    return this._param6
  }
  set longitude(value: number) {
    this._param6 = value
  }

  /**
   * Altitude (MSL)
   *
   * @units m
   */
  get altitude() {
    return this._param7
  }
  set altitude(value: number) {
    this._param7 = value
  }
}

/**
 * User defined spatial item. Ground Station will not show the Vehicle as flying through this item.
 * Example: ROI item.
 *
 * This command has location.
 */
export class SpatialUser1Command extends CommandLong {
  constructor(targetSystem = 1, targetComponent = 1) {
    super()
    this.command = MavCmd.SPATIAL_USER_1 as number
    this.targetSystem = targetSystem
    this.targetComponent = targetComponent
  }

  /**
   * Latitude unscaled
   */
  get latitude() {
    return this._param5
  }
  set latitude(value: number) {
    this._param5 = value
  }

  /**
   * Longitude unscaled
   */
  get longitude() {
    return this._param6
  }
  set longitude(value: number) {
    this._param6 = value
  }

  /**
   * Altitude (MSL)
   *
   * @units m
   */
  get altitude() {
    return this._param7
  }
  set altitude(value: number) {
    this._param7 = value
  }
}

/**
 * User defined spatial item. Ground Station will not show the Vehicle as flying through this item.
 * Example: ROI item.
 *
 * This command has location.
 */
export class SpatialUser2Command extends CommandLong {
  constructor(targetSystem = 1, targetComponent = 1) {
    super()
    this.command = MavCmd.SPATIAL_USER_2 as number
    this.targetSystem = targetSystem
    this.targetComponent = targetComponent
  }

  /**
   * Latitude unscaled
   */
  get latitude() {
    return this._param5
  }
  set latitude(value: number) {
    this._param5 = value
  }

  /**
   * Longitude unscaled
   */
  get longitude() {
    return this._param6
  }
  set longitude(value: number) {
    this._param6 = value
  }

  /**
   * Altitude (MSL)
   *
   * @units m
   */
  get altitude() {
    return this._param7
  }
  set altitude(value: number) {
    this._param7 = value
  }
}

/**
 * User defined spatial item. Ground Station will not show the Vehicle as flying through this item.
 * Example: ROI item.
 *
 * This command has location.
 */
export class SpatialUser3Command extends CommandLong {
  constructor(targetSystem = 1, targetComponent = 1) {
    super()
    this.command = MavCmd.SPATIAL_USER_3 as number
    this.targetSystem = targetSystem
    this.targetComponent = targetComponent
  }

  /**
   * Latitude unscaled
   */
  get latitude() {
    return this._param5
  }
  set latitude(value: number) {
    this._param5 = value
  }

  /**
   * Longitude unscaled
   */
  get longitude() {
    return this._param6
  }
  set longitude(value: number) {
    this._param6 = value
  }

  /**
   * Altitude (MSL)
   *
   * @units m
   */
  get altitude() {
    return this._param7
  }
  set altitude(value: number) {
    this._param7 = value
  }
}

/**
 * User defined spatial item. Ground Station will not show the Vehicle as flying through this item.
 * Example: ROI item.
 *
 * This command has location.
 */
export class SpatialUser4Command extends CommandLong {
  constructor(targetSystem = 1, targetComponent = 1) {
    super()
    this.command = MavCmd.SPATIAL_USER_4 as number
    this.targetSystem = targetSystem
    this.targetComponent = targetComponent
  }

  /**
   * Latitude unscaled
   */
  get latitude() {
    return this._param5
  }
  set latitude(value: number) {
    this._param5 = value
  }

  /**
   * Longitude unscaled
   */
  get longitude() {
    return this._param6
  }
  set longitude(value: number) {
    this._param6 = value
  }

  /**
   * Altitude (MSL)
   *
   * @units m
   */
  get altitude() {
    return this._param7
  }
  set altitude(value: number) {
    this._param7 = value
  }
}

/**
 * User defined spatial item. Ground Station will not show the Vehicle as flying through this item.
 * Example: ROI item.
 *
 * This command has location.
 */
export class SpatialUser5Command extends CommandLong {
  constructor(targetSystem = 1, targetComponent = 1) {
    super()
    this.command = MavCmd.SPATIAL_USER_5 as number
    this.targetSystem = targetSystem
    this.targetComponent = targetComponent
  }

  /**
   * Latitude unscaled
   */
  get latitude() {
    return this._param5
  }
  set latitude(value: number) {
    this._param5 = value
  }

  /**
   * Longitude unscaled
   */
  get longitude() {
    return this._param6
  }
  set longitude(value: number) {
    this._param6 = value
  }

  /**
   * Altitude (MSL)
   *
   * @units m
   */
  get altitude() {
    return this._param7
  }
  set altitude(value: number) {
    this._param7 = value
  }
}

/**
 * User defined command. Ground Station will not show the Vehicle as flying through this item. Example:
 * MAV_CMD_DO_SET_PARAMETER item.
 */
export class User1Command extends CommandLong {
  constructor(targetSystem = 1, targetComponent = 1) {
    super()
    this.command = MavCmd.USER_1 as number
    this.targetSystem = targetSystem
    this.targetComponent = targetComponent
  }
}

/**
 * User defined command. Ground Station will not show the Vehicle as flying through this item. Example:
 * MAV_CMD_DO_SET_PARAMETER item.
 */
export class User2Command extends CommandLong {
  constructor(targetSystem = 1, targetComponent = 1) {
    super()
    this.command = MavCmd.USER_2 as number
    this.targetSystem = targetSystem
    this.targetComponent = targetComponent
  }
}

/**
 * User defined command. Ground Station will not show the Vehicle as flying through this item. Example:
 * MAV_CMD_DO_SET_PARAMETER item.
 */
export class User3Command extends CommandLong {
  constructor(targetSystem = 1, targetComponent = 1) {
    super()
    this.command = MavCmd.USER_3 as number
    this.targetSystem = targetSystem
    this.targetComponent = targetComponent
  }
}

/**
 * User defined command. Ground Station will not show the Vehicle as flying through this item. Example:
 * MAV_CMD_DO_SET_PARAMETER item.
 */
export class User4Command extends CommandLong {
  constructor(targetSystem = 1, targetComponent = 1) {
    super()
    this.command = MavCmd.USER_4 as number
    this.targetSystem = targetSystem
    this.targetComponent = targetComponent
  }
}

/**
 * User defined command. Ground Station will not show the Vehicle as flying through this item. Example:
 * MAV_CMD_DO_SET_PARAMETER item.
 */
export class User5Command extends CommandLong {
  constructor(targetSystem = 1, targetComponent = 1) {
    super()
    this.command = MavCmd.USER_5 as number
    this.targetSystem = targetSystem
    this.targetComponent = targetComponent
  }
}

export const REGISTRY: MavLinkPacketRegistry = {
  1: SysStatus,
  2: SystemTime,
  4: Ping,
  5: ChangeOperatorControl,
  6: ChangeOperatorControlAck,
  7: AuthKey,
  11: SetMode,
  20: ParamRequestRead,
  21: ParamRequestList,
  22: ParamValue,
  23: ParamSet,
  24: GpsRawInt,
  25: GpsStatus,
  26: ScaledImu,
  27: RawImu,
  28: RawPressure,
  29: ScaledPressure,
  30: Attitude,
  31: AttitudeQuaternion,
  32: LocalPositionNed,
  33: GlobalPositionInt,
  34: RcChannelsScaled,
  35: RcChannelsRaw,
  36: ServoOutputRaw,
  37: MissionRequestPartialList,
  38: MissionWritePartialList,
  39: MissionItem,
  40: MissionRequest,
  41: MissionSetCurrent,
  42: MissionCurrent,
  43: MissionRequestList,
  44: MissionCount,
  45: MissionClearAll,
  46: MissionItemReached,
  47: MissionAck,
  48: SetGpsGlobalOrigin,
  49: GpsGlobalOrigin,
  50: ParamMapRc,
  51: MissionRequestInt,
  54: SafetySetAllowedArea,
  55: SafetyAllowedArea,
  61: AttitudeQuaternionCov,
  62: NavControllerOutput,
  63: GlobalPositionIntCov,
  64: LocalPositionNedCov,
  65: RcChannels,
  66: RequestDataStream,
  67: DataStream,
  69: ManualControl,
  70: RcChannelsOverride,
  73: MissionItemInt,
  74: VfrHud,
  75: CommandInt,
  76: CommandLong,
  77: CommandAck,
  81: ManualSetpoint,
  82: SetAttitudeTarget,
  83: AttitudeTarget,
  84: SetPositionTargetLocalNed,
  85: PositionTargetLocalNed,
  86: SetPositionTargetGlobalInt,
  87: PositionTargetGlobalInt,
  89: LocalPositionNedSystemGlobalOffset,
  90: HilState,
  91: HilControls,
  92: HilRcInputsRaw,
  93: HilActuatorControls,
  100: OpticalFlow,
  101: GlobalVisionPositionEstimate,
  102: VisionPositionEstimate,
  103: VisionSpeedEstimate,
  104: ViconPositionEstimate,
  105: HighresImu,
  106: OpticalFlowRad,
  107: HilSensor,
  108: SimState,
  109: RadioStatus,
  110: FileTransferProtocol,
  111: TimeSync,
  112: CameraTrigger,
  113: HilGps,
  114: HilOpticalFlow,
  115: HilStateQuaternion,
  116: ScaledImu2,
  117: LogRequestList,
  118: LogEntry,
  119: LogRequestData,
  120: LogData,
  121: LogErase,
  122: LogRequestEnd,
  123: GpsInjectData,
  124: Gps2Raw,
  125: PowerStatus,
  126: SerialControl,
  127: GpsRtk,
  128: Gps2Rtk,
  129: ScaledImu3,
  130: DataTransmissionHandshake,
  131: EncapsulatedData,
  132: DistanceSensor,
  133: TerrainRequest,
  134: TerrainData,
  135: TerrainCheck,
  136: TerrainReport,
  137: ScaledPressure2,
  138: MotionCaptureAttPos,
  139: SetActuatorControlTarget,
  140: ActuatorControlTarget,
  141: Altitude,
  142: ResourceRequest,
  143: ScaledPressure3,
  144: FollowTarget,
  146: ControlSystemState,
  147: BatteryStatus,
  148: AutopilotVersion,
  149: LandingTarget,
  162: FenceStatus,
  192: MagCalReport,
  225: EfiStatus,
  175: RallyPoint,
  176: RallyFetchPoint,
  177: CompassMotStatus,
  178: Ahrs2,
  179: CameraStatus,
  180: CameraFeedback,
  181: Battery2,
  182: Ahrs3,
  183: AutopilotVersionRequest,
  184: RemoteLogDataBlock,
  185: RemoteLogBlockStatus,
  186: LedControl,
  201: AcflyhtlPwmchansFb,
  202: AcflyhtlImuRegister,
  203: AcflyhtlImuUpdate,
  206: AcflypossensorInfo,
  208: AcflyRegeisterpossensor,
  209: AcflyUpdatepossensor,
  230: EstimatorStatus,
  231: WindCov,
  232: GpsInput,
  233: GpsRtcmData,
  234: HighLatency,
  235: HighLatency2,
  241: Vibration,
  242: HomePosition,
  243: SetHomePosition,
  244: MessageInterval,
  245: ExtendedSysState,
  246: AdsbVehicle,
  247: Collision,
  248: V2Extension,
  249: MemoryVect,
  250: DebugVect,
  251: NamedValueFloat,
  252: NamedValueInt,
  253: StatusText,
  254: Debug,
  256: SetupSigning,
  257: ButtonChange,
  258: PlayTune,
  259: CameraInformation,
  260: CameraSettings,
  261: StorageInformation,
  262: CameraCaptureStatus,
  263: CameraImageCaptured,
  264: FlightInformation,
  265: MountOrientation,
  266: LoggingData,
  267: LoggingDataAcked,
  268: LoggingAck,
  269: VideoStreamInformation,
  270: VideoStreamStatus,
  299: WifiConfigAp,
  310: UavcanNodeStatus,
  311: UavcanNodeInfo,
  320: ParamExtRequestRead,
  321: ParamExtRequestList,
  322: ParamExtValue,
  323: ParamExtSet,
  324: ParamExtAck,
  330: ObstacleDistance,
  331: Odometry,
  332: TrajectoryRepresentationWaypoints,
  333: TrajectoryRepresentationBezier,
  334: CellularStatus,
  335: IsbdLinkStatus,
  336: CellularConfig,
  339: RawRpm,
  340: UtmGlobalPosition,
  350: DebugFloatArray,
  370: SmartBatteryInfo,
  373: GeneratorStatus,
  375: ActuatorOutputStatus,
  385: Tunnel,
  400: PlayTuneV2,
  401: SupportedTunes,
  500: ComponentExtension43,
  602: BatteryStatusAcfly,
  800: OneToMoreAddrRequestXinguangfei,
  801: OneToMoreAddrXinguangfei,
  9000: WheelDistance,
  9005: WinchStatus,
  12918: OpenDroneIdArmStatus,
  12919: OpenDroneIdSystemUpdate,
  12920: HygrometerSensor,
}

export const COMMANDS: MavLinkCommandRegistry = {
  [MavCmd.SEND_WGA]: SendWgaCommand,
  [MavCmd.WRITE_WGA]: WriteWgaCommand,
  [MavCmd.SET_RTC]: SetRtcCommand,
  [MavCmd.REQUEST_ACFLY_POSSENSOR_INFO_STREAM]: RequestAcflyPossensorInfoStreamCommand,
  [MavCmd.REQUEST_HTL]: RequestHtlCommand,
  [MavCmd.NAV_WAYPOINT]: NavWaypointCommand,
  [MavCmd.NAV_LOITER_UNLIM]: NavLoiterUnlimCommand,
  [MavCmd.NAV_LOITER_TURNS]: NavLoiterTurnsCommand,
  [MavCmd.NAV_LOITER_TIME]: NavLoiterTimeCommand,
  [MavCmd.NAV_RETURN_TO_LAUNCH]: NavReturnToLaunchCommand,
  [MavCmd.NAV_LAND]: NavLandCommand,
  [MavCmd.NAV_TAKEOFF]: NavTakeoffCommand,
  [MavCmd.NAV_LAND_LOCAL]: NavLandLocalCommand,
  [MavCmd.NAV_TAKEOFF_LOCAL]: NavTakeoffLocalCommand,
  [MavCmd.NAV_FOLLOW]: NavFollowCommand,
  [MavCmd.NAV_CONTINUE_AND_CHANGE_ALT]: NavContinueAndChangeAltCommand,
  [MavCmd.NAV_LOITER_TO_ALT]: NavLoiterToAltCommand,
  [MavCmd.DO_FOLLOW]: DoFollowCommand,
  [MavCmd.DO_FOLLOW_REPOSITION]: DoFollowRepositionCommand,
  [MavCmd.DO_ORBIT]: DoOrbitCommand,
  [MavCmd.NAV_ROI]: NavRoiCommand,
  [MavCmd.NAV_PATHPLANNING]: NavPathplanningCommand,
  [MavCmd.NAV_SPLINE_WAYPOINT]: NavSplineWaypointCommand,
  [MavCmd.NAV_VTOL_TAKEOFF]: NavVtolTakeoffCommand,
  [MavCmd.NAV_VTOL_LAND]: NavVtolLandCommand,
  [MavCmd.NAV_GUIDED_ENABLE]: NavGuidedEnableCommand,
  [MavCmd.NAV_DELAY]: NavDelayCommand,
  [MavCmd.NAV_PAYLOAD_PLACE]: NavPayloadPlaceCommand,
  [MavCmd.NAV_LAST]: NavLastCommand,
  [MavCmd.CONDITION_DELAY]: ConditionDelayCommand,
  [MavCmd.CONDITION_CHANGE_ALT]: ConditionChangeAltCommand,
  [MavCmd.CONDITION_DISTANCE]: ConditionDistanceCommand,
  [MavCmd.CONDITION_YAW]: ConditionYawCommand,
  [MavCmd.CONDITION_LAST]: ConditionLastCommand,
  [MavCmd.DO_SET_MODE]: DoSetModeCommand,
  [MavCmd.DO_JUMP]: DoJumpCommand,
  [MavCmd.DO_CHANGE_SPEED]: DoChangeSpeedCommand,
  [MavCmd.DO_SET_HOME]: DoSetHomeCommand,
  [MavCmd.DO_SET_PARAMETER]: DoSetParameterCommand,
  [MavCmd.DO_SET_RELAY]: DoSetRelayCommand,
  [MavCmd.DO_REPEAT_RELAY]: DoRepeatRelayCommand,
  [MavCmd.DO_SET_SERVO]: DoSetServoCommand,
  [MavCmd.DO_REPEAT_SERVO]: DoRepeatServoCommand,
  [MavCmd.DO_FLIGHTTERMINATION]: DoFlightterminationCommand,
  [MavCmd.DO_CHANGE_ALTITUDE]: DoChangeAltitudeCommand,
  [MavCmd.DO_SET_ACTUATOR]: DoSetActuatorCommand,
  [MavCmd.DO_LAND_START]: DoLandStartCommand,
  [MavCmd.DO_RALLY_LAND]: DoRallyLandCommand,
  [MavCmd.DO_GO_AROUND]: DoGoAroundCommand,
  [MavCmd.DO_REPOSITION]: DoRepositionCommand,
  [MavCmd.DO_PAUSE_CONTINUE]: DoPauseContinueCommand,
  [MavCmd.DO_SET_REVERSE]: DoSetReverseCommand,
  [MavCmd.DO_SET_ROI_LOCATION]: DoSetRoiLocationCommand,
  [MavCmd.DO_SET_ROI_WPNEXT_OFFSET]: DoSetRoiWpnextOffsetCommand,
  [MavCmd.DO_SET_ROI_NONE]: DoSetRoiNoneCommand,
  [MavCmd.DO_SET_ROI_SYSID]: DoSetRoiSysidCommand,
  [MavCmd.DO_CONTROL_VIDEO]: DoControlVideoCommand,
  [MavCmd.DO_SET_ROI]: DoSetRoiCommand,
  [MavCmd.DO_DIGICAM_CONFIGURE]: DoDigicamConfigureCommand,
  [MavCmd.DO_DIGICAM_CONTROL]: DoDigicamControlCommand,
  [MavCmd.DO_MOUNT_CONFIGURE]: DoMountConfigureCommand,
  [MavCmd.DO_MOUNT_CONTROL]: DoMountControlCommand,
  [MavCmd.DO_SET_CAM_TRIGG_DIST]: DoSetCamTriggDistCommand,
  [MavCmd.DO_FENCE_ENABLE]: DoFenceEnableCommand,
  [MavCmd.DO_PARACHUTE]: DoParachuteCommand,
  [MavCmd.DO_MOTOR_TEST]: DoMotorTestCommand,
  [MavCmd.DO_INVERTED_FLIGHT]: DoInvertedFlightCommand,
  [MavCmd.DO_GRIPPER]: DoGripperCommand,
  [MavCmd.DO_AUTOTUNE_ENABLE]: DoAutotuneEnableCommand,
  [MavCmd.NAV_SET_YAW_SPEED]: NavSetYawSpeedCommand,
  [MavCmd.DO_SET_CAM_TRIGG_INTERVAL]: DoSetCamTriggIntervalCommand,
  [MavCmd.DO_MOUNT_CONTROL_QUAT]: DoMountControlQuatCommand,
  [MavCmd.DO_GUIDED_MASTER]: DoGuidedMasterCommand,
  [MavCmd.DO_GUIDED_LIMITS]: DoGuidedLimitsCommand,
  [MavCmd.DO_ENGINE_CONTROL]: DoEngineControlCommand,
  [MavCmd.DO_SET_MISSION_CURRENT]: DoSetMissionCurrentCommand,
  [MavCmd.DO_LAST]: DoLastCommand,
  [MavCmd.PREFLIGHT_CALIBRATION]: PreflightCalibrationCommand,
  [MavCmd.PREFLIGHT_SET_SENSOR_OFFSETS]: PreflightSetSensorOffsetsCommand,
  [MavCmd.PREFLIGHT_UAVCAN]: PreflightUavcanCommand,
  [MavCmd.PREFLIGHT_STORAGE]: PreflightStorageCommand,
  [MavCmd.PREFLIGHT_REBOOT_SHUTDOWN]: PreflightRebootShutdownCommand,
  [MavCmd.OVERRIDE_GOTO]: OverrideGotoCommand,
  [MavCmd.OBLIQUE_SURVEY]: ObliqueSurveyCommand,
  [MavCmd.EXT_DRONE_TAKEOFF]: ExtDroneTakeoffCommand,
  [MavCmd.EXT_DRONE_LAND]: ExtDroneLandCommand,
  [MavCmd.EXT_DRONE_MOVE]: ExtDroneMoveCommand,
  [MavCmd.EXT_DRONE_CIRCLE]: ExtDroneCircleCommand,
  [MavCmd.EXT_DRONE_WAYPOINT]: ExtDroneWaypointCommand,
  [MavCmd.EXT_DRONE_CHANGE_SPEED]: ExtDroneChangeSpeedCommand,
  [MavCmd.EXT_DRONE_LIGHT_RGB]: ExtDroneLightRgbCommand,
  [MavCmd.EXT_DRONE_SET_MODE]: ExtDroneSetModeCommand,
  [MavCmd.EXT_DRONE_VERSION_DETECT_MODE_SET]: ExtDroneVersionDetectModeSetCommand,
  [MavCmd.EXT_DRONE_GOTO_CMD]: ExtDroneGotoCmdCommand,
  [MavCmd.EXT_DRONE_OPEMMV_CMD]: ExtDroneOpemmvCmdCommand,
  [MavCmd.EXT_DRONE_TOTAL]: ExtDroneTotalCommand,
  [MavCmd.ACTUATOR_TEST]: ActuatorTestCommand,
  [MavCmd.CONFIGURE_ACTUATOR]: ConfigureActuatorCommand,
  [MavCmd.COMPONENT_ARM_DISARM]: ComponentArmDisarmCommand,
  [MavCmd.RUN_PREARM_CHECKS]: RunPrearmChecksCommand,
  [MavCmd.ILLUMINATOR_ON_OFF]: IlluminatorOnOffCommand,
  [MavCmd.GET_HOME_POSITION]: GetHomePositionCommand,
  [MavCmd.INJECT_FAILURE]: InjectFailureCommand,
  [MavCmd.START_RX_PAIR]: StartRxPairCommand,
  [MavCmd.GET_MESSAGE_INTERVAL]: GetMessageIntervalCommand,
  [MavCmd.SET_MESSAGE_INTERVAL]: SetMessageIntervalCommand,
  [MavCmd.REQUEST_MESSAGE]: RequestMessageCommand,
  [MavCmd.REQUEST_PROTOCOL_VERSION]: RequestProtocolVersionCommand,
  [MavCmd.REQUEST_AUTOPILOT_CAPABILITIES]: RequestAutopilotCapabilitiesCommand,
  [MavCmd.REQUEST_CAMERA_INFORMATION]: RequestCameraInformationCommand,
  [MavCmd.REQUEST_CAMERA_SETTINGS]: RequestCameraSettingsCommand,
  [MavCmd.REQUEST_STORAGE_INFORMATION]: RequestStorageInformationCommand,
  [MavCmd.STORAGE_FORMAT]: StorageFormatCommand,
  [MavCmd.REQUEST_CAMERA_CAPTURE_STATUS]: RequestCameraCaptureStatusCommand,
  [MavCmd.REQUEST_FLIGHT_INFORMATION]: RequestFlightInformationCommand,
  [MavCmd.RESET_CAMERA_SETTINGS]: ResetCameraSettingsCommand,
  [MavCmd.SET_CAMERA_MODE]: SetCameraModeCommand,
  [MavCmd.SET_CAMERA_ZOOM]: SetCameraZoomCommand,
  [MavCmd.SET_CAMERA_FOCUS]: SetCameraFocusCommand,
  [MavCmd.SET_STORAGE_USAGE]: SetStorageUsageCommand,
  [MavCmd.JUMP_TAG]: JumpTagCommand,
  [MavCmd.DO_JUMP_TAG]: DoJumpTagCommand,
  [MavCmd.DO_GIMBAL_MANAGER_PITCHYAW]: DoGimbalManagerPitchyawCommand,
  [MavCmd.DO_GIMBAL_MANAGER_CONFIGURE]: DoGimbalManagerConfigureCommand,
  [MavCmd.DO_GIMBAL_MANAGER_LASER]: DoGimbalManagerLaserCommand,
  [MavCmd.DO_GIMBAL_MANAGER_THERMAL]: DoGimbalManagerThermalCommand,
  [MavCmd.DO_GIMBAL_MANAGER_CENTER]: DoGimbalManagerCenterCommand,
  [MavCmd.DO_GIMBAL_MANAGER_UP]: DoGimbalManagerUpCommand,
  [MavCmd.DO_GIMBAL_MANAGER_DOWN]: DoGimbalManagerDownCommand,
  [MavCmd.DO_GIMBAL_START_TARGET_DETECTION]: DoGimbalStartTargetDetectionCommand,
  [MavCmd.DO_GIMBAL_STOP_TARGET_DETECTION]: DoGimbalStopTargetDetectionCommand,
  [MavCmd.DO_GIMBAL_ENABLE_OSD]: DoGimbalEnableOsdCommand,
  [MavCmd.DO_GIMBAL_DISABLE_OSD]: DoGimbalDisableOsdCommand,
  [MavCmd.IMAGE_START_CAPTURE]: ImageStartCaptureCommand,
  [MavCmd.IMAGE_STOP_CAPTURE]: ImageStopCaptureCommand,
  [MavCmd.REQUEST_CAMERA_IMAGE_CAPTURE]: RequestCameraImageCaptureCommand,
  [MavCmd.DO_TRIGGER_CONTROL]: DoTriggerControlCommand,
  [MavCmd.CAMERA_TRACK_POINT]: CameraTrackPointCommand,
  [MavCmd.CAMERA_TRACK_RECTANGLE]: CameraTrackRectangleCommand,
  [MavCmd.CAMERA_STOP_TRACKING]: CameraStopTrackingCommand,
  [MavCmd.VIDEO_START_CAPTURE]: VideoStartCaptureCommand,
  [MavCmd.VIDEO_STOP_CAPTURE]: VideoStopCaptureCommand,
  [MavCmd.VIDEO_START_STREAMING]: VideoStartStreamingCommand,
  [MavCmd.VIDEO_STOP_STREAMING]: VideoStopStreamingCommand,
  [MavCmd.REQUEST_VIDEO_STREAM_INFORMATION]: RequestVideoStreamInformationCommand,
  [MavCmd.REQUEST_VIDEO_STREAM_STATUS]: RequestVideoStreamStatusCommand,
  [MavCmd.LOGGING_START]: LoggingStartCommand,
  [MavCmd.LOGGING_STOP]: LoggingStopCommand,
  [MavCmd.AIRFRAME_CONFIGURATION]: AirframeConfigurationCommand,
  [MavCmd.CONTROL_HIGH_LATENCY]: ControlHighLatencyCommand,
  [MavCmd.PANORAMA_CREATE]: PanoramaCreateCommand,
  [MavCmd.DO_VTOL_TRANSITION]: DoVtolTransitionCommand,
  [MavCmd.ARM_AUTHORIZATION_REQUEST]: ArmAuthorizationRequestCommand,
  [MavCmd.SET_GUIDED_SUBMODE_STANDARD]: SetGuidedSubmodeStandardCommand,
  [MavCmd.SET_GUIDED_SUBMODE_CIRCLE]: SetGuidedSubmodeCircleCommand,
  [MavCmd.CONDITION_GATE]: ConditionGateCommand,
  [MavCmd.NAV_FENCE_RETURN_POINT]: NavFenceReturnPointCommand,
  [MavCmd.NAV_FENCE_POLYGON_VERTEX_INCLUSION]: NavFencePolygonVertexInclusionCommand,
  [MavCmd.NAV_FENCE_POLYGON_VERTEX_EXCLUSION]: NavFencePolygonVertexExclusionCommand,
  [MavCmd.NAV_FENCE_CIRCLE_INCLUSION]: NavFenceCircleInclusionCommand,
  [MavCmd.NAV_FENCE_CIRCLE_EXCLUSION]: NavFenceCircleExclusionCommand,
  [MavCmd.NAV_RALLY_POINT]: NavRallyPointCommand,
  [MavCmd.UAVCAN_GET_NODE_INFO]: UavcanGetNodeInfoCommand,
  [MavCmd.DO_ADSB_OUT_IDENT]: DoAdsbOutIdentCommand,
  [MavCmd.PAYLOAD_PREPARE_DEPLOY]: PayloadPrepareDeployCommand,
  [MavCmd.PAYLOAD_CONTROL_DEPLOY]: PayloadControlDeployCommand,
  [MavCmd.FIXED_MAG_CAL_YAW]: FixedMagCalYawCommand,
  [MavCmd.DO_WINCH]: DoWinchCommand,
  [MavCmd.WAYPOINT_USER_1]: WaypointUser1Command,
  [MavCmd.WAYPOINT_USER_2]: WaypointUser2Command,
  [MavCmd.WAYPOINT_USER_3]: WaypointUser3Command,
  [MavCmd.WAYPOINT_USER_4]: WaypointUser4Command,
  [MavCmd.WAYPOINT_USER_5]: WaypointUser5Command,
  [MavCmd.SPATIAL_USER_1]: SpatialUser1Command,
  [MavCmd.SPATIAL_USER_2]: SpatialUser2Command,
  [MavCmd.SPATIAL_USER_3]: SpatialUser3Command,
  [MavCmd.SPATIAL_USER_4]: SpatialUser4Command,
  [MavCmd.SPATIAL_USER_5]: SpatialUser5Command,
  [MavCmd.USER_1]: User1Command,
  [MavCmd.USER_2]: User2Command,
  [MavCmd.USER_3]: User3Command,
  [MavCmd.USER_4]: User4Command,
  [MavCmd.USER_5]: User5Command,
}
