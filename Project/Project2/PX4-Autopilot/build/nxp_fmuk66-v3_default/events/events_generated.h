// auto-generated from events.h.jinja



#pragma once

#include <stdint.h>
#include <string.h>

#include <libevents_definitions.h>

namespace events
{
static constexpr int MAX_ARGUMENTS_SIZE = 25; ///< maximum number of bytes for all arguments

static_assert(MAX_ARGUMENTS_SIZE <= sizeof(EventType::arguments), "Argument size mismatch");

enum class LogLevel : uint8_t {
	Emergency = 0,
	Alert = 1,
	Critical = 2,
	Error = 3,
	Warning = 4,
	Notice = 5,
	Info = 6,
	Debug = 7,
	Protocol = 8,
	Disabled = 9,

	Count
};


enum class LogLevelInternal : uint8_t {
	Emergency = 0,
	Alert = 1,
	Critical = 2,
	Error = 3,
	Warning = 4,
	Notice = 5,
	Info = 6,
	Debug = 7,
	Protocol = 8,
	Disabled = 9,

	Count
};


using Log = LogLevel;
using LogInternal = LogLevelInternal;

struct LogLevels {
	LogLevels() {}
	LogLevels(Log external_level) : external(external_level), internal((LogInternal)external_level) {}
	LogLevels(Log external_level, LogInternal internal_level)
		: external(external_level), internal(internal_level) {}

	Log external{Log::Info};
	LogInternal internal{LogInternal::Info};
};

static inline LogInternal internalLogLevel(uint8_t log_levels) {
	return (LogInternal)(log_levels >> 4);
}

static inline Log externalLogLevel(uint8_t log_levels) {
	return (Log)(log_levels & 0xF);
}


namespace px4 // component id: 1
{
namespace enums
{

/**
 Sensor type for failover reporting
*/
enum class sensor_type_t : uint8_t {
	accel = 0, ///< Accelerometer
	gyro = 1, ///< Gyroscope
	mag = 2, ///< Magnetometer

	_max = 2
};


/**
 Bitfield for sensor failover reason
*/
enum class sensor_failover_reason_t : uint16_t {
	no_data = 1, ///< No data
	stale_data = 2, ///< Stale data
	timeout = 4, ///< Timeout
	high_error_count = 8, ///< High Error Count
	high_error_density = 16, ///< High Error Density

	_max = 16
};


static inline sensor_failover_reason_t operator|(sensor_failover_reason_t a, sensor_failover_reason_t b)
{
	return static_cast<sensor_failover_reason_t>(static_cast<uint16_t>(a) | static_cast<uint16_t>(b));
}

static inline bool operator&(sensor_failover_reason_t a, sensor_failover_reason_t b)
{
	return static_cast<uint16_t>(a) & static_cast<uint16_t>(b);
}
/**
 State of the main arming state machine
*/
enum class arming_state_t : uint8_t {
	init = 0, ///< Init
	standby = 1, ///< Standby
	armed = 2, ///< Armed
	standby_error = 3, ///< Standby Error
	shutdown = 4, ///< Shutdown
	inair_restore = 5, ///< In-air Restore

	_max = 5
};


/**
 Reason for entering failsafe
*/
enum class failsafe_reason_t : uint8_t {
	no_rc = 0, ///< No manual control stick input
	no_offboard = 1, ///< No offboard control inputs
	no_rc_and_no_offboard = 2, ///< No manual control stick and no offboard control inputs
	no_local_position = 3, ///< No local position estimate
	no_global_position = 4, ///< No global position estimate
	no_datalink = 5, ///< No datalink
	no_rc_and_no_datalink = 6, ///< No RC and no datalink
	no_gps = 7, ///< No valid GPS

	_max = 7
};


/**
 Reason for arming/disarming
*/
enum class arm_disarm_reason_t : uint8_t {
	transition_to_standby = 0, ///< Transition to standby
	rc_stick = 1, ///< RC
	rc_switch = 2, ///< RC (switch)
	command_internal = 3, ///< internal command
	command_external = 4, ///< external command
	mission_start = 5, ///< mission start
	safety_button = 6, ///< safety button
	auto_disarm_land = 7, ///< landing
	auto_disarm_preflight = 8, ///< auto preflight disarming
	kill_switch = 9, ///< kill switch
	lockdown = 10, ///< lockdown
	failure_detector = 11, ///< failure detector
	shutdown = 12, ///< shutdown request
	unit_test = 13, ///< unit tests
	rc_button = 14, ///< RC (button)

	_max = 14
};


/**
 Flight mode
*/
enum class navigation_mode_t : uint8_t {
	manual = 0, ///< Manual
	altctl = 1, ///< Altitude control
	posctl = 2, ///< Position control
	auto_mission = 3, ///< Mission
	auto_loiter = 4, ///< Hold
	auto_rtl = 5, ///< RTL
	acro = 6, ///< Acro
	offboard = 7, ///< Offboard
	stab = 8, ///< Stabilized
	auto_takeoff = 10, ///< Takeoff
	auto_land = 11, ///< Land
	auto_follow_target = 12, ///< Follow Target
	auto_precland = 13, ///< Precision Landing
	orbit = 14, ///< Orbit
	auto_vtol_takeoff = 15, ///< Vtol Takeoff
	unknown = 255, ///< [Unknown]

	_max = 255
};


/**
 Reason for battery fault
*/
enum class battery_fault_reason_t : uint8_t {
	deep_discharge = 0, ///< Battery has deep discharged
	voltage_spikes = 1, ///< Battery detected voltage spikes
	cell_fail = 2, ///< One or more battery cells have failed
	over_current = 3, ///< Battery reported over-current
	fault_temperature = 4, ///< Battery has reached a critical temperature which can result in a critical failure
	under_temperature = 5, ///< Battery temperature is below operating range
	incompatible_voltage = 6, ///< Vehicle voltage is not compatible with battery one
	incompatible_firmware = 7, ///< Battery firmware is not compatible with current autopilot firmware
	incompatible_model = 8, ///< Battery model is not supported by the system
	hardware_fault = 9, ///< Battery reported an hardware problem
	over_temperature = 10, ///< Battery is over-heating

	_max = 10
};


/**
 Smart battery modes of operation
*/
enum class battery_mode_t : uint8_t {
	unknown = 0, ///< unknown
	autodischarging = 1, ///< auto discharging towards storage level
	hotswap = 2, ///< hot-swap

	_max = 2
};


/**
 Bitfield for ESC failure reason
*/
enum class esc_fault_reason_t : uint8_t {
	over_current = 0, ///< detected over current
	over_voltage = 1, ///< detected over voltage
	motor_over_temp = 2, ///< Motor has reached a critical temperature
	over_rpm = 3, ///< Motor RPM is exceeding the limits
	inconsistent_cmd = 4, ///< received an invalid control command
	motor_stuck = 5, ///< Motor stalled
	failure_generic = 6, ///< detected a generic hardware failure
	motor_warn_temp = 7, ///< Motor is over-heating
	esc_warn_temp = 8, ///< is over-heating
	esc_over_temp = 9, ///< reached a critical temperature

	_max = 9
};


/**
 Suggested actions for the user in case of a safety critical event is triggered
*/
enum class suggested_action_t : uint8_t {
	none = 0, ///< 
	land = 1, ///< Land now!
	reduce_throttle = 2, ///< Reduce throttle!

	_max = 2
};



} // namespace enums
} // namespace px4

namespace common // component id: 0
{
namespace enums
{

/**
 Bitfield for subsystems & components
*/
enum class health_component_t : uint64_t {
	sensor_imu = 1, ///< IMU
	sensor_absolute_pressure = 2, ///< Absolute pressure
	sensor_differential_pressure = 4, ///< Differential pressure
	sensor_gps = 8, ///< GPS
	sensor_optical_flow = 16, ///< Optical flow
	sensor_vision_position = 32, ///< Vision position estimate
	sensor_distance = 64, ///< Distance sensor
	manual_control_input = 128, ///< RC or virtual joystick input
	motors_escs = 256, ///< Motors/ESCs
	utm = 512, ///< UTM
	logging = 1024, ///< Logging
	battery = 2048, ///< Battery
	communication_links = 4096, ///< Communication links
	rate_controller = 8192, ///< Rate controller
	attitude_controller = 16384, ///< Attitude controller
	position_controller = 32768, ///< Position controller
	attitude_estimate = 65536, ///< Attitude estimate
	local_position_estimate = 131072, ///< Local position estimate
	mission = 262144, ///< Mission
	avoidance = 524288, ///< Avoidance
	system = 1048576, ///< System
	camera = 2097152, ///< Camera
	gimbal = 4194304, ///< Gimbal
	payload = 8388608, ///< Payload
	global_position_estimate = 16777216, ///< Global position estimate
	storage = 33554432, ///< Storage

	_max = 33554432
};


static inline health_component_t operator|(health_component_t a, health_component_t b)
{
	return static_cast<health_component_t>(static_cast<uint64_t>(a) | static_cast<uint64_t>(b));
}

static inline bool operator&(health_component_t a, health_component_t b)
{
	return static_cast<uint64_t>(a) & static_cast<uint64_t>(b);
}
/**
 Navigation/flight mode category bits
*/
enum class navigation_mode_category_t : uint8_t {
	current = 1, ///< Current mode
	manual__unused = 2, ///< Fully manual modes (w/o any controller support) (enable once needed)
	rate__unused = 4, ///< Rate-controlled modes (enable once needed)
	attitude__unused = 8, ///< Attitude-controlled modes (enable once needed)
	altitude = 16, ///< Altitude-controlled modes
	position = 32, ///< Position-controlled modes
	autonomous = 64, ///< Autonomous navigation modes
	mission = 128, ///< (Planned) Mission modes

	_max = 128
};


static inline navigation_mode_category_t operator|(navigation_mode_category_t a, navigation_mode_category_t b)
{
	return static_cast<navigation_mode_category_t>(static_cast<uint8_t>(a) | static_cast<uint8_t>(b));
}

static inline bool operator&(navigation_mode_category_t a, navigation_mode_category_t b)
{
	return static_cast<uint8_t>(a) & static_cast<uint8_t>(b);
}
/**
 Calibration type
*/
enum class calibration_type_t : uint16_t {
	accel = 1, ///< Accelerometer
	mag = 2, ///< Magnetometer
	gyro = 4, ///< Gyroscope
	level = 8, ///< Level
	airspeed = 16, ///< Airspeed
	rc = 32, ///< RC

	_max = 32
};


static inline calibration_type_t operator|(calibration_type_t a, calibration_type_t b)
{
	return static_cast<calibration_type_t>(static_cast<uint16_t>(a) | static_cast<uint16_t>(b));
}

static inline bool operator&(calibration_type_t a, calibration_type_t b)
{
	return static_cast<uint16_t>(a) & static_cast<uint16_t>(b);
}
/**
 Calibration Sides Bitfield
*/
enum class calibration_sides_t : uint8_t {
	tail_down = 1, ///< Tail Down
	nose_down = 2, ///< Nose Down
	left_side_down = 4, ///< Left Side Down
	right_side_down = 8, ///< Right Side Down
	upside_down = 16, ///< Upside Down
	down = 32, ///< Down

	_max = 32
};


static inline calibration_sides_t operator|(calibration_sides_t a, calibration_sides_t b)
{
	return static_cast<calibration_sides_t>(static_cast<uint8_t>(a) | static_cast<uint8_t>(b));
}

static inline bool operator&(calibration_sides_t a, calibration_sides_t b)
{
	return static_cast<uint8_t>(a) & static_cast<uint8_t>(b);
}
/**
 Calibration Action/next step
*/
enum class calibration_action_t : uint8_t {
	already_completed = 0, ///< Side already completed, switch to one of the remaining sides
	next_orientation = 1, ///< Switch to next orientation
	rotate = 2, ///< Rotate as shown
	hold_still = 3, ///< Hold still

	_max = 3
};


/**
 Calibration Result
*/
enum class calibration_result_t : uint8_t {
	success = 0, ///< Success
	failed = 1, ///< Failed
	aborted = 2, ///< Aborted

	_max = 2
};



} // namespace enums
} // namespace common




namespace px4 // component id: 1
{




enum class event_id_t : uint32_t {
};

} // namespace px4
namespace common // component id: 0
{

// Event group default


// Event group health

/**
 * Create event 'health_summary'.
 * Message: Health report summary event
 */
static inline EventType create_health_summary(const LogLevels &log_levels, common::enums::health_component_t is_present, common::enums::health_component_t error, common::enums::health_component_t warning)
{
	EventType event{};
	event.id = 1000;
	event.log_levels = ((uint8_t)log_levels.internal << 4) | (uint8_t)log_levels.external;
	memcpy(event.arguments+0, &is_present, sizeof(common::enums::health_component_t));
	memcpy(event.arguments+8, &error, sizeof(common::enums::health_component_t));
	memcpy(event.arguments+16, &warning, sizeof(common::enums::health_component_t));
	return event;
}

/**
 * Decode event 'health_summary'.
 * Message: Health report summary event
 */
static inline void decode_health_summary(const EventType &event, common::enums::health_component_t &is_present, common::enums::health_component_t &error, common::enums::health_component_t &warning)
{
	memcpy(&is_present, event.arguments+0, sizeof(common::enums::health_component_t));
	memcpy(&error, event.arguments+8, sizeof(common::enums::health_component_t));
	memcpy(&warning, event.arguments+16, sizeof(common::enums::health_component_t));
}

// Event group arming_check

/**
 * Create event 'arming_check_summary'.
 * Message: Arming check summary event
 */
static inline EventType create_arming_check_summary(const LogLevels &log_levels, common::enums::health_component_t error, common::enums::health_component_t warning, common::enums::navigation_mode_category_t can_arm)
{
	EventType event{};
	event.id = 1010;
	event.log_levels = ((uint8_t)log_levels.internal << 4) | (uint8_t)log_levels.external;
	memcpy(event.arguments+0, &error, sizeof(common::enums::health_component_t));
	memcpy(event.arguments+8, &warning, sizeof(common::enums::health_component_t));
	memcpy(event.arguments+16, &can_arm, sizeof(common::enums::navigation_mode_category_t));
	return event;
}

/**
 * Decode event 'arming_check_summary'.
 * Message: Arming check summary event
 */
static inline void decode_arming_check_summary(const EventType &event, common::enums::health_component_t &error, common::enums::health_component_t &warning, common::enums::navigation_mode_category_t &can_arm)
{
	memcpy(&error, event.arguments+0, sizeof(common::enums::health_component_t));
	memcpy(&warning, event.arguments+8, sizeof(common::enums::health_component_t));
	memcpy(&can_arm, event.arguments+16, sizeof(common::enums::navigation_mode_category_t));
}

// Event group calibration

/**
 * Create event 'cal_progress'.
 * Message: Calibration progress: {2}%
 */
static inline EventType create_cal_progress(const LogLevels &log_levels, uint8_t proto_ver, int8_t progress, common::enums::calibration_type_t calibration_type, common::enums::calibration_sides_t required_sides)
{
	EventType event{};
	event.id = 1100;
	event.log_levels = ((uint8_t)log_levels.internal << 4) | (uint8_t)log_levels.external;
	memcpy(event.arguments+0, &proto_ver, sizeof(uint8_t));
	memcpy(event.arguments+1, &progress, sizeof(int8_t));
	memcpy(event.arguments+2, &calibration_type, sizeof(common::enums::calibration_type_t));
	memcpy(event.arguments+4, &required_sides, sizeof(common::enums::calibration_sides_t));
	return event;
}

/**
 * Decode event 'cal_progress'.
 * Message: Calibration progress: {2}%
 */
static inline void decode_cal_progress(const EventType &event, uint8_t &proto_ver, int8_t &progress, common::enums::calibration_type_t &calibration_type, common::enums::calibration_sides_t &required_sides)
{
	memcpy(&proto_ver, event.arguments+0, sizeof(uint8_t));
	memcpy(&progress, event.arguments+1, sizeof(int8_t));
	memcpy(&calibration_type, event.arguments+2, sizeof(common::enums::calibration_type_t));
	memcpy(&required_sides, event.arguments+4, sizeof(common::enums::calibration_sides_t));
}
/**
 * Create event 'cal_orientation_detected'.
 * Message: Orientation detected: {1}
 */
static inline EventType create_cal_orientation_detected(const LogLevels &log_levels, common::enums::calibration_sides_t orientation, common::enums::calibration_action_t action)
{
	EventType event{};
	event.id = 1101;
	event.log_levels = ((uint8_t)log_levels.internal << 4) | (uint8_t)log_levels.external;
	memcpy(event.arguments+0, &orientation, sizeof(common::enums::calibration_sides_t));
	memcpy(event.arguments+1, &action, sizeof(common::enums::calibration_action_t));
	return event;
}

/**
 * Decode event 'cal_orientation_detected'.
 * Message: Orientation detected: {1}
 */
static inline void decode_cal_orientation_detected(const EventType &event, common::enums::calibration_sides_t &orientation, common::enums::calibration_action_t &action)
{
	memcpy(&orientation, event.arguments+0, sizeof(common::enums::calibration_sides_t));
	memcpy(&action, event.arguments+1, sizeof(common::enums::calibration_action_t));
}
/**
 * Create event 'cal_orientation_done'.
 * Message: Orientation Complete: {1}, next step: {2}
 */
static inline EventType create_cal_orientation_done(const LogLevels &log_levels, common::enums::calibration_sides_t orientation, common::enums::calibration_action_t action)
{
	EventType event{};
	event.id = 1102;
	event.log_levels = ((uint8_t)log_levels.internal << 4) | (uint8_t)log_levels.external;
	memcpy(event.arguments+0, &orientation, sizeof(common::enums::calibration_sides_t));
	memcpy(event.arguments+1, &action, sizeof(common::enums::calibration_action_t));
	return event;
}

/**
 * Decode event 'cal_orientation_done'.
 * Message: Orientation Complete: {1}, next step: {2}
 */
static inline void decode_cal_orientation_done(const EventType &event, common::enums::calibration_sides_t &orientation, common::enums::calibration_action_t &action)
{
	memcpy(&orientation, event.arguments+0, sizeof(common::enums::calibration_sides_t));
	memcpy(&action, event.arguments+1, sizeof(common::enums::calibration_action_t));
}
/**
 * Create event 'cal_done'.
 * Message: Calibration Complete: {1}
 */
static inline EventType create_cal_done(const LogLevels &log_levels, common::enums::calibration_result_t result)
{
	EventType event{};
	event.id = 1103;
	event.log_levels = ((uint8_t)log_levels.internal << 4) | (uint8_t)log_levels.external;
	memcpy(event.arguments+0, &result, sizeof(common::enums::calibration_result_t));
	return event;
}

/**
 * Decode event 'cal_done'.
 * Message: Calibration Complete: {1}
 */
static inline void decode_cal_done(const EventType &event, common::enums::calibration_result_t &result)
{
	memcpy(&result, event.arguments+0, sizeof(common::enums::calibration_result_t));
}




enum class event_id_t : uint32_t {
	health_summary = 1000,
	arming_check_summary = 1010,
	cal_progress = 1100,
	cal_orientation_detected = 1101,
	cal_orientation_done = 1102,
	cal_done = 1103,
};

} // namespace common


} // namespace events
