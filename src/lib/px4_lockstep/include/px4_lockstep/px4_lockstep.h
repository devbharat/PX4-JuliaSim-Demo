#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#if defined(_WIN32)
#  define PX4_LOCKSTEP_EXPORT __declspec(dllexport)
#else
#  define PX4_LOCKSTEP_EXPORT __attribute__((visibility("default")))
#endif

// Opaque handle managed by the library.
typedef void *px4_lockstep_handle_t;

// Minimal runtime config.
typedef struct px4_lockstep_config_t {
	// If non-zero, initialize Dataman with RAM backend.
	int32_t dataman_use_ram;

	// If non-zero, step Commander in lockstep.
	// Commander will own vehicle_status/vehicle_control_mode/actuator_armed/home_position.
	int32_t enable_commander;          // 0/1
	int32_t commander_rate_hz;         // <=0 => step every tick

	// Module stepping rates in Hz. If <= 0, the module is stepped every call.
	int32_t navigator_rate_hz;
	int32_t mc_pos_control_rate_hz;
	int32_t mc_att_control_rate_hz;
	int32_t mc_rate_control_rate_hz;

	// Enable and step the PX4 control allocator (mixing).
	//
	// If enabled, the lockstep harness will also configure a minimal
	// multicopter actuator geometry (quad-X) via CA_* parameters at startup
	// unless you already configured geometry via an airframe config/params.
	int32_t enable_control_allocator;      // 0/1
	int32_t control_allocator_rate_hz;     // <=0 => step every tick
} px4_lockstep_config_t;

// Inputs that your Julia simulator provides each lockstep tick.
// These map directly to uORB topics that Navigator + controllers expect.
typedef struct px4_lockstep_inputs_t {
	// Absolute sim time in microseconds (monotonic). Must be non-decreasing.
	uint64_t time_us;

	// High-level requests.
	//
	// - If Commander is enabled, these are interpreted as *requests* and
	//   translated into vehicle_command messages (arm + mode change).
	// - If Commander is disabled, these directly drive the minimal
	//   Commander-replacement publications done by the harness.
	int32_t armed;              // 0/1 desired arm state
	int32_t nav_auto_mission;   // 0/1 request AUTO.MISSION
	int32_t nav_auto_rtl;       // 0/1 request AUTO.RTL (optional)
	int32_t landed;             // 0/1

	// Local position in NED (meters) + velocity (m/s)
	float x;
	float y;
	float z;
	float vx;
	float vy;
	float vz;
	float yaw; // rad

	// Global position (degrees + meters). Provide something consistent with the mission file.
	double lat_deg;
	double lon_deg;
	float alt_msl_m;

	// Attitude quaternion (w,x,y,z) and body rates (rad/s)
	float q[4];
	float rates_xyz[3];

	// Battery (optional but required for battery RTL/failsafe integration).
	//
	// battery_warning should use the battery_status_s::BATTERY_WARNING_* enum.
	int32_t battery_connected;     // 0/1
	float battery_voltage_v;       // V
	float battery_current_a;       // A
	float battery_remaining;       // 0..1
	int32_t battery_warning;       // enum
} px4_lockstep_inputs_t;

// Outputs produced by PX4 modules each tick.
typedef struct px4_lockstep_outputs_t {
	// Actuator controls output (roll, pitch, yaw, thrust).
	//
	// Note: in newer PX4 this is populated from vehicle_torque_setpoint +
	// vehicle_thrust_setpoint rather than actuator_controls_0.
	float actuator_controls[8];

	// Mixed/allocated actuator outputs (if control allocator is enabled)
	//
	// - actuator_motors: normalised thrust for up to 12 motors (range [-1, 1], NaN = disarmed)
	// - actuator_servos: normalised positions for up to 8 servos (range [-1, 1], NaN = disarmed)
	float actuator_motors[12];
	float actuator_servos[8];

	// Debug: attitude + rate setpoints (if available)
	float attitude_setpoint_q[4];
	float rates_setpoint_xyz[3];
	float thrust_setpoint_body[3];

	// Debug: mission progress (if available)
	int32_t mission_seq;
	int32_t mission_count;
	int32_t mission_finished; // 0/1

	// Debug: commander/nav state (if Commander is enabled)
	int32_t nav_state;
	int32_t arming_state;
	int32_t battery_warning;

	// Flight task output (trajectory setpoint)
	float trajectory_setpoint_position[3];
	float trajectory_setpoint_velocity[3];
	float trajectory_setpoint_acceleration[3];
	float trajectory_setpoint_yaw;
	float trajectory_setpoint_yawspeed;
} px4_lockstep_outputs_t;

// Create/destroy.
PX4_LOCKSTEP_EXPORT px4_lockstep_handle_t px4_lockstep_create(const px4_lockstep_config_t *cfg);
PX4_LOCKSTEP_EXPORT void px4_lockstep_destroy(px4_lockstep_handle_t handle);

// Load a mission (QGC WPL 110) and preload it into Dataman.
// Returns 0 on success.
PX4_LOCKSTEP_EXPORT int px4_lockstep_load_mission_qgc_wpl(px4_lockstep_handle_t handle, const char *mission_path);

// One lockstep tick.
// Returns 0 on success.
PX4_LOCKSTEP_EXPORT int px4_lockstep_step(px4_lockstep_handle_t handle,
					 const px4_lockstep_inputs_t *in,
					 px4_lockstep_outputs_t *out);

#ifdef __cplusplus
}
#endif
