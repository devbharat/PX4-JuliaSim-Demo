#include <px4_lockstep/px4_lockstep.h>

#include <cmath>
#include <limits>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include <new>

#include <px4_platform_common/init.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/log.h>
#include <drivers/drv_hrt.h>

#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>

#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/home_position.h>

#include <uORB/topics/battery_status.h>
#include <uORB/topics/vehicle_command.h>

#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_rates_setpoint.h>
#include <uORB/topics/vehicle_thrust_setpoint.h>
#include <uORB/topics/actuator_motors.h>
#include <uORB/topics/actuator_servos.h>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/mission_result.h>
#include <uORB/topics/mission.h>
#include <uORB/topics/geofence_status.h>
#include <uORB/topics/vehicle_torque_setpoint.h>
#include <uORB/topics/vehicle_constraints.h>
#include <uORB/topics/trajectory_setpoint.h>
#include <uORB/topics/position_setpoint_triplet.h>

#include <dataman/dataman.h>
#include <lib/parameters/param.h>

// Module headers (make sure px4_lockstep/CMakeLists adds these include dirs)
#include "Commander.hpp"
#include "navigator.h"
#include "navigation.h"
#include "MulticopterPositionControl.hpp"
#include "mc_att_control.hpp"
#include "MulticopterRateControl.hpp"
#include "FlightModeManager.hpp"
#include "ControlAllocator.hpp"

// For VEHICLE_CMD_DO_SET_MODE payloads
#include "px4_custom_mode.h"

// -----------------------------------------------------------------------------
// ABI compatibility helpers
// -----------------------------------------------------------------------------

extern "C" PX4_LOCKSTEP_EXPORT uint32_t px4_lockstep_abi_version(void)
{
	return PX4_LOCKSTEP_ABI_VERSION;
}

extern "C" PX4_LOCKSTEP_EXPORT void px4_lockstep_sizes(uint32_t *in_sz,
				    uint32_t *out_sz,
				    uint32_t *cfg_sz)
{
	if (in_sz) {
		*in_sz = static_cast<uint32_t>(sizeof(px4_lockstep_inputs_t));
	}
	if (out_sz) {
		*out_sz = static_cast<uint32_t>(sizeof(px4_lockstep_outputs_t));
	}
	if (cfg_sz) {
		*cfg_sz = static_cast<uint32_t>(sizeof(px4_lockstep_config_t));
	}
}

namespace {

constexpr float kPi = 3.14159265358979323846f;

static inline float deg2rad(float deg) { return deg * (kPi / 180.0f); }

struct StepRateLimiter {
	uint64_t last_run_us{0};
	uint64_t period_us{0}; // 0 => always

	void set_hz(int32_t hz)
	{
		if (hz > 0) {
			period_us = static_cast<uint64_t>(1000000ULL / static_cast<uint64_t>(hz));
		} else {
			period_us = 0;
		}
	}

	bool should_run(uint64_t now_us)
	{
		if (period_us == 0) {
			return true;
		}
		if ((now_us - last_run_us) >= period_us) {
			last_run_us = now_us;
			return true;
		}
		return false;
	}
};

struct LockstepRuntime {
	px4_lockstep_config_t cfg{};

	// Publishers for the minimum set of topics we inject from Julia.
	uORB::Publication<vehicle_status_s>           pub_vstatus{ORB_ID(vehicle_status)};
	uORB::Publication<vehicle_control_mode_s>     pub_vctl_mode{ORB_ID(vehicle_control_mode)};
	uORB::Publication<actuator_armed_s>           pub_act_armed{ORB_ID(actuator_armed)};
	uORB::Publication<vehicle_local_position_s>   pub_lpos{ORB_ID(vehicle_local_position)};
	uORB::Publication<vehicle_global_position_s>  pub_gpos{ORB_ID(vehicle_global_position)};
	uORB::Publication<vehicle_attitude_s>         pub_att{ORB_ID(vehicle_attitude)};
	uORB::Publication<vehicle_angular_velocity_s> pub_rates{ORB_ID(vehicle_angular_velocity)};
	uORB::Publication<vehicle_land_detected_s>    pub_land{ORB_ID(vehicle_land_detected)};
	uORB::Publication<home_position_s>            pub_home{ORB_ID(home_position)};
	uORB::Publication<battery_status_s>           pub_battery{ORB_ID(battery_status)};
	uORB::Publication<vehicle_command_s>          pub_vehicle_command{ORB_ID(vehicle_command)};
	uORB::Publication<mission_s>                  pub_mission{ORB_ID(mission)};
	uORB::Publication<geofence_status_s>          pub_geofence_status{ORB_ID(geofence_status)};

	// Output subscriptions
	uORB::Subscription sub_vehicle_status{ORB_ID(vehicle_status)};
	uORB::Subscription sub_actuator_armed{ORB_ID(actuator_armed)};
	uORB::Subscription sub_battery_status{ORB_ID(battery_status)};
	uORB::Subscription sub_torque_sp{ORB_ID(vehicle_torque_setpoint)};
	uORB::Subscription sub_act_motors{ORB_ID(actuator_motors)};
	uORB::Subscription sub_act_servos{ORB_ID(actuator_servos)};
	uORB::Subscription sub_att_sp{ORB_ID(vehicle_attitude_setpoint)};
	uORB::Subscription sub_rates_sp{ORB_ID(vehicle_rates_setpoint)};
	uORB::Subscription sub_thrust_sp{ORB_ID(vehicle_thrust_setpoint)};
	uORB::Subscription sub_mission_result{ORB_ID(mission_result)};

	// Debug subscriptions
	uORB::Subscription sub_vehicle_status_dbg{ORB_ID(vehicle_status)};
	uORB::Subscription sub_vehicle_control_mode_dbg{ORB_ID(vehicle_control_mode)};
	uORB::Subscription sub_pos_sp_triplet{ORB_ID(position_setpoint_triplet)};
	uORB::Subscription sub_traj_sp{ORB_ID(trajectory_setpoint)};
	uORB::Subscription sub_vehicle_constraints_dbg{ORB_ID(vehicle_constraints)};

	// PX4 modules we step
	Commander *commander{nullptr};
	Navigator *navigator{nullptr};
	MulticopterPositionControl *mc_pos{nullptr};
	MulticopterAttitudeControl *mc_att{nullptr};
	MulticopterRateControl *mc_rate{nullptr};
	FlightModeManager *flight_mode_mgr{nullptr};
	ControlAllocator *control_alloc{nullptr};

	// Rate limiters
	StepRateLimiter cmd_rate;
	StepRateLimiter nav_rate;
	StepRateLimiter pos_rate;
	StepRateLimiter att_rate;
	StepRateLimiter rate_rate;
	StepRateLimiter alloc_rate;

	// Home position handling
	bool home_set{false};
	uint32_t home_update_count{0};
	double home_lat{std::numeric_limits<double>::quiet_NaN()};
	double home_lon{std::numeric_limits<double>::quiet_NaN()};
	float home_alt{NAN};
	uint64_t last_time_us{0};
	bool last_armed{false};
	int32_t last_req_armed{-1};
	int32_t last_req_nav_mission{-1};
	int32_t last_req_nav_rtl{-1};
	uint64_t last_debug_us{0};

	// Last outputs (so callers get stable outputs even when controllers run at lower rates)
	px4_lockstep_outputs_t last_out{};

	LockstepRuntime()
	{
		cfg = {};
		cfg.dataman_use_ram = 1;
		cfg.enable_commander = 1;
		cfg.commander_rate_hz = 100;
		cfg.navigator_rate_hz = 20;
		cfg.mc_pos_control_rate_hz = 100;
		cfg.mc_att_control_rate_hz = 250;
		cfg.mc_rate_control_rate_hz = 250;
		cfg.enable_control_allocator = 1;
		cfg.control_allocator_rate_hz = 250;

		// Use NaN to indicate "no output" / disarmed for mixed outputs by default.
		for (float &m : last_out.actuator_motors) { m = NAN; }
		for (float &s : last_out.actuator_servos) { s = NAN; }
	}

	~LockstepRuntime()
	{
		delete commander;
		delete navigator;
		delete mc_pos;
		delete mc_att;
		delete mc_rate;
		delete flight_mode_mgr;
		delete control_alloc;
	}
};

// --- Mission file loader (QGC WPL 110) ---

struct WplLine {
	int seq{0};
	int current{0};
	int frame{0};
	int command{0};
	float param1{0.f};
	float param2{0.f};
	float param3{0.f};
	float param4{0.f};
	double x{0.0}; // lat
	double y{0.0}; // lon
	float z{0.f};  // alt
	int autocontinue{1};
};

static bool parse_wpl_line(const std::string &line, WplLine &out)
{
	std::stringstream ss(line);
	ss >> out.seq;
	ss >> out.current;
	ss >> out.frame;
	ss >> out.command;
	ss >> out.param1;
	ss >> out.param2;
	ss >> out.param3;
	ss >> out.param4;
	ss >> out.x;
	ss >> out.y;
	ss >> out.z;
	ss >> out.autocontinue;
	return !ss.fail();
}

static void wpl_to_mission_item(const WplLine &wpl, mission_item_s &mi)
{
	mi = {};
	mi.nav_cmd = static_cast<uint16_t>(wpl.command);
	mi.lat = wpl.x;
	mi.lon = wpl.y;
	mi.altitude = wpl.z;
	mi.autocontinue = (wpl.autocontinue != 0);

	// Frame handling (best-effort): MAV_FRAME_GLOBAL_RELATIVE_ALT = 3
	mi.altitude_is_relative = (wpl.frame == 3);

	// Common MAVLink semantics:
	// param4 is yaw in degrees for many NAV_* commands
	mi.yaw = deg2rad(wpl.param4);

	// Waypoint params
	// MAV_CMD_NAV_WAYPOINT (16): param1 = hold, param2 = acceptance radius
	// MAV_CMD_NAV_TAKEOFF (22): param1 = pitch
	if (wpl.command == 16) {
		mi.time_inside = wpl.param1;
		mi.acceptance_radius = wpl.param2;

	} else if (wpl.command == 22) {
		// acceptance_radius sometimes used by PX4 for takeoff
		mi.acceptance_radius = wpl.param2;
	}

	// Reasonable defaults
	if (!PX4_ISFINITE(mi.acceptance_radius) || mi.acceptance_radius <= 0.f) {
		mi.acceptance_radius = 1.0f;
	}
}

} // namespace

extern "C" {

px4_lockstep_handle_t px4_lockstep_create(const px4_lockstep_config_t *cfg)
{
	static bool platform_initialized = false;
	if (!platform_initialized) {
		if (px4_platform_init() != PX4_OK) {
			PX4_ERR("px4_platform_init failed");
		}
		platform_initialized = true;
	}

	LockstepRuntime *rt = new (std::nothrow) LockstepRuntime();
	if (!rt) {
		return nullptr;
	}

	if (cfg) {
		rt->cfg = *cfg;
	}

	rt->cmd_rate.set_hz(rt->cfg.commander_rate_hz);
	rt->nav_rate.set_hz(rt->cfg.navigator_rate_hz);
	rt->pos_rate.set_hz(rt->cfg.mc_pos_control_rate_hz);
	rt->att_rate.set_hz(rt->cfg.mc_att_control_rate_hz);
	rt->rate_rate.set_hz(rt->cfg.mc_rate_control_rate_hz);
	rt->alloc_rate.set_hz(rt->cfg.control_allocator_rate_hz);

	// Enable lockstep HRT time on POSIX (patched in platforms/posix).
	// Safe to call even if the lockstep functions are stubs on other platforms.
	hrt_lockstep_enable(true);

	// Dataman: initialize RAM backend and enable synchronous mode
	if (rt->cfg.dataman_use_ram) {
		// Patched API: initializes RAM backend without worker thread.
		if (dm_lockstep_init(true) != 0) {
			PX4_ERR("dm_lockstep_init failed");
		}
	}
	dm_lockstep_set_sync(true);

	// Parameters: we keep things minimal, but control allocation needs some CA_* parameters.
	// In normal PX4 these come from the airframe config file + Actuators setup.
	// In lockstep we default to a simple quad-X geometry unless you already set these elsewhere.
	(void)param_init();

	auto set_param_i32 = [](const char *name, int32_t v) {
		param_t h = param_find(name);
		if (h == PARAM_INVALID) {
			PX4_WARN("param not found: %s", name);
			return;
		}
		(void)param_set(h, &v);
	};

	auto set_param_f32 = [](const char *name, float v) {
		param_t h = param_find(name);
		if (h == PARAM_INVALID) {
			PX4_WARN("param not found: %s", name);
			return;
		}
		(void)param_set(h, &v);
	};

	// Commander-related defaults for lockstep simulation.
	//
	// - COM_LOW_BAT_ACT=3: RTL at critical battery, land at emergency.
	// - COM_RC_IN_MODE=1: allow running without RC input (common in pure SITL).
	//
	// These are only applied in the lockstep harness build and can be overridden
	// by normal PX4 param mechanisms if desired.
	if (rt->cfg.enable_commander) {
		set_param_i32("COM_LOW_BAT_ACT", 3);
		set_param_i32("COM_RC_IN_MODE", 1);
	}

	if (rt->cfg.enable_control_allocator) {
		// Enable dynamic control allocation.
		set_param_i32("SYS_CTRL_ALLOC", 1);

		// Minimal multicopter (CA_AIRFRAME=0) quad-X geometry.
		// NOTE: these should eventually come from your sim/vehicle config.
		set_param_i32("CA_AIRFRAME", 0);
		set_param_i32("CA_ROTOR_COUNT", 4);

		const float km_ccw = +0.05f;
		const float km_cw  = -0.05f;

		// Rotor order/geometry matching the Iris defaults (see 10016_none_iris).
		set_param_f32("CA_ROTOR0_PX", +0.1515f); set_param_f32("CA_ROTOR0_PY", +0.2450f); set_param_f32("CA_ROTOR0_PZ", 0.f); set_param_f32("CA_ROTOR0_KM", km_ccw);
		set_param_f32("CA_ROTOR1_PX", -0.1515f); set_param_f32("CA_ROTOR1_PY", -0.1875f); set_param_f32("CA_ROTOR1_PZ", 0.f); set_param_f32("CA_ROTOR1_KM", km_ccw);
		set_param_f32("CA_ROTOR2_PX", +0.1515f); set_param_f32("CA_ROTOR2_PY", -0.2450f); set_param_f32("CA_ROTOR2_PZ", 0.f); set_param_f32("CA_ROTOR2_KM", km_cw);
		set_param_f32("CA_ROTOR3_PX", -0.1515f); set_param_f32("CA_ROTOR3_PY", +0.1875f); set_param_f32("CA_ROTOR3_PZ", 0.f); set_param_f32("CA_ROTOR3_KM", km_cw);
	}

	// Create PX4 modules (do NOT start their tasks).
	if (rt->cfg.enable_commander) {
		rt->commander = new (std::nothrow) Commander();
	}
	rt->navigator = new (std::nothrow) Navigator();
	rt->flight_mode_mgr = new (std::nothrow) FlightModeManager();
	rt->mc_pos = new (std::nothrow) MulticopterPositionControl();
	rt->mc_att = new (std::nothrow) MulticopterAttitudeControl();
	rt->mc_rate = new (std::nothrow) MulticopterRateControl();
	if (rt->cfg.enable_control_allocator) {
		rt->control_alloc = new (std::nothrow) ControlAllocator();
	}

	if ((rt->cfg.enable_commander && !rt->commander)
		|| !rt->navigator
		|| !rt->flight_mode_mgr
		|| !rt->mc_pos
		|| !rt->mc_att
		|| !rt->mc_rate
		|| (rt->cfg.enable_control_allocator && !rt->control_alloc)) {
		PX4_ERR("lockstep: failed to allocate modules");
		px4_lockstep_destroy(rt);
		return nullptr;
	}

	// Put modules into lockstep mode (patched API in each module).
	if (rt->commander) {
		rt->commander->enable_lockstep(true);
	}
	rt->navigator->enable_lockstep(true);
	rt->flight_mode_mgr->enable_lockstep(true);
	rt->mc_pos->enable_lockstep(true);
	rt->mc_att->enable_lockstep(true);
	rt->mc_rate->enable_lockstep(true);
	if (rt->control_alloc) {
		rt->control_alloc->enable_lockstep(true);
	}

	// Run module-specific lockstep init (no callbacks, no scheduling).
	if (rt->commander) {
		(void)rt->commander->init_lockstep();
	}
	// Navigator initialization happens in run_once via its internal guard.
	(void)rt->flight_mode_mgr->init_lockstep();
	(void)rt->mc_pos->init_lockstep();
	(void)rt->mc_att->init_lockstep();
	(void)rt->mc_rate->init_lockstep();
	if (rt->control_alloc) {
		(void)rt->control_alloc->init_lockstep();
	}

	PX4_INFO("px4_lockstep created");
	return reinterpret_cast<px4_lockstep_handle_t>(rt);
}

void px4_lockstep_destroy(px4_lockstep_handle_t handle)
{
	LockstepRuntime *rt = reinterpret_cast<LockstepRuntime *>(handle);
	if (!rt) {
		return;
	}

	delete rt;
}

int px4_lockstep_load_mission_qgc_wpl(px4_lockstep_handle_t handle, const char *mission_path)
{
	LockstepRuntime *rt = reinterpret_cast<LockstepRuntime *>(handle);
	if (!rt || !mission_path) {
		return -1;
	}

	std::ifstream f(mission_path);
	if (!f.is_open()) {
		PX4_ERR("failed to open mission file: %s", mission_path);
		return -2;
	}

	std::string line;
	if (!std::getline(f, line)) {
		PX4_ERR("empty mission file");
		return -3;
	}

	// Header: "QGC WPL 110"
	if (line.find("QGC") == std::string::npos) {
		PX4_WARN("mission header not QGC WPL, trying to parse anyway");
	}

	std::vector<mission_item_s> items;
	items.reserve(128);

	while (std::getline(f, line)) {
		if (line.empty()) {
			continue;
		}
		WplLine wpl;
		if (!parse_wpl_line(line, wpl)) {
			PX4_WARN("skip malformed WPL line: %s", line.c_str());
			continue;
		}
		mission_item_s mi{};
		wpl_to_mission_item(wpl, mi);
		items.push_back(mi);
	}

	if (items.empty()) {
		PX4_ERR("mission contains no items");
		return -4;
	}

	// Preload into Dataman using the same keys Mission uses.
	mission_s mission_state{};
	mission_state.timestamp = hrt_absolute_time();
	if (mission_state.timestamp == 0) {
		mission_state.timestamp = 1;
	}
	mission_state.mission_dataman_id = DM_KEY_WAYPOINTS_OFFBOARD_0;
	mission_state.fence_dataman_id = DM_KEY_FENCE_POINTS_0;
	mission_state.safepoint_dataman_id = DM_KEY_SAFE_POINTS_0;
	mission_state.count = static_cast<uint16_t>(items.size());
	mission_state.current_seq = 0;
	mission_state.land_start_index = -1;
	mission_state.land_index = -1;
	mission_state.mission_id = mission_state.timestamp & 0xffffffffu;
	mission_state.geofence_id = 0u;
	mission_state.safe_points_id = 0u;

	// Store mission state
	if (!dm_lockstep_write(DM_KEY_MISSION_STATE, 0, &mission_state, sizeof(mission_state))) {
		PX4_ERR("dm_lockstep_write mission_state failed");
		return -5;
	}

	// Store mission items
	for (unsigned i = 0; i < items.size(); i++) {
		mission_item_s mi = items[i];
		if (!dm_lockstep_write(static_cast<dm_item_t>(mission_state.mission_dataman_id), i, &mi, sizeof(mi))) {
			PX4_ERR("dm_lockstep_write mission item %u failed", i);
			return -6;
		}
	}

	rt->pub_mission.publish(mission_state);

	PX4_INFO("mission loaded: %u items", (unsigned)items.size());
	return 0;
}

static void publish_inputs(LockstepRuntime &rt, const px4_lockstep_inputs_t &in)
{
	// Battery (for commander failsafe + logging; required for battery RTL).
	battery_status_s bat{};
	bat.timestamp = in.time_us;
	bat.connected = (in.battery_connected != 0);
	bat.voltage_v = in.battery_voltage_v;
	bat.current_a = in.battery_current_a;
	bat.remaining = in.battery_remaining;
	bat.warning = static_cast<uint8_t>(in.battery_warning);
	rt.pub_battery.publish(bat);

	// Geofence status: mark as ready so mission feasibility checks can run.
	geofence_status_s geofence_status{};
	geofence_status.timestamp = in.time_us;
	geofence_status.geofence_id = 0u;
	geofence_status.status = geofence_status_s::GF_STATUS_READY;
	rt.pub_geofence_status.publish(geofence_status);

	// If Commander is disabled, we publish a minimal replacement set of topics.
	if (!rt.cfg.enable_commander) {
		rt.last_armed = (in.armed != 0);
		const bool auto_mode = (in.nav_auto_mission || in.nav_auto_rtl);

		// Vehicle status
		vehicle_status_s vstatus{};
		vstatus.timestamp = in.time_us;
		vstatus.arming_state = in.armed ? vehicle_status_s::ARMING_STATE_ARMED : vehicle_status_s::ARMING_STATE_DISARMED;
		if (in.nav_auto_rtl) {
			vstatus.nav_state = vehicle_status_s::NAVIGATION_STATE_AUTO_RTL;
		} else if (in.nav_auto_mission) {
			vstatus.nav_state = vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION;
		} else {
			vstatus.nav_state = vehicle_status_s::NAVIGATION_STATE_MANUAL;
		}
		vstatus.nav_state_user_intention = vstatus.nav_state;
		vstatus.vehicle_type = vehicle_status_s::VEHICLE_TYPE_ROTARY_WING;
		vstatus.is_vtol = false;
		rt.pub_vstatus.publish(vstatus);

		// Control mode (Commander replacement)
		vehicle_control_mode_s vcm{};
		vcm.timestamp = in.time_us;
		vcm.flag_armed = in.armed;
		vcm.flag_control_auto_enabled = auto_mode;
		vcm.flag_multicopter_position_control_enabled = auto_mode;
		vcm.flag_control_position_enabled = true;
		vcm.flag_control_velocity_enabled = true;
		vcm.flag_control_attitude_enabled = true;
		vcm.flag_control_rates_enabled = true;
		vcm.flag_control_altitude_enabled = true;
		vcm.flag_control_climb_rate_enabled = true;
		vcm.flag_control_manual_enabled = !auto_mode;
		vcm.flag_control_allocation_enabled = (rt.cfg.enable_control_allocator != 0);
		vcm.source_id = vstatus.nav_state;
		rt.pub_vctl_mode.publish(vcm);

		// Actuator arming state (used by allocators/output drivers to gate output)
		actuator_armed_s armed{};
		armed.timestamp = in.time_us;
		armed.armed = in.armed;
		armed.prearmed = in.armed;
		armed.ready_to_arm = true;
		armed.lockdown = false;
		rt.pub_act_armed.publish(armed);
	}

	// Local position
	vehicle_local_position_s lpos{};
	lpos.timestamp = in.time_us;
	lpos.timestamp_sample = in.time_us;
	lpos.x = in.x;
	lpos.y = in.y;
	lpos.z = in.z;
	lpos.vx = in.vx;
	lpos.vy = in.vy;
	lpos.vz = in.vz;
	lpos.z_deriv = in.vz;
	lpos.heading = in.yaw;
	lpos.xy_valid = true;
	lpos.z_valid = true;
	lpos.v_xy_valid = true;
	lpos.v_z_valid = true;
	lpos.heading_good_for_control = true;
	// Do not apply estimator-imposed speed/height limits in lockstep.
	lpos.vxy_max = NAN;
	lpos.vz_max = NAN;
	lpos.hagl_min = NAN;
	lpos.hagl_max_z = NAN;
	lpos.hagl_max_xy = NAN;

	const double ref_lat = rt.home_set ? rt.home_lat : in.lat_deg;
	const double ref_lon = rt.home_set ? rt.home_lon : in.lon_deg;
	const float ref_alt = rt.home_set ? rt.home_alt : in.alt_msl_m;
	const bool has_ref = PX4_ISFINITE(static_cast<float>(ref_lat))
			&& PX4_ISFINITE(static_cast<float>(ref_lon))
			&& PX4_ISFINITE(ref_alt);
	lpos.xy_global = has_ref;
	lpos.z_global = has_ref;
	lpos.ref_timestamp = in.time_us;
	lpos.ref_lat = ref_lat;
	lpos.ref_lon = ref_lon;
	lpos.ref_alt = ref_alt;
	rt.pub_lpos.publish(lpos);

	// Global position
	vehicle_global_position_s gpos{};
	gpos.timestamp = in.time_us;
	gpos.timestamp_sample = in.time_us;
	gpos.lat = in.lat_deg;
	gpos.lon = in.lon_deg;
	gpos.alt = in.alt_msl_m;
	gpos.lat_lon_valid = true;
	gpos.alt_valid = true;
	gpos.eph = 1.0f;
	gpos.epv = 1.0f;
	rt.pub_gpos.publish(gpos);

	// Attitude
	vehicle_attitude_s att{};
	att.timestamp = in.time_us;
	att.q[0] = in.q[0];
	att.q[1] = in.q[1];
	att.q[2] = in.q[2];
	att.q[3] = in.q[3];
	rt.pub_att.publish(att);

	// Body rates
	vehicle_angular_velocity_s rates{};
	rates.timestamp = in.time_us;
	rates.xyz[0] = in.rates_xyz[0];
	rates.xyz[1] = in.rates_xyz[1];
	rates.xyz[2] = in.rates_xyz[2];
	rt.pub_rates.publish(rates);

	// Land detector
	vehicle_land_detected_s land{};
	land.timestamp = in.time_us;
	land.landed = in.landed;
	land.ground_contact = in.landed;
	land.maybe_landed = in.landed;
	rt.pub_land.publish(land);

	// Home position: publish every tick in lockstep (Commander disabled).
	if (!rt.cfg.enable_commander) {
		if (!rt.home_set && PX4_ISFINITE(static_cast<float>(in.lat_deg)) && PX4_ISFINITE(static_cast<float>(in.lon_deg))) {
			rt.home_set = true;
			rt.home_lat = in.lat_deg;
			rt.home_lon = in.lon_deg;
			rt.home_alt = in.alt_msl_m;
		}

		if (rt.home_set) {
			home_position_s home{};
			home.timestamp = in.time_us;
			home.lat = rt.home_lat;
			home.lon = rt.home_lon;
			home.alt = rt.home_alt;
			home.valid_hpos = true;
			home.valid_lpos = true;
			home.valid_alt = true;
			home.update_count = ++rt.home_update_count;
			rt.pub_home.publish(home);
		}
	}

	// When Commander is enabled, translate requested actions into vehicle_command messages.
	// These are edge-triggered (we only send on change) to avoid fighting Commander failsafe logic.
	if (rt.cfg.enable_commander) {
		auto send_vehicle_command = [&](uint16_t command, float p1 = 0.f, float p2 = 0.f, float p3 = 0.f,
					float p4 = 0.f, double p5 = 0.0, double p6 = 0.0, float p7 = 0.f) {
			vehicle_command_s cmd{};
			cmd.timestamp = in.time_us;
			cmd.param1 = p1;
			cmd.param2 = p2;
			cmd.param3 = p3;
			cmd.param4 = p4;
			cmd.param5 = p5;
			cmd.param6 = p6;
			cmd.param7 = p7;
			cmd.command = command;
			cmd.target_system = 1;
			cmd.target_component = 1;
			cmd.source_system = 1;
			cmd.source_component = 1;
			cmd.confirmation = 0;
			cmd.from_external = false;
			rt.pub_vehicle_command.publish(cmd);
		};

		// Arm/disarm request
		if (rt.last_req_armed != in.armed) {
			send_vehicle_command(vehicle_command_s::VEHICLE_CMD_COMPONENT_ARM_DISARM, in.armed ? 1.f : 0.f);
			rt.last_req_armed = in.armed;
		}

		// Mode requests: RTL takes priority.
		// Only send on rising edge to avoid overriding failsafe mode changes (e.g. battery RTL).
		if (rt.last_req_nav_rtl != in.nav_auto_rtl) {
			if (in.nav_auto_rtl) {
				union px4_custom_mode custom{};
				custom.data = 0;
				custom.main_mode = PX4_CUSTOM_MAIN_MODE_AUTO;
				custom.sub_mode = PX4_CUSTOM_SUB_MODE_AUTO_RTL;
				send_vehicle_command(vehicle_command_s::VEHICLE_CMD_DO_SET_MODE,
					1.f /* MAV_MODE_FLAG_CUSTOM_MODE_ENABLED */,
					static_cast<float>(custom.data));
			}
			rt.last_req_nav_rtl = in.nav_auto_rtl;
		}

		if (!in.nav_auto_rtl && (rt.last_req_nav_mission != in.nav_auto_mission)) {
			if (in.nav_auto_mission) {
				union px4_custom_mode custom{};
				custom.data = 0;
				custom.main_mode = PX4_CUSTOM_MAIN_MODE_AUTO;
				custom.sub_mode = PX4_CUSTOM_SUB_MODE_AUTO_MISSION;
				send_vehicle_command(vehicle_command_s::VEHICLE_CMD_DO_SET_MODE,
					1.f /* MAV_MODE_FLAG_CUSTOM_MODE_ENABLED */,
					static_cast<float>(custom.data));
			}
			rt.last_req_nav_mission = in.nav_auto_mission;
		}
	}
}

static void read_outputs(LockstepRuntime &rt, px4_lockstep_outputs_t &out)
{
	// Start from the last outputs so callers get stable values even if PX4 modules
	// run at lower rates than the Julia sim step.
	out = rt.last_out;

	// Actuator controls (mapped from torque/thrust setpoints in newer PX4)
	vehicle_torque_setpoint_s torque_sp{};
	if (rt.sub_torque_sp.update(&torque_sp)) {
		out.actuator_controls[0] = torque_sp.xyz[0];
		out.actuator_controls[1] = torque_sp.xyz[1];
		out.actuator_controls[2] = torque_sp.xyz[2];
	}

	vehicle_thrust_setpoint_s thrust_sp{};
	if (rt.sub_thrust_sp.update(&thrust_sp)) {
		out.actuator_controls[3] = thrust_sp.xyz[2];
	}

	// Mixed/allocated outputs (from control allocator)
	actuator_motors_s motors{};
	if (rt.sub_act_motors.update(&motors)) {
		for (int i = 0; i < 12; i++) {
			out.actuator_motors[i] = motors.control[i];
		}
	}

	actuator_servos_s servos{};
	if (rt.sub_act_servos.update(&servos)) {
		for (int i = 0; i < 8; i++) {
			out.actuator_servos[i] = servos.control[i];
		}
	}

	vehicle_attitude_setpoint_s att_sp{};
	if (rt.sub_att_sp.update(&att_sp)) {
		out.attitude_setpoint_q[0] = att_sp.q_d[0];
		out.attitude_setpoint_q[1] = att_sp.q_d[1];
		out.attitude_setpoint_q[2] = att_sp.q_d[2];
		out.attitude_setpoint_q[3] = att_sp.q_d[3];
		out.thrust_setpoint_body[0] = att_sp.thrust_body[0];
		out.thrust_setpoint_body[1] = att_sp.thrust_body[1];
		out.thrust_setpoint_body[2] = att_sp.thrust_body[2];
	}

	vehicle_rates_setpoint_s rates_sp{};
	if (rt.sub_rates_sp.update(&rates_sp)) {
		out.rates_setpoint_xyz[0] = rates_sp.roll;
		out.rates_setpoint_xyz[1] = rates_sp.pitch;
		out.rates_setpoint_xyz[2] = rates_sp.yaw;
	}

	mission_result_s mres{};
	if (rt.sub_mission_result.update(&mres)) {
		out.mission_seq = static_cast<int32_t>(mres.seq_current);
		out.mission_count = static_cast<int32_t>(mres.seq_total);
		out.mission_finished = mres.finished ? 1 : 0;
	}

	// Commander / navigation state
	vehicle_status_s vstatus{};
	if (rt.sub_vehicle_status.update(&vstatus)) {
		out.nav_state = static_cast<int32_t>(vstatus.nav_state);
		out.arming_state = static_cast<int32_t>(vstatus.arming_state);
	}

	// Battery state
	battery_status_s bat{};
	if (rt.sub_battery_status.update(&bat)) {
		out.battery_warning = static_cast<int32_t>(bat.warning);
	}

	// Trajectory setpoint (from FlightModeManager)
	trajectory_setpoint_s traj{};
	if (rt.sub_traj_sp.update(&traj)) {
		for (int i = 0; i < 3; i++) {
			out.trajectory_setpoint_position[i] = traj.position[i];
			out.trajectory_setpoint_velocity[i] = traj.velocity[i];
			out.trajectory_setpoint_acceleration[i] = traj.acceleration[i];
		}
		out.trajectory_setpoint_yaw = traj.yaw;
		out.trajectory_setpoint_yawspeed = traj.yawspeed;
	}

	// Track arming state (for gating outputs)
	actuator_armed_s armed{};
	if (rt.sub_actuator_armed.update(&armed)) {
		rt.last_armed = armed.armed;
	}

	const bool is_armed = rt.last_armed;

	// Ensure disarmed output is clearly communicated to the sim.
	if (!is_armed) {
		for (float &m : out.actuator_motors) { m = NAN; }
		for (float &s : out.actuator_servos) { s = NAN; }
	}

	rt.last_out = out;
}

static void debug_state(LockstepRuntime &rt, uint64_t now_us)
{
	if (rt.last_debug_us != 0 && (now_us - rt.last_debug_us) < 1000000ULL) {
		return;
	}
	rt.last_debug_us = now_us;

	vehicle_status_s vstatus{};
	const bool has_status = rt.sub_vehicle_status_dbg.update(&vstatus);

	vehicle_control_mode_s vcm{};
	const bool has_vcm = rt.sub_vehicle_control_mode_dbg.update(&vcm);

	position_setpoint_triplet_s triplet{};
	const bool has_triplet = rt.sub_pos_sp_triplet.update(&triplet);

	trajectory_setpoint_s traj{};
	const bool has_traj = rt.sub_traj_sp.copy(&traj);
	vehicle_constraints_s constraints{};
	const bool has_constraints = rt.sub_vehicle_constraints_dbg.update(&constraints);

	PX4_INFO("lockstep dbg t=%llu status=%d auto=%d armed=%d", (unsigned long long)now_us,
		int(has_status), int(has_vcm ? vcm.flag_control_auto_enabled : false),
		int(has_status ? vstatus.arming_state == vehicle_status_s::ARMING_STATE_ARMED : false));

	if (has_triplet) {
		PX4_INFO("lockstep triplet valid=%d type=%u alt=%.2f lat=%.6f lon=%.6f", (int)triplet.current.valid,
			(unsigned)triplet.current.type, (double)triplet.current.alt, triplet.current.lat, triplet.current.lon);
	} else {
		PX4_INFO("lockstep triplet none");
	}

	if (has_traj) {
		PX4_INFO("lockstep traj z=%.2f vz=%.2f az=%.2f yaw=%.2f", (double)traj.position[2],
			(double)traj.velocity[2], (double)traj.acceleration[2], (double)traj.yaw);
	} else {
		PX4_INFO("lockstep traj none");
	}

	if (has_constraints) {
		PX4_INFO("lockstep constraints takeoff=%d speed_up=%.2f", (int)constraints.want_takeoff,
			(double)constraints.speed_up);
	} else {
		PX4_INFO("lockstep constraints none");
	}

	PX4_INFO("lockstep outputs thrust=%.3f rates=(%.3f,%.3f,%.3f)",
		(double)rt.last_out.thrust_setpoint_body[2],
		(double)rt.last_out.rates_setpoint_xyz[0],
		(double)rt.last_out.rates_setpoint_xyz[1],
		(double)rt.last_out.rates_setpoint_xyz[2]);
}

int px4_lockstep_step(px4_lockstep_handle_t handle,
			 const px4_lockstep_inputs_t *in,
			 px4_lockstep_outputs_t *out)
{
	LockstepRuntime *rt = reinterpret_cast<LockstepRuntime *>(handle);
	if (!rt || !in || !out) {
		return -1;
	}

	// Basic monotonic guarantee
	if (rt->last_time_us != 0 && in->time_us < rt->last_time_us) {
		PX4_WARN("lockstep: time went backwards (%llu -> %llu)",
			(unsigned long long)rt->last_time_us, (unsigned long long)in->time_us);
	}
	rt->last_time_us = in->time_us;

	// Drive PX4 timebase
	hrt_lockstep_set_absolute_time(in->time_us);

	// Publish injected topics
	publish_inputs(*rt, *in);

	// Deterministic stepping order
	const uint64_t now = in->time_us;

	// Commander first: it updates vehicle_status/nav_state (and triggers RTL on battery, etc).
	if (rt->commander && rt->cmd_rate.should_run(now)) {
		rt->commander->run_once();
	}

	if (rt->nav_rate.should_run(now)) {
		rt->navigator->run_once();
	}

	if (rt->flight_mode_mgr) {
		rt->flight_mode_mgr->run_once();
	}

	if (rt->pos_rate.should_run(now)) {
		rt->mc_pos->run_once();
	}

	if (rt->att_rate.should_run(now)) {
		rt->mc_att->run_once();
	}

	if (rt->rate_rate.should_run(now)) {
		rt->mc_rate->run_once();
	}

	if (rt->control_alloc && rt->alloc_rate.should_run(now)) {
		rt->control_alloc->run_once();
	}

	// Read outputs
	read_outputs(*rt, *out);
	debug_state(*rt, in->time_us);
	return 0;
}

} // extern "C"
