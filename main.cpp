#include "vec3.h"
#include "actor.h"

#include <Windows.h>
#include "detours.h"

#include <fstream>
#include <string>

namespace cfg {
	float tickrate = 128.F;

	float slope_gravity = 10.F;
	float stop_speed = 2.6F;
	float min_speed_scale = 2.9F;

	float kinetic_friction = 6.F;
	float static_friction = 3.F;
	float static_threshold = .5F;

	float walk_accel = 7.F;
	float air_accel = 5.F;
	float air_max_speed = 3.F;
};

const float *frame_delta = (float*)(0x1B4ADE0);
bool enable_physics = true;
float tick_timer = 0.f;
float tick_time;
vec3 velocity;

/*
 * Perform friction and sliding when on the ground or in water
 */
void friction(actor::physics_data *phys_data, const bool on_ground, const bool in_water)
{
	const auto ground_normal = phys_data->ground_normal();
	if (!in_water) {
		auto accel = ground_normal * cfg::slope_gravity;
		accel.z = 0.f;
		if (velocity.length() > cfg::static_threshold || accel.length() > ground_normal.z * cfg::static_friction)
			velocity += accel * tick_time; //slope sliding
	}

	const auto speed = velocity.length();
	auto friction = max(cfg::stop_speed, speed) * cfg::kinetic_friction * tick_time;
	if (!in_water)
		friction *= ground_normal.z;

	if (speed > friction)
		velocity *= 1.F - friction / speed;
	else
		velocity = vec3();
}

/*
 * Simulate a single physics tick
 */
void physics_tick(const vec3 &input_world, actor::physics_data *phys_data,
	const float max_speed, const bool on_ground, const bool in_water)
{
	if (on_ground)
		friction(phys_data, on_ground, in_water);
	else //falling speed will get projected to 2d velocity on landing
		velocity.z = phys_data->velocity().z;

	const auto ground_normal = phys_data->ground_normal();
	//project velocity onto surface
	if (on_ground && !in_water)
		velocity += ground_normal * -velocity.z * ground_normal.z;

	if (input_world.length() == 0.F)
		return;

	const auto input_dir = input_world.normal();
	const auto target_vel = input_dir * max_speed;
	const auto delta_vel = target_vel - velocity;
	const auto move_dir = delta_vel.normal();
	const auto cur_speed = vec3::dot(velocity, move_dir);

	//don't allow accelerating over max speed in that direction.
	//if the pc is going over the limit in one direction they'll
	//still be able to move in other directions though.
	if (cur_speed < max_speed) {
		const auto speed_scale = max(max_speed, cfg::min_speed_scale);
		const auto accel =
			in_water ? cfg::walk_accel * speed_scale :
			on_ground ? cfg::walk_accel * speed_scale * ground_normal.z :
			cfg::air_accel;

		velocity += move_dir * min(max_speed - cur_speed, accel * tick_time);
	}
}

struct move_params {
	float delta_time;
private:
	char padding[0xC];
public:
	vec3 input;
	float max_speed;
};

using move_t = void(__thiscall*)(actor::physics_data*, move_params*);
move_t orig_move;

/*
 * Override the original Skyrim physics
 */
void __fastcall hook_move(actor::physics_data *phys_data, const void *edx, move_params *params)
{
	if (!enable_physics)
		return orig_move(phys_data, params);

	auto *player = actor::player();
	if (player == nullptr || player->phys_data() != phys_data)
		return orig_move(phys_data, params);

	const auto on_ground = phys_data->fall_time() == 0.F;
	const auto in_water = on_ground && phys_data->ground_entity() == nullptr;
	//params->max_speed is in cm/s, we want m/s
	const auto max_speed = (on_ground ? params->input.length() : min(params->input.length(), cfg::air_max_speed));

	//transform from actor local space to world space
	vec3 input_world;
	input_world.x = params->input.x *  cosf(player->yaw()) + params->input.y * sinf(player->yaw());
	input_world.y = params->input.x * -sinf(player->yaw()) + params->input.y * cosf(player->yaw());
	input_world.z = params->input.z;

	tick_timer += *frame_delta;
	while (tick_timer >= tick_time) {
		physics_tick(input_world, phys_data, max_speed, on_ground, in_water);
		tick_timer -= tick_time;
	}

	//transform velocity back to local space to be fed into input
	params->input.x = velocity.x *  cosf(-player->yaw()) + velocity.y * sinf(-player->yaw());
	params->input.y = velocity.x * -sinf(-player->yaw()) + velocity.y * cosf(-player->yaw());
	params->input.z = in_water ? velocity.z : 0.f;

	return orig_move(phys_data, params);
}

using change_cam_t = void(__thiscall*)(uintptr_t, uintptr_t);
change_cam_t orig_change_cam;
/*
 * Disable physics during the "VATS" (killcam) camera
 */
void __fastcall hook_change_cam(uintptr_t camera, const void *edx, uintptr_t new_state)
{
	const auto cam_id = *(int*)(new_state + 0xC);

	enable_physics = cam_id != 2;
	orig_change_cam(camera, new_state);
}

using load_game_t = bool(__thiscall*)(void*, void*, bool);
load_game_t orig_load_game;
/*
 * Zero out velocity when loading
 */
void __fastcall hook_load_game(void *thisptr, const void *edx, void *a1, bool a2)
{
	velocity = vec3();
	orig_load_game(thisptr, a1, a2);
}

struct PluginInfo {
	unsigned int infoVersion;
	const char *name;
	unsigned int version;
};

extern "C" __declspec(dllexport) bool SKSEPlugin_Query(void *skse, PluginInfo *info)
{
	info->infoVersion = 3;
	info->version = 3;
	info->name = "Player Physics";

	return true;
}

/*
 * Read the config file
 */
void read_cfg()
{
	std::ifstream config("Data/SKSE/Plugins/skyrim_physics.cfg");
	if (config.fail())
		return;

	while (!config.eof()) {
		std::string line;
		std::getline(config, line);

		char key[32];
		float value;
		if (sscanf_s(line.c_str(), "%s %f", key, 32, &value) != 2)
			continue;

		if (strncmp(key, "//", 2) == 0)
			continue;

		if (strcmp(key, "tickrate") == 0)
			cfg::tickrate = value;
		else if (strcmp(key, "slope_gravity") == 0)
			cfg::slope_gravity = value;
		else if (strcmp(key, "stop_speed") == 0)
			cfg::stop_speed = value;
		else if (strcmp(key, "min_speed_scale") == 0)
			cfg::min_speed_scale = value;
		else if (strcmp(key, "kinetic_friction") == 0)
			cfg::kinetic_friction = value;
		else if (strcmp(key, "static_friction") == 0)
			cfg::static_friction = value;
		else if (strcmp(key, "static_threshold") == 0)
			cfg::static_threshold = value;
		else if (strcmp(key, "walk_accel") == 0)
			cfg::walk_accel = value;
		else if (strcmp(key, "air_accel") == 0)
			cfg::air_accel = value;
		else if (strcmp(key, "air_max_speed") == 0)
			cfg::air_max_speed = value;
	}
}

/*
 * Detour move, camera state change and loading functions
 */
extern "C" __declspec(dllexport) bool SKSEPlugin_Load(void *skse)
{
	read_cfg();
	tick_time = 1.f / cfg::tickrate;
	orig_move = (move_t)(DetourFunction((byte*)(0xD1DA60), (byte*)(hook_move)));
	orig_change_cam = (change_cam_t)(DetourFunction((byte*)(0x6533D0), (byte*)(hook_change_cam)));
	orig_load_game = (load_game_t)(DetourFunction((byte*)(0x67B720), (byte*)(hook_load_game)));

	return true;
}