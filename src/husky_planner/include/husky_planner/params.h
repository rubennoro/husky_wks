#pragma once
#include <cstdint>
#include <vector>

/*
Given Physical Bounds:

Physical properties from MATLAB:
robot.mass = 4.4; %kg

%% Robot world
robot.world.g = 9.81;
robot.world.air_density = 1.3; %kg/m3 

%% Legged
robot.leg.speed = 0.1; %m/s
robot.leg.COT = 4;
robot.leg.climbing_height = 0.25; %m

%% FLying
robot.fly.speed = 2; %m/s
robot.fly.power_consumption = 2400;

%% Transition cost
robot.costs.leg_to_fly = 800;%800; % energy computed form the M4 with a power consumption of 80W and 10s
robot.costs.fly_to_leg = 800;%800;
*/
namespace Params{

    struct{
        float mass{4.4};
    } physical;

    struct{
        float g{9.81};
        float air_density{1.3};
    } world;

    struct{
        float speed{0.1};
        float COT{4};
        float climbing_height{0.25};
    } leg;

    struct{
        float speed{2};
        float power_consumption{2400};
    } fly;

    struct{
        float leg_to_fly{800};
        float fly_to_leg{800};
    } transition;

    struct{
        float number_nodes{1000};
        float number_ground_nodes{500};
        float radius{4};
    } prm;

    struct{
        struct{
            float x{0.5f};
            float y{0.5f};
            float z{0};
        } start;
        struct{
            float x{9.5f};
            float y{2.5f};
            float z{1.0f};
        } goal;
        struct{
            /*
             * Walls from 1 to 3 AND the platform
             */
            const uint32_t num_blocks = 4;
            std::vector<float> x_min{1, 3, 5, 8};
            std::vector<float> x_max{2, 4, 6, 10};
            std::vector<float> y_min{0, 1, 0, 1.5};
            std::vector<float> y_max{4, 5, 5, 3.5};
            std::vector<float> z_min{0, 0, 0, 0};
            std::vector<float> z_max{1, 1, 0.25, 1};
        } blocks;
        struct{
            uint32_t active_platform{1};
            float x_min{8};
            float x_max{10};
            float y_min{1.5};
            float y_max{3.5};
            float z_min{0};
            float z_max{1};
        } platform;
        struct{
            float obstacles{0.3};
            float platform{0.2};
            float landing_area{0.5};
        } clearance;
        struct{
            float x_min{0};
            float x_max{10};
            float y_min{0};
            float y_max{5};
            float z_min{0};
            float z_max{2.5};
        } limits;
    } env;
    /*
    TODO(): Add any constant params here
    */
    struct{
        /* 
         * Must be static constexpr defined at compile-time to be used as a default param for a constructor.
         */
        float res{0.1};
        float kernel_x{0.2};
        float kernel_y{0.2};
    } sampling;
    
}
/*
Environment Specifications:
%% define limits
map.limits.x_min = 0;
map.limits.x_max = 10;
map.limits.y_min = 0;
map.limits.y_max = 5;
map.limits.z_min = 0;
map.limits.z_max = 2.5;


%% Define start and goal

map.start.x = 0.5;
map.start.y = 0.5;
map.start.z = 0;

map.goal.x = 9;
map.goal.y = 2.5;
map.goal.z = 1;

%% Obstacles clearances
map.obstacles_clearance = 0.3;
map.platform_clearance = 0.2;
map.landing_area_clearance = 0.5;

%% Define obstacles

name = "wall_1";

map.obstacles.(name).x_min = 1;
map.obstacles.(name).x_max = 2;
map.obstacles.(name).y_min = 0;
map.obstacles.(name).y_max = 4;
map.obstacles.(name).z_min = 0;
map.obstacles.(name).z_max = 1;

name = "wall_2";

map.obstacles.(name).x_min = 3;
map.obstacles.(name).x_max = 4;
map.obstacles.(name).y_min = 1;
map.obstacles.(name).y_max = 5;
map.obstacles.(name).z_min = 0;
map.obstacles.(name).z_max = 1;


name = "wall_3";

map.obstacles.(name).x_min = 5;
map.obstacles.(name).x_max = 6;
map.obstacles.(name).y_min = 0;
map.obstacles.(name).y_max = 5;
map.obstacles.(name).z_min = 0;
map.obstacles.(name).z_max = 0.25;


%% Save obstalcles names

map.obstacles_names = string(fields(map.obstacles));
%% Define goal platform
map.active_platform = 1;
map.platform.x_min = 8;
map.platform.x_max = 10;
map.platform.y_min = 1.5;
map.platform.y_max = 3.5;
map.platform.z_min = 0;
map.platform.z_max = 1;

%% plot the env.
my_plot(map);

%% clear useless variables
clear name
*/
