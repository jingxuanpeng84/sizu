#ifndef MAIN_CONTROL__HPP
#define MAIN_CONTROL__HPP
#include "../lad/lad.hpp"
#include "../vision/vision.hpp"
#include "../pure_trace/pure_pursuit.hpp"
#include "../octirobot/octirobot.hpp"
#include "../astar/a_star.hpp"
#include "../xml_rw/xml_rw.hpp"
#include "../pgm/pgm.hpp"
#include "../speaker/speaker.hpp"
#include <math.h>
#include <stack>
namespace OCTI_MAIN_CONTROL
{
    enum OCTI_ROBOT_WORK_STATE
    {
        CHARGING,
        INSPECTING,
        WAITING_CHARGED,
    };
    struct robot_coordination
    {
        float x;
        float y;
    };

    struct path_vector_stack_struct
    {
        std::vector<PUREPURSUIT::path_type> *path_vector_point;
        unsigned pre_vector_start_index; // record pre path vector now index
    };

    float EMERGENCY_MIN_STOP_DISTANCE = 0.02;
    float EMERGENCY_STOP_DISTANCE = 0.35;
    float NONE_ACTION_DISTANCE = 0.50;
    float OBSTACLE_AVOID_DISTANCE = 0.65;

    float MAX_WIDTH = 1;    //max obs width which can avoid
    float obs_width_expansion = 0.30;   //
    float left_obs_thresh = 0.35;
    float right_obs_thresh = 0.35;

    enum POINT_DIRCTION
    {
        LEFT,
        RIGHT,
    };

    class maincontrol
    {
    public:
        maincontrol(ASTAR::f_coordination robot_origin_, float laddar_vision_distance_);
        ASTAR::f_coordination transToRobotCoordination(ASTAR::f_coordination map_coordination_);
        void transToRobotCoordination(std::vector<ASTAR::f_coordination> *map_coordination_);

        ASTAR::f_coordination transToMapCoordination(ASTAR::f_coordination robot_coordination_);
        std::vector<ASTAR::f_coordination> transToMapCoordination(std::vector<PUREPURSUIT::path_type> pathVector_);
        ASTAR::f_coordination transObsCoordination();

        // expansion : only front expansion
        ASTAR::f_coordination computerCoordination(PUREPURSUIT::purePursuitCoordination dog_coord_, float depth_, float width_, float expansion_, POINT_DIRCTION direction_);

        void drawRobotcoordnaition(ASTAR::f_coordination robotCoordonation_, ASTAR::astar *myastar);

        float laddar_vision_distance = 0.2;

        int obs_meet_count = 0;
        char get_charge_low_thresh();
        char get_charge_high_thresh();
        void set_charge_low_thresh(char thresh_);
        void set_charge_high_thresh(char thresh_);

    private:
        ASTAR::f_coordination robot_origin;
        // vision in the front of laddar
        char charge_low_thresh;
        char charge_high_thresh;

        enum OCTI_ROBOT_WORK_STATE Robotstate;
    };
};

#endif