#include <iostream>
#include "./astar/a_star.hpp"
#include "./pure_trace/pure_pursuit.hpp"
#include "pgm/pgm.hpp"

int main()
{
    using namespace PGM_SPACE;
    PGM_SPACE::PGM pgm;
    if (pgm.LoadYamlFile("/home/dog/桌面/unitree/unitree_sdk2/path_control/map/2dmap.yaml") == PGM_SUCCESS)
    {
        pgm.setgray_type(0xFE, 0xCD, 0x00);
    }
    else
    {
        return 0;
    }
    pgm_header mypgmheader = pgm.getpgm_header();

    pgm.PGM_handle(0x00, 0xFF, 0.5, 0.2, 0xAA);

    pgm.SavePGM("../expansion", pgm.getPdata(), mypgmheader.width, mypgmheader.height, mypgmheader.max_gray);

    // set robot axis original coordination
    ASTAR::coordination origin;
    origin.x = 0;
    origin.y = mypgmheader.height - 1;
    ASTAR::astar myastar(pgm.getMap(), mypgmheader.width, mypgmheader.height, 0xFF, origin, ASTAR::CALLER_AXIS::X_LEFT_Y_DOWN, pgm.getResolution());
    std::vector<ASTAR::f_coordination> path_vector;
    // ASTAR::coordination source = {.x = 127, .y = 70};
    // ASTAR::coordination dest = {.x = 235, .y = 160};
    // myastar.find_path(path_vector, source, dest);
    // ASTAR::coordination obs = {.x = 140, .y = 45};
    // myastar.addObstacle(-7.85, -9.55, 0.1, 2.0, 0.2);
    // pgm.SavePGM("../astarobs", myastar.getAstarMap(), mypgmheader.width, mypgmheader.height, mypgmheader.max_gray);
    // if (myastar.find_path(path_vector, -7.60, -9.55, -41.85, -13, 1) == ASTAR::ASTAR_STATE::ASTAR_SUCCESS)
    // {
    //     // for(std::vector<ASTAR::f_coordination>::iterator it = path_vector.begin(); it < path_vector.end(); it++)
    //     // {
    //     //     std::cout << " x = " << (*it).x << " y = " << (*it).y << std::endl;
    //     // }
    //     myastar.compressPath(path_vector);
    //     // for (std::vector<ASTAR::f_coordination>::iterator it = path_vector.begin(); it < path_vector.end(); it++)
    //     // {
    //     //     std::cout << "compress : x = " << (*it).x << " y = " << (*it).y << std::endl;
    //     // }
    //     PUREPURSUIT::PurePursuitNode mypurepursuit(0.1, 0.2, 0);
    //     std::vector<PUREPURSUIT::path_type> pure_path = mypurepursuit.CreatePath(path_vector);
    //     std::cout << "PurePath\n";
    //     // for (std::vector<PUREPURSUIT::path_type>::iterator it = pure_path.begin(); it < pure_path.end(); it++)
    //     // {
    //     //     std::cout << "PurePath : x = " << (*it).x << " y = " << (*it).y << std::endl;
    //     // }
    //     PUREPURSUIT::purePursuitCoordination now_coord;
    //     now_coord.x = -7.3;
    //     now_coord.y = -3.1;
    //     now_coord.yaw = 2;
    //     PUREPURSUIT::pursuit_result res = mypurepursuit.PurePursuitRun(pure_path, now_coord);

    //     std::cout << "pure = x " << res.speed_x << " angle " << res.yaw_angle << std::endl;

    //     std::vector<ASTAR::coordination> inte_path_vector = myastar.coordination_trans_to_astar(path_vector);
    //     myastar.printPath(0x0, inte_path_vector);
    //     pgm.SavePGM("../astar", myastar.getAstarMap(), mypgmheader.width, mypgmheader.height, mypgmheader.max_gray);
    //     myastar.removeObstacle(-6.5, -0.55, 0.1, 2.0, 0.2);
    //     // pgm.SavePGM("../astarremove", myastar.getAstarMap(), mypgmheader.width, mypgmheader.height, mypgmheader.max_gray);
    // }
    // else{
    //     std::cout << "astar path not found\n";
    // }
    ASTAR::f_coordination xs = {.x = -10, .y = -30};
    ASTAR::f_coordination xe = {.x = -10, .y = -10};
    ASTAR::f_coordination ys = {.x = -2, .y = -20};
    ASTAR::f_coordination ye = {.x = -30, .y = -20};
    // ASTAR::f_coordination startLine = {.x = 0, .y = -2};
    // ASTAR::f_coordination endLine = {.x = -1, .y = -2};
    ASTAR::f_coordination startLine1 = {.x = -6.55013, .y = -11.7879};
    ASTAR::f_coordination endLine1 = {.x = -7.01999, .y = -11.4862};
    // ASTAR::f_coordination startLine2 = {.x = -4, .y = -2};
    // ASTAR::f_coordination endLine2 = {.x = -4, .y = -1};
    myastar.drawLine(xs, xe);
    myastar.drawLine(ys, ye);
    // myastar.drawLine(startLine, endLine);
    // myastar.drawLine(startLine1, endLine1);
    // myastar.drawRectangle(startLine1, endLine1, startLine2, endLine2);
    myastar.drawRectangle(startLine1, endLine1, 2, 1);
    pgm.SavePGM("../astarrectangle", myastar.getAstarMap(), mypgmheader.width, mypgmheader.height, mypgmheader.max_gray);
}