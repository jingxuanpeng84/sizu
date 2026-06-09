#include "main_control.hpp"

OCTI_MAIN_CONTROL::maincontrol::maincontrol(ASTAR::f_coordination robot_origin_, float laddar_vision_distance_)
{
    this->robot_origin = robot_origin_;
    this->laddar_vision_distance = laddar_vision_distance_;
}

ASTAR::f_coordination OCTI_MAIN_CONTROL::maincontrol::transToRobotCoordination(ASTAR::f_coordination map_coordination_)
{
    ASTAR::f_coordination temp;
    temp.x = (-map_coordination_.x) + this->robot_origin.x;
    temp.y = -map_coordination_.y + this->robot_origin.y;
    return temp;
}

void OCTI_MAIN_CONTROL::maincontrol::transToRobotCoordination(std::vector<ASTAR::f_coordination> *map_coordination_)
{
    ASTAR::f_coordination temp;
    for (std::vector<ASTAR::f_coordination>::iterator it = map_coordination_->begin(); it < map_coordination_->end(); it++)
    {
        temp = this->transToRobotCoordination(*it);
        *it = temp;
    }
}

ASTAR::f_coordination OCTI_MAIN_CONTROL::maincontrol::transToMapCoordination(ASTAR::f_coordination robot_coordination_)
{
    ASTAR::f_coordination temp;
    temp.x = (-robot_coordination_.x) + this->robot_origin.x;
    temp.y = -robot_coordination_.y + this->robot_origin.y;
    return temp;
}

ASTAR::f_coordination OCTI_MAIN_CONTROL::maincontrol::computerCoordination(PUREPURSUIT::purePursuitCoordination dog_coord_, float depth_, float width_, float expansion_, POINT_DIRCTION direction_)
{
    ASTAR::f_coordination res;
    depth_ = depth_ - expansion_; // only front expansion
    float angle = atan(width_ / depth_);
    if (direction_ == POINT_DIRCTION::RIGHT)
    {
        // right
        res.x = dog_coord_.x + sqrt(depth_ * depth_ + width_ * width_) * cos(dog_coord_.yaw + angle);
        res.y = dog_coord_.y - sqrt(depth_ * depth_ + width_ * width_) * sin(dog_coord_.yaw + angle);
    }
    else
    {
        // left
        res.x = dog_coord_.x + sqrt(depth_ * depth_ + width_ * width_) * cos(dog_coord_.yaw - angle);
        res.y = dog_coord_.y - sqrt(depth_ * depth_ + width_ * width_) * sin(dog_coord_.yaw - angle);
    }
    return res;
}

std::vector<ASTAR::f_coordination> OCTI_MAIN_CONTROL::maincontrol::transToMapCoordination(std::vector<PUREPURSUIT::path_type> pathVector_)
{
    std::vector<ASTAR::f_coordination> tempVector;
    ASTAR::f_coordination tempNode;
    for (std::vector<PUREPURSUIT::path_type>::iterator it = pathVector_.begin(); it < pathVector_.end(); it++)
    {
        tempNode.x = it->x;
        tempNode.y = it->y;
        tempVector.push_back(this->transToMapCoordination(tempNode));
    }
    return tempVector;
}

void OCTI_MAIN_CONTROL::maincontrol::drawRobotcoordnaition(ASTAR::f_coordination robotCoordonation_, ASTAR::astar *myastar)
{
    ASTAR::f_coordination xs;
    ASTAR::f_coordination xe;
    ASTAR::f_coordination ys;
    ASTAR::f_coordination ye;
    xs = {.x = robotCoordonation_.x, .y = robotCoordonation_.y + 4};
    xe = {.x = robotCoordonation_.x, .y = robotCoordonation_.y - 4};
    ys = {.x = robotCoordonation_.x + 4, .y = robotCoordonation_.y};
    ye = {.x = robotCoordonation_.x - 4, .y = robotCoordonation_.y};
    xs = this->transToMapCoordination(xs);
    xe = this->transToMapCoordination(xe);
    ys = this->transToMapCoordination(ys);
    ye = this->transToMapCoordination(ye);
    myastar->drawLine(xs, xe);
    myastar->drawLine(ys, ye);
}

void OCTI_MAIN_CONTROL::maincontrol::set_charge_low_thresh(char thresh_)
{
}

void task2(OCTI_LADDER::octiLadder *ladder, OCTI_VISION::octiVision *vision)
{
    // Init---------------------
    // create motion_control
    OCTIROBOT_ChaannelFactory_Instance_init(0, "enP4p65s0");
    OCTIROBOT::Robot myrobot(10.0f, 430000);

    // load map
    PGM_SPACE::PGM pgm;
    if (pgm.LoadYamlFile("/home/dog/桌面/unitree/OctiRobot/path_control/map/2dmap.yaml") == PGM_SPACE::PGM_SUCCESS)
    {
        pgm.setgray_type(0xFE, 0xCD, 0x00);
    }
    else
    {
        perror("map load fail");
        exit(0);
    }
    pgm.PGM_handle(0x00, 0xFF, 0.5, 0.2, 0xAA);
    PGM_SPACE::pgm_header mypgmheader = pgm.getpgm_header();

    //
    PGM_SPACE::coord ori_coord = pgm.getOrigin();
    ASTAR::f_coordination origin_coordination;
    origin_coordination.x = ori_coord.x;
    origin_coordination.y = ori_coord.y;
    OCTI_MAIN_CONTROL::maincontrol mymaincontrol(origin_coordination, 0.1);

    // A star init
    ASTAR::coordination map_origin;
    map_origin.x = 0;
    map_origin.y = mypgmheader.height - 1;
    ASTAR::astar myastar(pgm.getMap(), mypgmheader.width, mypgmheader.height, 0xFF, map_origin, ASTAR::CALLER_AXIS::X_LEFT_Y_DOWN, pgm.getResolution());

    // read path
    OCTI_XMLRW::octi_xml myxml("/home/dog/桌面/unitree/OctiRobot/path_control/path.xml");
    std::vector<ASTAR::f_coordination> path_vector = myxml.xml_read_path();
    if (path_vector.empty())
    {
        perror("path is empty");
        exit(0);
    }

    // speaker
    // OCTI_SPEAKER::speaker myspeaker("192.168.124.110", 8001, "192.168.124.110", 6004);
    // std::thread spTask(OCTI_SPEAKER::speakerThread, &myspeaker);
    // pure pursuit
    PUREPURSUIT::PurePursuitNode mypurepursuit(0.1, 0.2, 0);
    std::vector<PUREPURSUIT::path_type> pure_path = mypurepursuit.CreatePath(path_vector);
    PUREPURSUIT::purePursuitCoordination now_coord;
    // for(std::vector<PUREPURSUIT::path_type>::iterator it = pure_path.begin(); it < pure_path.end(); it++)
    // {
    //     std::cout << "x = " << it->x << " | y = " << it->y << std::endl;
    // }
    //
    std::stack<OCTI_MAIN_CONTROL::path_vector_stack_struct> path_vector_stack;
    OCTI_MAIN_CONTROL::path_vector_stack_struct temp_p_s;
    temp_p_s.path_vector_point = &pure_path;
    temp_p_s.pre_vector_start_index = mypurepursuit.getIndexTrace();
    path_vector_stack.push(temp_p_s);
    std::vector<PUREPURSUIT::path_type> *current_path_vector_point = path_vector_stack.top().path_vector_point;
    mypurepursuit.setPathType(PUREPURSUIT::PATHTYPE::OVERALLPATH);

    OCTI_XMLRW::octi_xml myxmlCoord("/home/dog/桌面/unitree/OctiRobot/path_control/coordinationTrace.xml");
    OCTI_XMLRW::coordiantion_trace_struct coor;
    coor = myxmlCoord.xml_read_coordinationTrace();
    std::cout << "index = " << coor.index << " x = " << coor.x << " y = " << coor.y << std::endl;
    mypurepursuit.setIndexTrace(coor.index);
    std::thread th_coord(OCTI_XMLRW::xml_record_coord_thread, &myxmlCoord);
    // coor.index = 11;
    // myxmlCoord.xml_write_coordinationTrace(coor);
    std::vector<ASTAR::coordination> temp_path = myastar.coordination_trans_to_astar(mymaincontrol.transToMapCoordination(pure_path));
    std::cout << "path size = " << temp_path.size() << std::endl;
    // for(std::vector<ASTAR::coordination>::iterator it = temp_path.begin(); it < temp_path.end(); it++)
    // {
    //     std::cout << "x = " << it->x << " | y = " << it->y <<std::endl;
    // }
    // myastar.printPath(0xAA, temp_path);
    pgm.SavePGM("../astarrectangleobs", myastar.getAstarMap(), mypgmheader.width, mypgmheader.height, mypgmheader.max_gray);

    myrobot.Octi_StandUp();
    myrobot.Octi_BalanceStand();

    ASTAR::f_coordination obs_map_l;
    ASTAR::f_coordination obs_map_r;

    while (1)
    {
        // handle ladder data
        OCTI_LADDER::ladder_return_data data_ladder = ladder->getLadderdata();
        while (data_ladder.flag != OCTI_LADDER::DATA_UPDATE::DATA)
        {
            // std::cout << "no laddar data \n";
            data_ladder = ladder->getLadderdata();
        }
        now_coord.x = data_ladder.x;
        now_coord.y = data_ladder.y;
        now_coord.yaw = data_ladder.yaw_angle; // left yaw positive, right yaw nagtive

        std::cout << "coor = " << now_coord.x << " | y = " << now_coord.y << " angle = " << now_coord.yaw << std::endl;
        float assumeDepth_ = 0.5 + 0.25;

        // handle vision data
        OCTI_VISION::vision_return_data data_vision = vision->getVisondata();
        while (data_vision.flag != OCTI_VISION::DATA_UPDATE::DATA)
        {
            // std::cout << "no vision data \n";
            data_vision = vision->getVisondata();
        }
        if (data_vision.distance <= OCTI_MAIN_CONTROL::EMERGENCY_STOP_DISTANCE && data_vision.distance > OCTI_MAIN_CONTROL::EMERGENCY_MIN_STOP_DISTANCE)
        {
            mypurepursuit.setSpeed_X(0.12);
            // emergency stop
            mymaincontrol.obs_meet_count = 0;
            if (myrobot.Octi_Is_Emergency_Stop() == false)
            {
                myrobot.Octi_Emergency_Stop();
            }
#ifdef SPEAKER_BUG
            myspeaker.addMessage("[b0]紧急停止[d]");
#endif
        }
        else if (data_vision.distance <= OCTI_MAIN_CONTROL::NONE_ACTION_DISTANCE && data_vision.distance > OCTI_MAIN_CONTROL::EMERGENCY_STOP_DISTANCE)
        {
            // slow down
            mypurepursuit.setSpeed_X(0.12);
            mymaincontrol.obs_meet_count = 0;
            if (myrobot.Octi_Is_Emergency_Stop() == true)
            {
                myrobot.Octi_Emergency_Recovery_Move();
            }
        }
        else if (0 && data_vision.distance <= OCTI_MAIN_CONTROL::OBSTACLE_AVOID_DISTANCE && data_vision.distance > OCTI_MAIN_CONTROL::NONE_ACTION_DISTANCE)
        {
            // slow down
            mypurepursuit.setSpeed_X(0.12);
            // avoid obs judge
            if (data_vision.flag_obs == 1)
            {
                //
                mymaincontrol.obs_meet_count++;
            }
            if (data_vision.flag_obs == 1 && mymaincontrol.obs_meet_count >= 2)
            {
                mymaincontrol.obs_meet_count = 0;
                if (data_vision.width > OCTI_MAIN_CONTROL::MAX_WIDTH)
                {
                    // call speaker and stop
                    if (myrobot.Octi_Is_Emergency_Stop() == false)
                    {
                        myrobot.Octi_Emergency_Stop();
                    }
#ifdef SPEAKER_BUG
                    myspeaker.addMessage("[b0]障碍物太宽，无法避开[d]");
#endif
                }
                else
                {
                    // enter A star
                    std::vector<ASTAR::f_coordination> astar_path_vector;
                    // the dog axis need to transform to the map axis
                    ASTAR::f_coordination obs_coord_l;
                    ASTAR::f_coordination obs_coord_r;
                    /*
                    need
                    */
                    // std::cout << " now x = " << now_coord.x << " y = " << now_coord.y << " angle = " << now_coord.yaw << std::endl;
                    // compute obs coordination and add obs to map
                    obs_coord_l = mymaincontrol.computerCoordination(now_coord, data_vision.distance + mymaincontrol.laddar_vision_distance, abs(data_vision.left_width + OCTI_MAIN_CONTROL::obs_width_expansion), 0.25, (data_vision.left_width + OCTI_MAIN_CONTROL::obs_width_expansion) >= 0 ? OCTI_MAIN_CONTROL::POINT_DIRCTION::LEFT : OCTI_MAIN_CONTROL::POINT_DIRCTION::RIGHT);
                    obs_coord_r = mymaincontrol.computerCoordination(now_coord, data_vision.distance + mymaincontrol.laddar_vision_distance, abs(data_vision.right_width + OCTI_MAIN_CONTROL::obs_width_expansion), 0.25, (data_vision.right_width + OCTI_MAIN_CONTROL::obs_width_expansion) >= 0 ? OCTI_MAIN_CONTROL::POINT_DIRCTION::RIGHT : OCTI_MAIN_CONTROL::POINT_DIRCTION::LEFT);
                    // std::cout << "coord x = " << obs_coord_l.x << " y = " << obs_coord_l.y << " rx = " << obs_coord_r.x << " ry = " << obs_coord_r.y << std::endl;
                    obs_map_l = mymaincontrol.transToMapCoordination(obs_coord_l); // make a transformation of coordination to Map axis
                    obs_map_r = mymaincontrol.transToMapCoordination(obs_coord_r);
                    // std::cout << "coord x = " << obs_map_l.x << " y = " << obs_map_l.y << " rx = " << obs_map_r.x << " ry = " << obs_map_r.y << std::endl;

                    myastar.addObstacle(obs_map_l, obs_map_r, assumeDepth_, -now_coord.yaw); // need to make a transformation of coordination to Map axis

                    // test --------------------------------------
                    pgm.SavePGM("../astarrectangleobs", myastar.getAstarMap(), mypgmheader.width, mypgmheader.height, mypgmheader.max_gray);
                    // test --------------------------------------

                    // need find destiantion
                    // can reversal the path in the map .if(not in obstacle) .then is the dest index
                    // reversal path index from now index
                    unsigned int tempIndex = mypurepursuit.getIndexTrace();
                    ASTAR::ASTAR_STATE state = myastar.getDestIndexOfPath(mymaincontrol.transToMapCoordination(*current_path_vector_point), tempIndex, current_path_vector_point->size());
                    // according state to judge
                    if (state == ASTAR::ASTAR_SUCCESS && tempIndex == mypurepursuit.getIndexTrace())
                    {
                        // 1-
                        //  obs is not in the path
                        //  no need to avoid obs
                        //  remove obs
                        myastar.removeObstacle(obs_map_l, obs_map_r, assumeDepth_, -now_coord.yaw);
                        std::cout << "obs is not in the path\n";
                    }
                    else if ((state == ASTAR::ASTAR_SUCCESS && (tempIndex != mypurepursuit.getIndexTrace())) || state == ASTAR::ASTAR_ERROR)
                    {
                        // 2-
                        //  find dest index
                        //  set index
                        //  avoid obs
                        //  if tempIndex < mypurepursuit.getIndexTrace() and path vector size > 1 .then also pop path vector stack and change current path vector and set current path index and find next path vector
                        if ((tempIndex < mypurepursuit.getIndexTrace() && path_vector_stack.size() > 1) || state == ASTAR::ASTAR_ERROR)
                        {
                            while (path_vector_stack.size() > 1)
                            {

                                tempIndex = path_vector_stack.top().pre_vector_start_index;
                                unsigned int preIndex_ = tempIndex;
                                delete path_vector_stack.top().path_vector_point;
                                path_vector_stack.pop();
                                std::cout << "find path dest pop\n";
                                current_path_vector_point = path_vector_stack.top().path_vector_point;
                                ASTAR::ASTAR_STATE state_t = myastar.getDestIndexOfPath(mymaincontrol.transToMapCoordination(*current_path_vector_point), tempIndex, current_path_vector_point->size());
                                if ((state_t == ASTAR::ASTAR_SUCCESS) && tempIndex >= preIndex_ && path_vector_stack.size() > 1)
                                {
                                    // shoud change in the moment of a star success?
                                    break;
                                }
                            }
                        }
                        // reversal path vector if in this vector not found correct destination then pop this vector and reversal next vector
                        // use stack
                        ASTAR::f_coordination now_coord_astar, now_coord_astar_dest;
                        now_coord_astar.x = now_coord.x;
                        now_coord_astar.y = now_coord.y;
                        now_coord_astar_dest.x = (*current_path_vector_point)[tempIndex].x;
                        now_coord_astar_dest.y = (*current_path_vector_point)[tempIndex].y;
                        now_coord_astar = mymaincontrol.transToMapCoordination(now_coord_astar);
                        now_coord_astar_dest = mymaincontrol.transToMapCoordination(now_coord_astar_dest);
                        if (myastar.find_path(astar_path_vector, now_coord_astar.x, now_coord_astar.y, now_coord_astar_dest.x, now_coord_astar_dest.y, 1) == ASTAR::ASTAR_STATE::ASTAR_SUCCESS)
                        {
                            std::vector<ASTAR::coordination> astarpath = myastar.coordination_trans_to_astar(astar_path_vector);

                            pgm.SavePGM("../astarrectangleobs", myastar.getAstarMap(), mypgmheader.width, mypgmheader.height, mypgmheader.max_gray);
                            myastar.compressPath(astar_path_vector); // need to transform to dog axis
                            // trans map axis to dog axis
                            mymaincontrol.transToRobotCoordination(&astar_path_vector);
                            // remove obs in the map
                            myastar.removeObstacle(obs_map_l, obs_map_r, assumeDepth_, -now_coord.yaw);
                            myastar.printPath(0xFF, astarpath);
                            auto pure_path = new std::vector<PUREPURSUIT::path_type>;
                            *pure_path = mypurepursuit.CreatePath(astar_path_vector);
                            temp_p_s.path_vector_point = pure_path;
                            temp_p_s.pre_vector_start_index = tempIndex; // tempIndex is pre path start index
                            path_vector_stack.push(temp_p_s);
                            // test----------
                            std::cout << " path stack size = " << path_vector_stack.size() << "current path size = " << astar_path_vector.size() << std::endl;
                            // test----------
                            current_path_vector_point = pure_path;
                            mypurepursuit.setPathType(PUREPURSUIT::PATHTYPE::ASTARPATH);
                            mypurepursuit.setIndexTrace(0);

                            // if a star success and emergency .then recovery move
                            if (myrobot.Octi_Is_Emergency_Stop() == true)
                            {
                                myrobot.Octi_Emergency_Recovery_Move();
                            }
                        }
                        else
                        {
                            // avoid obs fail
                            // call speaker
                            if (myrobot.Octi_Is_Emergency_Stop() == false)
                            {
                                myrobot.Octi_Emergency_Stop();
                            }
#ifdef SPEAKER_BUG
                            myspeaker.addMessage("[b0]前方物体太近,无法避开[d]");
#endif
                            // do need to while check distance of obs?
                        }
                    }
                    else
                    {
                        // not find dest need pop the top of path vector stack and change current path vector and set current path index and find next path vector
                        //  avoid obs fail
                        //  call speaker
                        myastar.removeObstacle(obs_map_l, obs_map_r, assumeDepth_, -now_coord.yaw);
                        if (myrobot.Octi_Is_Emergency_Stop() == false)
                        {
                            myrobot.Octi_Emergency_Stop();
                        }
#ifdef SPEAKER_BUG
                        myspeaker.addMessage("[b0]无法避开[d]");
#endif
                    }
                }
            }
        }
        else
        {
            mymaincontrol.obs_meet_count = 0;
            if (myrobot.Octi_Is_Emergency_Stop() == true)
            {
                myrobot.Octi_Emergency_Recovery_Move();
            }
            mypurepursuit.setSpeed_X(0.2);
        }

        // pure pursuit
        PUREPURSUIT::pursuit_result res = mypurepursuit.PurePursuitRun(*current_path_vector_point, now_coord);
        std::cout << "pure angle = " << res.yaw_angle << std::endl;
        while (mypurepursuit.getIndexTrace() == current_path_vector_point->size())
        {
            // change path vector
            mypurepursuit.setIndexTrace(path_vector_stack.top().pre_vector_start_index);
            std::cout << "path stack size = " << path_vector_stack.size() << std::endl;
            delete (path_vector_stack.top().path_vector_point);
            path_vector_stack.pop();
            std::cout << "pop path\n";
            current_path_vector_point = path_vector_stack.top().path_vector_point;
            if (path_vector_stack.size() > 1)
            {
                mypurepursuit.setPathType(PUREPURSUIT::PATHTYPE::ASTARPATH);
            }
            else
            {
                mypurepursuit.setPathType(PUREPURSUIT::PATHTYPE::OVERALLPATH);
            }
            res = mypurepursuit.PurePursuitRun(*current_path_vector_point, now_coord);
            std::cout << "pure angle = " << res.yaw_angle << std::endl;
        }

        // check left and right turn around whether exist obs -----------------------
        if ((res.yaw_angle > 0 && data_ladder.left_obs_dist <= OCTI_MAIN_CONTROL::left_obs_thresh) || (res.yaw_angle < 0 && data_ladder.right_obs_dist <= OCTI_MAIN_CONTROL::right_obs_thresh))
        {
            // left yaw > 0; right yaw < 0
            if (myrobot.Octi_Is_Emergency_Stop() == false)
            {
                myrobot.Octi_Emergency_Stop();
            }
        }
        else
        {
            // end check -------------------------
            myrobot.Octi_Move(res.speed_x, res.speed_y, res.yaw_angle);
        }

        // save overall path current index and coordination
        if (path_vector_stack.size() == 1)
        {
            coor.index = mypurepursuit.getIndexTrace();
        }
        coor.x = data_ladder.x;
        coor.y = data_ladder.y;
        myxmlCoord.xml_write_coordinationTrace(coor);
    }
    th_coord.join();
#ifdef SPEAKER_BUG
    spTask.join();
#endif
}

int main()
{
    OCTI_LADDER::octiLadder ladder("192.168.124.120", 6001, "192.168.124.110", 6002);
    OCTI_VISION::octiVision vision("192.168.124.110", 6000, "192.168.124.110", 6001);
    std::thread ladder_thread_t(OCTI_LADDER::ladder_thread, &ladder);
    std::thread vision_thread_t(OCTI_VISION::vision_thread, &vision);
    std::thread th2(task2, &ladder, &vision);
    th2.join();
    ladder_thread_t.join();
    vision_thread_t.join();
}