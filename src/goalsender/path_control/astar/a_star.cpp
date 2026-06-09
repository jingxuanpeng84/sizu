#include "./a_star.hpp"
#include <algorithm>

// namespace ASTAR
bool ASTAR::coordination::operator==(coordination coordination_)
{
    return (this->x == coordination_.x && this->y == coordination_.y);
}

unsigned int ASTAR::Node::getScore()
{
    return this->F_cost + this->G_cost;
}

ASTAR::FLAG_STATE ASTAR::astar::getFlag_state(coordination coord_)
{
    if (coord_.x >= 0 && coord_.y >= 0)
    {
        return flag_table[coord_.y * this->width + coord_.x] == ASTAR_NO_TRAVERED ? ASTAR_NO_TRAVERED : ASTAR_TRAVERED;
    }
    else
    {
        return ASTAR_FLAG_ERROR;
    }
}

ASTAR::Node::Node(coordination coord_, Node *parent_)
{
    this->parent = parent_;
    this->coord = coord_;
    this->F_cost = 0;
    this->G_cost = 0;
}

bool ASTAR::comp(Node *node1, Node *node2)
{
    return node1->getScore() > node2->getScore() ? true : false;
}
// class astar
void ASTAR::astar::setFlag_state(coordination coord_, FLAG_STATE state_)
{
    flag_table[coord_.y * this->width + coord_.x] = state_;
}

bool ASTAR::astar::isInboundary(int x, int y)
{
    if (x < 0 || y < 0 || x >= this->width || y >= this->height)
    {
        return false;
    }
    return true;
}

bool ASTAR::astar::isNoObstacle(coordination coord_)
{
    return this->map[coord_.y * this->width + coord_.x] == this->none_obstacle_flag ? true : false;
}

char *ASTAR::astar::getAstarMap()
{
    return this->map;
}

ASTAR::ASTAR_STATE ASTAR::astar::printPath(char path_color, std::vector<coordination> &path_vector)
{
    if (path_vector.empty())
    {
        return ASTAR_ERROR;
    }
    for (std::vector<coordination>::iterator it = path_vector.begin(); it < path_vector.end(); it++)
    {
        this->map[(*it).y * this->width + (*it).x] = path_color;
    }
    return ASTAR_SUCCESS;
}

ASTAR::ASTAR_STATE ASTAR::astar::addObstacle(float obs_coord_left_up_x, float obs_coord_left_up_y, float x_len, float y_len, float expasion_)
{
    coordination obs_coord_left_up = this->coordination_trans_to_astar(obs_coord_left_up_x, obs_coord_left_up_y);
    return this->addObstacle_in(obs_coord_left_up, (x_len / this->resolution), (y_len / this->resolution), expasion_);
}

ASTAR::ASTAR_STATE ASTAR::astar::addObstacle(f_coordination point_1, f_coordination point_2, float depth, float yaw)
{
    // int expasion_len = expasion_ / this->resolution;
    this->drawRectangle(point_1, point_2, depth, yaw);
    return ASTAR_SUCCESS;
}

float ASTAR::astar::getMinfloat(float val_1, float val_2, float val_3, float val_4)
{
    float temp_min;
    if (val_1 < val_2)
    {
        temp_min = val_1;
    }
    else
    {
        temp_min = val_2;
    }
    if (temp_min > val_3)
    {
        temp_min = val_3;
    }
    if (temp_min > val_4)
    {
        temp_min = val_4;
    }
    return temp_min;
}

float ASTAR::astar::getMaxfloat(float val_1, float val_2, float val_3, float val_4)
{
    float temp_max;
    if (val_1 > val_2)
    {
        temp_max = val_1;
    }
    else
    {
        temp_max = val_2;
    }
    if (temp_max < val_3)
    {
        temp_max = val_3;
    }
    if (temp_max < val_4)
    {
        temp_max = val_4;
    }
    return temp_max;
}

ASTAR::ASTAR_STATE ASTAR::astar::removeObstacle(f_coordination point_1, f_coordination point_2, float depth, float yaw)
{
    // f_coordination line_1_start, f_coordination line_1_end, unsigned int depth, float direction
    f_coordination line_2_point;
    f_coordination line_2_temp;
    f_coordination line_3_point;
    f_coordination line_4_point;
    f_coordination line_start;
    f_coordination line_end;
    if (point_1.x > point_2.x)
    {
        line_start = point_2;
        line_end = point_1;
    }
    else
    {
        line_start = point_1;
        line_end = point_2;
    }
    double angle = atan((point_2.y - point_1.y) / (point_2.x - point_1.x));

    line_2_temp.x = line_start.x;
    line_2_temp.y = line_start.y;
    if (yaw <= 0 && yaw > -M_PI)
    {
        line_2_point.x = line_2_temp.x - depth * sin(angle);
        line_2_point.y = line_2_temp.y + depth * cos(angle);
    }
    else
    {
        line_2_point.x = line_2_temp.x + depth * sin(angle);
        line_2_point.y = line_2_temp.y - depth * cos(angle);
    }
    line_2_temp.x = line_end.x;
    line_2_temp.y = line_end.y;
    if (yaw <= 0 && yaw > -M_PI)
    {
        line_3_point.x = line_2_temp.x - depth * sin(angle);
        line_3_point.y = line_2_temp.y + depth * cos(angle);
    }
    else
    {
        line_3_point.x = line_2_temp.x + depth * sin(angle);
        line_3_point.y = line_2_temp.y - depth * cos(angle);
    }
    // std::cout << " remx = " << line_start.x << " " << line_end.x << " " << line_2_point.x << " " << line_3_point.x << std::endl;
    // std::cout << " remy = " << line_start.y << " " << line_end.y << " " << line_2_point.y << " " << line_3_point.y << std::endl;
    float min_x = this->getMinfloat(line_start.x, line_end.x, line_2_point.x, line_3_point.x) - 0.1;
    float max_x = this->getMaxfloat(line_start.x, line_end.x, line_2_point.x, line_3_point.x) + 0.1;
    float min_y = this->getMinfloat(line_start.y, line_end.y, line_2_point.y, line_3_point.y) - 0.1;
    float max_y = this->getMaxfloat(line_start.y, line_end.y, line_2_point.y, line_3_point.y) + 0.1;
    // std::cout << "max x = " << max_x << " min y = " << min_y << std::endl;
    // std::cout << "sin() = " << sin(angle) << std::endl;
    coordination astar_point = this->coordination_trans_to_astar(max_x, min_y); // in the astar axis the minx is max_x of map axis
    unsigned int x_len = ceil((max_x - min_x) / this->resolution);
    unsigned int y_len = ceil((max_y - min_y) / this->resolution);
    for (int i = 0; i <= y_len; i++)
    {
        for (int j = 0; j <= x_len; j++)
        {
            if ((i + astar_point.y) >= 0 && (i + astar_point.y) < this->height && (j + astar_point.x) >= 0 && (j + astar_point.x) < this->width)
            {
                this->map[(i + astar_point.y) * this->width + (j + astar_point.x)] = this->map_const[(i + astar_point.y) * this->width + (j + astar_point.x)];
            }
        }
    }
    return ASTAR_SUCCESS;
}

ASTAR::ASTAR_STATE ASTAR::astar::addObstacle_in(coordination obs_coord_left_up, unsigned int x_len, unsigned int y_len, float expasion_)
{
    int expasion_len = expasion_ / this->resolution;
    coordination obs_coor_l_u;
    obs_coor_l_u.x = obs_coord_left_up.x - expasion_len > 0 ? obs_coord_left_up.x - expasion_len : 0;
    obs_coor_l_u.y = obs_coord_left_up.y - expasion_len > 0 ? obs_coord_left_up.y - expasion_len : 0;
    unsigned int x_obs = obs_coord_left_up.x + x_len + expasion_len < this->width ? obs_coord_left_up.x + x_len + expasion_len : this->width - 1;
    unsigned int y_obs = obs_coord_left_up.y + y_len + expasion_len < this->height ? obs_coord_left_up.y + y_len + expasion_len : this->height;
    for (unsigned int i = obs_coor_l_u.y; i <= y_obs; i++)
    {
        for (unsigned int j = obs_coor_l_u.x; j <= x_obs; j++)
        {
            this->map[i * this->width + j] = 0x88; // 0x88 obstacle color
        }
    }
    return ASTAR_SUCCESS;
}

ASTAR::ASTAR_STATE ASTAR::astar::drawLine(f_coordination start, f_coordination end)
{
    coordination A_start_temp = this->coordination_trans_to_astar(start.x, start.y);
    coordination A_end_temp = this->coordination_trans_to_astar(end.x, end.y);
    coordination A_start;
    coordination A_end;
    if (A_start_temp.x > A_end_temp.x)
    {
        A_start = A_end_temp;
        A_end = A_start_temp;
    }
    else
    {
        A_start = A_start_temp;
        A_end = A_end_temp;
    }
    // std::cout << "1 = " << A_start.x << " 1 y = " << A_start.y << " 2 = " << A_end.x << " " << A_end.y << std::endl;
    double angle = atan((float)(A_end.y - A_start.y) / (float)(A_end.x - A_start.x));
    // std::cout << "angle = " << angle << std::endl;
    if (abs(A_end.x - A_start.x) >= abs(A_end.y - A_start.y))
    {
        float i = 0; // 控制分辨率
        while (i <= abs(A_end.x - A_start.x))
        {
            int j_y = round(A_start.y + i * tan(angle));
            int j_x = round(i + A_start.x);
            // std::cout << " x_y = " << j_x << " y = " << j_y << " i*a = " << i * tan(angle) << std::endl;
            if (j_x >= 0 && j_x < this->width && j_y >= 0 && j_y < this->height)
            {
                this->map[j_y * this->width + j_x] = 0x88;
                // std::cout << "in\n";
            }
            i += (this->resolution) / 8;
        }
    }
    else
    {
        float i = 0;
        while (i <= abs(A_end.y - A_start.y))
        {
            int j_y;
            int j_x;
            if (angle < 0)
            {
                j_y = round(A_end.y + i);
                j_x = round(A_end.x + i / tan(angle)); // because tan(angle) < 0 ,so - abs(tan(angle)) = + tan(angle)
            }
            else
            {
                j_y = round(A_end.y - i);
                j_x = round(A_end.x - i / tan(angle));
            }
            // std::cout << " x_y = " << j_x << " y = " << j_y << " i*a = " << i * tan(angle) << std::endl;
            if ((j_x) >= 0 && (j_x) < this->width && j_y >= 0 && j_y < this->height)
            {
                // std::cout << "in\n";
                this->map[j_y * this->width + j_x] = 0x88;
            }
            i += (this->resolution) / 8;
        }
    }
    return ASTAR_SUCCESS;
}

ASTAR::ASTAR_STATE ASTAR::astar::drawLine_delete(f_coordination start, f_coordination end)
{
    coordination A_start_temp = this->coordination_trans_to_astar(start.x, start.y);
    coordination A_end_temp = this->coordination_trans_to_astar(end.x, end.y);
    coordination A_start;
    coordination A_end;
    if (A_start_temp.x > A_end_temp.x)
    {
        A_start = A_end_temp;
        A_end = A_start_temp;
    }
    else
    {
        A_start = A_start_temp;
        A_end = A_end_temp;
    }
    // std::cout << "1 = " << A_start.x << " " << A_start.y << " 2 = " << A_end.x << " " << A_end.y << std::endl;
    double angle = atan((float)(A_end.y - A_start.y) / (float)(A_end.x - A_start.x));
    // std::cout << "angle = " << angle << std::endl;
    if (abs(A_end.x - A_start.x) >= abs(A_end.y - A_start.y))
    {
        float i = 0; // 控制分辨率
        while (i < abs(A_end.x - A_start.x))
        {
            int j_y = round(A_start.y + i * tan(angle));
            int j_x = round(i + A_start.x);
            // std::cout << " x_y = " << j_x << " y = " << j_y << " i*a = " << i * tan(angle) << std::endl;
            if (j_x >= 0 && j_x < this->width && j_y >= 0 && j_y < this->height)
            {
                this->map[j_y * this->width + j_x] = this->map_const[j_y * this->width + j_x];
            }
            i += (this->resolution) / 8;
        }
    }
    else
    {
        float i = 0;
        while (i <= abs(A_end.y - A_start.y))
        {
            int j_y;
            int j_x;
            if (angle < 0)
            {
                j_y = round(A_end.y + i);
                j_x = round(A_end.x + i / tan(angle)); // because tan(angle) < 0 ,so - abs(tan(angle)) = + tan(angle)
            }
            else
            {
                j_y = round(A_end.y - i);
                j_x = round(A_end.x - i / tan(angle));
            }
            // std::cout << " x_y = " << j_x << " y = " << j_y << " i*a = " << i * tan(angle) << std::endl;
            if ((j_x) >= 0 && (j_x) < this->width && j_y >= 0 && j_y < this->height)
            {
                this->map[j_y * this->width + j_x] = this->map_const[j_y * this->width + j_x];
            }
            i += (this->resolution) / 8;
        }
    }
    return ASTAR_SUCCESS;
}

ASTAR::ASTAR_STATE ASTAR::astar::drawRectangle(f_coordination line_1_start, f_coordination line_1_end, float depth, float direction)
{
    f_coordination line_2_point;
    f_coordination line_2_temp;
    f_coordination line_start;
    f_coordination line_end;
    if (line_1_start.x > line_1_end.x)
    {
        line_start = line_1_end;
        line_end = line_1_start;
    }
    else
    {
        line_start = line_1_start;
        line_end = line_1_end;
    }
    double angle = atan((line_1_end.y - line_1_start.y) / (line_1_end.x - line_1_start.x));
    // double angle2 = angle + M_PI;
    //
    std::cout << "angle = " << (angle) << std::endl;
    
    // std::cout << " rec coord x = " << line_1_start.x << " y = " << line_1_start.y << std::endl;
    if (abs(line_end.x - line_start.x) >= abs(line_end.y - line_start.y))
    {
        float i = 0; // 控制分辨率
        while (i <= abs(line_end.x - line_start.x))
        {
            float j_y = (line_start.y + i * tan(angle));
            float j_x = (i + line_start.x);
            line_2_temp.x = j_x;
            line_2_temp.y = j_y;
            if (direction <= 0 && direction > -M_PI)
            {
                line_2_point.x = j_x - depth * sin(angle);
                line_2_point.y = j_y + depth * cos(angle);
            }
            else
            {
                line_2_point.x = j_x + depth * sin(angle);
                line_2_point.y = j_y - depth * cos(angle);
            }
            // std::cout << " line coord x = " << line_2_point.x << " y = " << line_2_point.y << std::endl;
            this->drawLine(line_2_temp, line_2_point);
            i += (this->resolution) / 8;
        }
    }
    else
    {
        float i = 0;
        while (i <= abs(line_end.y - line_start.y))
        {
            float j_y;
            float j_x;
            if (angle < 0)
            {
                j_y = (line_end.y + i);
                j_x = (line_end.x + i / tan(angle)); // because tan(angle) < 0 ,so - abs(tan(angle)) = + tan(angle)
                line_2_temp.x = j_x;
                line_2_temp.y = j_y;
                if (direction <= 0 && direction > -M_PI)
                {
                    line_2_point.x = j_x - depth * sin(angle);
                    line_2_point.y = j_y + depth * cos(angle);
                }
                else
                {
                    line_2_point.x = j_x + depth * sin(angle);
                    line_2_point.y = j_y - depth * cos(angle);
                }
                this->drawLine(line_2_temp, line_2_point);
            }
            else
            {
                j_y = (line_end.y - i);
                j_x = (line_end.x - i / tan(angle));
                line_2_temp.x = j_x;
                line_2_temp.y = j_y;
                if (direction <= 0 && direction > -M_PI)
                {
                    line_2_point.x = j_x - depth * sin(angle);
                    line_2_point.y = j_y + depth * cos(angle);
                }
                else
                {
                    line_2_point.x = j_x + depth * sin(angle);
                    line_2_point.y = j_y - depth * cos(angle);
                }
                // std::cout << "depth = " << depth << std::endl;
                // std::cout << " line coord x = " << line_2_temp.x << " y = " << line_2_temp.y << std::endl;
                // std::cout << " line coord x = " << line_2_point.x << " y = " << line_2_point.y << std::endl;
                this->drawLine(line_2_temp, line_2_point);
            }
            i += (this->resolution) / 8;
        }
    }
    return ASTAR_SUCCESS;
}

ASTAR::ASTAR_STATE ASTAR::astar::removeObstacle(float obs_coord_left_up_x, float obs_coord_left_up_y, float x_len_, float y_len_, float expasion_)
{
    coordination obs_coord_left_up = this->coordination_trans_to_astar(obs_coord_left_up_x, obs_coord_left_up_y);
    int expasion_len = expasion_ / this->resolution;
    unsigned int x_len = x_len_ / this->resolution;
    unsigned int y_len = y_len_ / this->resolution;
    coordination obs_coor_l_u;
    obs_coor_l_u.x = obs_coord_left_up.x - expasion_len > 0 ? obs_coord_left_up.x - expasion_len : 0;
    obs_coor_l_u.y = obs_coord_left_up.y - expasion_len > 0 ? obs_coord_left_up.y - expasion_len : 0;
    unsigned int x_obs = obs_coord_left_up.x + x_len + expasion_len < this->width ? obs_coord_left_up.x + x_len + expasion_len : this->width - 1;
    unsigned int y_obs = obs_coord_left_up.y + y_len + expasion_len < this->height ? obs_coord_left_up.y + y_len + expasion_len : this->height;
    for (unsigned int i = obs_coor_l_u.y; i <= y_obs; i++)
    {
        for (unsigned int j = obs_coor_l_u.x; j <= x_obs; j++)
        {
            this->map[i * this->width + j] = this->map_const[i * this->width + j];
        }
    }
    return ASTAR_SUCCESS;
}

ASTAR::coordination ASTAR::astar::coordination_trans_to_astar(float x, float y)
{
    coordination transformed_coord;
    // printf("%f\n", x / this->resolution);
    transformed_coord.x = round(x / this->resolution); // > 0 ? ceil(x / this->resolution ) : floor(x / this->resolution);
    transformed_coord.y = round(y / this->resolution); // > 0 ? ceil(y / this->resolution ) : floor(y / this->resolution);
    switch (this->caller_axis)
    {
    case CALLER_AXIS::X_RIGHT_Y_UP:
        transformed_coord.x = transformed_coord.x + (this->caller_original.x - 0);
        transformed_coord.y = -transformed_coord.y;
        break;
    case CALLER_AXIS::X_LEFT_Y_DOWN:
        transformed_coord.x = -transformed_coord.x + (this->caller_original.x - 0);
        transformed_coord.y = transformed_coord.y + this->caller_original.y;
        break;
    default:
        break;
    }
    return transformed_coord;
}

std::vector<ASTAR::coordination> ASTAR::astar::coordination_trans_to_astar(std::vector<ASTAR::f_coordination> f_path_vector)
{
    std::vector<ASTAR::coordination> path_temp;
    for (std::vector<f_coordination>::iterator it = f_path_vector.begin(); it < f_path_vector.end(); it++)
    {
        path_temp.push_back(this->coordination_trans_to_astar(it->x, it->y));
    }
    return path_temp;
}

ASTAR::ASTAR_STATE ASTAR::astar::coordination_trans_to_caller_axis(std::vector<coordination> &source_path_vector, std::vector<ASTAR::f_coordination> &dest_path_vector)
{
    f_coordination f_temp_coord;
    coordination temp_coord;
    switch (this->caller_axis)
    {
    case CALLER_AXIS::X_RIGHT_Y_UP:
        while (!source_path_vector.empty())
        {
            temp_coord = source_path_vector.back();
            source_path_vector.pop_back();
            f_temp_coord.x = (temp_coord.x - (this->caller_original.x - 0)) * this->resolution;
            f_temp_coord.y = (-temp_coord.y) * this->resolution;
            dest_path_vector.push_back(f_temp_coord);
        }
        break;
    case CALLER_AXIS::X_LEFT_Y_DOWN:
        while (!source_path_vector.empty())
        {
            temp_coord = source_path_vector.back();
            source_path_vector.pop_back();
            f_temp_coord.x = (-temp_coord.x - (this->caller_original.x - 0)) * this->resolution;
            f_temp_coord.y = (temp_coord.y - this->caller_original.y) * this->resolution;
            dest_path_vector.push_back(f_temp_coord);
        }
        break;

    default:
        break;
    }
    return ASTAR_SUCCESS;
}

ASTAR::ASTAR_STATE ASTAR::astar::compressPath(std::vector<f_coordination> &dest_path_vector)
{
    if (dest_path_vector.empty())
    {
        return ASTAR_ERROR;
    }
    std::vector<f_coordination>::iterator it = dest_path_vector.begin();
    f_coordination pre = *it;
    f_coordination temp;
    it++;
    while (it != dest_path_vector.end())
    {
        temp = *it;
        if (it == dest_path_vector.end() - 1)
        {
            break;
        }
        else
        {
            if (pre.x == (*(it + 1)).x || pre.y == (*(it + 1)).y)
            {
                dest_path_vector.erase(it);
            }
            else
            {
                pre = *it;
                it++;
            }
        }
    }
    return ASTAR_SUCCESS;
}

ASTAR::ASTAR_STATE ASTAR::astar::find_path(std::vector<f_coordination> &dest_path_vector, float source_x, float source_y, float destination_x, float destination_y, int IsprintPath, char path_color)
{
    coordination source;
    coordination dest;
    std::vector<coordination> path_vector;
    source = coordination_trans_to_astar(source_x, source_y);
    dest = coordination_trans_to_astar(destination_x, destination_y);
    std::cout << "source : " << source.x << " y " << source.y << std::endl;
    std::cout << "dest : " << dest.x << " y " << dest.y << std::endl;
    if (this->find_path(path_vector, source, dest) == ASTAR_SUCCESS)
    {
        if (IsprintPath == 1)
        {
            this->printPath(path_color, path_vector);
        }
        return this->coordination_trans_to_caller_axis(path_vector, dest_path_vector);
    }
    else
        return ASTAR_ERROR;
}

ASTAR::ASTAR_STATE ASTAR::astar::find_path(std::vector<coordination> &path_vector, coordination source_, coordination destination_)
{
    if (&path_vector == NULL)
    {
        return ASTAR_ERROR;
    }

    if (this->path_node_reversal(path_vector, source_, destination_) == ASTAR_ERROR)
    {
        std::cout << "a star error\n";
        return ASTAR_ERROR;
    }
    std::cout << "a star success\n";
    return ASTAR_SUCCESS;
}

ASTAR::ASTAR_STATE ASTAR::astar::path_node_reversal(std::vector<coordination> &path_vector, coordination source_, coordination destination_)
{
    // X_POSITIVE_RIGHT, Y_POSITIVE_DOWN
    Nodevector node_vector;
    Nodevector close_vector;
    node_vector.reserve(100);
    close_vector.reserve(100);
    coordination start = source_;
    Node *star_node = new Node(start, nullptr);
    star_node->coord = source_;
    star_node->F_cost = star_node->G_cost = 0;
    star_node->parent = NULL;
    node_vector.push_back(star_node);
    if (this->flag_table != NULL)
    {
        memset(this->flag_table, ASTAR_NO_TRAVERED, width * height);
    }
    unsigned int count = 0;
    while (!node_vector.empty())
    {
        Node *current_node = nullptr;
        current_node = node_vector.back();
        if (current_node == nullptr)
        {
            std::cout << "fail\n";
            return ASTAR_ERROR;
        }
        if (this->getFlag_state(current_node->coord) == ASTAR_TRAVERED)
        {
            // delete multi TRAVERED node
            delete current_node;
            node_vector.pop_back();
            continue;
        }

        this->setFlag_state(current_node->coord, ASTAR_TRAVERED);
        // print reversal node --------------------------
        // this->map[current_node->coord.y * this->width + current_node->coord.x] = 0xDD;
        // end print-------------------------------------
        //  move node to close vector
        if (current_node != nullptr)
        {
            close_vector.push_back(current_node);
        }
        node_vector.pop_back();
        // success
        if (current_node->coord == destination_)
        {
            for (Nodevector::iterator it = node_vector.begin(); it < node_vector.end(); it++)
            {
                delete *(it);
            }
            node_vector.clear();
            Node *pathtrace = close_vector.back();
            while (1)
            {
                path_vector.push_back(pathtrace->coord);
                if (pathtrace->parent == nullptr)
                {
                    break;
                }
                pathtrace = pathtrace->parent;
            }
            return ASTAR_SUCCESS;
        }
        coordination temp_coord;
        // left
        temp_coord.x = current_node->coord.x - 1;
        temp_coord.y = current_node->coord.y;
        if (this->isInboundary(temp_coord.x, temp_coord.y) && this->getFlag_state(temp_coord) == ASTAR_NO_TRAVERED && this->isNoObstacle(temp_coord))
        {
            Node *temp_node = new Node(temp_coord, current_node);
            temp_node->coord.x = temp_coord.x;
            temp_node->coord.y = temp_coord.y;
            temp_node->F_cost = current_node->F_cost + 1;
            temp_node->G_cost = abs(temp_node->coord.x - destination_.x) + abs(temp_node->coord.y - destination_.y);
            node_vector.push_back(temp_node);
        }
        // down
        temp_coord.x = current_node->coord.x;
        temp_coord.y = current_node->coord.y - 1;
        if (this->isInboundary(temp_coord.x, temp_coord.y) && this->getFlag_state(temp_coord) == ASTAR_NO_TRAVERED && this->isNoObstacle(temp_coord))
        {
            Node *temp_node = new Node(temp_coord, current_node);
            temp_node->coord.x = temp_coord.x;
            temp_node->coord.y = temp_coord.y;
            temp_node->F_cost = current_node->F_cost + 1;
            temp_node->G_cost = abs(temp_node->coord.x - destination_.x) + abs(temp_node->coord.y - destination_.y);
            node_vector.push_back(temp_node);
        }
        // up
        temp_coord.x = current_node->coord.x;
        temp_coord.y = current_node->coord.y + 1;
        if (this->isInboundary(temp_coord.x, temp_coord.y) && this->getFlag_state(temp_coord) == ASTAR_NO_TRAVERED && this->isNoObstacle(temp_coord))
        {
            Node *temp_node = new Node(temp_coord, current_node);
            temp_node->coord.x = temp_coord.x;
            temp_node->coord.y = temp_coord.y;
            temp_node->F_cost = current_node->F_cost + 1;
            temp_node->G_cost = abs(temp_node->coord.x - destination_.x) + abs(temp_node->coord.y - destination_.y);
            node_vector.push_back(temp_node);
        }
        // up
        temp_coord.x = current_node->coord.x + 1;
        temp_coord.y = current_node->coord.y;
        if (this->isInboundary(temp_coord.x, temp_coord.y) && this->getFlag_state(temp_coord) == ASTAR_NO_TRAVERED && this->isNoObstacle(temp_coord))
        {
            Node *temp_node = new Node(temp_coord, current_node);
            temp_node->coord.x = temp_coord.x;
            temp_node->coord.y = temp_coord.y;
            temp_node->F_cost = current_node->F_cost + 1;
            temp_node->G_cost = abs(temp_node->coord.x - destination_.x) + abs(temp_node->coord.y - destination_.y);
            node_vector.push_back(temp_node);
        }
        // sort from big to small (avoid to move element in the vector when delete element, save time)
        std::sort(node_vector.begin(), node_vector.end(), comp);
    }
    return ASTAR_ERROR;
}

ASTAR::ASTAR_STATE ASTAR::astar::getDestIndexOfPath(std::vector<f_coordination> pathVector_, unsigned int &index, unsigned int reversal_index_count)
{
    unsigned int i = 0;
    unsigned int len = pathVector_.size();
    int flag = 0;
    std::vector<ASTAR::coordination> astartPath_ = this->coordination_trans_to_astar(pathVector_);
    if (this->isNoObstacle(astartPath_[(index + i) % len]) == false)
    {
        //开始起点落在障碍物
        if (this->isNoObstacle(astartPath_[(index + 1) % len]) == true)
        {
            index = index + 1;
        }
        else if (index - 1 >= 0)
        {
            if(this->isNoObstacle(astartPath_[(index - 1) % len]) == true)
            {
                index = index - 1;
            }
            else
            {
                return ASTAR_ERROR;
            }
        }
        else
        {
            return ASTAR_ERROR;
        }
    }
    if (reversal_index_count > len)
    {
        reversal_index_count = len;
    }
    while (reversal_index_count > 0)
    {
        if (this->isNoObstacle(astartPath_[(index + i) % len]) == false)
        {
            flag = 1;
        }
        if (this->isNoObstacle(astartPath_[(index + i) % len]) == true && flag == 1)
        {
            flag = 2;
            index = (index + i) % len;
            break;
        }
        reversal_index_count--;
        i++;
    }
    if (flag == 0)
    {
        // obs not in the path
        return ASTAR_SUCCESS;
    }
    else if (flag == 1)
    {
        // not find dest
        return ASTAR_ERROR;
    }
    else
    {
        // dest index find
        return ASTAR_SUCCESS;
    }
}