#ifndef A_STAR__H
#define A_STAR__H

#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <iostream>
#include <vector>

namespace ASTAR
{
    enum CALLER_AXIS
    {
        X_RIGHT_Y_UP,
        X_RIGHT_Y_DOWN,
        X_LEFT_Y_UP,
        X_LEFT_Y_DOWN,
    };
    enum ASTAR_STATE
    {
        ASTAR_SUCCESS,
        ASTAR_ERROR,
    };
    struct coordination
    {
        int x;
        int y;
        bool operator==(coordination coordination_);
    };

    struct f_coordination
    {
        double x;
        double y;
    };

    struct Node
    {
        Node(coordination coord_, Node *parent_ = nullptr);
        unsigned int G_cost;
        unsigned int F_cost; // front cost
        coordination coord;
        Node *parent;
        unsigned int getScore();
    };

    enum FLAG_STATE
    {
        ASTAR_NO_TRAVERED,
        ASTAR_TRAVERED,
        ASTAR_FLAG_ERROR,
    };

    bool comp(Node *node1, Node *node2);

    using Nodevector = std::vector<Node *>;
    class astar
    {
    public:
        astar(char *map, unsigned int width, unsigned int height, char none_obstacle_flag, coordination original, CALLER_AXIS caller_axis_, float resolution_)
        {
            if (map == NULL)
            {
                return;
            }
            this->map_const = map;
            this->map = (char *)malloc(sizeof(char) * width * height);
            memcpy(this->map, map, width * height);
            this->resolution = resolution_;
            this->width = width;
            this->height = height;
            this->none_obstacle_flag = none_obstacle_flag;
            this->caller_original = original;
            this->caller_axis = caller_axis_;
            this->flag_table = (char *)malloc(sizeof(char) * width * height);
            if (this->flag_table != NULL)
            {
                memset(this->flag_table, ASTAR_NO_TRAVERED, width * height);
            }
        }

        ~astar()
        {
            if (this->map == NULL)
            {
                free(this->map);
            }
            if (this->flag_table != NULL)
            {
                free(this->flag_table);
            }
        }

        
        // the coordonation is in the caller axis
        ASTAR_STATE find_path(std::vector<f_coordination> &dest_path_vector, float source_x, float source_y, float destination_x, float destination_y, int IsprintPath, char path_color = 0x77);

        char* getAstarMap();

        ASTAR_STATE compressPath(std::vector<f_coordination> &dest_path_vector);

        // the coordonation is in the caller axis
        ASTAR_STATE addObstacle(float obs_coord_left_up_x, float obs_coord_left_up_y, float x_len, float y_len, float expasion_);

        ASTAR_STATE addObstacle(f_coordination point_1, f_coordination point_2, float depth, float yaw);

        // the coordonation is in the caller axis
        coordination coordination_trans_to_astar(float x, float y);
        std::vector<coordination> coordination_trans_to_astar(std::vector<f_coordination> f_path_vector);

        //
        ASTAR_STATE printPath(char path_color, std::vector<coordination> &path_vector);

        ASTAR_STATE coordination_trans_to_caller_axis(std::vector<coordination> &source_path_vector, std::vector<f_coordination> &dest_path_vector);

        //
        ASTAR_STATE removeObstacle(float obs_coord_left_up_x, float obs_coord_left_up_y, float x_len, float y_len, float expasion_);
        ASTAR_STATE removeObstacle(f_coordination point_1, f_coordination point_2, float depth, float yaw);
        //
        ASTAR_STATE drawLine(f_coordination start, f_coordination end);

        ASTAR_STATE drawLine_delete(f_coordination start, f_coordination end);
        // ASTAR_STATE drawRectangle(f_coordination line_1_start, f_coordination line_1_end, f_coordination line_2_start, f_coordination line_2_end);
        ASTAR_STATE drawRectangle(f_coordination line_1_start, f_coordination line_1_end, float depth, float direction);
    
        float getMinfloat(float val_1, float val_2, float val_3, float val_4);
        float getMaxfloat(float val_1, float val_2, float val_3, float val_4);

        //index is current coordination index
        //return ASTAR_SUCCESS and index = index -> obs not in the path
        //return ASTAR_SUCCESS and index != index -> //dest index find
        //return ASTAR_ERROR -> not find dest index
        ASTAR_STATE getDestIndexOfPath(std::vector<f_coordination> pathVector_, unsigned int& index, unsigned int reversal_index_count);
    private:
        char *map;
        char *map_const;
        char *flag_table;
        unsigned int width;
        unsigned int height;
        float resolution;
        char none_obstacle_flag;    //mark the none-obstacle
        coordination caller_original;   //caller original in the astar axis
        CALLER_AXIS caller_axis;

        ASTAR_STATE path_node_reversal(std::vector<coordination> &path_vector, coordination source_, coordination destination_);

        FLAG_STATE getFlag_state(coordination coord_);

        void setFlag_state(coordination coord_, FLAG_STATE state_);

        bool isInboundary(int x, int y);

        bool isNoObstacle(coordination coord_);

        

        ASTAR_STATE find_path(std::vector<coordination> &path_vector, coordination source_, coordination destination_);

        ASTAR_STATE addObstacle_in(ASTAR::coordination obs_coord_left_up, unsigned int x_len, unsigned int y_len, float expasion_);
    };
};
#endif