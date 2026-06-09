#ifndef PGM__HPP
#define PGM__HPP

#include <iostream>
#include <fstream>
#include <string>
#include <math.h>
#include <yaml-cpp/yaml.h>
namespace PGM_SPACE
{
    enum PGM_STATE
    {
        PGM_SUCCESS,
        PGM_ERROR,
    };
    struct pgm_header
    {
        unsigned int width;
        unsigned int height;
        unsigned int max_gray;
        unsigned int type; // p2 / p5
    };
    struct gary_type_struct
    {
        char visiable;
        char unkonw;
        char obstacle;
        char handle_visible;
        char handle_obstacle;
    };

    struct coord
    {
        float x;
        float y;
        float z;
    };

    class PGM
    {
    public:
        PGM()
        {
            this->header.height = 0;
            this->header.width = 0;
            this->header.max_gray = 0;
            this->header.type = 0;
            this->pdata = NULL;
        }
        ~PGM()
        {
            if (this->pdata != NULL)
            {
                free(this->pdata);
            }
            std::cout << "Deinit\n";
        }

        // read yaml file and then load pgm file
        PGM_STATE LoadYamlFile(const char *file_path);

        // set the different gray type from the pgm file
        PGM_STATE setgray_type(char visiable, char unkonw, char obstacle);

        PGM_STATE SavePGM(const char *path, char *pdata_, unsigned int width_, unsigned int height_, unsigned int max_gray_);

        // handle the map
        // obstacle: set the value you want to present obstacle
        // no_obstacle: set the value you want to present no obstacle
        // expansion_tag_value: set the value you want to present expansion
        // expansion_value: how big the expansion you want to set
        PGM_STATE PGM_handle(char obstacle, char no_obstacle, float obj_width, float expansion_value, char expansion_tag_value);

        char *getPdata()
        {
            return this->pdata;
        }

        struct pgm_header getpgm_header()
        {
            return this->header;
        }

        PGM_STATE setOrigin(coord coord_);

        coord getOrigin();

        float getResolution();

#define getMap getPdata

        coord origin;

    private:
        struct pgm_header header;
        char *pdata;
        struct gary_type_struct gray_type; // record types of point
        float resolution;

        PGM_STATE Loadfile(const char *file_path);

        // make multi gray_state to tow state
        PGM_STATE pre_handle_pgm(char obstacle, char no_obstacle);

        // file the narrow empty to obstacle
        PGM_STATE PGM_handle_fill(float obj_width);

        // expasion the obstacle (keep the bondary)
        // expansion_value need to be greater then half of (width or lens) of robot
        PGM_STATE PGM_handle_expansion(float expansion_value, char expansion_tag_value);
    };

}

#endif