#include "./pgm.hpp"

PGM_SPACE::PGM_STATE PGM_SPACE::PGM::LoadYamlFile(const char *file_path)
{
    YAML::Node yaml = YAML::LoadFile(file_path);
    std::string image_path = yaml["image"].as<std::string>();
    this->resolution = yaml["resolution"].as<float>();
    PGM_SPACE::coord temp_coord;
    YAML::iterator it = yaml["origin"].begin();
    temp_coord.x = (*it).as<float>();
    it++;
    temp_coord.y = (*it).as<float>();
    it++;
    if ((*it).as<std::string>() != "nan")
    {
        temp_coord.z = (*it).as<float>();
    }
    else
    {
        temp_coord.z = 0;
    }
    this->setOrigin(temp_coord);
    // std::cout << "ori = x " << this->origin.x << " y = " << this->origin.y << " z = " << this->origin.z << std::endl;
    std::cout << "image path : " << image_path << std::endl;
    std::cout << "resolution : " << this->resolution << std::endl;
    if (this->Loadfile(image_path.c_str()) != PGM_SUCCESS)
    {
        return PGM_ERROR;
    }
    return PGM_SUCCESS;
}

PGM_SPACE::PGM_STATE PGM_SPACE::PGM::setOrigin(coord coord_)
{
    this->origin = coord_;
    return PGM_SUCCESS;
}

PGM_SPACE::coord PGM_SPACE::PGM::getOrigin()
{
    return this->origin;
}

PGM_SPACE::PGM_STATE PGM_SPACE::PGM::setgray_type(char visiable, char unkonw, char obstacle)
{
    this->gray_type.visiable = visiable;
    this->gray_type.unkonw = unkonw;
    this->gray_type.obstacle = obstacle;
    return PGM_SUCCESS;
}

float PGM_SPACE::PGM::getResolution()
{
    return this->resolution;
}

PGM_SPACE::PGM_STATE PGM_SPACE::PGM::pre_handle_pgm(char obstacle, char no_obstacle)
{
    unsigned int index = 0;
    this->gray_type.handle_obstacle = obstacle;
    this->gray_type.handle_visible = no_obstacle;
    while (index < this->header.width * this->header.height)
    {
        if (this->pdata[index] == this->gray_type.obstacle || this->pdata[index] == this->gray_type.unkonw)
        {
            this->pdata[index] = obstacle;
        }
        else
        {
            this->pdata[index] = no_obstacle;
        }
        index++;
    }
    return PGM_SUCCESS;
}

PGM_SPACE::PGM_STATE PGM_SPACE::PGM::SavePGM(const char *path, char *pdata_, unsigned int width_, unsigned int height_, unsigned int max_gray_)
{
    std::ofstream ofs;
    std::string str = std::string(path);
    ofs.open(str + "map.pgm");
    ofs << "P5\n";
    ofs << width_ << " " << height_ << std::endl;
    ofs << max_gray_ << std::endl;
    if (pdata_ != NULL)
    {
        ofs.write(pdata_, width_ * height_);
    }
    return PGM_SUCCESS;
}

PGM_SPACE::PGM_STATE PGM_SPACE::PGM::PGM_handle_fill(float obj_width)
{
    unsigned int obj_w = obj_width / this->resolution;
    unsigned int begin = 0;
    unsigned int obstacle_start = 0;
    unsigned int visible_width = 0;
    // handle line
    for (unsigned int i = 0; i < this->header.height; i++)
    {

        begin = i * this->header.width;
        obstacle_start = begin;
        visible_width = 0;
        for (unsigned int j = 0; j < this->header.width; j++)
        {
            if (this->pdata[begin + j] == this->gray_type.handle_visible)
            {
                visible_width++;
            }
            else
            { // obstacle
                if (visible_width < obj_w)
                {
                    // print_obs
                    for (unsigned int k = obstacle_start; k <= begin + j; k++)
                    {
                        this->pdata[k] = this->gray_type.handle_obstacle;
                    }
                }
                obstacle_start = begin + j;
                visible_width = 0;
            }
            if (j == this->header.width - 1 && visible_width < obj_w)
            {
                for (unsigned int k = obstacle_start; k <= begin + j; k++)
                {
                    this->pdata[k] = this->gray_type.handle_obstacle;
                }
            }
        }
    }

    // handle h
    for (unsigned int i = 0; i < this->header.width; i++)
    {

        begin = i;
        obstacle_start = 0;
        visible_width = 0;
        for (unsigned int j = 0; j < this->header.height; j++)
        {
            if (this->pdata[begin + j * this->header.width] == this->gray_type.handle_visible)
            {
                visible_width++;
            }
            else
            { // obstacle
                if (visible_width < obj_w)
                {
                    // print_obs
                    for (unsigned int k = obstacle_start; k <= j; k++)
                    {
                        this->pdata[begin + k * this->header.width] = this->gray_type.handle_obstacle;
                    }
                }
                obstacle_start = j;
                visible_width = 0;
            }
            if (j == this->header.height - 1 && visible_width < obj_w)
            {
                for (unsigned int k = obstacle_start; k <= j; k++)
                {
                    this->pdata[begin + k * this->header.width] = this->gray_type.handle_obstacle;
                }
            }
        }
    }
    return PGM_SUCCESS;
}

PGM_SPACE::PGM_STATE PGM_SPACE::PGM::PGM_handle_expansion(float expansion_value, char expansion_tag_value)
{
    unsigned int pgm_expansiaon = expansion_value / this->resolution;
    std::cout << "expansion = " << pgm_expansiaon << std::endl;
    // from up to down
    for (unsigned int i = 0; i < this->header.height - pgm_expansiaon; i++)
    {
        for (unsigned int j = 0; j < this->header.width; j++)
        {
            if (this->pdata[i * this->header.width + j] == this->gray_type.handle_obstacle && this->pdata[(i + 1) * this->header.width + j] != this->gray_type.handle_obstacle)
            {
                //
                for (unsigned int k = 0; k < pgm_expansiaon; k++)
                {
                    this->pdata[(i + 1 + k) * this->header.width + j] = expansion_tag_value;
                }
            }
        }
    }
    // from down to up
    for (unsigned int i = this->header.height - 1; i > pgm_expansiaon; i--)
    {
        for (unsigned int j = 0; j < this->header.width; j++)
        {
            if (this->pdata[i * this->header.width + j] == this->gray_type.handle_obstacle && this->pdata[(i - 1) * this->header.width + j] != this->gray_type.handle_obstacle)
            {
                //
                for (unsigned int k = 0; k < pgm_expansiaon; k++)
                {
                    this->pdata[(i - 1 - k) * this->header.width + j] = expansion_tag_value;
                }
            }
        }
    }
    // from left to right
    for (unsigned int i = 0; i < this->header.width - pgm_expansiaon; i++)
    {
        for (unsigned int j = 0; j < this->header.height; j++)
        {
            if (this->pdata[j * this->header.width + i] == this->gray_type.handle_obstacle && this->pdata[j * this->header.width + i + 1] != this->gray_type.handle_obstacle)
            {
                for (unsigned int k = 0; k < pgm_expansiaon; k++)
                {
                    this->pdata[j * this->header.width + i + 1 + k] = expansion_tag_value;
                }
            }
        }
    }
    // from right to left
    for (unsigned int i = this->header.width - 1; i > pgm_expansiaon - 1; i--)
    {
        for (unsigned int j = 0; j < this->header.height; j++)
        {
            if (this->pdata[j * this->header.width + i] == this->gray_type.handle_obstacle && this->pdata[j * this->header.width + i - 1] != this->gray_type.handle_obstacle)
            {
                for (unsigned int k = 0; k < pgm_expansiaon; k++)
                {
                    this->pdata[j * this->header.width + i - 1 - k] = expansion_tag_value;
                }
            }
        }
    }
    // handle turn concer
    float rotate_angle = 1.0 / pgm_expansiaon;
    float rotate_counter = 0;
    for (unsigned int i = pgm_expansiaon; i < this->header.height - pgm_expansiaon; i++)
    {
        for (unsigned int j = pgm_expansiaon; j < this->header.width - pgm_expansiaon; j++)
        {
            // obstacle handle up down left right has visiable point
            if (this->pdata[i * this->header.width + j] == this->gray_type.handle_obstacle && (this->pdata[(i + 1) * this->header.width + j] != this->gray_type.handle_obstacle || this->pdata[(i - 1) * this->header.width + j] != this->gray_type.handle_obstacle || this->pdata[i * this->header.width + j - 1] != this->gray_type.handle_obstacle || this->pdata[i * this->header.width + j + 1] != this->gray_type.handle_obstacle))
            {
                rotate_counter = 0;
                while (rotate_angle * rotate_counter <= 2 * M_PI)
                {
                    for (unsigned int k = 1; k <= pgm_expansiaon; k++)
                    {
                        int x_inc = cos(rotate_angle * rotate_counter) > 0 ? (int)ceil(k * cos(rotate_angle * rotate_counter)) : (int)floor(k * cos(rotate_angle * rotate_counter));
                        int y_inc = sin(rotate_angle * rotate_counter) > 0 ? (int)ceil(k * sin(rotate_angle * rotate_counter)) : (int)floor(k * sin(rotate_angle * rotate_counter));
                        unsigned int index = j + x_inc + this->header.width * (i - y_inc);
                        if (this->pdata[index] == this->gray_type.handle_visible)
                        {
                            this->pdata[index] = expansion_tag_value;
                        }
                    }
                    rotate_counter++;
                }
            }
        }
    }
    return PGM_SUCCESS;
}

PGM_SPACE::PGM_STATE PGM_SPACE::PGM::Loadfile(const char *file_path)
{
    std::ifstream ifs;
    ifs.open(file_path, std::ios::in);
    if (!ifs.is_open())
    {
        std::cout << "file open fail!\n";
        return PGM_ERROR;
    }
    std::string buf;
    // read type
    getline(ifs, buf);
    if (buf == "P5")
    {
        this->header.type = 1;
    }
    else if (buf == "P2")
    {
        this->header.type = 2;
    }
    else
    {
        std::cout << "file type error!\n";
        return PGM_ERROR;
    }
    // pass the comments
    while (getline(ifs, buf))
    {
        if (buf[0] != '#')
            break;
    }
    if (buf[0] == '#')
    {
        std::cout << "file type error!\n";
        return PGM_ERROR;
    }
    // read PGM header
    // save format : "width height"
    std::cout << buf << std::endl;
    int j = buf.find(' ');
    for (unsigned int i = 0; i < j; i++)
    {
        this->header.width += (buf[i] - '0') * pow(10, j - i - 1);
    }

    for (unsigned int i = j + 1; i < buf.length(); i++)
    {
        this->header.height += (buf[i] - '0') * pow(10, buf.length() - i - 1);
    }

    // read max_gray
    if (getline(ifs, buf))
    {
        for (unsigned int i = 0; i < buf.length(); i++)
        {
            this->header.max_gray += (buf[i] - '0') * pow(10, buf.length() - i - 1);
        }
    }
    std::cout << "PGM width = " << this->header.width << " , height = " << this->header.height << " , gary = " << this->header.max_gray << std::endl;
    // read data #element 1 byte size
    this->pdata = (char *)malloc(sizeof(char) * this->header.width * this->header.height);
    for (unsigned int i = 0; i < this->header.height; i++)
    {
        ifs.read((this->pdata + i * this->header.width), this->header.width);
        // print test
        if (ifs.gcount() == 0)
        {
            // file end or encounter error
            break;
        }
    }
    std::cout << "map size = " << this->header.width * this->resolution << " x " << this->header.height * this->resolution << "m" << std::endl;
    ifs.close();
    return PGM_SUCCESS;
}

PGM_SPACE::PGM_STATE PGM_SPACE::PGM::PGM_handle(char obstacle, char no_obstacle, float obj_width, float expansion_value, char expansion_tag_value)
{
    if (this->pre_handle_pgm(obstacle, no_obstacle) == PGM_ERROR)
    {
        return PGM_ERROR;
    }
    if (this->PGM_handle_fill(obj_width) == PGM_ERROR)
    {
        return PGM_ERROR;
    }
    if (this->PGM_handle_expansion(expansion_value, expansion_tag_value) == PGM_ERROR)
    {
        return PGM_ERROR;
    }
    return PGM_SUCCESS;
}