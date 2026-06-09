#ifndef OCTI_XML__HPP
#define OCTI_XML__HPP

#include <iostream>
#include <string>
#include <math.h>
#include <thread>
#include <vector>
#include <mutex>
#include <atomic>
#include <algorithm>
#include <functional>
#include <SFML/Graphics.hpp>
#include <TGUI/TGUI.hpp>
#include <TGUI/Backend/SFML-Graphics.hpp>
#include "tinyxml/tinyxml2.h"
#define GLFW_INCLUDE_NONE // Don't let GLFW include an OpenGL extention loader
#include <GLFW/glfw3.h>

namespace OCTIXML
{
    enum KEYPONITTYPE
    {
        NORMAL = 0,
        VISION = 1,
        CHARGE = 2,
        WAIT = 3,
        INELEVATOR = 4,
        MAPSWITCH = 5,
    };
    struct pathNodeStruct
    {
        unsigned long sequence;
        double x;
        double y;
        double yaw;
        unsigned int keyPointType;
        unsigned int mapnum;
    };

    struct currentGoalStruct
    {
        unsigned long index;
        double x;
        double y;
        double yaw;
    };
};

class octiXml
{
private:
    std::thread *xmlUiRenderThreadHandler;
    std::atomic<bool> threadEndFlag;
    std::atomic<bool> pathRecordFlag;
    // std::atomic<bool> keyPathRecordFlag;
    std::atomic<bool> pathRecordCancelFlag;
    std::mutex DataAccessMutex;
    OCTIXML::pathNodeStruct pathData;
    unsigned long sequence;
    OCTIXML::KEYPONITTYPE pointType;
    std::mutex pointTypeAccessMutex;

private:
    void UiThreadFunc(void);
    void adjustTextSize(sf::Text &text, sf::RenderWindow &window, float scale);

private:
bool pathIndexCompare(OCTIXML::pathNodeStruct pathNode_1, OCTIXML::pathNodeStruct pathNode_2);
public:
    tinyxml2::XMLDocument *createXMLDocument(void);
    tinyxml2::XMLElement *createXMLRoot(tinyxml2::XMLDocument *doc);
    void insertPathNode(tinyxml2::XMLDocument *doc, tinyxml2::XMLElement *parentEle, OCTIXML::pathNodeStruct pathNode_);
    bool saveXmlFile(tinyxml2::XMLDocument *doc, const char *filePath);
    std::vector<OCTIXML::pathNodeStruct> readPath(const char *file_path);
    void updatePathData(OCTIXML::pathNodeStruct pathData_);
    bool getPathRecordFlag(void);
    void setPathRecordFlag(bool flag);
    // bool getKeyPathRecordFlag(void);
    // void setKeyPathRecordFlag(bool flag);
    void setSequence(unsigned long sequence_);
    OCTIXML::KEYPONITTYPE getPointType();

    bool getUiThreadEndFlag();
    bool getCancelFlag();
    bool startUiWork();
    bool endUiWork();
    void watingUiEnding();

    octiXml();
    ~octiXml();
};

bool octiXml::pathIndexCompare(OCTIXML::pathNodeStruct pathNode_1, OCTIXML::pathNodeStruct pathNode_2)
{
    return pathNode_1.sequence < pathNode_2.sequence;
}

#endif