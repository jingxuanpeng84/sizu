#include "path_control/include/octi_xml.hpp"
#include "path_control/include/laddar_device.hpp"
#include "path_control/include/octi_yaml.hpp"

int main()
{
    std::cout << "Loading config.yaml\n";
    octi_yaml yaml("/home/orangepi/Desktop/work_station/OctiRobotVersion2/path_control/target/config/config.yaml");
    std::string laddarIp = "nan";
    uint32_t laddarPort = 0;

    std::string myIp = "nan";
    uint32_t myPort = 0;

    std::string reversalPathXmlPath = "nan";

    // load reversalPathXmlPath
    if (yaml.IsDefine("reversalPathXmlPath") == false)
    {
        perror("target/config/config.yaml: Please define reversalPathXmlPath !");
        exit(1);
    }
    reversalPathXmlPath = yaml.getString("reversalPathXmlPath");
    std::cout << "reversalPathXmlPath = " << reversalPathXmlPath << std::endl;

    // load laddar
    if (yaml.IsDefine("laddarIp") == false || yaml.IsDefine("laddarPort") == false)
    {
        perror("target/config/config.yaml: Please define laddarIp / laddarPort, when need load laddar !");
        exit(1);
    }
    laddarIp = yaml.getString("laddarIp");
    laddarPort = yaml.getUint("laddarPort");
    std::cout << "laddarIp = " << laddarIp << " laddarPort = " << laddarPort << std::endl;

    // load my ip and my port
    if (yaml.IsDefine("myIp") == false || yaml.IsDefine("myPort") == false)
    {
        perror("target/config/config.yaml: Please define myIp / myPort !");
        exit(1);
    }
    myIp = yaml.getString("myIp");
    myPort = yaml.getUint("myPort");
    std::cout << "myIp = " << myIp << " myPort = " << myPort << std::endl;

    octiXml myxml;
    tinyxml2::XMLDocument *doc = myxml.createXMLDocument();
    tinyxml2::XMLElement *root = myxml.createXMLRoot(doc);

    //
    std::vector<OCTIXML::pathNodeStruct> pathVector = myxml.readPath(reversalPathXmlPath.c_str());
    if (pathVector.size() != 0)
    {
        for (auto pathNode : pathVector)
        {
            std::cout << "Seq = " << pathNode.sequence << " x = " << pathNode.x << " y = " << pathNode.y << " yaw = " << pathNode.yaw << " keypoint = " << pathNode.keyPointType << std::endl;
        }
    }
    //

    OCTIXML::pathNodeStruct pathNode;
    unsigned long sequence = 0;

    deviceBase *laddar = new laddarDevice(laddarIp.c_str(), laddarPort, myIp.c_str(), myPort);
    if (laddar->startWork() == false)
    {
        if (laddar != NULL)
        {
            delete laddar;
        }
        perror("laddar start error");
        return 0;
    }

    myxml.startUiWork();
    LADDAR::dataStruct laddarData;
    unsigned int dataSize;
    while (true)
    {
        if (laddar->readDeviceData(&laddarData, dataSize) != true)
        {
            std::cout << "Laddar may disconnect! Please check!\n";
            laddar->endWork();
            myxml.endUiWork();
            break;
        }
        else
        {
            pathNode.sequence = sequence;
            pathNode.x = laddarData.x;
            pathNode.y = laddarData.y;
            pathNode.yaw = laddarData.yaw;
            myxml.updatePathData(pathNode);
            if (myxml.getPathRecordFlag())
            {
                sequence++;
                pathNode.sequence = sequence;
                pathNode.keyPointType = myxml.getPointType();
                myxml.insertPathNode(doc, root, pathNode);
                myxml.setPathRecordFlag(false);
            }
            if (myxml.getUiThreadEndFlag())
            {
                if (sequence != 0)
                {
                    myxml.saveXmlFile(doc, reversalPathXmlPath.c_str());
                }
                break;
            }
            if (myxml.getCancelFlag())
            {
                // cancel record
                std::cout << "Record Path Cancel !\n";
                break;
            }
        }
    }
    laddar->endWork();
    laddar->watingDeviceEnding();

    myxml.watingUiEnding();
    delete laddar;
    return 0;
}