#include "path_control/include/octi_yaml.hpp"

int main()
{
    octi_yaml yaml("/home/dog/桌面/unitree/OctiRobotVersion2/path_control/target/config/config.yaml");
    std::string robotNetInterface = "nan";
    std::string reversalPathXmlPath = "nan";
    std::string currentGoalIndexXmlPath = "nan";

    std::string laddarIp = "nan";
    uint32_t laddarPort = 0;
    uint32_t laddarLoad = 0;

    std::string visionIp = "nan";
    uint32_t visionPort = 0;
    uint32_t visionLoad = 0;

    std::string myIp = "nan";
    uint32_t myPort = 0;

    std::string speakerIp = "nan";
    uint32_t speakerPort = 0;
    uint32_t speakerLoad = 0;

    std::string localPlanner = "nan";
    double sampleDetaTime;
    double vMin;
    double vMax;
    double wMin;
    double wMax;
    double aVMax;
    double aWMax;
    double vSampleDetaV;
    double wSampleDetaW;
    double predictTime;
    double alpha;
    double beta;
    double gamma;
    double keyPointRadius;
    
    // load robotNetInterface
    if (yaml.IsDefine("robotNetInterface") == false)
    {
        perror("target/config/config.yaml: Please define robotNetInterface !");
        exit(1);
    }
    robotNetInterface = yaml.getString("robotNetInterface");
    std::cout << "robotNetInterface = " << robotNetInterface << std::endl;

    // load reversalPathXmlPath
    if (yaml.IsDefine("reversalPathXmlPath") == false)
    {
        perror("target/config/config.yaml: Please define reversalPathXmlPath !");
        exit(1);
    }
    reversalPathXmlPath = yaml.getString("reversalPathXmlPath");
    std::cout << "reversalPathXmlPath = " << reversalPathXmlPath << std::endl;

    // load currentGoalIndexRecord
    if (yaml.IsDefine("currentGoalIndexRecordLoad") == true)
    {
        if (yaml.getUint("currentGoalIndexRecordLoad") == 1)
        {
            if (yaml.IsDefine("currentGoalIndexXmlPath") == false)
            {
                perror("target/config/config.yaml: Please define currentGoalIndexXmlPath, when need load currentGoalIndexRecord !");
                exit(1);
            }
            currentGoalIndexXmlPath = yaml.getString("currentGoalIndexXmlPath");
            std::cout << "currentGoalIndexXmlPath = " << currentGoalIndexXmlPath << std::endl;
        }
    }

    // load laddar
    if (yaml.IsDefine("laddarLoad") == true)
    {
        if (yaml.getUint("laddarLoad") == 1)
        {
            if (yaml.IsDefine("laddarIp") == false || yaml.IsDefine("laddarPort") == false)
            {
                perror("target/config/config.yaml: Please define laddarIp / laddarPort, when need load laddar !");
                exit(1);
            }
            laddarLoad = 1;
            laddarIp = yaml.getString("laddarIp");
            laddarPort = yaml.getUint("laddarPort");
            std::cout << "laddarIp = " << laddarIp << " laddarPort = " << laddarPort << std::endl;
        }
    }

    // load vision
    if (yaml.IsDefine("visionLoad") == true)
    {
        if (yaml.getUint("visionLoad") == 1)
        {
            if (yaml.IsDefine("visionIp") == false || yaml.IsDefine("visionPort") == false)
            {
                perror("target/config/config.yaml: Please define visionIp / visionPort, when need load vision !");
                exit(1);
            }
            visionLoad = 1;
            visionIp = yaml.getString("visionIp");
            visionPort = yaml.getUint("visionPort");
            std::cout << "visionIp = " << visionIp << " visionPort = " << visionPort << std::endl;
        }
    }

    // load my ip and my port
    if (yaml.IsDefine("myIp") == false || yaml.IsDefine("myPort") == false)
    {
        perror("target/config/config.yaml: Please define myIp / myPort !");
        exit(1);
    }
    myIp = yaml.getString("myIp");
    myPort = yaml.getUint("myPort");
    std::cout << "myIp = " << myIp << " myPort = " << myPort << std::endl;

    // load speaker
    if (yaml.IsDefine("speakerLoad") == true)
    {
        if (yaml.getUint("speakerLoad") == 1)
        {
            if (yaml.IsDefine("speakerIp") == false || yaml.IsDefine("speakerPort") == false)
            {
                perror("target/config/config.yaml: Please define speakerIp / speakerPort, when need load speaker !");
                exit(1);
            }
            speakerLoad = 1;
            laddarIp = yaml.getString("speakerIp");
            laddarPort = yaml.getUint("speakerPort");
            std::cout << "speakerIp = " << speakerIp << " speakerPort = " << speakerPort << std::endl;
        }
    }

    // local planner
    if (yaml.IsDefine("localPlanner") == false)
    {
        perror("target/config/config.yaml: Please define localPlanner !");
        exit(1);
    }
    localPlanner = yaml.getString("localPlanner");
    std::cout << "localPlanner = " << localPlanner << std::endl;
    if (localPlanner == "DWA")
    {
        if (yaml.IsDefine("sampleDetaTime") == false ||
            yaml.IsDefine("vMin") == false ||
            yaml.IsDefine("vMax") == false ||
            yaml.IsDefine("wMin") == false ||
            yaml.IsDefine("wMax") == false ||
            yaml.IsDefine("aVMax") == false ||
            yaml.IsDefine("aWMax") == false ||
            yaml.IsDefine("vSampleDetaV") == false ||
            yaml.IsDefine("wSampleDetaW") == false ||
            yaml.IsDefine("predictTime") == false ||
            yaml.IsDefine("alpha") == false ||
            yaml.IsDefine("beta") == false ||
            yaml.IsDefine("gamma") == false ||
            yaml.IsDefine("keyPointRadius") == false)
        {
            perror("target/config/config.yaml: Please define DWA parameters when need load DWA local planner !");
            exit(1);
        }
        sampleDetaTime = yaml.getDouble("sampleDetaTime");
        vMin = yaml.getDouble("vMin");
        vMax = yaml.getDouble("vMax");
        wMin = yaml.getDouble("wMin");
        wMax = yaml.getDouble("wMax");
        aVMax = yaml.getDouble("aVMax");
        aWMax = yaml.getDouble("aWMax");
        vSampleDetaV = yaml.getDouble("vSampleDetaV");
        wSampleDetaW = yaml.getDouble("wSampleDetaW");
        predictTime = yaml.getDouble("predictTime");
        alpha = yaml.getDouble("alpha");
        beta = yaml.getDouble("beta");
        gamma = yaml.getDouble("gamma");
        keyPointRadius = yaml.getDouble("keyPointRadius");
        std::cout 
        << "DWA parameters : -> "
        << " sampleDetaTime = " << sampleDetaTime 
        << " vMin = " << vMin 
        << " vMax = " << vMax 
        << " wMin = " << wMin 
        << " wMax = " << wMax 
        << " aVMax = " << aVMax 
        << " aWMax = " << aWMax 
        << " vSampleDetaV = " << vSampleDetaV 
        << " wSampleDetaW = " << wSampleDetaW 
        << " predictTime = " << predictTime 
        << " alpha = " << alpha 
        << " beta = " << beta 
        << " gamma = " << gamma 
        << " keyPointRadius = " << keyPointRadius << std::endl;
    }

    return 0;
}