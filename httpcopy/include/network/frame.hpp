#ifndef __FRAME_DATA__HPP
#define __FRAME_DATA__HPP

#define FRAME_MAX_SIZE (400 + 24u)

// 帧结构定义
struct frame {
    unsigned int source;
    unsigned int dest;
    unsigned int type;
    unsigned int len_data;
    unsigned int checksum;
    unsigned int reserve;
    char data[];
};

struct mydata {
    int seq;
    int eof;
    int sub_obj;
    int type;
};

// 帧类型定义 (frameType)
enum FRAME_TYPE {
    REQUEST = 0,          // 请求帧/命令
    RESPONSE,             // 数据上报，底下各部分上传到服务器
    COMNMAND,            // 控制命令，包含开启、关闭等各种控制命令
    INQUIRY,             // 状态查询
    BACK                 // 状态反馈
};

// 源地址定义 (source) / 目标地址定义 (dest)
// 与 goalsender 的 MODULE_TYPE 枚举保持一致
enum LOCATION {
    ROBOT_SYSTEM = 0,      // 机器人系统 (对应 ROBOT_OBJ)
    VISION_SYSTEM,         // 视觉系统 (对应 VISION_OBJ)
    HTTP_SERVER,           // HTTP服务器 (对应 HTTPSERVER_OBJ) = 2
    GAS_SYSTEM,            // 气体检测系统 (对应 GAS_OBJ)
    LADDAR_SYSTEM,         // 雷达系统 (对应 LADDAR_OBJ) = 4
    METER_SYSTEM,          // 仪表系统
    TEMP_SYSTEM,           // 温度检测系统
    VOICEACTOR_SYSTEM      // 语音串口系统
};

// 子对象定义 (subObj) - 视觉系统
enum SUBOBJ {
    FACE = 0,              // 人脸检测
    METER,                 // 仪表检测
    HARDHAT,               // 安全帽检测
    INVASION,              // 人员入侵检测
    FLAME,                 // 火焰检测
    SMOKE                  // 吸烟检测
};

// 子对象定义 (subObj) - 气体系统
#define SUBOBJ_YANWU1      0   // 烟雾1
#define SUBOBJ_YANWU2      1   // 烟雾2

// 子对象定义 (subObj) - 仪表系统
#define SUBOBJ_METER       1   // 仪表数据

// 子对象定义 (subObj) - 温度系统
#define SUBOBJ_TEMPERATURE 0   // 温度数据

// 子对象定义 (subObj) - 机器人系统
#define SUBOBJ_ROBOT       0   // 机器人数据
#define SUBOBJ_MAP         1   // 地图数据

// 子对象定义 (subObj) 控制数据开关
#define DATA_STOP          5   // 关闭
#define DATA_BEGIN         6   // 开启

// 数据标志定义 (hasData)
#define HAS_DATA_NODATA    0   // 无数据
#define HAS_DATA_OPEN      1   // 开启
#define HAS_DATA_CLOSE     2   // 关闭

// data1定义
#define VISION_DATA_CLOSE  0   // 关闭
#define VISION_DATA_OPEN   1   // 开启

#endif
