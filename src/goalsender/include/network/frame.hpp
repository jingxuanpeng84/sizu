#ifndef __FRAME_DATA__HPP
#define __FRAME_DATA__HPP

#define FRAME_MAX_SIZE (400 + 24u)
enum HEARTBEAT_TYPE{
    HEARTBEAT_STATUS = 0, //普通状态心跳
    HEARTBEAT_REQUEST_RESTART = 1,//请求重启
    HEARTBEAT_COMMAND_RESTART = 2
};
enum FRAME_TYPE{
    FRAME_TYPE_DATA = 0,//普通数据帧
    FRAME_TYPE_HEARTBEAT = 1,//心跳帧
    FRAME_TYPE_CONTROL = 2  //控制帧
};
struct frame {
    // head 24B
    unsigned int source;    // 源模块对象
    unsigned int dest;      // 目的模块对象
    unsigned int type;      // req请求数据, push发送数据，heartbeat心跳帧
    unsigned int len_data;  // 数据部分的长度，max_len = 400B
    unsigned int checksum;  // 校验和：偶校验
    unsigned int reserve;   // 保留填充
    // data
    char data[];
};

// 示例char data[];实现
struct mydata {
    int seq;        //
    int eof;        // 数据结尾
    int sub_obj;    // 子对象
    int type;       // data, ack
    char data[];
};

struct heartdancefra {
    int type; // 类型为心跳帧
    int status; 
    int len;
    char info[];
};

#endif