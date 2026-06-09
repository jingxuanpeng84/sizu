#ifndef __FRAME_DATA__HPP
#define __FRAME_DATA__HPP

#define FRAME_MAX_SIZE (3000 + 24u)

// ============================================
// 心跳帧类型定义
// ============================================
enum HEARTBEAT_TYPE{
    HEARTBEAT_STATUS = 0,           // 普通状态心跳
    HEARTBEAT_REQUEST_RESTART = 1,  // 请求重启
    HEARTBEAT_COMMAND_RESTART = 2   // 重启命令
};

// ============================================
// 帧类型定义（frame.type）
// ============================================
enum FRAME_TYPE{
    FRAME_TYPE_DATA = 0,        // 普通数据帧（已废弃，建议使用下面的明确类型）
    FRAME_TYPE_HEARTBEAT = 1,   // 心跳帧
    FRAME_TYPE_CONTROL = 2,     // 控制帧
    
    // 新增：明确的请求/响应类型
    FRAME_TYPE_REQUEST = 3,     // 请求帧（Server → Vision：请求开始检测）
    FRAME_TYPE_RESPONSE = 4,    // 响应帧（Vision → Server：上报检测结果）
    FRAME_TYPE_ACK = 5          // 应答帧（确认收到）
};

// ============================================
// 帧头结构（24字节）
// ============================================
struct frame {
    // head 24B
    unsigned int source;    // 源模块对象（MODULE_TYPE）
    unsigned int dest;      // 目的模块对象（MODULE_TYPE）
    unsigned int type;      // 帧类型（FRAME_TYPE）
    unsigned int len_data;  // 数据部分的长度，max_len = 400B
    unsigned int checksum;  // 校验和：偶校验
    unsigned int reserve;   // 保留填充
    // data
    char data[];
};

// ============================================
// 数据帧负载结构
// ============================================
struct mydata {
    int seq;        // 序列号（用于数据包排序）
    int eof;        // 结束标志/计数（Vision→Server时表示连续异常帧数）
    int sub_obj;    // 子任务类型（TASK_HARDHAT, TASK_FLAME等）
    int type;       // 状态类型（STATE_NORMAL=0, STATE_ABNORMAL=1）
    char data[];    // 扩展数据
};

// ============================================
// 心跳帧负载结构
// ============================================
struct heartdancefra {
    int type;       // 心跳类型（HEARTBEAT_TYPE）
    int status;     // 状态码（0=正常，<0=错误，>0=警告）
    int len;        // info字段长度
    char info[];    // 附加信息（如错误描述、重启原因等）
};

#endif