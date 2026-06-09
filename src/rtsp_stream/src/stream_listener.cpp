/**
 * @file stream_listener_standalone.cpp
 * @brief RTSP 推流监听程序（独立版本，不依赖 ROS）
 * 
 * 功能：
 * - 监听 UDP 端口接收控制指令
 * - 启动/停止 ROS 推流节点
 * - 可作为系统服务开机自启动
 * - 不依赖 ROS 环境
 */

#include <iostream>
#include <string>
#include <thread>
#include <mutex>
#include <atomic>
#include <csignal>
#include <cstring>
#include <ctime>
#include <fstream>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <sys/wait.h>
#include <sys/stat.h>

// 配置参数
struct Config {
    int listen_port;
    std::string launch_command;
    std::string log_file;
    bool enable_log;
    
    Config() : listen_port(8113),
               launch_command("bash -c 'source /opt/ros/melodic/setup.bash && source ~/slam_ws/devel/setup.bash && roslaunch rtsp_stream map_publisher.launch'"),
               log_file("/var/log/rtsp_stream/listener.log"),
               enable_log(true) {}
};

Config g_config;
std::atomic<bool> g_running(true);
int g_sockfd = -1;
pid_t g_stream_pid = -1;
std::mutex g_pid_mutex;
std::mutex g_log_mutex;

// 请求帧结构
struct RequestFrame {
    uint32_t command;  // 1=启动, 2=停止, 3=查询
    uint32_t seq;
};

// 应答帧结构
struct ReplyFrame {
    bool result;
    uint32_t seq;
    uint32_t status;  // 0=停止, 1=运行
};

/**
 * @brief 日志函数
 */
void log_message(const std::string& level, const std::string& message) {
    // 控制台输出
    std::time_t now = std::time(nullptr);
    char time_buf[64];
    std::strftime(time_buf, sizeof(time_buf), "%Y-%m-%d %H:%M:%S", std::localtime(&now));
    
    std::cout << "[" << time_buf << "] [" << level << "] " << message << std::endl;
    
    // 文件输出
    if (g_config.enable_log) {
        std::lock_guard<std::mutex> lock(g_log_mutex);
        std::ofstream log_file(g_config.log_file, std::ios::app);
        if (log_file.is_open()) {
            log_file << "[" << time_buf << "] [" << level << "] " << message << std::endl;
            log_file.close();
        }
    }
}

#define LOG_INFO(msg) log_message("INFO", msg)
#define LOG_WARN(msg) log_message("WARN", msg)
#define LOG_ERROR(msg) log_message("ERROR", msg)

/**
 * @brief 信号处理
 */
void signalHandler(int signum) {
    LOG_INFO("Received signal " + std::to_string(signum) + ", shutting down...");
    g_running = false;
    
    if (g_sockfd >= 0) {
        close(g_sockfd);
        g_sockfd = -1;
    }
    
    // 停止推流进程
    std::lock_guard<std::mutex> lock(g_pid_mutex);
    if (g_stream_pid > 0) {
        LOG_INFO("Stopping stream process (PID: " + std::to_string(g_stream_pid) + ")");
        kill(g_stream_pid, SIGTERM);
        g_stream_pid = -1;
    }
}

/**
 * @brief 创建日志目录
 */
void createLogDirectory() {
    std::string log_dir = g_config.log_file.substr(0, g_config.log_file.find_last_of('/'));
    
    struct stat st;
    if (stat(log_dir.c_str(), &st) != 0) {
        // 目录不存在，创建它
        std::string cmd = "mkdir -p " + log_dir;
        system(cmd.c_str());
        LOG_INFO("Created log directory: " + log_dir);
    }
}

/**
 * @brief 创建并绑定 UDP socket
 */
int setupUdpSocket(int port) {
    int sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    if (sockfd < 0) {
        LOG_ERROR("Failed to create socket: " + std::string(strerror(errno)));
        return -1;
    }
    
    // 设置地址重用
    int opt = 1;
    setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
    
    // 绑定地址
    struct sockaddr_in server_addr;
    memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = INADDR_ANY;
    server_addr.sin_port = htons(port);
    
    if (bind(sockfd, (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0) {
        LOG_ERROR("Failed to bind socket: " + std::string(strerror(errno)));
        close(sockfd);
        return -1;
    }
    
    LOG_INFO("UDP socket bound to port " + std::to_string(port));
    return sockfd;
}

/**
 * @brief 发送应答
 */
void sendReply(int sockfd, const ReplyFrame& reply, const struct sockaddr_in& client_addr) {
    ssize_t sent = sendto(sockfd, &reply, sizeof(reply), 0,
                          (struct sockaddr*)&client_addr, sizeof(client_addr));
    if (sent < 0) {
        LOG_ERROR("Failed to send reply: " + std::string(strerror(errno)));
    } else {
        char client_ip[INET_ADDRSTRLEN];
        inet_ntop(AF_INET, &client_addr.sin_addr, client_ip, INET_ADDRSTRLEN);
        LOG_INFO("Sent reply to " + std::string(client_ip) + ":" + 
                 std::to_string(ntohs(client_addr.sin_port)) +
                 " (seq: " + std::to_string(reply.seq) + 
                 ", result: " + std::to_string(reply.result) + 
                 ", status: " + std::to_string(reply.status) + ")");
    }
}

/**
 * @brief 启动推流进程
 */
bool startStreamProcess() {
    std::lock_guard<std::mutex> lock(g_pid_mutex);
    
    if (g_stream_pid > 0) {
        LOG_WARN("Stream process already running (PID: " + std::to_string(g_stream_pid) + ")");
        return true;
    }
    
    LOG_INFO("Starting stream process: " + g_config.launch_command);
    
    pid_t pid = fork();
    if (pid == 0) {
        // 子进程：执行 launch 命令
        // 重定向输出到日志文件
        if (g_config.enable_log) {
            std::string stream_log = g_config.log_file + ".stream";
            freopen(stream_log.c_str(), "a", stdout);
            freopen(stream_log.c_str(), "a", stderr);
        }
        
        execlp("bash", "bash", "-c", g_config.launch_command.c_str(), nullptr);
        // 如果 exec 失败
        LOG_ERROR("Failed to execute launch command");
        exit(1);
    } else if (pid > 0) {
        g_stream_pid = pid;
        LOG_INFO("Stream process started (PID: " + std::to_string(g_stream_pid) + ")");
        return true;
    } else {
        LOG_ERROR("Failed to fork process: " + std::string(strerror(errno)));
        return false;
    }
}

/**
 * @brief 停止推流进程
 */
bool stopStreamProcess() {
    std::lock_guard<std::mutex> lock(g_pid_mutex);
    
    if (g_stream_pid <= 0) {
        LOG_WARN("No stream process running");
        return true;
    }
    
    LOG_INFO("Stopping stream process (PID: " + std::to_string(g_stream_pid) + ")");
    
    // 发送 SIGTERM 信号
    if (kill(g_stream_pid, SIGTERM) == 0) {
        // 等待进程结束（最多 5 秒）
        for (int i = 0; i < 50; i++) {
            if (kill(g_stream_pid, 0) != 0) {
                // 进程已结束
                g_stream_pid = -1;
                LOG_INFO("Stream process stopped successfully");
                return true;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        
        // 超时，强制杀死
        LOG_WARN("Process did not stop gracefully, sending SIGKILL");
        kill(g_stream_pid, SIGKILL);
        g_stream_pid = -1;
        return true;
    } else {
        LOG_ERROR("Failed to stop process: " + std::string(strerror(errno)));
        return false;
    }
}

/**
 * @brief 获取推流状态
 */
bool getStreamStatus() {
    std::lock_guard<std::mutex> lock(g_pid_mutex);
    
    if (g_stream_pid <= 0) {
        return false;
    }
    
    // 检查进程是否还在运行
    if (kill(g_stream_pid, 0) == 0) {
        return true;
    } else {
        // 进程已结束
        g_stream_pid = -1;
        return false;
    }
}

/**
 * @brief UDP 接收循环
 */
void udpReceiveLoop() {
    char buffer[1024];
    struct sockaddr_in client_addr;
    socklen_t client_len = sizeof(client_addr);
    
    LOG_INFO("UDP receive loop started, listening on port " + std::to_string(g_config.listen_port));
    
    while (g_running) {
        // 设置接收超时
        struct timeval tv;
        tv.tv_sec = 1;
        tv.tv_usec = 0;
        setsockopt(g_sockfd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
        
        ssize_t recv_len = recvfrom(g_sockfd, buffer, sizeof(buffer), 0,
                                     (struct sockaddr*)&client_addr, &client_len);
        
        if (recv_len < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                // 超时，继续循环
                continue;
            }
            LOG_ERROR("recvfrom error: " + std::string(strerror(errno)));
            break;
        }
        
        if (recv_len < static_cast<ssize_t>(sizeof(RequestFrame))) {
            LOG_WARN("Received incomplete frame (" + std::to_string(recv_len) + " bytes)");
            continue;
        }
        
        // 解析请求
        RequestFrame* request = reinterpret_cast<RequestFrame*>(buffer);
        
        char client_ip[INET_ADDRSTRLEN];
        inet_ntop(AF_INET, &client_addr.sin_addr, client_ip, INET_ADDRSTRLEN);
        
        LOG_INFO("========================================");
        LOG_INFO("Received request from " + std::string(client_ip) + ":" + 
                 std::to_string(ntohs(client_addr.sin_port)));
        LOG_INFO("  Command: " + std::to_string(request->command));
        LOG_INFO("  Sequence: " + std::to_string(request->seq));
        
        // 处理命令
        ReplyFrame reply;
        reply.seq = request->seq;
        reply.result = false;
        reply.status = getStreamStatus() ? 1 : 0;
        
        switch (request->command) {
            case 1:  // 启动推流
                LOG_INFO("Command: START STREAM");
                reply.result = startStreamProcess();
                reply.status = 1;
                break;
                
            case 2:  // 停止推流
                LOG_INFO("Command: STOP STREAM");
                reply.result = stopStreamProcess();
                reply.status = 0;
                break;
                
            case 3:  // 查询状态
                LOG_INFO("Command: QUERY STATUS");
                reply.result = true;
                reply.status = getStreamStatus() ? 1 : 0;
                LOG_INFO("  Status: " + std::string(reply.status ? "RUNNING" : "STOPPED"));
                break;
                
            default:
                LOG_WARN("Unknown command: " + std::to_string(request->command));
                reply.result = false;
                break;
        }
        
        // 发送应答
        sendReply(g_sockfd, reply, client_addr);
        LOG_INFO("========================================");
    }
    
    LOG_INFO("UDP receive loop stopped");
}

/**
 * @brief 打印使用说明
 */
void printUsage(const char* program_name) {
    std::cout << "RTSP Stream Listener (Standalone Version)" << std::endl;
    std::cout << std::endl;
    std::cout << "Usage: " << program_name << " [options]" << std::endl;
    std::cout << std::endl;
    std::cout << "Options:" << std::endl;
    std::cout << "  -p PORT        Listen port (default: 8113)" << std::endl;
    std::cout << "  -c COMMAND     Launch command" << std::endl;
    std::cout << "  -l LOGFILE     Log file path" << std::endl;
    std::cout << "  --no-log       Disable file logging" << std::endl;
    std::cout << "  -h, --help     Show this help" << std::endl;
    std::cout << std::endl;
    std::cout << "Example:" << std::endl;
    std::cout << "  " << program_name << " -p 8113" << std::endl;
    std::cout << std::endl;
}

/**
 * @brief 解析命令行参数
 */
bool parseArguments(int argc, char** argv) {
    for (int i = 1; i < argc; i++) {
        std::string arg = argv[i];
        
        if (arg == "-h" || arg == "--help") {
            printUsage(argv[0]);
            return false;
        } else if (arg == "-p" && i + 1 < argc) {
            g_config.listen_port = std::stoi(argv[++i]);
        } else if (arg == "-c" && i + 1 < argc) {
            g_config.launch_command = argv[++i];
        } else if (arg == "-l" && i + 1 < argc) {
            g_config.log_file = argv[++i];
        } else if (arg == "--no-log") {
            g_config.enable_log = false;
        } else {
            std::cerr << "Unknown option: " << arg << std::endl;
            printUsage(argv[0]);
            return false;
        }
    }
    
    return true;
}

int main(int argc, char** argv) {
    // 解析命令行参数
    if (!parseArguments(argc, argv)) {
        return 0;
    }
    
    // 创建日志目录
    if (g_config.enable_log) {
        createLogDirectory();
    }
    
    LOG_INFO("========================================");
    LOG_INFO("Stream Listener (Standalone) Started");
    LOG_INFO("========================================");
    LOG_INFO("Listen Port: " + std::to_string(g_config.listen_port));
    LOG_INFO("Launch Command: " + g_config.launch_command);
    if (g_config.enable_log) {
        LOG_INFO("Log File: " + g_config.log_file);
    }
    LOG_INFO("========================================");
    LOG_INFO("Commands:");
    LOG_INFO("  1 = Start Stream");
    LOG_INFO("  2 = Stop Stream");
    LOG_INFO("  3 = Query Status");
    LOG_INFO("========================================");
    
    // 注册信号处理
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);
    
    // 创建 UDP socket
    g_sockfd = setupUdpSocket(g_config.listen_port);
    if (g_sockfd < 0) {
        LOG_ERROR("Failed to setup UDP socket");
        return -1;
    }
    
    LOG_INFO("Waiting for commands...");
    
    // 进入接收循环
    udpReceiveLoop();
    
    // 清理
    if (g_sockfd >= 0) {
        close(g_sockfd);
    }
    
    LOG_INFO("Program shutdown complete");
    
    return 0;
}
