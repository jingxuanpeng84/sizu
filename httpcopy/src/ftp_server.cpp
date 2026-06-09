#include "ftp_server.h"
#include <iostream>
#include <fstream>
#include <thread>
#include <mutex>
#include <vector>
#include <cstring>
#include <cerrno>
#include <cstdlib>
#include <sys/types.h>
#include <sys/stat.h>
#include <dirent.h>
#include <sstream>
#include <iomanip>

#ifdef _WIN32
    #include <winsock2.h>
    #include <ws2tcpip.h>
    #include <direct.h>
    #pragma comment(lib, "ws2_32.lib")
    #define mkdir(path, mode) _mkdir(path)
    #define close closesocket
    typedef int socklen_t;
#else
    #include <unistd.h>
    #include <arpa/inet.h>
    #include <sys/socket.h>
    #include <netinet/in.h>
#endif

// FTP响应码
#define FTP_READY           "220 FTP Server Ready\r\n"
#define FTP_USER_OK         "331 User name okay, need password\r\n"
#define FTP_LOGIN_OK        "230 User logged in\r\n"
#define FTP_TYPE_OK         "200 Type set to I\r\n"
#define FTP_TYPE_A_OK       "200 Type set to A\r\n"
#define FTP_PWD_REPLY       "257 \"/\" is current directory\r\n"
#define FTP_PASV_MODE       "227 Entering Passive Mode (%d,%d,%d,%d,%d,%d)\r\n"
#define FTP_TRANSFER_START  "150 Opening BINARY mode data connection\r\n"
#define FTP_LIST_START      "150 Opening ASCII mode data connection for file list\r\n"
#define FTP_TRANSFER_OK     "226 Transfer complete\r\n"
#define FTP_GOODBYE         "221 Goodbye\r\n"
#define FTP_CMD_OK          "200 Command okay\r\n"

class FTPServer::Impl {
public:
    FTPConfig config;
    int server_socket = -1;
    bool running = false;
    std::thread server_thread;
    FileReceivedCallback file_callback;
    std::vector<std::string> received_images;
    std::mutex images_mutex;
    
    Impl(const FTPConfig& cfg) : config(cfg) {
        // 创建目录
        mkdir(config.root_dir.c_str(), 0755);
        mkdir(config.image_dir.c_str(), 0755);
    }
    
    ~Impl() {
        stop();
    }
    
    bool init_socket() {
#ifdef _WIN32
        WSADATA wsa;
        if (WSAStartup(MAKEWORD(2, 2), &wsa) != 0) {
            std::cerr << "WSAStartup failed" << std::endl;
            return false;
        }
#endif
        
        server_socket = socket(AF_INET, SOCK_STREAM, 0);
        if (server_socket < 0) {
            std::cerr << "Failed to create FTP socket" << std::endl;
            return false;
        }
        
        int opt = 1;
        setsockopt(server_socket, SOL_SOCKET, SO_REUSEADDR, (char*)&opt, sizeof(opt));
        
        struct sockaddr_in addr;
        memset(&addr, 0, sizeof(addr));
        addr.sin_family = AF_INET;
        addr.sin_port = htons(config.port);
        addr.sin_addr.s_addr = INADDR_ANY;
        
        if (bind(server_socket, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
            std::cerr << "Failed to bind FTP socket to port " << config.port << std::endl;
            close(server_socket);
            return false;
        }
        
        if (listen(server_socket, 5) < 0) {
            std::cerr << "Failed to listen on FTP socket" << std::endl;
            close(server_socket);
            return false;
        }
        
        return true;
    }
    
    void handle_client(int client_sock) {
        char buffer[4096];
        int data_listen_socket = -1;  // 监听socket
        int data_socket = -1;          // 数据连接socket
        int data_port = 0;
        std::string filename;
        
        // 发送欢迎消息
        send(client_sock, FTP_READY, strlen(FTP_READY), 0);
        
        while (running) {
            memset(buffer, 0, sizeof(buffer));
            int n = recv(client_sock, buffer, sizeof(buffer) - 1, 0);
            if (n <= 0) break;
            
            std::string cmd(buffer);
            // 移除换行符
            while (!cmd.empty() && (cmd.back() == '\r' || cmd.back() == '\n')) {
                cmd.pop_back();
            }
            
            std::cout << "FTP命令: " << cmd << std::endl;
            
            // 解析命令
            if (cmd.substr(0, 4) == "USER") {
                send(client_sock, FTP_USER_OK, strlen(FTP_USER_OK), 0);
            }
            else if (cmd.substr(0, 4) == "PASS") {
                send(client_sock, FTP_LOGIN_OK, strlen(FTP_LOGIN_OK), 0);
            }
            else if (cmd.substr(0, 4) == "TYPE") {
                if (cmd.find('I') != std::string::npos) {
                    send(client_sock, FTP_TYPE_OK, strlen(FTP_TYPE_OK), 0);
                } else if (cmd.find('A') != std::string::npos) {
                    send(client_sock, FTP_TYPE_A_OK, strlen(FTP_TYPE_A_OK), 0);
                } else {
                    send(client_sock, FTP_TYPE_OK, strlen(FTP_TYPE_OK), 0);
                }
            }
            else if (cmd.substr(0, 3) == "PWD") {
                send(client_sock, FTP_PWD_REPLY, strlen(FTP_PWD_REPLY), 0);
            }
            else if (cmd.substr(0, 4) == "PASV") {
                // 被动模式：创建监听socket（不阻塞）
                std::cout << "收到 PASV 命令，创建数据端口..." << std::endl;
                
                // 关闭之前的监听socket
                if (data_listen_socket >= 0) {
                    close(data_listen_socket);
                    data_listen_socket = -1;
                }
                
                // 获取服务器IP（从客户端连接的本地地址获取）
                struct sockaddr_in server_addr;
                socklen_t addr_len = sizeof(server_addr);
                if (getsockname(client_sock, (struct sockaddr*)&server_addr, &addr_len) < 0) {
                    std::cerr << "无法获取服务器地址" << std::endl;
                    const char* err = "425 Can't get server address\r\n";
                    send(client_sock, err, strlen(err), 0);
                } else {
                    unsigned char* ip = (unsigned char*)&server_addr.sin_addr.s_addr;
                    std::cout << "服务器地址: " << (int)ip[0] << "." << (int)ip[1] << "." 
                              << (int)ip[2] << "." << (int)ip[3] << std::endl;
                    
                    // 创建监听socket（不阻塞）
                    data_listen_socket = create_listen_socket(data_port);
                    
                    if (data_listen_socket >= 0) {
                        char response[256];
                        int p1 = data_port / 256;
                        int p2 = data_port % 256;
                        
                        snprintf(response, sizeof(response), FTP_PASV_MODE, 
                                 ip[0], ip[1], ip[2], ip[3], p1, p2);
                        send(client_sock, response, strlen(response), 0);
                        std::cout << "PASV 响应已发送: " << (int)ip[0] << "." << (int)ip[1] << "." 
                                  << (int)ip[2] << "." << (int)ip[3] << ":" << data_port << std::endl;
                    } else {
                        std::cerr << "创建数据端口失败" << std::endl;
                        const char* err = "425 Can't open data connection\r\n";
                        send(client_sock, err, strlen(err), 0);
                    }
                }
            }
            else if (cmd.substr(0, 4) == "STOR") {
                // 存储文件
                filename = cmd.substr(5);
                std::cout << "收到 STOR 命令，文件名: " << filename << std::endl;
                
                if (data_listen_socket >= 0) {
                    send(client_sock, FTP_TRANSFER_START, strlen(FTP_TRANSFER_START), 0);
                    
                    // 现在才 accept 数据连接
                    std::cout << "等待客户端连接数据端口..." << std::endl;
                    data_socket = accept_data_connection(data_listen_socket);
                    close(data_listen_socket);
                    data_listen_socket = -1;
                    
                    if (data_socket >= 0) {
                        std::cout << "数据连接已建立，开始接收文件..." << std::endl;
                        receive_file(data_socket, filename);
                        close(data_socket);
                        data_socket = -1;
                        send(client_sock, FTP_TRANSFER_OK, strlen(FTP_TRANSFER_OK), 0);
                    } else {
                        std::cerr << "数据连接建立失败" << std::endl;
                        const char* err = "425 Can't open data connection\r\n";
                        send(client_sock, err, strlen(err), 0);
                    }
                } else {
                    std::cerr << "没有数据连接（需要先执行 PASV）" << std::endl;
                    const char* err = "425 Use PASV first\r\n";
                    send(client_sock, err, strlen(err), 0);
                }
            }
            else if (cmd.substr(0, 4) == "LIST") {
                // 列出文件
                std::cout << "收到 LIST 命令" << std::endl;
                
                if (data_listen_socket >= 0) {
                    send(client_sock, FTP_LIST_START, strlen(FTP_LIST_START), 0);
                    
                    // accept 数据连接
                    std::cout << "等待客户端连接数据端口..." << std::endl;
                    data_socket = accept_data_connection(data_listen_socket);
                    close(data_listen_socket);
                    data_listen_socket = -1;
                    
                    if (data_socket >= 0) {
                        std::cout << "数据连接已建立，发送文件列表..." << std::endl;
                        send_file_list(data_socket);
                        close(data_socket);
                        data_socket = -1;
                        send(client_sock, FTP_TRANSFER_OK, strlen(FTP_TRANSFER_OK), 0);
                    } else {
                        std::cerr << "数据连接建立失败" << std::endl;
                        const char* err = "425 Can't open data connection\r\n";
                        send(client_sock, err, strlen(err), 0);
                    }
                } else {
                    std::cerr << "没有数据连接（需要先执行 PASV）" << std::endl;
                    const char* err = "425 Use PASV first\r\n";
                    send(client_sock, err, strlen(err), 0);
                }
            }
            else if (cmd.substr(0, 4) == "QUIT") {
                send(client_sock, FTP_GOODBYE, strlen(FTP_GOODBYE), 0);
                break;
            }
            else {
                send(client_sock, FTP_CMD_OK, strlen(FTP_CMD_OK), 0);
            }
        }
        
        if (data_listen_socket >= 0) close(data_listen_socket);
        if (data_socket >= 0) close(data_socket);
        close(client_sock);
    }
    
    int create_listen_socket(int& port) {
        int sock = socket(AF_INET, SOCK_STREAM, 0);
        if (sock < 0) {
            std::cerr << "创建socket失败" << std::endl;
            return -1;
        }
        
        // 设置 socket 选项
        int opt = 1;
        setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, (char*)&opt, sizeof(opt));
        
        struct sockaddr_in addr;
        memset(&addr, 0, sizeof(addr));
        addr.sin_family = AF_INET;
        addr.sin_addr.s_addr = INADDR_ANY;
        addr.sin_port = 0; // 自动分配端口
        
        if (bind(sock, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
            std::cerr << "bind失败" << std::endl;
            close(sock);
            return -1;
        }
        
        socklen_t len = sizeof(addr);
        if (getsockname(sock, (struct sockaddr*)&addr, &len) < 0) {
            std::cerr << "getsockname失败" << std::endl;
            close(sock);
            return -1;
        }
        
        port = ntohs(addr.sin_port);
        
        if (listen(sock, 1) < 0) {
            std::cerr << "listen失败" << std::endl;
            close(sock);
            return -1;
        }
        
        std::cout << "数据端口监听在: " << port << std::endl;
        return sock;
    }
    
    int accept_data_connection(int listen_sock) {
        // 设置 accept 超时（30秒）
        struct timeval tv;
        tv.tv_sec = 30;
        tv.tv_usec = 0;
        setsockopt(listen_sock, SOL_SOCKET, SO_RCVTIMEO, (char*)&tv, sizeof(tv));
        
        int client = accept(listen_sock, nullptr, nullptr);
        
        if (client < 0) {
            std::cerr << "数据连接 accept 失败: " << strerror(errno) << std::endl;
            return -1;
        }
        
        std::cout << "数据连接已建立" << std::endl;
        return client;
    }
    
    void receive_file(int sock, const std::string& filename) {
        std::string filepath = config.image_dir + "/" + filename;
        std::ofstream file(filepath, std::ios::binary);
        
        if (!file.is_open()) {
            std::cerr << "无法创建文件: " << filepath << std::endl;
            return;
        }
        
        char buffer[8192];
        int total = 0;
        while (true) {
            int n = recv(sock, buffer, sizeof(buffer), 0);
            if (n <= 0) break;
            file.write(buffer, n);
            total += n;
        }
        
        file.close();
        std::cout << "接收文件完成: " << filepath << " (" << total << " bytes)" << std::endl;
        
        // 如果是 PGM 文件，尝试转换为 PNG
        if (filepath.find(".pgm") != std::string::npos) {
            std::string png_path = filepath.substr(0, filepath.size() - 4) + ".png";
            std::string cmd = "convert " + filepath + " " + png_path + " 2>/dev/null";
            int ret = system(cmd.c_str());
            if (ret == 0) {
                std::cout << "已转换为 PNG: " << png_path << std::endl;
                filepath = png_path; // 更新为 PNG 路径
            } else {
                std::cout << "PGM 转 PNG 失败（可能未安装 ImageMagick），保持 PGM 格式" << std::endl;
            }
        }
        
        // 添加到已接收列表
        {
            std::lock_guard<std::mutex> lock(images_mutex);
            received_images.push_back(filepath);
        }
        
        // 调用回调
        if (file_callback) {
            file_callback(filename, filepath);
        }
    }
    
    void send_file_list(int sock) {
        // 读取图片目录中的文件
        DIR* dir = opendir(config.image_dir.c_str());
        if (!dir) {
            std::cerr << "无法打开目录: " << config.image_dir << std::endl;
            return;
        }
        
        std::ostringstream list;
        struct dirent* entry;
        while ((entry = readdir(dir)) != nullptr) {
            if (entry->d_name[0] == '.') continue; // 跳过隐藏文件
            
            std::string filepath = config.image_dir + "/" + entry->d_name;
            struct stat st;
            if (stat(filepath.c_str(), &st) == 0) {
                // 格式化为类似 ls -l 的输出
                // -rw-r--r-- 1 user group size date filename
                list << "-rw-r--r-- 1 ftp ftp " 
                     << std::setw(10) << st.st_size << " "
                     << "Jan 01 00:00 " << entry->d_name << "\r\n";
            }
        }
        closedir(dir);
        
        std::string list_str = list.str();
        if (!list_str.empty()) {
            send(sock, list_str.c_str(), list_str.size(), 0);
            std::cout << "已发送文件列表 (" << list_str.size() << " bytes)" << std::endl;
        } else {
            std::cout << "目录为空" << std::endl;
        }
    }
    
    void run() {
        if (!init_socket()) {
            std::cerr << "FTP服务器初始化失败" << std::endl;
            return;
        }
        
        std::cout << "FTP服务器启动在端口 " << config.port << std::endl;
        running = true;
        
        while (running) {
            struct sockaddr_in client_addr;
            socklen_t addr_len = sizeof(client_addr);
            int client_sock = accept(server_socket, (struct sockaddr*)&client_addr, &addr_len);
            
            if (client_sock < 0) {
                if (running) {
                    std::cerr << "Accept failed" << std::endl;
                }
                continue;
            }
            
            std::cout << "FTP客户端连接" << std::endl;
            
            // 在新线程中处理客户端
            std::thread([this, client_sock]() {
                handle_client(client_sock);
            }).detach();
        }
        
        close(server_socket);
#ifdef _WIN32
        WSACleanup();
#endif
    }
    
    void stop() {
        running = false;
        if (server_socket >= 0) {
            close(server_socket);
            server_socket = -1;
        }
        if (server_thread.joinable()) {
            server_thread.join();
        }
    }
};

// FTPServer 实现
FTPServer::FTPServer(const FTPConfig& config) {
    pImpl = new Impl(config);
}

FTPServer::~FTPServer() {
    delete pImpl;
}

bool FTPServer::start() {
    pImpl->run();
    return true;
}

bool FTPServer::start_async() {
    pImpl->server_thread = std::thread([this]() {
        pImpl->run();
    });
    return true;
}

void FTPServer::stop() {
    pImpl->stop();
}

void FTPServer::set_file_received_callback(FileReceivedCallback callback) {
    pImpl->file_callback = callback;
}

std::vector<std::string> FTPServer::get_received_images() const {
    std::lock_guard<std::mutex> lock(pImpl->images_mutex);
    return pImpl->received_images;
}
