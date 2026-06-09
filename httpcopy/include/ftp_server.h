#ifndef FTP_SERVER_H
#define FTP_SERVER_H

#include <string>
#include <functional>

// FTP服务器配置
struct FTPConfig {
    std::string ip = "0.0.0.0";
    int port = 21;
    std::string root_dir = "./ftp_files";
    std::string image_dir = "./ftp_files/images";
};

// 文件接收回调函数类型
using FileReceivedCallback = std::function<void(const std::string& filename, const std::string& filepath)>;

// FTP服务器类
class FTPServer {
public:
    FTPServer(const FTPConfig& config);
    ~FTPServer();
    
    // 启动FTP服务器（阻塞式）
    bool start();
    
    // 在独立线程中启动FTP服务器
    bool start_async();
    
    // 停止FTP服务器
    void stop();
    
    // 设置文件接收回调
    void set_file_received_callback(FileReceivedCallback callback);
    
    // 获取接收到的图片列表
    std::vector<std::string> get_received_images() const;
    
private:
    class Impl;
    Impl* pImpl;
};

#endif // FTP_SERVER_H
