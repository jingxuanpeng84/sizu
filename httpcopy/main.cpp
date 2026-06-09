#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <mutex>
#include <thread>
#include <chrono>
#include <cstring>
#include <atomic>
#include <ctime>
#include <cerrno>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include "httplib.h"
#include "socket.h"
#include "ftp_server.h"
#include "ftp_server.h"

#define HTTP_PORT 8081
#define SERVER_IP "0.0.0.0"
#define UDP_PORT  8110  // UDP端口（改为8110）
#define FTP_PORT  2121  // FTP端口（使用2121避免需要root权限）

// SLAM 地址（可按需改为从配置读取）
#define SLAM_IP   "192.168.110.205"
#define SLAM_PORT 8080

// 地图文件存储路径
#define MAP_FILE  "slam_map.pgm"

// ---------- 路径点数据 ----------
struct PathPoint {
    int    seq;
    double x;
    double y;
    double yaw;
    int    keypointflag; // 0普通 1充电 2待梯 3乘梯 4出梯 5换图点
    int    mapnum;       // 地图编号
};

// ---------- 地图管理 ----------
struct MapInfo {
    int    map_id;
    std::string map_name;
    std::string xml_file;
    std::string map_file;
};

static std::vector<MapInfo> g_maps;
static int g_current_map_id = 1;
static std::mutex g_map_list_mutex;

// 像素坐标点（SLAM 推送）
struct PixelPoint {
    int seq;
    int px;
    int py;
};

static std::vector<PathPoint>  g_points;
static std::vector<PixelPoint> g_pixels;   // SLAM 推送的像素坐标
static std::mutex              g_points_mutex;
static std::mutex              g_map_mutex;
static bool                    g_map_ready = false;

static SlamPose                g_latest_pose = {0.0, 0.0, 0.0, 0, 0, 1, false};
static std::mutex              g_pose_mutex;

// ---------- 地图切换状态管理 ----------
static std::atomic<bool> g_map_switch_in_progress(false);
static std::atomic<bool> g_map_switch_completed(false);

// ---------- XML 序列化 ----------
static std::string points_to_xml(const std::vector<PathPoint>& pts) {
    std::ostringstream ss;
    ss << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n<Root>\n";
    for (const auto& p : pts) {
        ss << "    <path Seq=\"" << p.seq << "\" Type=\"ulong\">\n"
           << "        <x Type=\"double\">"            << p.x            << "</x>\n"
           << "        <y Type=\"double\">"            << p.y            << "</y>\n"
           << "        <yaw Type=\"double\">"          << p.yaw          << "</yaw>\n"
           << "        <keypointflag Type=\"uint\">"   << p.keypointflag << "</keypointflag>\n"
           << "        <mapnum Type=\"int\">"          << p.mapnum       << "</mapnum>\n"
           << "    </path>\n";
    }
    ss << "</Root>";
    return ss.str();
}

static std::string get_current_xml_file() {
    return "/home/orangepi/Desktop/work_station/OctiRobotVersion2/path_control/test1.xml";
}

static void save_xml_file() {
    std::string xml_file = get_current_xml_file();
    std::ofstream f(xml_file);
    if (f.is_open()) {
        f << points_to_xml(g_points);
        std::cout << "路径已保存到: " << xml_file << std::endl;
    }
}

static void load_xml_file(int map_id) {
    std::string xml_file = "/home/orangepi/Desktop/work_station/OctiRobotVersion2/path_control/test1.xml";
    std::ifstream f(xml_file);
    if (!f.is_open()) {
        std::cout << "地图 " << map_id << " 的路径文件不存在，将创建新路径" << std::endl;
        return;
    }
    // 简单实现：这里只是示例，实际需要完整的XML解析
    std::cout << "已加载地图 " << map_id << " 的路径文件: " << xml_file << std::endl;
}

// ---------- 简单 JSON 解析辅助 ----------
static std::string json_get(const std::string& body, const std::string& key) {
    // 找 "key": value 或 "key":"value"
    std::string search = "\"" + key + "\"";
    auto pos = body.find(search);
    if (pos == std::string::npos) return "";
    pos = body.find(':', pos + search.size());
    if (pos == std::string::npos) return "";
    ++pos;
    while (pos < body.size() && (body[pos] == ' ' || body[pos] == '"')) ++pos;
    std::string val;
    for (; pos < body.size(); ++pos) {
        char c = body[pos];
        if (c == '"' || c == ',' || c == '}' || c == '\n') break;
        val += c;
    }
    return val;
}

// ---------- HTML 读取 ----------
static std::string read_html_file(const std::string& filename) {
    std::vector<std::string> paths = {filename, "../" + filename, "../../" + filename};
    for (const auto& path : paths) {
        std::ifstream file(path);
        if (file.is_open()) {
            std::string content((std::istreambuf_iterator<char>(file)),
                                 std::istreambuf_iterator<char>());
            std::cout << "成功加载HTML文件: " << path << std::endl;
            return content;
        }
    }
    std::cerr << "错误: 无法找到HTML文件 " << filename << std::endl;
    return "";
}

// ---------- main ----------
int main() {
    std::cout << "=== 配置服务器启动 ===" << std::endl;

    int sockfd = setup_udp_socket(UDP_PORT);
    if (sockfd < 0) {
        std::cerr << "Failed to set up UDP socket!" << std::endl;
        return -1;
    }
    std::cout << "UDP Socket initialized on port " << UDP_PORT << std::endl;

    // 启动FTP服务器
    FTPConfig ftp_config;
    ftp_config.port = FTP_PORT;
    ftp_config.root_dir = "../ftp_files";      // 相对于 build 目录的上级目录
    ftp_config.image_dir = "../ftp_files/images";
    
    FTPServer ftp_server(ftp_config);
    
    // 设置文件接收回调
    ftp_server.set_file_received_callback([](const std::string& filename, const std::string& filepath) {
        std::cout << "收到图片文件: " << filename << std::endl;
        std::cout << "保存路径: " << filepath << std::endl;
    });
    
    // 在独立线程中启动FTP服务器
    ftp_server.start_async();
    std::cout << "FTP服务器启动在端口 " << FTP_PORT << std::endl;

    // 后台轮询 SLAM 位置，每 100ms 更新缓存
    std::thread([](){
        while (true) {
            auto start = std::chrono::steady_clock::now();
            SlamPose pose = request_slam_pose(SLAM_IP, SLAM_PORT, 1000);
            if (pose.valid) {
                std::lock_guard<std::mutex> lk(g_pose_mutex);
                g_latest_pose = pose;
            }
            auto elapsed = std::chrono::steady_clock::now() - start;
            auto wait_time = std::chrono::milliseconds(100) - std::chrono::duration_cast<std::chrono::milliseconds>(elapsed);
            if (wait_time.count() > 0) {
                std::this_thread::sleep_for(wait_time);
            }
        }
    }).detach();

    httplib::Server svr;

    // ---- 配置页面 ----
    auto html_handler = [](const httplib::Request&, httplib::Response& res) {
        std::string html = read_html_file("peizhi.html");
        if (!html.empty()) res.set_content(html, "text/html; charset=utf-8");
        else { res.set_content("配置页面加载失败", "text/plain; charset=utf-8"); res.status = 404; }
    };
    svr.Get("/", html_handler);
    svr.Get("/peizhi.html", html_handler);

    // ---- 保存配置 ----
    svr.Post("/api/save_config", [&](const httplib::Request& req, httplib::Response& res) {
        res.set_header("Access-Control-Allow-Origin", "*");
        std::cout << "收到配置保存请求: " << req.body << std::endl;
        res.set_content("{\"status\":\"ok\"}", "application/json");
    });

    // ---- 获取当前 SLAM 位置 ----
    svr.Get("/api/get_position", [](const httplib::Request&, httplib::Response& res) {
        res.set_header("Access-Control-Allow-Origin", "*");
        std::lock_guard<std::mutex> lk(g_pose_mutex);
        if (g_latest_pose.valid) {
            std::ostringstream ss;
            ss << "{\"x\":" << g_latest_pose.x 
               << ",\"y\":" << g_latest_pose.y 
               << ",\"yaw\":" << g_latest_pose.yaw 
               << ",\"px\":" << g_latest_pose.px      // 添加像素坐标X
               << ",\"py\":" << g_latest_pose.py      // 添加像素坐标Y
               << ",\"map_id\":" << g_latest_pose.map_id  // 添加地图编号
               << "}";
            
            // 调试输出
            std::cout << "[DEBUG] 返回位置数据: " << ss.str() << std::endl;
            
            res.set_content(ss.str(), "application/json");
        } else {
            res.status = 504;
            res.set_content("{\"error\":\"SLAM position not ready\"}", "application/json");
        }
    });

    // ---- 添加路径点 ----
    svr.Post("/api/add_point", [](const httplib::Request& req, httplib::Response& res) {
        res.set_header("Access-Control-Allow-Origin", "*");
        std::string xs    = json_get(req.body, "x");
        std::string ys    = json_get(req.body, "y");
        std::string yaws  = json_get(req.body, "yaw");
        std::string pxs   = json_get(req.body, "px");      // 像素坐标X
        std::string pys   = json_get(req.body, "py");      // 像素坐标Y
        std::string flags = json_get(req.body, "keypointflag");
        std::string map_ids = json_get(req.body, "map_id"); // 当前地图编号
        std::string target_map_ids = json_get(req.body, "target_map_id"); // 目标地图编号（地图切换点专用）
        
        if (xs.empty() || ys.empty() || yaws.empty()) {
            res.status = 400;
            res.set_content("{\"error\":\"missing fields\"}", "application/json");
            return;
        }
        
        std::lock_guard<std::mutex> lk(g_points_mutex);
        
        // 添加世界坐标点
        PathPoint pt;
        pt.seq          = (int)g_points.size() + 1;
        pt.x            = std::stod(xs);
        pt.y            = std::stod(ys);
        pt.yaw          = std::stod(yaws);
        pt.keypointflag = flags.empty() ? 0 : std::stoi(flags);
        
        // 🔴 修改：如果是地图切换点（keypointflag=5），mapnum存储目标地图编号
        if (pt.keypointflag == 5 && !target_map_ids.empty()) {
            pt.mapnum = std::stoi(target_map_ids);  // 目标地图编号
            std::cout << "添加地图切换点 #" << pt.seq 
                      << " 当前地图: " << (map_ids.empty() ? 1 : std::stoi(map_ids))
                      << " → 目标地图: " << pt.mapnum << std::endl;
        } else {
            pt.mapnum = map_ids.empty() ? 1 : std::stoi(map_ids);  // 当前地图编号
        }
        
        g_points.push_back(pt);
        
        // 同时添加像素坐标点（如果提供了）
        if (!pxs.empty() && !pys.empty()) {
            PixelPoint pp;
            pp.seq = pt.seq;
            pp.px  = std::stoi(pxs);
            pp.py  = std::stoi(pys);
            g_pixels.push_back(pp);
        }
        
        // 打印日志
        std::cout << "添加路径点 #" << pt.seq 
                  << " 世界坐标: (" << pt.x << ", " << pt.y << ")"
                  << " 像素坐标: (" << (pxs.empty() ? 0 : std::stoi(pxs)) << ", " 
                  << (pys.empty() ? 0 : std::stoi(pys)) << ")"
                  << " 类型: " << pt.keypointflag
                  << " 地图编号: " << pt.mapnum << std::endl;
        
        std::ostringstream ss;
        ss << "{\"status\":\"ok\",\"seq\":" << pt.seq << "}";
        res.set_content(ss.str(), "application/json");
    });

    // ---- 删除最后一个点 ----
    svr.Delete("/api/undo_point", [](const httplib::Request&, httplib::Response& res) {
        res.set_header("Access-Control-Allow-Origin", "*");
        std::lock_guard<std::mutex> lk(g_points_mutex);
        if (!g_points.empty()) {
            g_points.pop_back();
            // 同时删除对应的像素坐标
            if (!g_pixels.empty()) {
                g_pixels.pop_back();
            }
        }
        res.set_content("{\"status\":\"ok\"}", "application/json");
    });

    // ---- 清空所有点 ----
    svr.Delete("/api/clear_points", [](const httplib::Request&, httplib::Response& res) {
        res.set_header("Access-Control-Allow-Origin", "*");
        std::lock_guard<std::mutex> lk(g_points_mutex);
        g_points.clear();
        g_pixels.clear();  // 同时清空像素坐标
        res.set_content("{\"status\":\"ok\"}", "application/json");
    });

    // ---- 获取当前所有点（JSON） ----
    svr.Get("/api/get_points", [](const httplib::Request&, httplib::Response& res) {
        res.set_header("Access-Control-Allow-Origin", "*");
        std::lock_guard<std::mutex> lk(g_points_mutex);
        std::ostringstream ss;
        ss << "[";
        for (size_t i = 0; i < g_points.size(); ++i) {
            const auto& p = g_points[i];
            if (i) ss << ",";
            ss << "{\"seq\":" << p.seq
               << ",\"x\":"   << p.x
               << ",\"y\":"   << p.y
               << ",\"yaw\":" << p.yaw
               << ",\"keypointflag\":" << p.keypointflag << "}";
        }
        ss << "]";
        res.set_content(ss.str(), "application/json");
    });

    // ---- 保存点到文件（保存按钮）----
    svr.Post("/api/save_points", [](const httplib::Request&, httplib::Response& res) {
        res.set_header("Access-Control-Allow-Origin", "*");
        std::lock_guard<std::mutex> lk(g_points_mutex);
        save_xml_file();
        std::ostringstream ss;
        ss << "{\"status\":\"ok\",\"message\":\"Points saved\",\"count\":" << g_points.size() << "}";
        res.set_content(ss.str(), "application/json");
        std::cout << "点已保存到文件，共 " << g_points.size() << " 个点" << std::endl;
    });

    // ---- 下载 XML 文件 ----
    svr.Get("/api/download_xml", [](const httplib::Request&, httplib::Response& res) {
        res.set_header("Access-Control-Allow-Origin", "*");
        std::lock_guard<std::mutex> lk(g_points_mutex);
        std::string xml = points_to_xml(g_points);
        std::string filename = "map_" + std::to_string(g_current_map_id) + "_waypoints.xml";
        res.set_header("Content-Disposition", "attachment; filename=\"" + filename + "\"");
        res.set_content(xml, "application/xml; charset=utf-8");
    });

    // ---- 切换地图（仅用于上传地图后切换显示，不清空点）----
    svr.Post("/api/switch_map", [](const httplib::Request& req, httplib::Response& res) {
        res.set_header("Access-Control-Allow-Origin", "*");
        std::string new_map_name = json_get(req.body, "map_name");
        if (new_map_name.empty()) new_map_name = "地图" + std::to_string(g_current_map_id + 1);
        
        // 🔴 注意：此API仅用于切换显示的地图，不清空点
        // 点应该一直保留在内存中，直到用户点击"保存"按钮
        // 如果需要清空点并开始新路径，应该使用单独的API
        
        std::cout << "切换显示地图: " << new_map_name << " (点不清空，保留在内存中)" << std::endl;
        
        std::ostringstream ss;
        ss << "{\"status\":\"ok\",\"map_id\":" << g_current_map_id 
           << ",\"map_name\":\"" << new_map_name << "\"}";
        res.set_content(ss.str(), "application/json");
    });

    // ---- 获取当前地图信息 ----
    svr.Get("/api/get_current_map", [](const httplib::Request&, httplib::Response& res) {
        res.set_header("Access-Control-Allow-Origin", "*");
        std::ostringstream ss;
        ss << "{\"map_id\":" << g_current_map_id 
           << ",\"xml_file\":\"" << get_current_xml_file() << "\"}";
        res.set_content(ss.str(), "application/json");
    });

    // ---- 加载指定地图的路径 ----
    svr.Post("/api/load_map", [](const httplib::Request& req, httplib::Response& res) {
        res.set_header("Access-Control-Allow-Origin", "*");
        std::string map_id_str = json_get(req.body, "map_id");
        if (map_id_str.empty()) {
            res.status = 400;
            res.set_content("{\"error\":\"missing map_id\"}", "application/json");
            return;
        }
        
        int map_id = std::stoi(map_id_str);
        {
            std::lock_guard<std::mutex> lk(g_points_mutex);
            g_current_map_id = map_id;
            load_xml_file(map_id);
        }
        
        std::ostringstream ss;
        ss << "{\"status\":\"ok\",\"map_id\":" << map_id << "}";
        res.set_content(ss.str(), "application/json");
    });

    // ---- SLAM 推送地图文件（POST 二进制，Content-Type: image/pgm 或 image/png）----
    svr.Post("/api/upload_map", [](const httplib::Request& req, httplib::Response& res) {
        res.set_header("Access-Control-Allow-Origin", "*");
        if (req.body.empty()) {
            res.status = 400;
            res.set_content("{\"error\":\"empty body\"}", "application/json");
            return;
        }
        // 根据 Content-Type 决定扩展名
        std::string ct = req.get_header_value("Content-Type");
        std::string ext = ".pgm";
        if (ct.find("png") != std::string::npos) ext = ".png";

        std::string mapPath = std::string("slam_map") + ext;
        {
            std::lock_guard<std::mutex> lk(g_map_mutex);
            std::ofstream f(mapPath, std::ios::binary);
            if (!f.is_open()) {
                res.status = 500;
                res.set_content("{\"error\":\"cannot write map file\"}", "application/json");
                return;
            }
            f.write(req.body.data(), req.body.size());
            g_map_ready = true;
        }
        std::cout << "地图文件已更新: " << mapPath << " (" << req.body.size() << " bytes)" << std::endl;
        res.set_content("{\"status\":\"ok\"}", "application/json");
    });

    // ---- 获取SLAM端地图列表 ----
    svr.Get("/api/list_slam_maps", [](const httplib::Request&, httplib::Response& res) {
        res.set_header("Access-Control-Allow-Origin", "*");
        std::cout << "获取SLAM端地图列表..." << std::endl;
        
        const char* slam_ip = SLAM_IP;
        const int upload_listen_port = 9998;
        
        // 创建 UDP socket
        int sock = socket(AF_INET, SOCK_DGRAM, 0);
        if (sock < 0) {
            std::cerr << "创建socket失败" << std::endl;
            res.status = 500;
            res.set_content("{\"error\":\"Failed to create socket\"}", "application/json");
            return;
        }
        
        // 设置超时
        struct timeval tv;
        tv.tv_sec = 5;
        tv.tv_usec = 0;
        setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
        
        // 设置目标地址
        struct sockaddr_in slam_addr;
        memset(&slam_addr, 0, sizeof(slam_addr));
        slam_addr.sin_family = AF_INET;
        slam_addr.sin_port = htons(upload_listen_port);
        inet_pton(AF_INET, slam_ip, &slam_addr.sin_addr);
        
        // 发送列表命令
        const char* cmd = "LIST_MAPS";
        std::cout << "发送命令到 SLAM端 " << slam_ip << ":" << upload_listen_port << std::endl;
        
        ssize_t sent = sendto(sock, cmd, strlen(cmd), 0,
                             (struct sockaddr*)&slam_addr, sizeof(slam_addr));
        
        if (sent < 0) {
            std::cerr << "发送命令失败" << std::endl;
            close(sock);
            res.status = 500;
            res.set_content("{\"error\":\"Failed to send command to SLAM\"}", "application/json");
            return;
        }
        
        // 接收响应（可能很长，需要大缓冲区）
        char buffer[8192];
        struct sockaddr_in from_addr;
        socklen_t from_len = sizeof(from_addr);
        
        ssize_t recv_len = recvfrom(sock, buffer, sizeof(buffer) - 1, 0,
                                    (struct sockaddr*)&from_addr, &from_len);
        close(sock);
        
        if (recv_len < 0) {
            std::cerr << "接收响应超时或失败" << std::endl;
            res.status = 504;
            res.set_content("{\"error\":\"SLAM response timeout\"}", "application/json");
            return;
        }
        
        buffer[recv_len] = '\0';
        std::string response(buffer);
        //std::cout << "收到SLAM端响应: " << response.substr(0, 100) << "..." << std::endl;
        
        // 直接返回SLAM端的JSON响应
        res.set_content(response, "application/json");
    });

    // ---- 上传指定地图 ----
    svr.Post("/api/upload_selected_map", [](const httplib::Request& req, httplib::Response& res) {
        res.set_header("Access-Control-Allow-Origin", "*");
        
        std::string map_number_str = json_get(req.body, "map_number");
        if (map_number_str.empty()) {
            res.status = 400;
            res.set_content("{\"error\":\"missing map_number\"}", "application/json");
            return;
        }
        
        std::cout << "请求上传地图序号: " << map_number_str << std::endl;
        
        const char* slam_ip = SLAM_IP;
        const int upload_listen_port = 9998;
        
        // 创建 UDP socket
        int sock = socket(AF_INET, SOCK_DGRAM, 0);
        if (sock < 0) {
            std::cerr << "创建socket失败" << std::endl;
            res.status = 500;
            res.set_content("{\"error\":\"Failed to create socket\"}", "application/json");
            return;
        }
        
        // 设置超时
        struct timeval tv;
        tv.tv_sec = 15;
        tv.tv_usec = 0;
        setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
        
        // 设置目标地址
        struct sockaddr_in slam_addr;
        memset(&slam_addr, 0, sizeof(slam_addr));
        slam_addr.sin_family = AF_INET;
        slam_addr.sin_port = htons(upload_listen_port);
        inet_pton(AF_INET, slam_ip, &slam_addr.sin_addr);
        
        // 发送上传命令（带序号）
        std::string cmd = "UPLOAD_MAP:" + map_number_str;
        std::cout << "发送命令到 SLAM端: " << cmd << std::endl;
        
        ssize_t sent = sendto(sock, cmd.c_str(), cmd.length(), 0, 
                             (struct sockaddr*)&slam_addr, sizeof(slam_addr));
        
        if (sent < 0) {
            std::cerr << "发送命令失败" << std::endl;
            close(sock);
            res.status = 500;
            res.set_content("{\"error\":\"Failed to send command to SLAM\"}", "application/json");
            return;
        }
        
        std::cout << "命令已发送，等待SLAM端响应..." << std::endl;
        
        // 接收响应
        char buffer[1024];
        struct sockaddr_in from_addr;
        socklen_t from_len = sizeof(from_addr);
        
        ssize_t recv_len = recvfrom(sock, buffer, sizeof(buffer) - 1, 0,
                                    (struct sockaddr*)&from_addr, &from_len);
        close(sock);
        
        if (recv_len < 0) {
            std::cerr << "接收响应超时或失败" << std::endl;
            res.status = 504;
            res.set_content("{\"error\":\"SLAM response timeout\"}", "application/json");
            return;
        }
        
        buffer[recv_len] = '\0';
        std::string response(buffer);
        std::cout << "收到SLAM端响应: " << response << std::endl;
        
        // 解析响应
        if (response.find("OK") == 0) {
            std::ostringstream ss;
            ss << "{\"status\":\"ok\","
               << "\"message\":\"" << response << "\","
               << "\"map_number\":\"" << map_number_str << "\"}";
            res.set_content(ss.str(), "application/json");
        } else {
            res.status = 500;
            std::ostringstream ss;
            ss << "{\"error\":\"" << response << "\"}";
            res.set_content(ss.str(), "application/json");
        }
    });

    // ---- 主动向 SLAM 请求地图文件（通过UDP通知SLAM端上传）----
    svr.Post("/api/request_map", [](const httplib::Request&, httplib::Response& res) {
        res.set_header("Access-Control-Allow-Origin", "*");
        std::cout << "收到主动获取地图请求，通知SLAM端上传地图..." << std::endl;
        
        // 通过 UDP 发送上传命令到 SLAM 端的上传监听服务
        const char* slam_ip = SLAM_IP;
        const int upload_listen_port = 9998;  // 上传监听服务端口
        
        // 创建 UDP socket
        int sock = socket(AF_INET, SOCK_DGRAM, 0);
        if (sock < 0) {
            std::cerr << "创建socket失败" << std::endl;
            res.status = 500;
            res.set_content("{\"error\":\"Failed to create socket\"}", "application/json");
            return;
        }
        
        // 设置超时
        struct timeval tv;
        tv.tv_sec = 15;  // 上传可能需要更长时间
        tv.tv_usec = 0;
        setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
        
        // 设置目标地址
        struct sockaddr_in slam_addr;
        memset(&slam_addr, 0, sizeof(slam_addr));
        slam_addr.sin_family = AF_INET;
        slam_addr.sin_port = htons(upload_listen_port);
        inet_pton(AF_INET, slam_ip, &slam_addr.sin_addr);
        
        // 发送上传命令
        const char* cmd = "UPLOAD_MAP";
        std::cout << "发送上传命令到 SLAM端 " << slam_ip << ":" << upload_listen_port << std::endl;
        
        ssize_t sent = sendto(sock, cmd, strlen(cmd), 0,
                             (struct sockaddr*)&slam_addr, sizeof(slam_addr));
        
        if (sent < 0) {
            std::cerr << "发送命令失败" << std::endl;
            close(sock);
            res.status = 500;
            res.set_content("{\"error\":\"Failed to send command to SLAM\"}", "application/json");
            return;
        }
        
        std::cout << "命令已发送，等待SLAM端上传地图..." << std::endl;
        
        // 接收响应
        char buffer[1024];
        struct sockaddr_in from_addr;
        socklen_t from_len = sizeof(from_addr);
        
        ssize_t recv_len = recvfrom(sock, buffer, sizeof(buffer) - 1, 0,
                                    (struct sockaddr*)&from_addr, &from_len);
        
        close(sock);
        
        if (recv_len < 0) {
            std::cerr << "接收响应超时或失败" << std::endl;
            res.status = 504;
            res.set_content("{\"error\":\"SLAM response timeout\"}", "application/json");
            return;
        }
        
        buffer[recv_len] = '\0';
        std::string response(buffer);
        std::cout << "收到SLAM端响应: " << response << std::endl;
        
        // 解析响应
        if (response.find("OK") == 0) {
            std::cout << "SLAM端地图上传成功" << std::endl;
            res.set_content("{\"status\":\"ok\",\"message\":\"Map uploaded from SLAM successfully\"}", "application/json");
        } else {
            res.status = 500;
            res.set_content("{\"error\":\"SLAM failed to upload map\"}", "application/json");
        }
    });

    // ---- 保存地图并退出建图程序（通过UDP通知SLAM端）----
    svr.Post("/api/save_map_and_exit", [](const httplib::Request&, httplib::Response& res) {
        res.set_header("Access-Control-Allow-Origin", "*");
        std::cout << "收到保存地图并退出建图的请求" << std::endl;
        
        // 通过 UDP 发送保存命令到 SLAM 端
        const char* slam_ip = SLAM_IP;
        const int slam_listen_port = 9999;  // SLAM端监听端口
        
        // 创建 UDP socket
        int sock = socket(AF_INET, SOCK_DGRAM, 0);
        if (sock < 0) {
            std::cerr << "创建socket失败" << std::endl;
            res.status = 500;
            res.set_content("{\"error\":\"Failed to create socket\"}", "application/json");
            return;
        }
        
        // 设置超时
        struct timeval tv;
        tv.tv_sec = 10;
        tv.tv_usec = 0;
        setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
        
        // 设置目标地址
        struct sockaddr_in slam_addr;
        memset(&slam_addr, 0, sizeof(slam_addr));
        slam_addr.sin_family = AF_INET;
        slam_addr.sin_port = htons(slam_listen_port);
        inet_pton(AF_INET, slam_ip, &slam_addr.sin_addr);
        
        // 发送保存命令
        const char* cmd = "SAVE_MAP";
        std::cout << "发送保存命令到 SLAM端 " << slam_ip << ":" << slam_listen_port << std::endl;
        
        ssize_t sent = sendto(sock, cmd, strlen(cmd), 0,
                             (struct sockaddr*)&slam_addr, sizeof(slam_addr));
        
        if (sent < 0) {
            std::cerr << "发送命令失败" << std::endl;
            close(sock);
            res.status = 500;
            res.set_content("{\"error\":\"Failed to send command to SLAM\"}", "application/json");
            return;
        }
        
        std::cout << "命令已发送，等待SLAM端响应..." << std::endl;
        
        // 接收响应
        char buffer[1024];
        struct sockaddr_in from_addr;
        socklen_t from_len = sizeof(from_addr);
        
        ssize_t recv_len = recvfrom(sock, buffer, sizeof(buffer) - 1, 0,
                                    (struct sockaddr*)&from_addr, &from_len);
        
        close(sock);
        
        if (recv_len < 0) {
            std::cerr << "接收响应超时或失败" << std::endl;
            res.status = 504;
            res.set_content("{\"error\":\"SLAM response timeout\"}", "application/json");
            return;
        }
        
        buffer[recv_len] = '\0';
        std::string response(buffer);
        std::cout << "收到SLAM端响应: " << response << std::endl;
        
        // 解析响应
        if (response.find("OK") == 0) {
            std::ostringstream ss;
            ss << "{\"status\":\"ok\","
               << "\"message\":\"Map saved successfully on SLAM side\","
               << "\"map_file\":\"/home/orangepi/slam_ws/src/FAST_LIO_GLOBAL/PCD/scans.pgm\","
               << "\"yaml_file\":\"/home/orangepi/slam_ws/src/FAST_LIO_GLOBAL/PCD/scans.yaml\"}";
            res.set_content(ss.str(), "application/json");
        } else {
            res.status = 500;
            res.set_content("{\"error\":\"SLAM failed to save map\"}", "application/json");
        }
    });

    // ---- 🔴 新增：触发SLAM地图切换 ----
    svr.Post("/api/trigger_map_switch", [](const httplib::Request& req, httplib::Response& res) {
        res.set_header("Access-Control-Allow-Origin", "*");
        
        std::string target_map_id_str = json_get(req.body, "target_map_id");
        std::string current_x_str = json_get(req.body, "current_x");
        std::string current_y_str = json_get(req.body, "current_y");
        std::string current_yaw_str = json_get(req.body, "current_yaw");
        
        if (target_map_id_str.empty()) {
            res.status = 400;
            res.set_content("{\"error\":\"missing target_map_id\"}", "application/json");
            return;
        }
        
        int target_map_id = std::stoi(target_map_id_str);
        float current_x = current_x_str.empty() ? 0.0f : std::stof(current_x_str);
        float current_y = current_y_str.empty() ? 0.0f : std::stof(current_y_str);
        float current_yaw = current_yaw_str.empty() ? 0.0f : std::stof(current_yaw_str);
        
        std::cout << "[地图切换] 准备切换到地图" << target_map_id 
                  << " 当前位置: (" << current_x << ", " << current_y << ", " << current_yaw << ")" << std::endl;
        
        // 设置切换状态
        g_map_switch_in_progress = true;
        g_map_switch_completed = false;
        
        // 🔴 通过TCP向SLAM端发送地图切换请求
        const char* slam_ip = "192.168.2.100";  // SLAM端IP（根据实际情况修改）
        const int slam_port = 6051;             // SLAM端地图切换端口
        
        // 创建TCP socket
        int sock = socket(AF_INET, SOCK_DGRAM, 0);
        if (sock < 0) {
            std::cerr << "[地图切换] 创建socket失败" << std::endl;
            g_map_switch_in_progress = false;
            res.status = 500;
            res.set_content("{\"error\":\"Failed to create socket\"}", "application/json");
            return;
        }
        
        // 设置超时（接收超时需要足够长，因为SLAM切换需要40-50秒）
        struct timeval tv_send;
        tv_send.tv_sec = 5;   // 发送超时5秒
        tv_send.tv_usec = 0;
        setsockopt(sock, SOL_SOCKET, SO_SNDTIMEO, &tv_send, sizeof(tv_send));
        
        struct timeval tv_recv;
        tv_recv.tv_sec = 1;  // UDP receive tick; resend while waiting for final result
        tv_recv.tv_usec = 0;
        setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv_recv, sizeof(tv_recv));
        
        // 连接到SLAM端
        struct sockaddr_in slam_addr;
        memset(&slam_addr, 0, sizeof(slam_addr));
        slam_addr.sin_family = AF_INET;
        slam_addr.sin_port = htons(slam_port);
        inet_pton(AF_INET, slam_ip, &slam_addr.sin_addr);
        
        if (false) {
            std::cerr << "[地图切换] 连接SLAM端失败: " << strerror(errno) << std::endl;
            close(sock);
            g_map_switch_in_progress = false;
            res.status = 500;
            res.set_content("{\"error\":\"Failed to connect to SLAM\"}", "application/json");
            return;
        }
        
        // 🔴 构造地图切换请求（参考地图切换机制分析报告.md）
        struct req_frame {
            unsigned long frame_type;  // 目标地图ID
            unsigned long seq;         // 请求序列号
            float x;                   // 当前X坐标
            float y;                   // 当前Y坐标
            float yaw;                 // 当前航向角
        };
        
        req_frame request;
        request.frame_type = target_map_id;
        request.seq = time(nullptr);  // 使用时间戳作为序列号
        request.x = current_x;
        request.y = current_y;
        request.yaw = current_yaw;
        
        // 发送请求
        ssize_t sent = sendto(sock, &request, sizeof(request), 0,
                              (struct sockaddr*)&slam_addr, sizeof(slam_addr));
        if (sent < 0) {
            std::cerr << "[地图切换] 发送请求失败: " << strerror(errno) << std::endl;
            close(sock);
            g_map_switch_in_progress = false;
            res.status = 500;
            res.set_content("{\"error\":\"Failed to send request to SLAM\"}", "application/json");
            return;
        }
        
        std::cout << "[地图切换] 已发送切换请求到SLAM端 " << slam_ip << ":" << slam_port << std::endl;
        
        // 🔴 在后台线程中等待SLAM响应
        std::thread([sock, slam_addr, request]() mutable {
            struct replay_frame {
                bool result;
                unsigned long seq;
                unsigned char status;
            };
            
            std::cout << "[地图切换] 后台线程开始等待SLAM响应..." << std::endl;
            
            const unsigned char UDP_REPLY_ACK = 1;
            const unsigned char UDP_REPLY_FINAL = 2;
            const auto start_time = std::chrono::steady_clock::now();
            bool finished = false;
            bool success = false;

            while (true) {
                const auto now = std::chrono::steady_clock::now();
                const auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - start_time).count();
                if (elapsed >= 60) {
                    std::cerr << "[地图切换] 等待SLAM端最终响应超时" << std::endl;
                    break;
                }

                replay_frame udp_reply;
                memset(&udp_reply, 0, sizeof(udp_reply));
                struct sockaddr_in from_addr;
                socklen_t from_len = sizeof(from_addr);
                ssize_t udp_recv_len = recvfrom(sock, &udp_reply, sizeof(udp_reply), 0,
                                                (struct sockaddr*)&from_addr, &from_len);

                if (udp_recv_len < 0) {
                    if (errno == EAGAIN || errno == EWOULDBLOCK) {
                        ssize_t retry_sent = sendto(sock, &request, sizeof(request), 0,
                                                    (struct sockaddr*)&slam_addr, sizeof(slam_addr));
                        if (retry_sent < 0) {
                            std::cerr << "[地图切换] UDP重发请求失败: " << strerror(errno) << std::endl;
                            break;
                        }
                        continue;
                    }
                    if (errno == EINTR) continue;
                    std::cerr << "[地图切换] UDP接收响应失败: " << strerror(errno) << std::endl;
                    break;
                }

                if (udp_recv_len != (ssize_t)sizeof(udp_reply)) {
                    std::cerr << "[地图切换] 忽略长度不匹配的UDP响应: " << udp_recv_len
                              << " / " << sizeof(udp_reply) << std::endl;
                    continue;
                }

                if (udp_reply.seq != request.seq) {
                    std::cerr << "[地图切换] 忽略seq不匹配的响应: " << udp_reply.seq
                              << "，期望 " << request.seq << std::endl;
                    continue;
                }

                std::cout << "[地图切换] 收到UDP响应: status=" << (int)udp_reply.status
                          << ", result=" << (int)udp_reply.result
                          << ", seq=" << udp_reply.seq << std::endl;

                if (udp_reply.status == UDP_REPLY_ACK) {
                    continue;
                }

                if (udp_reply.status == UDP_REPLY_FINAL) {
                    finished = true;
                    success = udp_reply.result;
                    break;
                }
            }

            close(sock);
            g_map_switch_completed = finished && success;
            g_map_switch_in_progress = false;
            std::cout << "[地图切换] 后台线程结束，状态: "
                      << (g_map_switch_completed ? "成功" : "失败") << std::endl;
            return;

#if 0
            replay_frame reply;
            memset(&reply, 0, sizeof(reply));  // 初始化结构体
            ssize_t recv_len = recv(sock, &reply, sizeof(reply), 0);
            
            std::cout << "[地图切换] recv返回: recv_len=" << recv_len 
                      << ", result=" << (int)reply.result 
                      << ", seq=" << reply.seq << std::endl;
            
            close(sock);
            
            if (recv_len == sizeof(reply) && reply.result) {
                std::cout << "[地图切换] ✓ SLAM端切换成功，seq=" << reply.seq << std::endl;
                g_map_switch_completed = true;
            } else {
                std::cerr << "[地图切换] ✗ SLAM端切换失败或超时，recv_len=" << recv_len 
                          << " (期望=" << sizeof(reply) << ")" << std::endl;
                if (recv_len > 0) {
                    std::cerr << "[地图切换] reply.result=" << (int)reply.result 
                              << ", reply.seq=" << reply.seq << std::endl;
                }
                g_map_switch_completed = false;
            }
            
            g_map_switch_in_progress = false;
            std::cout << "[地图切换] 后台线程结束，状态: " 
                      << (g_map_switch_completed ? "成功" : "失败") << std::endl;
#endif
        }).detach();
        
        res.set_content("{\"status\":\"ok\",\"message\":\"Map switch request sent\"}", "application/json");
    });

    // ---- 🔴 新增：检查地图切换状态 ----
    svr.Get("/api/check_map_switch_status", [](const httplib::Request&, httplib::Response& res) {
        res.set_header("Access-Control-Allow-Origin", "*");
        
        std::ostringstream ss;
        if (g_map_switch_completed) {
            ss << "{\"status\":\"completed\"}";
        } else if (g_map_switch_in_progress) {
            ss << "{\"status\":\"in_progress\"}";
        } else {
            ss << "{\"status\":\"idle\"}";
        }
        
        res.set_content(ss.str(), "application/json");
    });

    // ---- 获取地图文件（前端拉取）----
    svr.Get("/api/map_image", [&ftp_server](const httplib::Request&, httplib::Response& res) {
        res.set_header("Access-Control-Allow-Origin", "*");
        std::lock_guard<std::mutex> lk(g_map_mutex);

        // 优先返回 FTP 上传的最新图片文件（跳过 .yaml 等非图片文件）
        auto ftp_images = ftp_server.get_received_images();
        if (!ftp_images.empty()) {
            // 从后往前找，找到第一个图片文件
            for (auto it = ftp_images.rbegin(); it != ftp_images.rend(); ++it) {
                const std::string& filepath = *it;
                // 只处理图片文件，跳过 .yaml 等配置文件
                if (filepath.find(".yaml") != std::string::npos || 
                    filepath.find(".yml") != std::string::npos) {
                    continue;
                }
                
                std::ifstream f(filepath, std::ios::binary);
                if (f.is_open()) {
                    std::string data((std::istreambuf_iterator<char>(f)),
                                      std::istreambuf_iterator<char>());
                    // 根据文件扩展名设置 Content-Type
                    std::string content_type = "image/x-portable-graymap";
                    if (filepath.find(".png") != std::string::npos) content_type = "image/png";
                    else if (filepath.find(".jpg") != std::string::npos || filepath.find(".jpeg") != std::string::npos) content_type = "image/jpeg";
                    res.set_content(data, content_type);
                    std::cout << "返回 FTP 地图: " << filepath << std::endl;
                    return;
                }
            }
        }

        // 其次尝试 SLAM 推送的地图文件（png 或 pgm）
        std::vector<std::pair<std::string,std::string>> candidates = {
            {"slam_map.png", "image/png"},
            {"slam_map.pgm", "image/x-portable-graymap"}
        };
        for (const auto& c : candidates) {
            std::ifstream f(c.first, std::ios::binary);
            if (f.is_open()) {
                std::string data((std::istreambuf_iterator<char>(f)),
                                  std::istreambuf_iterator<char>());
                res.set_content(data, c.second);
                std::cout << "返回 SLAM 地图: " << c.first << std::endl;
                return;
            }
        }
        res.status = 404;
        res.set_content("{\"error\":\"no map\"}", "application/json");
    });

    // ---- SLAM 推送像素坐标列表 ----
    // Body JSON: [{"seq":1,"px":120,"py":340}, ...]
    svr.Post("/api/upload_pixels", [](const httplib::Request& req, httplib::Response& res) {
        res.set_header("Access-Control-Allow-Origin", "*");
        // 简单手动解析 JSON 数组
        std::vector<PixelPoint> pts;
        const std::string& b = req.body;
        size_t pos = 0;
        while ((pos = b.find('{', pos)) != std::string::npos) {
            size_t end = b.find('}', pos);
            if (end == std::string::npos) break;
            std::string obj = b.substr(pos, end - pos + 1);
            PixelPoint p{};
            auto getInt = [&](const std::string& key) -> int {
                std::string k = "\"" + key + "\"";
                auto kp = obj.find(k);
                if (kp == std::string::npos) return 0;
                auto cp = obj.find(':', kp + k.size());
                if (cp == std::string::npos) return 0;
                ++cp;
                while (cp < obj.size() && obj[cp] == ' ') ++cp;
                std::string num;
                for (; cp < obj.size(); ++cp) {
                    char c = obj[cp];
                    if (c == ',' || c == '}' || c == ' ') break;
                    num += c;
                }
                return num.empty() ? 0 : std::stoi(num);
            };
            p.seq = getInt("seq");
            p.px  = getInt("px");
            p.py  = getInt("py");
            pts.push_back(p);
            pos = end + 1;
        }
        {
            std::lock_guard<std::mutex> lk(g_points_mutex);
            g_pixels = std::move(pts);
        }
        std::cout << "像素坐标已更新，共 " << g_pixels.size() << " 个点" << std::endl;
        res.set_content("{\"status\":\"ok\"}", "application/json");
    });

    // ---- 获取像素坐标列表（前端拉取）----
    svr.Get("/api/get_pixels", [](const httplib::Request&, httplib::Response& res) {
        res.set_header("Access-Control-Allow-Origin", "*");
        std::lock_guard<std::mutex> lk(g_points_mutex);
        std::ostringstream ss;
        ss << "[";
        for (size_t i = 0; i < g_pixels.size(); ++i) {
            if (i) ss << ",";
            ss << "{\"seq\":" << g_pixels[i].seq
               << ",\"px\":"  << g_pixels[i].px
               << ",\"py\":"  << g_pixels[i].py << "}";
        }
        ss << "]";
        res.set_content(ss.str(), "application/json");
    });

    // ---- 获取FTP接收到的图片列表 ----
    svr.Get("/api/get_ftp_images", [&ftp_server](const httplib::Request&, httplib::Response& res) {
        res.set_header("Access-Control-Allow-Origin", "*");
        auto images = ftp_server.get_received_images();
        std::ostringstream ss;
        ss << "[";
        for (size_t i = 0; i < images.size(); ++i) {
            if (i) ss << ",";
            ss << "\"" << images[i] << "\"";
        }
        ss << "]";
        res.set_content(ss.str(), "application/json");
    });

    // ---- 获取FTP接收的图片文件 ----
    svr.Get(R"(/api/ftp_image/(.+))", [](const httplib::Request& req, httplib::Response& res) {
        res.set_header("Access-Control-Allow-Origin", "*");
        std::string filename = req.matches[1];
        std::string filepath = "../ftp_files/images/" + filename;
        
        std::ifstream file(filepath, std::ios::binary);
        if (!file.is_open()) {
            res.status = 404;
            res.set_content("{\"error\":\"file not found\"}", "application/json");
            return;
        }
        
        std::string content((std::istreambuf_iterator<char>(file)),
                            std::istreambuf_iterator<char>());
        
        // 根据文件扩展名设置Content-Type
        std::string content_type = "image/jpeg";
        if (filename.find(".png") != std::string::npos) content_type = "image/png";
        else if (filename.find(".gif") != std::string::npos) content_type = "image/gif";
        else if (filename.find(".bmp") != std::string::npos) content_type = "image/bmp";
        else if (filename.find(".pgm") != std::string::npos) content_type = "image/x-portable-graymap";
        
        res.set_content(content, content_type);
    });

    std::cout << "HTTP服务器启动在 " << SERVER_IP << ":" << HTTP_PORT << std::endl;
    if (!svr.listen(SERVER_IP, HTTP_PORT)) {
        std::cerr << "HTTP服务器启动失败！" << std::endl;
        return -1;
    }
    return 0;
}
