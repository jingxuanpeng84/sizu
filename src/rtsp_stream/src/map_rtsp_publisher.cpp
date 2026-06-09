/**
 * @file map_rtsp_publisher.cpp
 * @brief ROS 地图 RTSP 推流节点 (C++ 实现)
 * 
 * 功能：将 ROS OccupancyGrid 地图转换为视频流并通过 RTSP 推送
 */

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <opencv2/opencv.hpp>
#include <thread>
#include <mutex>
#include <atomic>
#include <csignal>

// 全局变量
cv::Mat g_latest_map;
std::mutex g_map_mutex;
std::atomic<bool> g_running(true);
std::atomic<bool> g_has_map(false);
FILE* g_ffmpeg_pipe = nullptr;

// 配置参数
struct Config {
    std::string rtsp_url;
    int fps;
    int out_width;
    int out_height;
    std::string map_topic;
    
    Config() : rtsp_url("rtsp://127.0.0.1:8554/mapping"),
               fps(15),
               out_width(800),
               out_height(600),
               map_topic("/projected_map") {}
};

Config g_config;

// 信号处理
void signalHandler(int signum) {
    ROS_INFO("Received signal %d, shutting down...", signum);
    g_running = false;
    
    if (g_ffmpeg_pipe) {
        pclose(g_ffmpeg_pipe);
        g_ffmpeg_pipe = nullptr;
    }
    
    ros::shutdown();
}

/**
 * @brief 将 OccupancyGrid 转换为 OpenCV 图像
 */
cv::Mat mapToImage(const nav_msgs::OccupancyGrid::ConstPtr& map_msg) {
    int width = map_msg->info.width;
    int height = map_msg->info.height;
    
    // 创建图像
    cv::Mat img(height, width, CV_8UC3);
    
    // 填充颜色
    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            int index = (height - 1 - y) * width + x;  // 上下翻转
            int8_t value = map_msg->data[index];
            
            cv::Vec3b color;
            if (value == -1) {
                color = cv::Vec3b(128, 128, 128);  // 未知 → 灰色
            } else if (value == 0) {
                color = cv::Vec3b(255, 255, 255);  // 空闲 → 白色
            } else {
                color = cv::Vec3b(0, 0, 0);        // 占用 → 黑色
            }
            
            img.at<cv::Vec3b>(y, x) = color;
        }
    }
    
    // 缩放处理
    double scale = std::min(
        static_cast<double>(g_config.out_width) / width,
        static_cast<double>(g_config.out_height) / height
    );
    scale = std::min(scale, 1.0);  // 不放大
    
    if (scale < 1.0) {
        int new_w = static_cast<int>(width * scale);
        int new_h = static_cast<int>(height * scale);
        cv::resize(img, img, cv::Size(new_w, new_h), 0, 0, cv::INTER_NEAREST);
    }
    
    // 居中放置到画布
    cv::Mat canvas = cv::Mat::zeros(g_config.out_height, g_config.out_width, CV_8UC3);
    int h_offset = (g_config.out_height - img.rows) / 2;
    int w_offset = (g_config.out_width - img.cols) / 2;
    
    img.copyTo(canvas(cv::Rect(w_offset, h_offset, img.cols, img.rows)));
    
    return canvas;
}

/**
 * @brief ROS 地图回调
 */
void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(g_map_mutex);
    g_latest_map = mapToImage(msg);
    g_has_map = true;
}

/**
 * @brief FFmpeg 推流线程
 */
void streamingThread() {
    // 等待第一帧地图
    ROS_INFO("Waiting for first map...");
    while (!g_has_map && g_running) {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    
    if (!g_running) return;
    
    ROS_INFO("First map received, starting FFmpeg...");
    
    // 构建 FFmpeg 命令
    std::stringstream cmd;
    cmd << "ffmpeg -y"
        << " -f rawvideo"
        << " -pix_fmt bgr24"
        << " -s " << g_config.out_width << "x" << g_config.out_height
        << " -r " << g_config.fps
        << " -i -"  // 从 stdin 读取
        << " -c:v libx264"
        << " -preset ultrafast"
        << " -tune zerolatency"
        << " -profile:v baseline"
        << " -pix_fmt yuv420p"
        << " -f rtsp"
        << " -rtsp_transport tcp"
        << " " << g_config.rtsp_url
        << " 2>&1";  // 重定向错误输出
    
    ROS_INFO("FFmpeg command: %s", cmd.str().c_str());
    
    // 启动 FFmpeg
    g_ffmpeg_pipe = popen(cmd.str().c_str(), "w");
    if (!g_ffmpeg_pipe) {
        ROS_ERROR("Failed to start FFmpeg");
        g_running = false;
        return;
    }
    
    ROS_INFO("FFmpeg started successfully");
    ROS_INFO("Stream URL: %s", g_config.rtsp_url.c_str());
    
    // 推流循环
    ros::Rate rate(g_config.fps);
    while (g_running && ros::ok()) {
        cv::Mat frame;
        {
            std::lock_guard<std::mutex> lock(g_map_mutex);
            if (!g_latest_map.empty()) {
                frame = g_latest_map.clone();
            }
        }
        
        if (!frame.empty()) {
            // 写入 FFmpeg
            size_t written = fwrite(frame.data, 1, frame.total() * frame.elemSize(), g_ffmpeg_pipe);
            if (written != frame.total() * frame.elemSize()) {
                ROS_ERROR("FFmpeg pipe broken");
                break;
            }
            fflush(g_ffmpeg_pipe);
        }
        
        rate.sleep();
    }
    
    // 清理
    if (g_ffmpeg_pipe) {
        pclose(g_ffmpeg_pipe);
        g_ffmpeg_pipe = nullptr;
    }
    
    ROS_INFO("Streaming thread stopped");
}

int main(int argc, char** argv) {
    // 初始化 ROS
    ros::init(argc, argv, "map_rtsp_publisher");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");
    
    // 读取参数
    nh_private.param<std::string>("rtsp_url", g_config.rtsp_url, g_config.rtsp_url);
    nh_private.param<int>("fps", g_config.fps, g_config.fps);
    nh_private.param<int>("out_width", g_config.out_width, g_config.out_width);
    nh_private.param<int>("out_height", g_config.out_height, g_config.out_height);
    nh_private.param<std::string>("map_topic", g_config.map_topic, g_config.map_topic);
    
    ROS_INFO("========================================");
    ROS_INFO("Map RTSP Publisher Node Started");
    ROS_INFO("========================================");
    ROS_INFO("RTSP URL: %s", g_config.rtsp_url.c_str());
    ROS_INFO("FPS: %d", g_config.fps);
    ROS_INFO("Output Size: %dx%d", g_config.out_width, g_config.out_height);
    ROS_INFO("Map Topic: %s", g_config.map_topic.c_str());
    ROS_INFO("========================================");
    
    // 注册信号处理
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);
    
    // 订阅地图话题
    ros::Subscriber map_sub = nh.subscribe(g_config.map_topic, 1, mapCallback);
    
    // 启动推流线程
    std::thread stream_thread(streamingThread);
    
    // ROS 主循环
    ros::spin();
    
    // 等待推流线程结束
    g_running = false;
    if (stream_thread.joinable()) {
        stream_thread.join();
    }
    
    ROS_INFO("Node shutdown complete");
    
    return 0;
}
