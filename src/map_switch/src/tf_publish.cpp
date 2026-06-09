#include "tf_publish_node.hpp"
#include <signal.h>

// 全局指针，用于信号处理
TfPublish* g_tf_publish = nullptr;

/**
 * @brief 信号处理函数
 */
void signalHandler(int signum) {
    ROS_INFO("收到中断信号 (%d)，正在关闭TF发布服务...", signum);
    if (g_tf_publish) {
        g_tf_publish->endWork();
    }
    ros::shutdown();
}

int main(int argc,char *argv[])
{
    setlocale(LC_ALL,"");
    ros::init(argc,argv,"tf_publish");
    
    // 注册信号处理
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);
    
    
    // 创建TF发布服务实例
    g_tf_publish = new TfPublish();

    // 如果启用socket，启动网络服务
    if(g_tf_publish->socketEn_get()){
        if (!g_tf_publish->startWork()) {
            ROS_ERROR("[TfPublish] 网络服务启动失败!");
            delete g_tf_publish;
            return -1;
        }
    }

    // 启动主运行线程
    std::thread mainThread(&TfPublish::run, g_tf_publish);
    mainThread.detach();

    ROS_INFO("[TfPublish] 服务已启动，等待数据...");
    ros::spin();
    
    // 清理
    ROS_INFO("[TfPublish] 正在关闭服务...");
    if (g_tf_publish) {
        g_tf_publish->endWork();
        g_tf_publish->watingDeviceEnding();
        delete g_tf_publish;
        g_tf_publish = nullptr;
    }
    
    ROS_INFO("[TfPublish] 服务已关闭");
    return 0;
}
