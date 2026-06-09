// 地图切换器 - 机器狗运行时使用
// 当机器狗经过换图点时，自动切换到新地图的路径

#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <cmath>

struct PathPoint {
    int    seq;
    double x;
    double y;
    double yaw;
    int    keypointflag; // 5 表示换图点
};

struct MapSwitchPoint {
    int from_map_id;      // 当前地图ID
    int to_map_id;        // 目标地图ID
    double x, y;          // 换图点坐标
    double threshold;     // 触发距离阈值
};

class MapSwitcher {
private:
    int current_map_id_;
    std::vector<PathPoint> current_path_;
    std::vector<MapSwitchPoint> switch_points_;
    
public:
    MapSwitcher() : current_map_id_(1) {}
    
    // 加载指定地图的路径
    bool loadMapPath(int map_id) {
        std::string xml_file = "map_" + std::to_string(map_id) + "_waypoints.xml";
        std::ifstream f(xml_file);
        if (!f.is_open()) {
            std::cerr << "无法打开地图路径文件: " << xml_file << std::endl;
            return false;
        }
        
        // 这里需要实现完整的XML解析
        // 简化示例：假设已经解析完成
        current_path_.clear();
        current_map_id_ = map_id;
        
        std::cout << "已加载地图 " << map_id << " 的路径: " << xml_file << std::endl;
        return true;
    }
    
    // 注册换图点
    void registerSwitchPoint(int from_map, int to_map, double x, double y, double threshold = 0.5) {
        MapSwitchPoint sp;
        sp.from_map_id = from_map;
        sp.to_map_id = to_map;
        sp.x = x;
        sp.y = y;
        sp.threshold = threshold;
        switch_points_.push_back(sp);
        
        std::cout << "注册换图点: 地图" << from_map << " -> 地图" << to_map 
                  << " 位置(" << x << ", " << y << ")" << std::endl;
    }
    
    // 检查当前位置是否触发地图切换
    bool checkAndSwitch(double current_x, double current_y) {
        for (const auto& sp : switch_points_) {
            // 只检查当前地图的换图点
            if (sp.from_map_id != current_map_id_) continue;
            
            // 计算距离
            double dx = current_x - sp.x;
            double dy = current_y - sp.y;
            double distance = std::sqrt(dx * dx + dy * dy);
            
            // 如果在阈值范围内，触发地图切换
            if (distance <= sp.threshold) {
                std::cout << "触发地图切换！当前位置(" << current_x << ", " << current_y 
                          << ") 距离换图点 " << distance << "m" << std::endl;
                std::cout << "从地图 " << current_map_id_ << " 切换到地图 " << sp.to_map_id << std::endl;
                
                // 切换到新地图
                return loadMapPath(sp.to_map_id);
            }
        }
        return false;
    }
    
    // 获取当前地图ID
    int getCurrentMapId() const { return current_map_id_; }
    
    // 获取当前路径
    const std::vector<PathPoint>& getCurrentPath() const { return current_path_; }
    
    // 从路径文件中提取换图点并自动注册
    void autoRegisterSwitchPoints() {
        // 遍历所有地图文件，找出换图点
        for (int map_id = 1; map_id <= 10; ++map_id) {
            std::string xml_file = "map_" + std::to_string(map_id) + "_waypoints.xml";
            std::ifstream f(xml_file);
            if (!f.is_open()) continue;
            
            // 解析XML，找出 keypointflag == 5 的点
            // 简化示例：假设已经解析完成
            // 这里需要实现完整的XML解析逻辑
            
            std::cout << "扫描地图 " << map_id << " 的换图点..." << std::endl;
        }
    }
};

// 使用示例
int main() {
    MapSwitcher switcher;
    
    // 加载初始地图（一楼）
    switcher.loadMapPath(1);
    
    // 注册换图点：一楼到二楼
    // 假设一楼最后一个点坐标为 (10.5, 20.3)，这是换图点
    switcher.registerSwitchPoint(1, 2, 10.5, 20.3, 0.5);
    
    // 注册换图点：二楼到一楼
    // 假设二楼第一个点坐标为 (0.2, 0.5)，这是返回一楼的换图点
    switcher.registerSwitchPoint(2, 1, 0.2, 0.5, 0.5);
    
    // 模拟机器狗运行
    std::cout << "\n=== 模拟机器狗运行 ===" << std::endl;
    
    // 在一楼移动
    std::cout << "\n机器狗在一楼移动..." << std::endl;
    switcher.checkAndSwitch(5.0, 10.0);  // 普通位置，不切换
    
    // 接近换图点
    std::cout << "\n机器狗接近换图点..." << std::endl;
    switcher.checkAndSwitch(10.4, 20.2);  // 接近换图点，触发切换到二楼
    
    // 在二楼移动
    std::cout << "\n机器狗在二楼移动..." << std::endl;
    switcher.checkAndSwitch(5.0, 8.0);   // 二楼普通位置
    
    // 返回一楼
    std::cout << "\n机器狗返回一楼..." << std::endl;
    switcher.checkAndSwitch(0.3, 0.6);   // 触发切换回一楼
    
    std::cout << "\n当前地图ID: " << switcher.getCurrentMapId() << std::endl;
    
    return 0;
}
