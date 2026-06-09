
  // TCP方式
#pragma once

#include "elevator_controller.hpp"  
#include "map_switch.hpp"
#include <string>
#include <iostream>
#include <thread>
#include <chrono>
#include <sstream>
#include <iomanip>

class RobotElevatorClient {
public:
    // RobotElevatorClient(const std::string& device, int baudrate, char parity, int data_bits, int stop_bits, int slave_id);
    RobotElevatorClient(const std::string& ip, int port = 8000, int slave_id = 1);

    bool callElevatorAndOpenDoor(int FromFloor);
    bool rideToTargetFloorAndOpenDoor(int ToFloor);
    bool closeDoorAndSwitchMap(const req_frame& request, const char* server_addr, int map_switch_PORT);

private:
    ElevatorController m_controller;   

    // ===== 通用控制方法（带重试机制） =====
    bool sendRideCommandWithRetry(const std::string& floor, int retryLimit = 10, int intervalSec = 10);
    bool requestOpenMainDoorWithRetry(int retryLimit = 100, int intervalMs = 1000);
    bool requestCloseDoorWithRetry(int retryLimit = 100, int intervalMs = 1000);

    // ===== 电梯状态等待逻辑 =====
    bool waitElevatorOnlineAndActive(int timeout_sec = 100);
};


