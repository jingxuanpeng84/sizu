#ifndef ROBOT_UI__CPP
#define ROBOT_UI__CPP

#include <SFML/Graphics.hpp>
#include <eigen3/Eigen/Dense>
#include <vector>
#include <thread>
#include <mutex>
#include <cstdlib>
#include <ctime>
#include <iomanip>
#include <sstream>
#include <iostream>
#include <cmath>

using namespace Eigen;

const float UNIT_TO_PIXEL = 100.0f;     // 0.01m corresponds to 1 pixel
const float ROBOT_RADIUS_METERS = 0.25f; // Robot radius in meters
const float ROBOT_RADIUS_PIXELS = ROBOT_RADIUS_METERS * UNIT_TO_PIXEL;
const sf::Time UPDATE_INTERVAL = sf::milliseconds(100);
const float SCALE_INTERVAL = 0.5f;         // 0.5 meters between ticks
const float OBSTACLE_RADIUS_METERS = 0.05f; // Obstacle radius in meters
const float OBSTACLE_RADIUS_PIXELS = OBSTACLE_RADIUS_METERS * UNIT_TO_PIXEL;

class Obstacle
{
public:
    sf::CircleShape shape;
    sf::Vector2f position;

    Obstacle(float radius, sf::Color color, sf::Vector2f pos) : position(pos)
    {
        shape.setRadius(radius);
        shape.setFillColor(color);
        updateShapePosition();
    }

    void updatePosition(sf::Vector2f newPos)
    {
        position = newPos;
        updateShapePosition();
    }

private:
    void updateShapePosition()
    {
        shape.setPosition(position * UNIT_TO_PIXEL);
        shape.setOrigin(shape.getRadius(), shape.getRadius());
    }
};

class Robot_UI
{
public:
    sf::CircleShape shape;
    sf::RectangleShape directionArrow;
    sf::Vector2f position;
    float angle; // Rotation angle in degrees

    Robot_UI(float radius, sf::Color color, sf::Vector2f startPosition) : angle(0)
    {
        shape.setRadius(radius);
        shape.setFillColor(color);
        shape.setOrigin(radius, radius); // Set origin to center of the circle

        directionArrow.setSize(sf::Vector2f(radius, radius / 4)); // Arrow size
        directionArrow.setFillColor(sf::Color::Blue);
        directionArrow.setOrigin(directionArrow.getSize().x / 2, directionArrow.getSize().y / 2); // Center the arrow

        position = startPosition;
        updateShapePosition();
    }

    void updatePosition()
    {
        updateShapePosition();
        directionArrow.setRotation(angle); // Update arrow rotation
    }

private:
    void updateShapePosition()
    {
        sf::Vector2f tempposition;  //将机器人的显示位置限制在原点
        tempposition.x = 0;
        tempposition.y = 0;
        shape.setPosition(tempposition * UNIT_TO_PIXEL);
        directionArrow.setPosition(tempposition * UNIT_TO_PIXEL); // Arrow position matches the robot position
    }
};

namespace UIRENDER
{
    struct coordination
    {
        double x;
        double y;
    };
};

class uidata
{
    UIRENDER::coordination obs[181];
    unsigned int obs_num;
    UIRENDER::coordination robot_x_y;
    double robot_yaw;
    unsigned int update_flag = 0;
    std::mutex uidata_access_mutex;

public:
    void getObsRobotData(std::vector<Obstacle> &obstacles, Robot_UI &robot);
    void setObsRobotData(std::vector<Vector2d> obstacle, VectorXd state);
};

void robot_ui_thread(uidata* uihandle);
void updateObstacles(std::vector<Obstacle> &obstacles);
void drawAxes(sf::RenderWindow &window, sf::View &view, float unitToPixel);

#endif