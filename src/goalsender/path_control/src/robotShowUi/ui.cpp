#include "path_control/include/robotShowUi/ui.hpp"

void drawAxes(sf::RenderWindow &window, sf::View &view, float unitToPixel)
{
    sf::Vector2f viewCenter = view.getCenter();
    sf::Vector2f viewSize = view.getSize();

    sf::Vertex xAxis[] = {
        sf::Vertex(sf::Vector2f(viewCenter.x - viewSize.x / 2.0f, viewCenter.y), sf::Color::Black),
        sf::Vertex(sf::Vector2f(viewCenter.x + viewSize.x / 2.0f, viewCenter.y), sf::Color::Black)};
    window.draw(xAxis, 2, sf::Lines);

    sf::Vertex yAxis[] = {
        sf::Vertex(sf::Vector2f(viewCenter.x, viewCenter.y - viewSize.y / 2.0f), sf::Color::Black),
        sf::Vertex(sf::Vector2f(viewCenter.x, viewCenter.y + viewSize.y / 2.0f), sf::Color::Black)};
    window.draw(yAxis, 2, sf::Lines);

    sf::Font font;
    if (!font.loadFromFile("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"))
    {
        std::cerr << "Failed to load font" << std::endl;
        exit(-1);
    }

    sf::Text text;
    text.setFont(font);
    text.setCharacterSize(12);
    text.setFillColor(sf::Color::Black);

    float tickIntervalPixels = SCALE_INTERVAL * unitToPixel;
    int numXTicks = static_cast<int>(viewSize.x / tickIntervalPixels);

    // Draw x-axis ticks and labels
    for (int i = -numXTicks; i <= numXTicks; ++i)
    {
        float x = i * tickIntervalPixels + viewCenter.x;
        sf::Vertex line[] = {
            sf::Vertex(sf::Vector2f(x, viewCenter.y - 5), sf::Color::Black),
            sf::Vertex(sf::Vector2f(x, viewCenter.y + 5), sf::Color::Black)};
        window.draw(line, 2, sf::Lines);

        std::ostringstream oss;
        oss << std::fixed << std::setprecision(1) << i * SCALE_INTERVAL;
        text.setString(oss.str() + " m");
        text.setPosition(x + 2, viewCenter.y + 10);
        window.draw(text);
    }

    // Draw y-axis ticks and labels
    int numYTicks = static_cast<int>(viewSize.y / tickIntervalPixels);
    for (int i = -numYTicks; i <= numYTicks; ++i)
    {
        float y = i * tickIntervalPixels + viewCenter.y;
        sf::Vertex line[] = {
            sf::Vertex(sf::Vector2f(viewCenter.x - 5, y), sf::Color::Black),
            sf::Vertex(sf::Vector2f(viewCenter.x + 5, y), sf::Color::Black)};
        window.draw(line, 2, sf::Lines);

        std::ostringstream oss;
        oss << std::fixed << std::setprecision(1) << i * SCALE_INTERVAL;
        text.setString(oss.str() + " m");
        text.setPosition(viewCenter.x + 10, y - 10);
        window.draw(text);
    }
}

void uidata::getObsRobotData(std::vector<Obstacle> &obstacles, Robot_UI &robot)
{
    this->uidata_access_mutex.lock();
    if (this->update_flag == 1)
    {
        // update
        this->update_flag = 0;
        obstacles.clear();
        for (unsigned int i = 0; i < this->obs_num; i++)
        {
            sf::Vector2f position(this->obs[i].x - this->robot_x_y.x, this->obs[i].y - this->robot_x_y.y);
            obstacles.emplace_back(OBSTACLE_RADIUS_PIXELS, sf::Color::Red, position);
        }
        robot.position.x = this->robot_x_y.x;
        robot.position.y = this->robot_x_y.y;
        robot.angle = this->robot_yaw * 180 / M_PI;
        // std::cout << "bos num = " << obstacles.size() << std::endl;
    }
    this->uidata_access_mutex.unlock();
}

void uidata::setObsRobotData(std::vector<Vector2d> obstacle, VectorXd state)
{
    this->uidata_access_mutex.lock();
    this->obs_num = obstacle.size();
    for (unsigned int i = 0; i < obstacle.size(); i++)
    {
        this->obs[i].x = obstacle[i][0];
        this->obs[i].y = obstacle[i][1];
    }
    this->robot_x_y.x = state[0];
    this->robot_x_y.y = state[1];
    this->robot_yaw = state[2];
    this->update_flag = 1;
    this->uidata_access_mutex.unlock();
}

void updateObstacles(std::vector<Obstacle> &obstacles)
{
    for (auto &obstacle : obstacles)
    {
        // Adjust the position if necessary
        sf::Vector2f pos = obstacle.position;
        obstacle.updatePosition(pos);
    }
}

void robot_ui_thread(uidata *uihandle)
{
    sf::RenderWindow window(sf::VideoMode(800, 600), "Robot and Obstacles");
    window.setFramerateLimit(60);

    // Initialize the view to cover a large area
    sf::View view(sf::FloatRect(-1000, -750, 2000, 1500)); // Larger view area
    view.setCenter(0.f, 0.f);                              // Center view at origin

    sf::Clock clock;
    sf::Time elapsed = sf::Time::Zero;

    Robot_UI robot(ROBOT_RADIUS_PIXELS, sf::Color::Green, sf::Vector2f(0, 0)); // Robot starts at origin

    std::vector<Obstacle> obstacles;
    uihandle->getObsRobotData(obstacles, robot);
    sf::Font font;
    if (!font.loadFromFile("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"))
    {
        std::cerr << "Failed to load font" << std::endl;
        pthread_exit(NULL);
    }
    sf::Text coordinates;
    coordinates.setFont(font);
    coordinates.setCharacterSize(20);
    coordinates.setFillColor(sf::Color::Black);

    bool isDragging = false;
    sf::Vector2i lastMousePos;

    while (window.isOpen())
    {
        sf::Time deltaTime = clock.restart();
        elapsed += deltaTime;

        sf::Event event;
        while (window.pollEvent(event))
        {
            if (event.type == sf::Event::Closed)
            {
                window.close();
            }
            else if (event.type == sf::Event::Resized)
            {
                // Adjust view size on window resize
                view.setSize(static_cast<float>(event.size.width), static_cast<float>(event.size.height));
            }
            else if (event.type == sf::Event::MouseButtonPressed)
            {
                
                if (event.mouseButton.button == sf::Mouse::Left)
                {
                    // std::cout << "left pressed\n";
                    if (view.getViewport().contains(static_cast<sf::Vector2f>(sf::Mouse::getPosition(window))))
                    {
                        
                        isDragging = true;
                        lastMousePos = sf::Mouse::getPosition(window);
                    }
                }
            }
            else if (event.type == sf::Event::MouseButtonReleased)
            {
                if (event.mouseButton.button == sf::Mouse::Left)
                {
                    isDragging = false;
                }
            }
            else if (event.type == sf::Event::MouseMoved)
            {
                // std::cout << "move\n";
                if (isDragging)
                {
                    sf::Vector2i currentMousePos = sf::Mouse::getPosition(window);
                    sf::Vector2f delta = static_cast<sf::Vector2f>(currentMousePos - lastMousePos);
                    view.move(-delta.x, delta.y); // Invert Y to match SFML's coordinate system
                    lastMousePos = currentMousePos;
                    // std::cout << "drag move\n";
                }
            }
        }

        if (elapsed >= UPDATE_INTERVAL)
        {
            // Update robot position with continuous motion and random angle for demonstration
            uihandle->getObsRobotData(obstacles, robot);
            robot.updatePosition();

            // Draw obstacles
            updateObstacles(obstacles);

            elapsed = sf::Time::Zero;
        }

        window.clear(sf::Color::White);

        window.setView(view);

        // Draw coordinate axes and ticks
        drawAxes(window, view, UNIT_TO_PIXEL);
        for (const auto &obstacle : obstacles)
        {
            window.draw(obstacle.shape);
        }
        // Draw robot and its direction arrow
        window.draw(robot.shape);
        window.draw(robot.directionArrow); // Draw direction arrow

        // Draw robot coordinates
        coordinates.setString("Robot Position: (" +
                              std::to_string(robot.position.x) + " m, " +
                              std::to_string(robot.position.y) + " m)");
        coordinates.setPosition(-view.getSize().x / 2, -view.getSize().y / 2);
        window.draw(coordinates);

        window.display();
    }

    pthread_exit(NULL);
}