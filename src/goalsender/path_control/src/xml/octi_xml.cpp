#include "path_control/include/octi_xml.hpp"

octiXml::octiXml()
{
    this->threadEndFlag = false;
    this->pathRecordFlag = false;
    this->pathRecordCancelFlag = false;
    this->sequence = 0;
    this->xmlUiRenderThreadHandler = NULL; // type of pointer must set NULL before init ,otherwise will occur error when free obj
}

octiXml::~octiXml()
{
    if (this->xmlUiRenderThreadHandler != NULL)
    {
        if (this->xmlUiRenderThreadHandler->joinable())
        {
            this->xmlUiRenderThreadHandler->join();
            std::cout << "Ui close success\n";
        }
    }
}

bool octiXml::getUiThreadEndFlag()
{
    return this->threadEndFlag;
}

bool octiXml::getCancelFlag()
{
    return this->pathRecordCancelFlag;
}

bool octiXml::getPathRecordFlag(void)
{
    return this->pathRecordFlag;
}

void octiXml::setPathRecordFlag(bool flag)
{
    this->pathRecordFlag = flag;
}

// bool octiXml::getKeyPathRecordFlag(void)
// {
//     return this->keyPathRecordFlag;
// }

// void octiXml::setKeyPathRecordFlag(bool flag)
// {
//     this->keyPathRecordFlag = flag;
// }

void octiXml::updatePathData(OCTIXML::pathNodeStruct pathData_)
{
    this->DataAccessMutex.lock();
    this->pathData = pathData_;
    this->DataAccessMutex.unlock();
    return;
}

void octiXml::setSequence(unsigned long sequence_)
{
    this->DataAccessMutex.lock();
    this->sequence = sequence_;
    this->DataAccessMutex.unlock();
}

bool octiXml::startUiWork()
{
    this->xmlUiRenderThreadHandler = new std::thread(std::bind(&octiXml::UiThreadFunc, this));
    std::cout << "UI start success!\n";
    return true;
}

bool octiXml::endUiWork()
{
    this->threadEndFlag = true;
    return true;
}

void octiXml::watingUiEnding()
{
    if (this->xmlUiRenderThreadHandler != NULL)
    {
        this->xmlUiRenderThreadHandler->join();
        this->xmlUiRenderThreadHandler = NULL;
    }
    std::cout << "UI finished\n";
}

tinyxml2::XMLDocument *octiXml::createXMLDocument(void)
{
    tinyxml2::XMLDocument *doc = new tinyxml2::XMLDocument();
    tinyxml2::XMLDeclaration *declaration = doc->NewDeclaration();
    doc->InsertEndChild(declaration);
    return doc;
}

tinyxml2::XMLElement *octiXml::createXMLRoot(tinyxml2::XMLDocument *doc)
{
    tinyxml2::XMLElement *root = doc->NewElement("Root");
    doc->InsertEndChild(root);
    return root;
}

void octiXml::insertPathNode(tinyxml2::XMLDocument *doc, tinyxml2::XMLElement *parentEle, OCTIXML::pathNodeStruct pathNode_)
{
    tinyxml2::XMLElement *pathEle = doc->NewElement("path");
    pathEle->SetAttribute("Seq", pathNode_.sequence);
    pathEle->SetAttribute("Type", "ulong");
    tinyxml2::XMLElement *xEle = doc->NewElement("x");
    xEle->SetAttribute("Type", "double");
    xEle->SetText(pathNode_.x);
    pathEle->InsertEndChild(xEle);
    tinyxml2::XMLElement *yEle = doc->NewElement("y");
    yEle->SetAttribute("Type", "double");
    yEle->SetText(pathNode_.y);
    pathEle->InsertEndChild(yEle);
    tinyxml2::XMLElement *yawEle = doc->NewElement("yaw");
    yawEle->SetAttribute("Type", "double");
    yawEle->SetText(pathNode_.yaw);
    pathEle->InsertEndChild(yawEle);
    tinyxml2::XMLElement *keyFlagEle = doc->NewElement("keypointflag");
    keyFlagEle->SetAttribute("Type", "uint");
    keyFlagEle->SetText(pathNode_.keyPointType);
    pathEle->InsertEndChild(keyFlagEle);
    tinyxml2::XMLElement *mapnumEle = doc->NewElement("mapnum");
    mapnumEle->SetAttribute("Type", "uint");
    mapnumEle->SetText(pathNode_.mapnum);
    pathEle->InsertEndChild(mapnumEle);
    parentEle->InsertEndChild(pathEle);
    return;
}

OCTIXML::KEYPONITTYPE octiXml::getPointType()
{
    OCTIXML::KEYPONITTYPE pointType_;
    this->pointTypeAccessMutex.lock();
    pointType_ = this->pointType;
    this->pointTypeAccessMutex.unlock();
    return pointType_;
}

bool octiXml::saveXmlFile(tinyxml2::XMLDocument *doc, const char *filePath)
{
    tinyxml2::XMLError err = doc->SaveFile(filePath);
    if (err == tinyxml2::XML_SUCCESS)
    {
        return true;
    }
    else
    {
        return false;
    }
}

// 调整文本大小
void octiXml::adjustTextSize(sf::Text &text, sf::RenderWindow &window, float scale)
{
    text.setCharacterSize(static_cast<unsigned int>(window.getSize().y * scale));
    sf::FloatRect textRect = text.getLocalBounds();
    text.setOrigin(textRect.left + textRect.width / 2.0f, textRect.top + textRect.height / 2.0f);
    text.setPosition((window.getSize().x - textRect.width) / 2.0f, text.getPosition().y);
}

void octiXml::UiThreadFunc(void)
{
    OCTIXML::pathNodeStruct pathData_;
    this->DataAccessMutex.lock();
    pathData_ = this->pathData;
    this->DataAccessMutex.unlock();
    // 创建一个窗口，设定最小窗口大小为800x600
    sf::RenderWindow window(sf::VideoMode(750, 400), "Path Record Window", sf::Style::Default);
    tgui::Gui gui(window);        // Create the GUI and attach it to the window
    window.setFramerateLimit(60); // 设置帧率限制

    // 初始化变量
    double x = 0.0f;
    double y = 0.0f;
    double yaw = 0.0f;
    int pointCount = 0;
    int path_save_flag = 0;
    // std::mutex mutex;

    // 使用默认字体
    sf::Font font;
    if (!font.loadFromFile("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"))
    {
        std::cerr << "无法加载字体" << std::endl;
        std::exit(EXIT_FAILURE);
    }

    // 标签
    sf::Text xLabel;
    xLabel.setFont(font);
    xLabel.setString("x: " + std::to_string(x));
    xLabel.setFillColor(sf::Color::White);
    xLabel.setCharacterSize(24); // 设置文本大小

    sf::Text yLabel;
    yLabel.setFont(font);
    yLabel.setString("y: " + std::to_string(y));
    yLabel.setFillColor(sf::Color::White);
    yLabel.setCharacterSize(24); // 设置文本大小

    sf::Text yawLabel;
    yawLabel.setFont(font);
    yawLabel.setString("yaw: " + std::to_string(y));
    yawLabel.setFillColor(sf::Color::White);
    yawLabel.setCharacterSize(24); // 设置文本大小

    sf::Text pointCountLabel;
    pointCountLabel.setFont(font);
    pointCountLabel.setString("Points Count: " + std::to_string(pathData_.sequence));
    pointCountLabel.setFillColor(sf::Color::White);
    pointCountLabel.setCharacterSize(24); // 设置文本大小

    // 确认按钮
    sf::RectangleShape confirmButton(sf::Vector2f(200.0f, 50.0f));
    confirmButton.setFillColor(sf::Color(41, 84, 223));

    sf::Text confirmButtonText;
    confirmButtonText.setFont(font);
    confirmButtonText.setString("Confirm");
    confirmButtonText.setFillColor(sf::Color::White);
    confirmButtonText.setCharacterSize(24); // 设置文本大小

    // 关键点记录按钮
    // sf::RectangleShape keyButton(sf::Vector2f(200.0f, 50.0f));
    // keyButton.setFillColor(sf::Color(255, 111, 74));

    // sf::Text keyButtonText;
    // keyButtonText.setFont(font);
    // keyButtonText.setString("KeyConfirm");
    // keyButtonText.setFillColor(sf::Color::White);
    // keyButtonText.setCharacterSize(24); // 设置文本大小

    // TGUI
    // unsigned
    //  Create a ComboBox (dropdown list)
    auto comboBox = tgui::ComboBox::create();
    // comboBox->setPosition(100, 100);
    comboBox->setSize(200, 50);
    comboBox->addItem("Normal point", "1");
    comboBox->addItem("Vision point", "2");
    comboBox->addItem("Charge point", "3");
    comboBox->addItem("Wait point", "4");
    comboBox->addItem("Inelevator point", "5");
    comboBox->addItem("Mapswitch point", "6");
    // comboBox->addItem("Other point", "4");
    // 连接ComboBox的选择事件
    comboBox->onItemSelect([this](const tgui::String item)
                           { 
                            this->pointTypeAccessMutex.lock();
                            if(item == "Normal point")
                            {
                                this->pointType = OCTIXML::KEYPONITTYPE::NORMAL;
                            }
                            else if (item == "Vision point")
                            {
                               this->pointType = OCTIXML::KEYPONITTYPE::VISION;
                            }
                            else if (item == "Charge point")
                            {
                               this->pointType = OCTIXML::KEYPONITTYPE::CHARGE;
                            }
                            else if (item == "Wait point")
                            {
                               this->pointType = OCTIXML::KEYPONITTYPE::WAIT;
                            }
                            else if (item == "Inelevator point")
                            {
                               this->pointType = OCTIXML::KEYPONITTYPE::INELEVATOR;
                            }
                            else if (item == "Mapswitch point")
                            {
                               this->pointType = OCTIXML::KEYPONITTYPE::MAPSWITCH;
                            }
                            else 
                            {
                                std::cout << "point type error\n";
                            }
                            this->pointTypeAccessMutex.unlock();
                            std::cout << "Selected item : " << item << std::endl; });

    tgui::ComboBoxRenderer *Render = comboBox->getRenderer();
    tgui::Color textColor(255, 111, 74);

    Render->setTextColor(textColor);
    Render->setTextSize(22);
    tgui::Color arrowColor = tgui::Color(255, 111, 74);
    tgui::Color arrowColorHover = tgui::Color(255, 190, 0);
    Render->setArrowColor(arrowColor);
    Render->setArrowColorHover(arrowColorHover);
    comboBox->setSelectedItemByIndex(0);
    gui.add(comboBox);

    // 结束按钮
    sf::RectangleShape exitButton(sf::Vector2f(200.0f, 50.0f));
    exitButton.setFillColor(sf::Color(223, 41, 41));

    sf::Text exitButtonText;
    exitButtonText.setFont(font);
    exitButtonText.setString("Save");
    exitButtonText.setFillColor(sf::Color::White);
    exitButtonText.setCharacterSize(24); // 设置文本大小
    while (window.isOpen())
    {
        sf::Event event;
        while (window.pollEvent(event))
        {
            if (event.type == sf::Event::Closed)
            {
                window.close();
            }

            gui.handleEvent(event); // Pass the event to the GUI
            // 窗口缩放事件处理
            if (event.type == sf::Event::Resized)
            {
                // 限制窗口大小不小于800x600
                if (event.size.width < 750 || event.size.height < 400)
                {
                    window.setSize(sf::Vector2u(std::max(event.size.width, 750u), std::max(event.size.height, 400u)));
                }

                // 重新设置视口以匹配新的窗口尺寸
                sf::FloatRect visibleArea(0, 0, event.size.width, event.size.height);
                window.setView(sf::View(visibleArea));

                // 重新计算标签位置
                float width = window.getSize().x;
                float height = window.getSize().y;

                // 设置标签位置为居中对齐
                xLabel.setPosition((width - xLabel.getLocalBounds().width) / 2, 0.1f * height);
                yLabel.setPosition((width - yLabel.getLocalBounds().width) / 2, 0.2f * height);
                yawLabel.setPosition((width - yawLabel.getLocalBounds().width) / 2, 0.3f * height);
                pointCountLabel.setPosition((width - pointCountLabel.getLocalBounds().width) / 2, 0.5f * height);

                // 计算按钮位置，使它们左右排列且整体居中
                float buttonWidth = confirmButton.getSize().x;
                float buttonHeight = confirmButton.getSize().y;
                float totalButtonWidth = buttonWidth * 3 + 20 * 2; // 两个按钮和它们之间的间距
                float buttonsStartX = (width - totalButtonWidth) / 2;

                confirmButton.setPosition(buttonsStartX, 0.75f * height);
                comboBox->setPosition(buttonsStartX + buttonWidth + 20, 0.75f * height);
                // keyButton.setPosition(buttonsStartX + buttonWidth + 20, 0.75f * height);
                exitButton.setPosition(buttonsStartX + buttonWidth * 2 + 20 * 2, 0.75f * height);

                // 调整文本大小
                // adjustTextSize(confirmButtonText, window, 0.05f);
                // adjustTextSize(exitButtonText, window, 0.05f);
                // adjustTextSize(keyButtonText, window, 0.05f);

                // 中心对齐按钮文本
                sf::FloatRect confirmButtonRect = confirmButton.getGlobalBounds();
                sf::FloatRect confirmTextRect = confirmButtonText.getLocalBounds();
                confirmButtonText.setOrigin(confirmTextRect.left + confirmTextRect.width / 2.0f, confirmTextRect.top + confirmTextRect.height / 2.0f);
                confirmButtonText.setPosition(confirmButtonRect.left + confirmButtonRect.width / 2.0f, confirmButtonRect.top + confirmButtonRect.height / 2.0f);

                sf::FloatRect exitButtonRect = exitButton.getGlobalBounds();
                sf::FloatRect exitTextRect = exitButtonText.getLocalBounds();
                exitButtonText.setOrigin(exitTextRect.left + exitTextRect.width / 2.0f, exitTextRect.top + exitTextRect.height / 2.0f);
                exitButtonText.setPosition(exitButtonRect.left + exitButtonRect.width / 2.0f, exitButtonRect.top + exitButtonRect.height / 2.0f);

                // sf::FloatRect keyButtonRect = keyButton.getGlobalBounds();
                // sf::FloatRect keyTextRect = keyButtonText.getLocalBounds();
                // keyButtonText.setOrigin(keyTextRect.left + keyTextRect.width / 2.0f, keyTextRect.top + keyTextRect.height / 2.0f);
                // keyButtonText.setPosition(keyButtonRect.left + keyButtonRect.width / 2.0f, keyButtonRect.top + keyButtonRect.height / 2.0f);
            }

            // 检查按钮点击
            if (event.type == sf::Event::MouseButtonPressed)
            {
                if (event.mouseButton.button == sf::Mouse::Left)
                {
                    sf::Vector2i mousePos = sf::Mouse::getPosition(window);

                    // 检查确认按钮点击
                    if (confirmButton.getGlobalBounds().contains(static_cast<sf::Vector2f>(mousePos)))
                    {
                        this->pathRecordFlag = true;
                        std::cout << "path_saved" << std::endl;
                    }

                    // 检查关键按钮点击
                    // if (keyButton.getGlobalBounds().contains(static_cast<sf::Vector2f>(mousePos)))
                    // {
                    //     this->keyPathRecordFlag = true;
                    //     std::cout << "Key path saved" << std::endl;
                    // }

                    // 检查结束按钮点击
                    if (exitButton.getGlobalBounds().contains(static_cast<sf::Vector2f>(mousePos)))
                    {
                        this->threadEndFlag = true;
                        window.close();
                        pthread_exit(NULL);
                    }
                }
            }
        }
        if (this->threadEndFlag == true)
        {
            window.close();
            pthread_exit(NULL);
        }
        OCTIXML::pathNodeStruct pathData_ = {0};
        this->DataAccessMutex.lock();
        pathData_ = this->pathData;
        // sequence_ = this->sequence;
        this->DataAccessMutex.unlock();
        x = pathData_.x;
        y = pathData_.y;
        yaw = pathData_.yaw;
        // 更新标签
        xLabel.setString("x: " + std::to_string(x));
        yLabel.setString("y: " + std::to_string(y));
        yawLabel.setString("yaw: " + std::to_string(yaw));
        pointCountLabel.setString("Points Count: " + std::to_string(pathData_.sequence));

        // 清除窗口并设置背景颜色
        window.clear(sf::Color(50, 50, 50)); // 设置背景颜色为深灰色

        // 绘制元素
        window.draw(xLabel);
        window.draw(yLabel);
        window.draw(yawLabel);
        window.draw(pointCountLabel);
        window.draw(confirmButton);
        window.draw(confirmButtonText);
        // window.draw(keyButton);
        // window.draw(keyButtonText);
        window.draw(exitButton);
        window.draw(exitButtonText);

        gui.draw(); // Draw all widgets

        // 显示窗口内容
        window.display();
    }
    this->pathRecordCancelFlag = true;
}

std::vector<OCTIXML::pathNodeStruct> octiXml::readPath(const char *file_path)
{
    std::vector<OCTIXML::pathNodeStruct> path;
    OCTIXML::pathNodeStruct pathNode;
    tinyxml2::XMLDocument doc;
    if (doc.LoadFile(file_path) == tinyxml2::XML_SUCCESS)
    {
        std::cout << "Path Xml load success!" << std::endl;
        tinyxml2::XMLElement *root = doc.RootElement();
        unsigned long path_node_num = root->ChildElementCount("path");
        if (path_node_num > 0)
        {
            tinyxml2::XMLElement *pathNodeEle = root->FirstChildElement("path");
            std::cout << "path_node num = " << path_node_num << std::endl;
            for (size_t i = 0; i < path_node_num; i++)
            {
                // get seq num and path x y
                unsigned long index = atol(pathNodeEle->Attribute("Seq")) - 1;
                pathNode.sequence = index;
                pathNode.x = atof(pathNodeEle->FirstChildElement("x")->GetText());
                pathNode.y = atof(pathNodeEle->FirstChildElement("y")->GetText());
                pathNode.yaw = atof(pathNodeEle->FirstChildElement("yaw")->GetText());
                pathNode.keyPointType = atoi(pathNodeEle->FirstChildElement("keypointflag")->GetText());
                pathNode.mapnum = atoi(pathNodeEle->FirstChildElement("mapnum")->GetText());

                path.push_back(pathNode);
                pathNodeEle = pathNodeEle->NextSiblingElement("path");
            }
            std::sort(path.begin(), path.end(), std::bind(&octiXml::pathIndexCompare, this, std::placeholders::_1, std::placeholders::_2));
            return path;
        }
    }
    else
    {
        std::cout << "path xml load file fail!" << std::endl;
    }
    return path;
}