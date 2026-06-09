#include <SFML/Graphics.hpp>
#include <TGUI/TGUI.hpp>
#include <TGUI/Backend/SFML-Graphics.hpp>
#define GLFW_INCLUDE_NONE // Don't let GLFW include an OpenGL extention loader
#include <GLFW/glfw3.h>
#include <functional>
void mycallback(const sf::String &selectedItem)
{
    std::cout << "Selected item: " << selectedItem.toAnsiString() << std::endl;
}

int main()
{
    sf::RenderWindow window(sf::VideoMode(800, 600), "TGUI Dropdown Example");
    tgui::Gui gui(window); // Create the GUI and attach it to the window

    // Create a ComboBox (dropdown list)
    auto comboBox = tgui::ComboBox::create();
    comboBox->setPosition(100, 100);
    comboBox->setSize(220, 50);
    comboBox->addItem("Normal point", "1");
    comboBox->addItem("Vision point", "2");
    comboBox->addItem("Charge point", "3");
    comboBox->addItem("Other point", "4");
    // 连接ComboBox的选择事件
    comboBox->onItemSelect([](int index){
        std::cout << "Selected item : " << index << std::endl;
    });
    
    tgui::ComboBoxRenderer* Render = comboBox->getRenderer();
    tgui::Color textColor(255, 111, 74);
    
    Render->setTextColor(textColor);
    Render->setTextSize(24);
    tgui::Color arrowColor = tgui::Color(255, 111, 74);
    tgui::Color arrowColorHover = tgui::Color(255, 190, 0);
    Render->setArrowColor(arrowColor);
    Render->setArrowColorHover(arrowColorHover);

    comboBox->setSelectedItemByIndex(0);
    gui.add(comboBox);

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
        }

        window.clear();
        gui.draw(); // Draw all widgets
        window.display();
    }

    return 0;
}
