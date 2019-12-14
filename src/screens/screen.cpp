#include "robot/screens/screen.hpp"

#include "robot/screens/odom.hpp"
#include "robot/screens/selector.hpp"

namespace Screen{

void init(){
  screen->startTask("screenTask");
  initOdom();
  initSelector();
}

std::shared_ptr<GUI::Screen> screen = std::make_shared<GUI::Screen>(
    lv_scr_act(),
    LV_COLOR_MAKE(153, 51, 255)
);

}
