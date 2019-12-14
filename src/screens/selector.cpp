#include "robot/screens/selector.hpp"

#include "robot/modes/auto.hpp"
#include "robot/screens/screen.hpp"
#include "lib7842/api/gui/selector.hpp"

using namespace lib7842::GUI;

namespace Screen{

void initSelector(){
  screen->makePage<Selector>("Selector")
    .button("simpleForward", [&]() { ; })
    .button("simpleBackward", [&]() { ; })
    .build();

  screen->makePage<Selector>("Runner")
    .button("simpleForward", [&]() { ; })
    .button("simpleBackward", [&]() { ; })
    .build();
}

}//Screen
