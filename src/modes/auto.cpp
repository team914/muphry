#include "robot/modes/auto.hpp"

namespace Auto{

map<string, function<void()>> routines = {
    {
        "simpleForward",
        [&](){
            Robot::chassis->moveDistance(1_tl);
        }
    },
    {
        "simpleBackward",
        [&](){
            Robot::chassis->moveDistance(-1_tl);
        }
    }
};

std::string routine = "simpleForward";

void run(){
    runAuto(routine);
}

void addAuto(const std::string& id, const std::function<void()>& iroutine ){
    auto out = "Auto::addAuto " + std::string(id) + ".\n";
    printf( out.c_str() );
    routines[id] = iroutine;
}

void runAuto(const std::string& id){
    auto out = "Auto::runAuto running" + std::string(id) + ".\n";
    printf( out.c_str() );
    routines.at(id)();
}


}//Auto