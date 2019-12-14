#include "robot/screens/odom.hpp"

namespace Screen{

State resetState{6_ft, 6_ft};

void odomResetState( const State& state ){
  Robot::model->resetSensors();
  Robot::odom->setState( state );
}

void odomResetter(){
  odomResetState( resetState );
}

void initOdom(){
  screen->makePage<GUI::Odom>()
    .attachOdom( Robot::odom )
    .attachResetter( odomResetter );
}

}//Screen
