#pragma once

#include "okapi/api.hpp"
#include <memory>
#include <map>
#include <string>

namespace autolib{

using namespace okapi;

class Auton{
    public:
    Auton();

    void add( const std::string &id, std::function< void()> iroutine );
    void run( const std::string &id );

    protected:
    friend class Selector;

    std::map<std::string, std::function<void()>> routines;
};

}//autolib
