#pragma once

//Standard Includes
#include <exception>
#include <string>

//Lua Includes
#include "lua.h"
#include "lauxlib.h"
#include "lualib.h"

namespace Keeko
{

class LuaException: public std::exception
{
public:
    LuaException(lua_State* state);
    virtual const char* what() const throw()
    {
        return error.c_str();
        
    }
protected:
    std::string error;
};

} // namespace Keeko