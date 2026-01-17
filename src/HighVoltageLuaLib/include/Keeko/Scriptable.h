#pragma once

//Lua Includes
#include "lua.h"
#include "lauxlib.h"
#include "lualib.h"

#include "LuaBridge.h"

//Keeko includes
#include "Keeko/LuaState.h"


namespace Keeko
{

class Scriptable
{
public:
    virtual void addClassToState(lua_State* state) = 0;
    inline void addClassToState(const Keeko::LuaState& state)
    {
        lua_State* ls = state;
        addClassToState(ls);
    }
};

} // namespace Keeko