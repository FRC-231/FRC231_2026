#include "Keeko/LuaException.h"

namespace Keeko
{

LuaException::LuaException(lua_State* state)
{
    error = lua_tostring(state, -1);
}

}