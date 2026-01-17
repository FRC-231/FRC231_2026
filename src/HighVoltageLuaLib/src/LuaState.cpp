
#include "Keeko/LuaState.h"

#include <fstream>
#include <streambuf>

namespace Keeko
{

LuaState::LuaState()
{
    state = luaL_newstate();
    luaL_openlibs(state);
}

void LuaState::loadFile(const std::string& file_path)
{
    std::ifstream f(file_path);
    std::string str((std::istreambuf_iterator<char>(f)),
                     std::istreambuf_iterator<char>());
    parseString(str, file_path);
}

void LuaState::parseString(const std::string& str, const std::string& name)
{
    if(luaL_loadbuffer(state, str.c_str(), str.length(), name.c_str()))
    {
        throw LuaException{state};
    }
    if(lua_pcall(state, 0, 0, 0))
    {
        throw LuaException{state};
    }
}

LuaState::~LuaState()
{
    if(state != nullptr)
    {
        lua_close(state);
        state = nullptr;
    }
}

void LuaState::runFunction(const std::string& function_name)
{
    lua_getglobal(state, function_name.c_str());
    if(lua_pcall(state, 0, 0, 0))
    {
        throw LuaException{state};
    }
}

void LuaState::addGlobalObject(void* ptr, const luaL_Reg functions[], std::string name)
{
    lua_pushlightuserdata(state, ptr);
    luaL_newmetatable(state, name.c_str());
    lua_pushvalue(state, -1);/* Push table created by luaL_newmetatable */
    lua_setfield(state, -2, "__index");/* table.__index == table */
    luaL_setfuncs(state, functions, 0);
    lua_setmetatable(state, -2);/* Set meta for userdata */
    lua_setglobal(state, name.c_str());/* Make global */
}

} // namespace Keeko