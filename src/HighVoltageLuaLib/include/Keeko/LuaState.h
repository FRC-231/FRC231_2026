#pragma once

//Standard Includes
#include <memory>
#include <string>

//Lua Includes
#include "lua.h"
#include "lauxlib.h"
#include "lualib.h"
#include "LuaBridge.h"

//My Includes
#include "Keeko/LuaException.h"

namespace Keeko
{
class LuaState
{
public:
    LuaState();
    virtual ~LuaState();

    void loadFile(const std::string& file_path);
    void parseString(const std::string& str, const std::string& name = "loaded_string");
    void runFunction(const std::string& function_name);

    operator lua_State*() const {return state;};

    void addGlobalObject(void* ptr, const luaL_Reg functions[], std::string name);
    void addGlobalFunction(lua_CFunction, std::string);
    
    template <typename T>
    void addInstance(const std::string& name,T *const &t)
    {
        luabridge::Result res = luabridge::push(state, t);
        if(!res)
        {
            std::cerr << "Failed to load instance!" << std::endl; //TODO: Add exception to throw
        }
        lua_setglobal(state, name.c_str());
    }

protected:
    lua_State* state = nullptr;
};
} // namespace Keeko

