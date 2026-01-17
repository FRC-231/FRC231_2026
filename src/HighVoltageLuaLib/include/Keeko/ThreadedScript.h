#pragma once

//Standard includes
#include <string>
#include <thread>
#include <mutex>
#include <atomic>
#include <condition_variable>
#include <functional>
#include <cmath>
#include <iostream>
#include <memory>

//Lua includes
#include "lua.h"
#include "lauxlib.h"
#include "lualib.h"

#include "LuaBridge.h"

//Keeko includes
#include "Keeko/LuaState.h"
#include "Keeko/Scriptable.h"


namespace Keeko
{

class ThreadedScript
{
public:
    ThreadedScript();
    ThreadedScript(std::shared_ptr<LuaState> existing_state);
    ThreadedScript(const ThreadedScript&) = delete;//Removes the copy constructor
    virtual ~ThreadedScript();

    void sleep(double delay);
    void waitUntilTrue(luabridge::LuaRef callback);

    template <typename T>
    void addInstance(const std::string& name,T *const &t)
    {
        state->addInstance(name, t);
    }
    operator lua_State*() const {return *(state.get());};
    void runFile(const std::string& path);
    void loadFile(const std::string& path);
    void runFunction(const std::string& function_name);
    void stop();
    void join();
    bool isRunning() const {return running;}

    void setOnFailCallback(std::function<void()> cb);
    //TODO: Add get state

protected:
    static constexpr std::chrono::milliseconds DEFAULT_WAIT_UNTIL_DELAY{10};
    std::chrono::milliseconds wait_until_delay = DEFAULT_WAIT_UNTIL_DELAY;
    std::shared_ptr<LuaState> state = nullptr;
    bool running = false;
    std::string runningPath;

    void addFunctions();

    std::function<void()> onFailCallback;
    

private:
    std::unique_ptr<std::thread> thread;
    std::condition_variable kill_cv;//A condition_variable that allows early canceling of the thread
    std::mutex mtx;//Mutex used for condition wait
};

} // namespace Keeko