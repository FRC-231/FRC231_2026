#include "Keeko/ThreadedScript.h"

namespace Keeko
{

ThreadedScript::ThreadedScript()
{
    state = std::make_shared<LuaState>();
    addFunctions();
}

ThreadedScript::ThreadedScript(std::shared_ptr<LuaState> existing_state)
{
    state = existing_state;
    addFunctions();
}

ThreadedScript::~ThreadedScript()
{
    running = false;
    join();
}

void ThreadedScript::runFile(const std::string& path)
{
    if(running)
    {
        std::cout << "WARNING: Script is already running, runFile for \"" << path << "\" will be ignored";
        return;
    }
    if(thread != nullptr && thread->joinable())
    {
        thread->join();//Very slight chance thread isn't fully dead, join just incase
    }
    runningPath = path;
    thread = std::make_unique<std::thread>([this]()
    {
        running = true;
        std::cout << "Starting Lua script \"" << runningPath << "\"" << std::endl;
        try
        {
            state->loadFile(runningPath);
        }
        catch(const std::exception& e)
        {
            std::cerr << e.what() << std::endl;
            onFailCallback();
        }
        std::cout << "Lua script \"" << runningPath << "\" has ended" << std::endl;
        running = false;
    });
}

void ThreadedScript::runFunction(const std::string& function_name)
{
    if(running)
    {
        std::cout << "WARNING: Script is already running, runFunction for \"" << function_name << "\" will be ignored";
        return;
    }
    if(thread != nullptr && thread->joinable())
    {
        thread->join();//Very slight chance thread isn't fully dead, join just incase
    }
    thread = std::make_unique<std::thread>([this, function_name]()
    {
        running = true;
        std::cout << "Starting Lua function \"" << function_name << "\"" << std::endl;
        try
        {
            state->runFunction(function_name);
        }
        catch(const std::exception& e)
        {
            std::cerr << "ERROR:" << e.what() << std::endl;
            onFailCallback();
        }
        std::cout << "Lua function \"" << function_name << "\" has ended" << std::endl;
        running = false;
    });
}

void ThreadedScript::loadFile(const std::string& path)
{
    try
    {
        state->loadFile(path);
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << std::endl;
    }
}

void ThreadedScript::stop()
{
    running = false;
    kill_cv.notify_one();
    if(thread != nullptr  && thread->joinable())
    {
        thread->join();
    }
}

void ThreadedScript::join()
{
    if(thread != nullptr && thread->joinable())
    {
        thread->join();//Very slight chance thread isn't fully dead, join just incase
    }
}

void ThreadedScript::sleep(double delay)
{
    std::unique_lock<std::mutex> lck{mtx};
    std::chrono::milliseconds delay_millisecond(static_cast<long>(delay*1000.0));
    kill_cv.wait_for(lck, delay_millisecond);
    if(!running)
    {
        luaL_error(*(state.get()), "Script stopped early\n");
    }
}

void ThreadedScript::waitUntilTrue(luabridge::LuaRef callback)
{
    if(!callback.isCallable())
    {
        luaL_error(callback.state(), "Callback was not given!");
        return;
    }
    while(true)
    {
        std::unique_lock<std::mutex> lck{mtx};
        kill_cv.wait_for(lck, wait_until_delay);
        if(!running)
        {
            luaL_error(*(state.get()), "Script stopped early\n");
            return;
        }
        auto ret = callback();
        if(ret.size() != 1)
        {
            luaL_error(callback.state(), "Callback got %d returns but needs 1", ret.size());
            return;
        }
        if(!ret[0].isBool())
        {
            luaL_error(callback.state(), "Callback needs to return an boolean");
            return;
        }
        if(ret[0].cast<bool>() == true)
        {
            break;
        }
    }
}

void ThreadedScript::addFunctions()
{
    luabridge::getGlobalNamespace(*(state.get()))
        .beginClass<ThreadedScript> ("ThreadedScript")
            .addFunction("sleep", &ThreadedScript::sleep)
            .addFunction("waitUntilTrue", &ThreadedScript::waitUntilTrue)
        .endClass();
    
    state->addInstance("control", this);
}

void ThreadedScript::setOnFailCallback(std::function<void()> cb)
{
    onFailCallback = cb;
}

} // namespace Keeko