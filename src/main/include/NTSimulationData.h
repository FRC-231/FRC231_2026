#include "NetworkTableValue.h"
#include <thread>
#include <memory>
#include <atomic>
#include <vector>
#include <string>

namespace NTSimulationData
{

class SimulationData
{
public:
    virtual void update() = 0;

private:

    std::string nt_name;
};

class Blinker: public SimulationData
{
public:
    Blinker(std::chrono::milliseconds interval, const std::string& nt_name);
    ~Blinker();
    void update() override;

private:
    std::chrono::milliseconds interval;
    std::string nt_name;
    std::chrono::steady_clock::time_point start_time;
    std::unique_ptr<HighVoltage::NetworkTableValue<bool>> nt_value{nullptr};
};


class NTSimulationData
{
public:
    NTSimulationData();
    ~NTSimulationData();
    
    inline static const std::string SIMULATION_TABLE_NAME = "SimulationData";

private:

    std::unique_ptr<std::thread> nt_update_thread{nullptr};
    std::atomic<bool> nt_update_thread_running{false};
    void ntUpdateLoop();

    std::vector<std::unique_ptr<SimulationData>> simulation_data_entries;

};




}// namespace NTSimulationData
