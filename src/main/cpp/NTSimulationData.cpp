#include "NTSimulationData.h"
#include <iostream>

namespace NTSimulationData
{



Blinker::Blinker(std::chrono::milliseconds interval, const std::string& nt_name)
{
    this->interval = interval;
    this->nt_name = nt_name;
    start_time = std::chrono::steady_clock::now();
    nt_value = std::make_unique<HighVoltage::NetworkTableValue<bool>>(nt_name, false);
}

Blinker::~Blinker()
{

}

void Blinker::update()
{
    auto now = std::chrono::steady_clock::now();
    int time_since_start = std::chrono::duration_cast<std::chrono::milliseconds>(now - start_time).count();
    bool new_value = (time_since_start / interval.count()) % 2 == 0;
    nt_value->setValue(new_value);
}

NTSimulationData::NTSimulationData()
{
    simulation_data_entries.push_back(
        std::make_unique<Blinker>(std::chrono::milliseconds(100), SIMULATION_TABLE_NAME + "/Blinker_100ms")
    );
    simulation_data_entries.push_back(
        std::make_unique<Blinker>(std::chrono::milliseconds(200), SIMULATION_TABLE_NAME + "/Blinker_200ms")
    );
    simulation_data_entries.push_back(
        std::make_unique<Blinker>(std::chrono::milliseconds(250), SIMULATION_TABLE_NAME + "/Blinker_250ms")
    );
    simulation_data_entries.push_back(
        std::make_unique<Blinker>(std::chrono::milliseconds(500), SIMULATION_TABLE_NAME + "/Blinker_500ms")
    );
    simulation_data_entries.push_back(
        std::make_unique<Blinker>(std::chrono::milliseconds(750), SIMULATION_TABLE_NAME + "/Blinker_750ms")
    );
    simulation_data_entries.push_back(
        std::make_unique<Blinker>(std::chrono::milliseconds(1000), SIMULATION_TABLE_NAME + "/Blinker_1000ms")
    );
    
    nt_update_thread_running = true;
    nt_update_thread = std::make_unique<std::thread>(&NTSimulationData::ntUpdateLoop, this);
}

NTSimulationData::~NTSimulationData()
{
    nt_update_thread_running = false;
    if(nt_update_thread && nt_update_thread->joinable())
    {
        nt_update_thread->join();
    }
}

void NTSimulationData::ntUpdateLoop()
{
    while(nt_update_thread_running)
    {
        for(auto& entry : simulation_data_entries)
        {
            entry->update();
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
}

} // namespace NTSimulationData
