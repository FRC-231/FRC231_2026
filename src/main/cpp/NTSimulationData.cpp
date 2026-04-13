#include "NTSimulationData.h"
#include <numbers>
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

Counter::Counter(const std::string& nt_name, double start_value, double end_value, double step)
{
    this->nt_name = nt_name;
    this->count = start_value;
    this->start_value = start_value;
    this->end_value = end_value;
    this->step = step;
    nt_value = std::make_unique<HighVoltage::NetworkTableValue<double>>(nt_name, count);
}

Counter::~Counter()
{

}

void Counter::update()
{
    count += step;
    if(count > end_value)
    {
        auto leftover = count - end_value;
        count = start_value + leftover;
    }
    nt_value->setValue(count);
}

SineWave::SineWave(const std::string& nt_name, double amplitude, double frequency)
{
    this->nt_name = nt_name;
    this->amplitude = amplitude;
    this->frequency = frequency;
    start_time = std::chrono::steady_clock::now();
    nt_value = std::make_unique<HighVoltage::NetworkTableValue<double>>(nt_name, 0.0);
}

SineWave::~SineWave()
{

}

void SineWave::update()
{
    auto now = std::chrono::steady_clock::now();
    double time_since_start = std::chrono::duration_cast<std::chrono::milliseconds>(now - start_time).count() / 1000.0;
    double new_value = amplitude * std::sin(2 * std::numbers::pi * frequency * time_since_start);
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


    simulation_data_entries.push_back(
        std::make_unique<Counter>(SIMULATION_TABLE_NAME + "/Counter_0-100_step1_076547", 0, 100, 0.076547)
    );

    simulation_data_entries.push_back(
        std::make_unique<SineWave>(SIMULATION_TABLE_NAME + "/SineWave_Amplitude1_Frequency1", 50.0, 1.0)
    );
    simulation_data_entries.push_back(
        std::make_unique<SineWave>(SIMULATION_TABLE_NAME + "/SineWave_Amplitude1_Frequency0.25", 50.0, 0.25)
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
