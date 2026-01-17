#pragma once

#include <frc/controller/PIDController.h>
#include <units/time.h>
#include <networktables/NetworkTableInstance.h>
#include <networktables/DoubleTopic.h>
#include <networktables/NetworkTable.h>
#include <networktables/DoubleArrayTopic.h>
#include <string>
#include <string_view>
#include <memory>
#include <optional>
#include <mutex>

#include "CommonConfig.h"

namespace HighVoltage
{

class SmartPID : public frc::PIDController
{
public:
    SmartPID(std::string name, double k_p, double k_i, double k_d, units::second_t period = 0.02_s);
    SmartPID(std::string name, PIDConfig config, units::second_t period = 0.02_s);
    
    static constexpr std::string_view K_P_KEY = "P";
    static constexpr std::string_view K_I_KEY = "I";
    static constexpr std::string_view K_D_KEY = "D";
    static constexpr std::string_view SETPOINT_KEY = "Setpoint";
    static constexpr std::string_view MEASUREMENT_KEY = "Measurement";
    static constexpr std::string_view OUTPUT_KEY = "Output";
    
    double Calculate(double measurement);
    double Calculate(double measurement, double setpoint);
    void SetSetpoint(double setpoint);



protected:

    std::mutex mutex;

    std::string name;
    nt::NetworkTableInstance network_table_inst;
    std::shared_ptr<nt::NetworkTable> pid_table;
    nt::DoubleTopic k_p_topic;
    nt::DoubleSubscriber k_p_subscriber;
    nt::DoublePublisher k_p_publisher;
    NT_Listener k_p_listener;
    nt::DoubleTopic k_i_topic;
    nt::DoubleSubscriber k_i_subscriber;
    nt::DoublePublisher k_i_publisher;
    NT_Listener k_i_listener;
    nt::DoubleTopic k_d_topic;
    nt::DoubleSubscriber k_d_subscriber;
    nt::DoublePublisher k_d_publisher;
    NT_Listener k_d_listener;

    nt::DoubleTopic setpoint_topic;
    nt::DoublePublisher setpoint_publisher;
    nt::DoubleTopic measurement_topic;
    nt::DoublePublisher measurement_publisher;
    nt::DoubleTopic output_topic;
    nt::DoublePublisher output_publisher;

    void populateNetworkTable(double measurement, double output, std::optional<double> setpoint = std::nullopt);

};

}
