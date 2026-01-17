#include "SmartPID.h"
#include <iostream>

namespace HighVoltage
{

SmartPID::SmartPID(std::string name, double k_p, double k_i, double k_d, units::second_t period) :
    frc::PIDController(k_p, k_i, k_d, period),
    name(name)
{
    network_table_inst = nt::NetworkTableInstance::GetDefault();
    pid_table = network_table_inst.GetTable(name);
    k_p_topic = pid_table->GetDoubleTopic(K_P_KEY);
    k_p_publisher = k_p_topic.Publish();
    k_p_publisher.SetDefault(k_p);
    k_p_subscriber = k_p_topic.Subscribe(k_p);
    

    k_p_listener = network_table_inst.AddListener(
        k_p_subscriber,
        nt::EventFlags::kValueAll,
        [this, name](const nt::Event& event)
        {
            std::scoped_lock lock{mutex};
            if(event.GetValueEventData()->value.IsDouble())
            {
                std::cout << "P value set to " << 
                    event.GetValueEventData()->value.GetDouble() <<
                    " For table " << name <<
                     std::endl;
                SetP(event.GetValueEventData()->value.GetDouble());
            }
        }
    );
    k_i_topic = pid_table->GetDoubleTopic(K_I_KEY);
    k_i_publisher = k_i_topic.Publish();
    k_i_publisher.SetDefault(k_i);
    k_i_subscriber = k_i_topic.Subscribe(GetI());
    k_i_listener = network_table_inst.AddListener(
        k_i_subscriber,
        nt::EventFlags::kValueAll,
        [this, name](const nt::Event& event)
        {
            std::scoped_lock lock{mutex};
            if(event.GetValueEventData()->value.IsDouble())
            {
                std::cout << "I value set to " << 
                    event.GetValueEventData()->value.GetDouble() <<
                    " For table " << name <<
                     std::endl;
                SetI(event.GetValueEventData()->value.GetDouble());
            }
        }
    );
    k_d_topic = pid_table->GetDoubleTopic(K_D_KEY);
    k_d_publisher = k_d_topic.Publish();
    k_d_publisher.SetDefault(k_d);
    k_d_subscriber = k_d_topic.Subscribe(GetD());
    k_d_listener = network_table_inst.AddListener(
        k_d_subscriber,
        nt::EventFlags::kValueAll,
        [this, name](const nt::Event& event)
        {
            std::scoped_lock lock{mutex};
            if(event.GetValueEventData()->value.IsDouble())
            {
                std::cout << "D value set to " << 
                    event.GetValueEventData()->value.GetDouble() <<
                    " For table " << name <<
                     std::endl;
                SetD(event.GetValueEventData()->value.GetDouble());
            }
        }
    );


    setpoint_topic = pid_table->GetDoubleTopic(SETPOINT_KEY);
    setpoint_publisher = setpoint_topic.Publish();
    setpoint_publisher.SetDefault(GetSetpoint());
    measurement_topic = pid_table->GetDoubleTopic(MEASUREMENT_KEY);
    measurement_publisher = measurement_topic.Publish();
    measurement_publisher.SetDefault(0.0);
    output_topic = pid_table->GetDoubleTopic(OUTPUT_KEY);
    output_publisher = output_topic.Publish();
    output_publisher.SetDefault(0.0);
}

SmartPID::SmartPID(std::string name, PIDConfig config, units::second_t period) :
    SmartPID(name, config.k_p, config.k_i, config.k_d, period)
{
}

void SmartPID::populateNetworkTable(double measurement, double output, std::optional<double> setpoint)
{
    if(setpoint.has_value())
    {
        pid_table->PutNumber(SETPOINT_KEY, setpoint.value());
    }
    measurement_publisher.Set(measurement);
    output_publisher.Set(output);
}

double SmartPID::Calculate(double measurement) {
    double result = frc::PIDController::Calculate(measurement);
    populateNetworkTable(measurement, result);
    return result;
}

double SmartPID::Calculate(double measurement, double setpoint) {
    double result = frc::PIDController::Calculate(measurement, setpoint);
    populateNetworkTable(measurement, result, setpoint);
    return result;
}

void SmartPID::SetSetpoint(double setpoint)
{
    frc::PIDController::SetSetpoint(setpoint);
    pid_table->PutNumber(SETPOINT_KEY, setpoint);
}

}
