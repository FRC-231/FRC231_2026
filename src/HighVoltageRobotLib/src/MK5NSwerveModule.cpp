#include "MK5NSwerveModule.h"

#include <iostream>

namespace HighVoltage
{

MK5NSwerveModule::MK5NSwerveModule(std::string name,
    MK5NSwerveModule::Mk5NConfig config) : SwerveModule{name}, config{config}
{
    drive_motor = std::make_unique<ctre::phoenix6::hardware::TalonFX>(config.drive_id);
    rotate_motor =  std::make_unique<ctre::phoenix6::hardware::TalonFX>(config.rotate_id);
    encoder = std::make_unique<ctre::phoenix6::hardware::CANcoder>(config.encoder_id);
    offset = config.encoder_offset;

    rotate_feedforward = std::make_unique<frc::SimpleMotorFeedforward<units::meters>>(
        units::volt_t{config.rotate_ff.k_s},
        VoltsPerMeterPerSecond{config.rotate_ff.k_v}, //TODO: Make this less hacky by making a custom feedforward class with angles
        VoltsPerMeterPerSecondSquared{config.rotate_ff.k_a}
    );
    rotate_velocity_pid = std::make_unique<frc::PIDController>(
        config.rotate_velocity_pid.k_p,
        config.rotate_velocity_pid.k_i,
        config.rotate_velocity_pid.k_d
    );
    rotate_angle_pid = std::make_unique<frc::PIDController>(
        config.rotate_angle_pid.k_p,
        config.rotate_angle_pid.k_i,
        config.rotate_angle_pid.k_d
    );

    //Init network table values
    std::cout << "Creating \"" << module_name + "/Target_Angle_Degrees\" network table value" << std::endl;
    nt_target_angle = std::make_unique<HighVoltage::NetworkTableValue<double>>(module_name + "/Target_Angle_Degrees", 0.0);
    nt_angle = std::make_unique<HighVoltage::NetworkTableValue<double>>(module_name + "/Angle_Degrees", 0.0);
    nt_raw_angle = std::make_unique<HighVoltage::NetworkTableValue<double>>(module_name + "/Raw_Angle_Degrees", 0.0);
    nt_rotate_velocity = std::make_unique<HighVoltage::NetworkTableValue<double>>(module_name + "/Rotate_Velocity", 0.0);

    nt_rotate_ff_voltage = std::make_unique<HighVoltage::NetworkTableValue<double>>(module_name + "/Rotate_FF_Voltage", 0.0);
    nt_rotate_pid_voltage = std::make_unique<HighVoltage::NetworkTableValue<double>>(module_name + "/Rotate_PID_Voltage", 0.0);

}

void MK5NSwerveModule::update(std::chrono::duration<double> seconds_passed)
{
    units::angle::degree_t current_angle = getAngle();
    units::angle::degree_t set_angle = closestAngle(current_angle, target_angle);
    units::angle::degree_t set_angle_flipped = closestAngle(current_angle, target_angle + 180.0_deg);
    units::angle::degree_t new_target = 0_deg;
    bool is_forward = false;
    if(std::abs(set_angle.value()) <= std::abs(set_angle_flipped.value()))
    {
        new_target = current_angle + set_angle;
        is_forward = true;
    }
    else
    {
        new_target = current_angle + set_angle_flipped;
        is_forward = false;
    }
    //turn motor
    units::angle::degree_t degrees_off = get180Range(current_angle - new_target);
    if(abs(degrees_off.value()) > 1)
    {
        rotate_velocity_pid->SetSetpoint(0);
        double velocity = rotate_velocity_pid->Calculate(degrees_off.value());
        rotate_angle_pid->SetSetpoint(0);
        units::voltage::volt_t turn_pid_voltage = units::voltage::volt_t{rotate_angle_pid->Calculate(degrees_off.value())};
        auto turn_feed = rotate_feedforward->Calculate(units::unit_t<units::compound_unit<units::meters, units::inverse<units::seconds>>>(velocity));
        rotate_motor->SetVoltage(turn_pid_voltage + turn_feed);
        nt_rotate_ff_voltage->setValue(turn_feed.value());
        nt_rotate_pid_voltage->setValue(turn_pid_voltage.value());
    }
    else
    {
        rotate_motor->SetVoltage(0_V);
        nt_rotate_ff_voltage->setValue(0);
        nt_rotate_pid_voltage->setValue(0);
    }
    //drive motor
    units::voltage::volt_t drive_voltage = 2_V;
    if(!is_forward)
    {
        drive_voltage *= -1;
    }
    //drive_motor->SetVoltage(drive_voltage);
    updateNetworkTables();
}

frc::SwerveModulePosition MK5NSwerveModule::getModulePosition()
{
    // TODO: Implement getModulePosition
    return frc::SwerveModulePosition{};
}

units::angle::degree_t MK5NSwerveModule::getRawAngle()
{
    return get180Range(encoder->GetPosition().GetValue().value() * 360.0_deg);
}

units::angle::degree_t MK5NSwerveModule::getAngle()
{
    return get180Range(getRawAngle() - offset);
}

units::length::meter_t MK5NSwerveModule::getEncoderPosition()
{
    // TODO: Implement getEncoderPosition
    return units::length::meter_t{0.0};
}

void MK5NSwerveModule::updateNetworkTables()
{
    nt_target_angle->setValue(target_angle.value());
    nt_angle->setValue(getAngle().value());
    nt_raw_angle->setValue(getRawAngle().value());
    nt_rotate_velocity->setValue(encoder->GetVelocity().GetValueAsDouble() * 360);
}

void from_json(const nlohmann::json& j, MK5NSwerveModule::Mk5NConfig& config)
{
    j.at(MK5NSwerveModule::Mk5NConfig::DRIVE_ID).get_to(config.drive_id);
    j.at(MK5NSwerveModule::Mk5NConfig::ROTATE_ID).get_to(config.rotate_id);
    j.at(MK5NSwerveModule::Mk5NConfig::ENCODER_ID).get_to(config.encoder_id);
    double offset_degrees = j.at(MK5NSwerveModule::Mk5NConfig::ENCODER_OFFSET_DEGREES).get<double>(); //Grab the value as a double
    config.encoder_offset = units::angle::degree_t{offset_degrees}; //Covert that double to degree_t and assign
    j.at(MK5NSwerveModule::Mk5NConfig::ROTATE_FF).get_to(config.rotate_ff);
    j.at(MK5NSwerveModule::Mk5NConfig::ROTATE_VELOCITY_PID).get_to(config.rotate_velocity_pid);
    j.at(MK5NSwerveModule::Mk5NConfig::ROTATE_ANGLE_PID).get_to(config.rotate_angle_pid);
}

} // namespace HighVoltage



