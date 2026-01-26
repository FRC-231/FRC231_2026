#include "MK5NSwerveModule.h"

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
    nt_target_angle = std::make_unique<HighVoltage::NetworkTableValue<double>>(module_name + "/Target_Angle_Degrees", 0.0);
    nt_angle = std::make_unique<HighVoltage::NetworkTableValue<double>>(module_name + "/Angle_Degrees", 0.0);
    nt_raw_angle = std::make_unique<HighVoltage::NetworkTableValue<double>>(module_name + "/Raw_Angle_Degrees", 0.0);

}

void MK5NSwerveModule::update(std::chrono::duration<double> seconds_passed)
{
    nt_angle->setValue(getAngle().value());
    nt_raw_angle->setValue(getRawAngle().value());
}

frc::SwerveModulePosition MK5NSwerveModule::getModulePosition()
{
    // TODO: Implement getModulePosition
    return frc::SwerveModulePosition{};
}

units::angle::degree_t MK5NSwerveModule::getRawAngle()
{
    return encoder->GetPosition().GetValue().value() * 360.0_deg;
}

units::angle::degree_t MK5NSwerveModule::getAngle()
{
    return getRawAngle() - offset;
}

units::length::meter_t MK5NSwerveModule::getEncoderPosition()
{
    // TODO: Implement getEncoderPosition
    return units::length::meter_t{0.0};
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



