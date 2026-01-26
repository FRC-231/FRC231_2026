#include "MK5NSwerveModule.h"

namespace HighVoltage
{

MK5NSwerveModule::MK5NSwerveModule(std::string name,
    MK5NSwerveModule::Mk5NConfig config) : SwerveModule{name}, config{config}
{
    drive_motor = std::make_unique<ctre::phoenix6::hardware::TalonFX>(config.drive_id);
    rotate_motor =  std::make_unique<ctre::phoenix6::hardware::TalonFX>(config.rotate_id);
    encoder = std::make_unique<ctre::phoenix6::hardware::CANcoder>(config.encoder_id);

    rotate_feedforward = std::make_unique<frc::SimpleMotorFeedforward<units::angle::degree_t>>(
        config.rotate_ff.K_S,
        config.rotate_ff.K_V,
        config.rotate_ff.K_A
    );

    rotate_velocity_pid = std::make_unique<frc::PIDController>(
        config.rotate_velocity_pid.K_P,
        config.rotate_velocity_pid.K_I,
        config.rotate_velocity_pid.K_D
    );

    rotate_angle_pid = std::make_unique<frc::PIDController>(
        config.rotate_angle_pid.K_P,
        config.rotate_angle_pid.K_I,
        config.rotate_angle_pid.K_D
    );

}

void from_json(const nlohmann::json& j, MK5NSwerveModule::Mk5NConfig& config)
{
    j.at("drive_id").get_to(config.drive_id);
    j.at("rotate_id").get_to(config.rotate_id);
    j.at("encoder_id").get_to(config.encoder_id);
    double offset_degrees = j.at("encoder_offset_degrees").get<double>(); //Grab the value as a double
    config.encoder_offset = units::angle::degree_t{offset_degrees}; //Covert that double to degree_t and assign
}

} // namespace HighVoltage



