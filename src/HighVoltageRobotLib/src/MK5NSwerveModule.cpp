#include "MK5NSwerveModule.h"

#include <iostream>

namespace HighVoltage
{

MK5NSwerveModule::MK5NSwerveModule(std::string name,
    MK5NSwerveModule::Mk5NConfig config) : SwerveModule{name}, config{config}
{
    drive_motor = std::make_shared<ctre::phoenix6::hardware::TalonFX>(config.drive_id, "Drivebase");
    rotate_motor =  std::make_shared<ctre::phoenix6::hardware::TalonFX>(config.rotate_id, "Drivebase");
    encoder = std::make_shared<ctre::phoenix6::hardware::CANcoder>(config.encoder_id, "Drivebase");
    // drive_motor = std::make_shared<ctre::phoenix6::hardware::TalonFX>(config.drive_id, "rio");
    // rotate_motor =  std::make_shared<ctre::phoenix6::hardware::TalonFX>(config.rotate_id, "rio");
    // encoder = std::make_shared<ctre::phoenix6::hardware::CANcoder>(config.encoder_id, "rio");
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
    drive_feedforward = std::make_unique<frc::SimpleMotorFeedforward<units::meters>>(
        units::volt_t{config.drive_ff.k_s},
        VoltsPerMeterPerSecond{config.drive_ff.k_v},
        VoltsPerMeterPerSecondSquared{config.drive_ff.k_a}
    );

    ctre::phoenix6::configs::TalonFXConfiguration rotate_config{};
    rotate_config.MotorOutput.Inverted = ctre::phoenix6::signals::InvertedValue::CounterClockwise_Positive;
    rotate_config.MotorOutput.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Brake;
    // Limit supply current to prevent brownouts
    rotate_config.CurrentLimits.SupplyCurrentLimitEnable = true;
    rotate_config.CurrentLimits.SupplyCurrentLimit = 35_A;
    rotate_motor->GetConfigurator().Apply(rotate_config);

    ctre::phoenix6::configs::TalonFXConfiguration drive_config{};
    drive_config.MotorOutput.Inverted = ctre::phoenix6::signals::InvertedValue::Clockwise_Positive;
    drive_config.MotorOutput.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Brake;
    // Limit supply current to prevent brownouts
    drive_config.CurrentLimits.SupplyCurrentLimitEnable = true;
    drive_config.CurrentLimits.SupplyCurrentLimit = 55_A;
    drive_motor->GetConfigurator().Apply(drive_config);

    drive_turns_per_meter = config.drive_turns_per_meter;

    //Init network table values
    std::cout << "Creating \"" << module_name + "/Target_Angle_Degrees\" network table value" << std::endl;
    nt_target_angle = std::make_unique<HighVoltage::NetworkTableValue<double>>(module_name + "/Target_Angle_Degrees", 0.0);
    nt_angle = std::make_unique<HighVoltage::NetworkTableValue<double>>(module_name + "/Angle_Degrees", 0.0);
    nt_raw_angle = std::make_unique<HighVoltage::NetworkTableValue<double>>(module_name + "/Raw_Angle_Degrees", 0.0);
    nt_rotate_velocity = std::make_unique<HighVoltage::NetworkTableValue<double>>(module_name + "/Rotate_Velocity", 0.0);

    nt_rotate_ff_voltage = std::make_unique<HighVoltage::NetworkTableValue<double>>(module_name + "/Rotate_FF_Voltage", 0.0);
    nt_rotate_pid_voltage = std::make_unique<HighVoltage::NetworkTableValue<double>>(module_name + "/Rotate_PID_Voltage", 0.0);
    nt_encoder_position = std::make_unique<HighVoltage::NetworkTableValue<double>>(module_name + "/Encoder_Postion", 0.0);
    nt_encoder_raw_position = std::make_unique<HighVoltage::NetworkTableValue<double>>(module_name + "/Encoder_Raw_Postion", 0.0);

}

//updates the turn motor based on the degrees off from the target
void MK5NSwerveModule::updateTurnMotor(units::angle::degree_t degrees_off)
{
    if(abs(degrees_off.value()) > 1)
    {
        rotate_velocity_pid->SetSetpoint(0);
        double velocity = rotate_velocity_pid->Calculate(degrees_off.value());
        rotate_angle_pid->SetSetpoint(0);
        units::voltage::volt_t turn_pid_voltage = units::voltage::volt_t{rotate_angle_pid->Calculate(degrees_off.value())};
        auto turn_feed = rotate_feedforward->Calculate(units::unit_t<units::compound_unit<units::meters, units::inverse<units::seconds>>>(velocity));
        rotate_motor->SetVoltage(turn_pid_voltage + turn_feed);
        cached_rotate_ff_voltage_ = turn_feed.value();
        cached_rotate_pid_voltage_ = turn_pid_voltage.value();
    }
    else
    {
        rotate_motor->SetVoltage(0_V);
        cached_rotate_ff_voltage_ = 0;
        cached_rotate_pid_voltage_ = 0;
    }
    // NOTE: updateNetworkTables() is called by update() — don't call it here
    // to avoid double-publishing every loop.
}

//updates the drive motor based on if the wheel is going forward or backward
void MK5NSwerveModule::updateDriveMotor(bool is_forward, units::degree_t degrees_off)
{
    units::velocity::meters_per_second_t drive_speed = target_speed;
    if(!is_forward)
    {
        drive_speed *= -1;
    }
    auto drive_ff_value = drive_feedforward->Calculate(drive_speed);
    //TODO: add pid
    auto drive_voltage = drive_ff_value * std::cos(units::radian_t{degrees_off}.to<double>());
    drive_motor->SetVoltage(drive_voltage);
}

//updates the drive motor voltage directly
void MK5NSwerveModule::updateDriveMotorVoltage(bool is_forward, units::volt_t voltage)
{
    auto drive_voltage = voltage;
    if(!is_forward)
    {
        drive_voltage *= -1;
    }
    drive_motor->SetVoltage(drive_voltage);
}

//updates the drive motor voltage directly
void MK5NSwerveModule::updateDriveMotorVoltage(units::volt_t voltage)
{
    drive_motor->SetVoltage(voltage);
}


//Called by swerve cordinator to update the module
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
    updateTurnMotor(degrees_off);
    //drive motor
    updateDriveMotor(is_forward);
    updateNetworkTables();
}

//Points the turn motor to a specific angle
void MK5NSwerveModule::pointTurnMotor(units::angle::degree_t target_angle)
{
    units::angle::degree_t current_angle = getAngle();
    units::angle::degree_t degrees_off = get180Range(current_angle - target_angle);
    updateTurnMotor(degrees_off);
}

//Resets the PID controllers' errors
void MK5NSwerveModule::resetError()
{
    rotate_velocity_pid->Reset();
    rotate_angle_pid->Reset();
}

//Gets the position of the module
frc::SwerveModulePosition MK5NSwerveModule::getModulePosition()
{
    frc::SwerveModulePosition result;
    result.angle = units::degree_t{getAngle().value()};
    result.distance = -units::meter_t{drive_motor->GetPosition().GetValue().value() * drive_turns_per_meter};
    return result;
}

//Gets the raw angle from the encoder without offset correction
units::angle::degree_t MK5NSwerveModule::getRawAngle()
{
    return get180Range(encoder->GetPosition().GetValue().value() * 360.0_deg);
}

//Gets the angle from the encoder with offset correction
units::angle::degree_t MK5NSwerveModule::getAngle()
{
    return get180Range(getRawAngle() - offset);
}

//Gets the distance the drive motor has traveled
units::length::meter_t MK5NSwerveModule::getEncoderPosition()
{
    return units::meter_t{drive_motor->GetPosition().GetValue().value() * drive_turns_per_meter};
}

double MK5NSwerveModule::getEncoderRawPosition()
{
    return drive_motor->GetPosition().GetValueAsDouble();
}


//Updates the network table values
void MK5NSwerveModule::updateNetworkTables()
{
    // Throttle NT updates: 0 or 1 = every loop, N = every Nth loop
    if (nt_throttle_interval_ > 1)
    {
        nt_update_counter_++;
        if (nt_update_counter_ < nt_throttle_interval_) return;
        nt_update_counter_ = 0;
    }

    nt_target_angle->setValue(target_angle.value());
    nt_angle->setValue(getAngle().value());
    nt_raw_angle->setValue(getRawAngle().value());
    nt_rotate_velocity->setValue(encoder->GetVelocity().GetValueAsDouble() * 360);
    nt_rotate_ff_voltage->setValue(cached_rotate_ff_voltage_);
    nt_rotate_pid_voltage->setValue(cached_rotate_pid_voltage_);
    nt_encoder_position->setValue(getEncoderPosition().value());
    nt_encoder_raw_position->setValue(getEncoderRawPosition());
}

//Function to convert from json to Mk5NConfig
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
    j.at(MK5NSwerveModule::Mk5NConfig::DRIVE_TURNS_PER_METER).get_to(config.drive_turns_per_meter);
    j.at(MK5NSwerveModule::Mk5NConfig::DRIVE_FF).get_to(config.drive_ff);
}

} // namespace HighVoltage



