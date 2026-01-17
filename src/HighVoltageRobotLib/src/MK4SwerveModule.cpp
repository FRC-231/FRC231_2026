#include "MK4SwerveModule.h"

namespace HighVoltage
{

Mk4SwerveModule::Mk4SwerveModule(std::string name,
    uint8_t drive_id,
    uint8_t rotate_id,
    uint8_t encoder_id,
    units::angle::degree_t encoder_offset,
    std::shared_ptr<frc::PIDController> drive_pid,
    std::shared_ptr<frc::ProfiledPIDController<units::meters>> angle_pid,
    std::shared_ptr<frc::SimpleMotorFeedforward<units::meters>> angle_feedforward,
    std::shared_ptr<frc::SimpleMotorFeedforward<units::meters>> drive_feedforward)
    : SwerveModule(name), offset(encoder_offset), drive_pid(drive_pid), angle_pid(angle_pid), angle_feedforward(angle_feedforward), drive_feedforward(drive_feedforward)
{
    drive_motor = std::make_unique<ctre::phoenix6::hardware::TalonFX>(drive_id);
    rotate_motor = std::make_unique<ctre::phoenix6::hardware::TalonFX>(rotate_id);
    
    ctre::phoenix6::configs::TalonFXConfiguration rotate_config{};
    rotate_config.MotorOutput.Inverted = ctre::phoenix6::signals::InvertedValue::CounterClockwise_Positive;
    rotate_config.MotorOutput.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Brake;
    rotate_motor->GetConfigurator().Apply(rotate_config);
    
    ctre::phoenix6::configs::TalonFXConfiguration drive_config{};
    drive_config.MotorOutput.Inverted = ctre::phoenix6::signals::InvertedValue::CounterClockwise_Positive;
    drive_config.MotorOutput.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Brake;
    drive_motor->GetConfigurator().Apply(drive_config);
    encoder = std::make_unique<ctre::phoenix6::hardware::CANcoder>(encoder_id);
    offset = encoder_offset + ALL_ENCODER_OFFSET;

    //Setup network table
    setupNetworkTable();

}

Mk4SwerveModule::Mk4SwerveModule(std::string name,
    uint8_t drive_id,
    uint8_t rotate_id,
    uint8_t encoder_id,
    units::angle::degree_t encoder_offset,
    PIDConfig drive_pid_config,
    PIDConfig angle_pid_config,
    TrapazoidalProfileConfig angle_constraints,
    FeedForwardConfig angle_feedforward_config,
    FeedForwardConfig drive_feedforward_config)
    : SwerveModule(name), offset(encoder_offset)
{
    drive_pid = std::make_shared<frc::PIDController>(drive_pid_config.k_p, drive_pid_config.k_i, drive_pid_config.k_d);
    frc::TrapezoidProfile<units::meters>::Constraints constraints{
        units::unit_t<units::compound_unit<units::meters, units::inverse<units::seconds>>>(angle_constraints.max_velocity),
        units::unit_t<units::compound_unit<units::meters, units::inverse<units::squared<units::seconds>>>>(angle_constraints.max_acceleration)
    };
    angle_pid = std::make_shared<frc::ProfiledPIDController<units::meters>>(angle_pid_config.k_p, angle_pid_config.k_d, angle_pid_config.k_d, constraints);
    angle_pid->SetTolerance(units::meter_t{1.0});
    angle_feedforward = std::make_shared<frc::SimpleMotorFeedforward<units::meters>>(
        units::volt_t(angle_feedforward_config.k_s),
        units::unit_t<frc::SimpleMotorFeedforward<units::meters>::kv_unit>{angle_feedforward_config.k_v},
        units::unit_t<frc::SimpleMotorFeedforward<units::meters>::ka_unit>{angle_feedforward_config.k_a}
    );
    drive_feedforward = std::make_shared<frc::SimpleMotorFeedforward<units::meters>>(
        units::volt_t(drive_feedforward_config.k_s),
        units::unit_t<frc::SimpleMotorFeedforward<units::meters>::kv_unit>{drive_feedforward_config.k_v},
        units::unit_t<frc::SimpleMotorFeedforward<units::meters>::ka_unit>{drive_feedforward_config.k_a}
    );
    drive_motor = std::make_unique<ctre::phoenix6::hardware::TalonFX>(drive_id);
    rotate_motor = std::make_unique<ctre::phoenix6::hardware::TalonFX>(rotate_id);
    
    ctre::phoenix6::configs::TalonFXConfiguration rotate_config{};
    rotate_config.MotorOutput.Inverted = ctre::phoenix6::signals::InvertedValue::Clockwise_Positive;
    rotate_config.MotorOutput.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Brake;
    rotate_motor->GetConfigurator().Apply(rotate_config);
    
    ctre::phoenix6::configs::TalonFXConfiguration drive_config{};
    drive_config.MotorOutput.Inverted = ctre::phoenix6::signals::InvertedValue::Clockwise_Positive;
    drive_config.MotorOutput.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Brake;
    drive_motor->GetConfigurator().Apply(drive_config);
    encoder = std::make_unique<ctre::phoenix6::hardware::CANcoder>(encoder_id);
    offset = encoder_offset + ALL_ENCODER_OFFSET;

    //Setup network table
    setupNetworkTable();

}

void Mk4SwerveModule::setupNetworkTable()
{
    network_table_inst = nt::NetworkTableInstance::GetDefault();
    network_table_name = module_name + " Module";
    module_table = network_table_inst.GetTable(network_table_name);

    drive_speed_topic = module_table->GetDoubleTopic("Drive Speed");
    drive_speed_publisher = drive_speed_topic.Publish();
    drive_speed_publisher.SetDefault(0.0);

    drive_target_topic = module_table->GetDoubleTopic("Drive Target");
    drive_target_publisher = drive_target_topic.Publish();
    drive_target_publisher.SetDefault(0.0);

    raw_angle_topic = module_table->GetDoubleTopic("Raw Angle");
    raw_angle_publisher = raw_angle_topic.Publish();
    raw_angle_publisher.SetDefault(0.0);

    target_angle_topic = module_table->GetDoubleTopic("Target Angle");
    target_angle_publisher = target_angle_topic.Publish();
    target_angle_publisher.SetDefault(0.0);

    rotate_voltage_topic = module_table->GetDoubleTopic("Rotate Voltage");
    rotate_voltage_publisher = rotate_voltage_topic.Publish();
    rotate_voltage_publisher.SetDefault(0.0);

    rotate_feedforward_voltage_topic = module_table->GetDoubleTopic("Rotate Feedforward Voltage");
    rotate_feedforward_voltage_publisher = rotate_feedforward_voltage_topic.Publish();
    rotate_feedforward_voltage_publisher.SetDefault(0.0);

    rotate_pid_voltage_topic = module_table->GetDoubleTopic("Rotate PID Voltage");
    rotate_pid_voltage_publisher = rotate_pid_voltage_topic.Publish();
    rotate_pid_voltage_publisher.SetDefault(0.0);

    rotate_error_topic = module_table->GetDoubleTopic("Rotate Error");
    rotate_error_publisher = rotate_error_topic.Publish();
    rotate_error_publisher.SetDefault(0.0);
}

void Mk4SwerveModule::populateNetworkTable()
{
    drive_speed_publisher.Set(drive_motor->GetVelocity().GetValue().value());
    drive_target_publisher.Set(target_speed.value());
    raw_angle_publisher.Set(getRawAngle().value());
    target_angle_publisher.Set(target_angle.value());
}

units::angle::degree_t Mk4SwerveModule::getAngle()
{
    return get180Range((encoder->GetAbsolutePosition().GetValue()) - offset);
}

units::angle::degree_t Mk4SwerveModule::getRawAngle()
{
    return get180Range(encoder->GetAbsolutePosition().GetValue());
}

units::length::meter_t Mk4SwerveModule::getEncoderPosition()
{
    return 0_m; //drive_motor->GetPosition().GetValue();//TODO: Fix this
}

frc::SwerveModulePosition Mk4SwerveModule::getModulePosition()
{
    frc::SwerveModulePosition result;
    result.angle = units::degree_t{getAngle().value() - 180};
    result.distance = units::meter_t{drive_motor->GetPosition().GetValue().value() * TURN_TO_METER_RATIO};
    return result;
}

void Mk4SwerveModule::update(std::chrono::duration<double> seconds_passed)
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

    //Turn Motor
    units::angle::degree_t degrees_off = get180Range(current_angle - new_target);
    if(std::abs(degrees_off.value()) > TURN_MOTOR_DEADZONE.value())
    {
        angle_pid->SetGoal(units::meter_t{0.0});
        double turn_power = angle_pid->Calculate(units::meter_t(degrees_off.value()));
        auto turn_feed = angle_feedforward->Calculate(units::unit_t<units::compound_unit<units::meters, units::inverse<units::seconds>>>((degrees_off * 8).value()));
        rotate_feedforward_voltage_publisher.Set(turn_feed.value());
        rotate_pid_voltage_publisher.Set(turn_power);
        rotate_voltage_publisher.Set((units::voltage::volt_t{turn_power} - turn_feed).value());
        rotate_error_publisher.Set(degrees_off.value());
        //TODO: Make the inversion configurable
        rotate_motor->SetVoltage(-(units::voltage::volt_t{turn_power}-turn_feed));
    }
    else
    {
        rotate_motor->Set(0.0);
    }

    //Drive Motor
    units::velocity::meters_per_second_t drive_speed = target_speed;
    if(is_forward)
    {
        drive_speed = -drive_speed;
    }
    auto drive_pid_value = units::volt_t{drive_pid->Calculate(drive_motor->GetVelocity().GetValue().value() * TURN_TO_METER_RATIO, drive_speed.value())};
    auto drive_ff_value = drive_feedforward->Calculate(units::meters_per_second_t{drive_speed});
    auto drive_power = (drive_pid_value + drive_ff_value) * std::cos(units::radian_t{degrees_off}.to<double>());
    drive_motor->SetVoltage(drive_power);

    populateNetworkTable();
}

void Mk4SwerveModule::resetError()
{
    drive_pid->Reset();
    angle_pid->Reset(0_m);
}

}
