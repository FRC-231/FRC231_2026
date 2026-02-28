#include <ShooterSystem.h>
#include <iostream> 

ShooterSystem::ShooterSystem(std::shared_ptr<RobotContext> context):
    context(context)
{
    const auto& shooter_config = context->getConfig()->shooter_config;
    hopper_motor = std::make_unique<ctre::phoenix6::hardware::TalonFX>(shooter_config.hopper_motor_id);
    feed_motor = std::make_unique<ctre::phoenix6::hardware::TalonFX>(shooter_config.feed_motor_id);
    left_shooter_motor = std::make_unique<ctre::phoenix6::hardware::TalonFX>(shooter_config.left_shooter_motor_id);
    right_shooter_motor = std::make_unique<ctre::phoenix6::hardware::TalonFX>(shooter_config.right_shooter_motor_id);
    hopper_voltage = units::volt_t{shooter_config.hopper_voltage};
    feed_voltage = units::volt_t{shooter_config.feed_voltage};
    shooter_gear_ratio = shooter_config.shooter_gear_ratio;
    max_shooter_voltage = units::volt_t{shooter_config.shooter_max_voltage};
    min_shooter_voltage = units::volt_t{shooter_config.shooter_min_voltage};
    servo_controller = std::make_unique<rev::servohub::ServoHub>(shooter_config.servo_controller_id);
    left_servo_id = shooter_config.left_servo_id;
    right_servo_id = shooter_config.right_servo_id;

    auto left_channel_id  = static_cast<rev::servohub::ServoChannel::ChannelId>(left_servo_id);
    auto right_channel_id = static_cast<rev::servohub::ServoChannel::ChannelId>(right_servo_id);

    // Explicitly configure pulse range (500–1500–2500 µs, standard servo range)
    // so the device range matches what we send regardless of hardware client state.
    rev::servohub::ServoChannelConfig left_servo_cfg{left_channel_id};
    left_servo_cfg.PulseRange(500, 1500, 2500);
    rev::servohub::ServoChannelConfig right_servo_cfg{right_channel_id};
    right_servo_cfg.PulseRange(500, 1500, 2500);
    rev::servohub::ServoHubConfig servo_hub_config{};
    servo_hub_config.Apply(left_channel_id, left_servo_cfg);
    servo_hub_config.Apply(right_channel_id, right_servo_cfg);
    servo_controller->Configure(servo_hub_config, rev::ResetMode::kNoResetSafeParameters);

    servo_controller->GetServoChannel(left_channel_id).SetEnabled(true);
    servo_controller->GetServoChannel(left_channel_id).SetPowered(true);
    servo_controller->GetServoChannel(right_channel_id).SetEnabled(true);
    servo_controller->GetServoChannel(right_channel_id).SetPowered(true);



    ctre::phoenix6::configs::TalonFXConfiguration shooter_left_config{};
    shooter_left_config.MotorOutput.Inverted = ctre::phoenix6::signals::InvertedValue::Clockwise_Positive;
    shooter_left_config.MotorOutput.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Coast;
    left_shooter_motor->GetConfigurator().Apply(shooter_left_config);

    ctre::phoenix6::configs::TalonFXConfiguration shooter_right_config{};
    shooter_right_config.MotorOutput.Inverted = ctre::phoenix6::signals::InvertedValue::Clockwise_Positive;
    shooter_right_config.MotorOutput.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Coast;
    right_shooter_motor->GetConfigurator().Apply(shooter_right_config);

    HighVoltage::applyMotionConfigToTalonFXFromRPM(shooter_config.shooter_pid, shooter_config.shooter_ff, *left_shooter_motor,  shooter_gear_ratio);
    HighVoltage::applyMotionConfigToTalonFXFromRPM(shooter_config.shooter_pid, shooter_config.shooter_ff, *right_shooter_motor, shooter_gear_ratio);

    // Seed the NT values with the config values so they display correctly on boot.
    nt_shooter_kP.setValue(shooter_config.shooter_pid.k_p);
    nt_shooter_kI.setValue(shooter_config.shooter_pid.k_i);
    nt_shooter_kD.setValue(shooter_config.shooter_pid.k_d);

    // Bind NT updates: writing kP/kI/kD from a dashboard will immediately
    // re-apply the scaled gains to both shooter motors.
    HighVoltage::bindPIDNetworkTableToTalonFXFromRPM(nt_shooter_kP, nt_shooter_kI, nt_shooter_kD,
                                                      shooter_config.shooter_ff, *left_shooter_motor,  shooter_gear_ratio);
    HighVoltage::bindPIDNetworkTableToTalonFXFromRPM(nt_shooter_kP, nt_shooter_kI, nt_shooter_kD,
                                                      shooter_config.shooter_ff, *right_shooter_motor, shooter_gear_ratio);

    ctre::phoenix6::configs::TalonFXConfiguration shooter_feed_config{};
    shooter_feed_config.MotorOutput.Inverted = ctre::phoenix6::signals::InvertedValue::Clockwise_Positive;
    shooter_feed_config.MotorOutput.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Brake;
    feed_motor->GetConfigurator().Apply(shooter_feed_config);

    nt_hopper_voltage.setOnUpdate([this](const double& value)
    {
        if(value != hopper_voltage.value())
        {
            std::cout << "Set hopper_voltage to " << value << std::endl;
            hopper_voltage = units::volt_t{value};
        }
    });

    nt_feed_voltage.setOnUpdate([this](const double& value)
    {
        if(value != feed_voltage.value())
        {
            std::cout << "Set feed_voltage to " << value << std::endl;
            feed_voltage = units::volt_t{value};
        }
    });

    nt_left_shooter_voltage.setOnUpdate([this](const double& value)
    {
        if(value != left_shooter_voltage.value())
        {
            std::cout << "Set left_shooter_voltage to " << value << std::endl;
            left_shooter_voltage = units::volt_t{value};
        }
    });

    nt_right_shooter_voltage.setOnUpdate([this](const double& value)
    {
        if(value != right_shooter_voltage.value())
        {
            std::cout << "Set right_shooter_voltage to " << value << std::endl;
            right_shooter_voltage = units::volt_t{value};
        }
    });

    nt_left_shooter_target_velocity.setOnUpdate([this](const double& value)
    {
        if(value != left_shooter_target_velocity.value())
        {
            std::cout << "Set left_shooter_target_velocity to " << value << std::endl;
            left_shooter_target_velocity = units::revolutions_per_minute_t{value};
        }
    });

    nt_right_shooter_target_velocity.setOnUpdate([this](const double& value)
    {
        if(value != right_shooter_target_velocity.value())
        {
            std::cout << "Set right_shooter_target_velocity to " << value << std::endl;
            right_shooter_target_velocity = units::revolutions_per_minute_t{value};
        }
    });
}

void ShooterSystem::Periodic()
{
    switch(shooter_mode)
    {
        case ShooterMode::VOLTAGE:
            doShooterVoltageControl();
            break;
        case ShooterMode::SPEED:
            doShooterSpeedControl();
            break;
        default:
            doShooterStop();
            break;
    }

    switch(loader_mode)
    {
        case LoaderMode::LOAD:
            doLoaderLoad();
            break;
        case LoaderMode::UNLOAD:
            doLoaderUnload();
            break;
        default:
            doLoaderStop();
            break;
    }

    auto left_channel_id  = static_cast<rev::servohub::ServoChannel::ChannelId>(left_servo_id);
    auto right_channel_id = static_cast<rev::servohub::ServoChannel::ChannelId>(right_servo_id);
    servo_controller->GetServoChannel(left_channel_id).SetEnabled(true);
    servo_controller->GetServoChannel(left_channel_id).SetPowered(true);
    servo_controller->GetServoChannel(right_channel_id).SetEnabled(true);
    servo_controller->GetServoChannel(right_channel_id).SetPowered(true);
    servo_controller->GetServoChannel(left_channel_id).SetPulseWidth(1200);
    servo_controller->GetServoChannel(right_channel_id).SetPulseWidth(1200);

    // Diagnostics: print servo status once per second (~50 loops at 20ms)
    servo_diag_counter_++;
    if (servo_diag_counter_ >= 50) {
        servo_diag_counter_ = 0;
        auto s0 = servo_controller->GetPeriodicStatus0();
        auto s2 = servo_controller->GetPeriodicStatus2();
        auto s3 = servo_controller->GetPeriodicStatus3();
        std::cout << "[ServoHub] systemEnabled=" << (int)s0.systemEnabled
                  << " primaryHeartbeatLock=" << (int)s0.primaryHeartbeatLock
                  << " commMode=" << (int)s0.communicationMode
                  << " servoVoltage=" << s0.servoVoltage
                  << " deviceVoltage=" << s0.voltage
                  << " | ch" << (int)left_servo_id
                  << ": enabled=" << s2.channel2Enabled
                  << " pulseWidth=" << s2.channel2PulseWidth
                  << " outOfRange=" << s2.channel2OutOfRange
                  << " | ch" << (int)right_servo_id
                  << ": enabled=" << s3.channel3Enabled
                  << " pulseWidth=" << s3.channel3PulseWidth
                  << " outOfRange=" << s3.channel3OutOfRange
                  << std::endl;
        if (servo_controller->HasActiveFault()) {
            auto faults = servo_controller->GetFaults();
            std::cout << "[ServoHub] FAULTS: regulatorPowerGood=" << faults.regulatorPowerGood
                      << " hardware=" << faults.hardware
                      << " firmware=" << faults.firmware
                      << " lowBattery=" << faults.lowBattery
                      << " rawBits=0x" << std::hex << faults.rawBits << std::dec << std::endl;
        }
    }

    updateNetworkTable();
}

void ShooterSystem::updateNetworkTable()
{
    nt_hopper_voltage.setValue(hopper_voltage.value());
    nt_feed_voltage.setValue(feed_voltage.value());
    nt_loader_mode.setValue(loaderModeToString(loader_mode));
    nt_shooter_mode.setValue(shooterModeToString(shooter_mode));
    nt_shooter_velocity.setValue(left_shooter_motor->GetVelocity().GetValueAsDouble());
    nt_shooter_gear_ratio.setValue(shooter_gear_ratio);
    nt_left_shooter_velocity.setValue(getLeftShooterVelocity().value());
    nt_right_shooter_velocity.setValue(getRightShooterVelocity().value());
    nt_left_shooter_voltage.setValue(left_shooter_voltage.value());
    nt_right_shooter_voltage.setValue(right_shooter_voltage.value());
    nt_shooter_velocities.setValue({left_shooter_voltage.value(),right_shooter_voltage.value(),getLeftShooterVelocity().value(), getRightShooterVelocity().value()});
}

std::string ShooterSystem::loaderModeToString(LoaderMode mode)
{
    switch(mode)
    {
        case LoaderMode::STOP:
            return "STOP";

        case LoaderMode::LOAD:
            return "LOAD";

        case LoaderMode::UNLOAD:
            return "UNLOAD";
    }
    return "UNKNOWN";
}

std::string ShooterSystem::shooterModeToString(ShooterMode mode)
{
    switch(mode)
    {
        case ShooterMode::STOP:
            return "STOP";

        case ShooterMode::SPEED:
            return "SPEED";

        case ShooterMode::VOLTAGE:
            return "VOLTAGE";
    }
    return "UNKNOWN";
}

void ShooterSystem::setShooterMode(ShooterMode mode)
{
    shooter_mode = mode;
}

void ShooterSystem::doShooterVoltageControl()
{
    left_shooter_motor->SetVoltage(left_shooter_voltage);
    right_shooter_motor->SetVoltage(right_shooter_voltage);
    feed_motor->SetVoltage(feed_voltage);
}


void ShooterSystem::doShooterStop()
{
    left_shooter_motor->SetVoltage(0_V);
    right_shooter_motor->SetVoltage(0_V);
    feed_motor->SetVoltage(0_V);
}

units::revolutions_per_minute_t ShooterSystem::getLeftShooterVelocity()
{
    return left_shooter_motor->GetVelocity().GetValue() / shooter_gear_ratio;
}

units::revolutions_per_minute_t ShooterSystem::getRightShooterVelocity()
{
    return right_shooter_motor->GetVelocity().GetValue() / shooter_gear_ratio;
}

void ShooterSystem::setShooterVoltages(units::volt_t left_voltage, units::volt_t right_voltage)
{
    left_shooter_voltage = left_voltage;
    right_shooter_voltage = right_voltage;
    shooter_mode = ShooterMode::VOLTAGE;
}

void ShooterSystem::setShooterTargetVelocities(units::revolutions_per_minute_t left_velocity, units::revolutions_per_minute_t right_velocity)
{
    right_shooter_target_velocity = left_velocity;
    left_shooter_target_velocity = right_velocity;
    shooter_mode = ShooterMode::SPEED;
}

void ShooterSystem::doShooterSpeedControl()
{
    // Convert output RPM to motor-shaft rot/s for the TalonFX closed-loop.
    // The motor's kP/kI/kD/kS/kV/kA were already scaled for this unit in the constructor.
    units::turns_per_second_t left_rps  = units::turns_per_second_t{left_shooter_target_velocity.value()  / 60.0 * shooter_gear_ratio};
    units::turns_per_second_t right_rps = units::turns_per_second_t{right_shooter_target_velocity.value() / 60.0 * shooter_gear_ratio};

    left_shooter_motor->SetControl(shooter_velocity_request.WithVelocity(left_rps));
    right_shooter_motor->SetControl(shooter_velocity_request.WithVelocity(right_rps));
}

void ShooterSystem::setLoaderMode(LoaderMode mode)
{
    loader_mode = mode;
}

void ShooterSystem::doLoaderStop()
{
    hopper_motor->SetVoltage(0_V);
    feed_motor->SetVoltage(0_V);
}

void ShooterSystem::doLoaderLoad()
{
    hopper_motor->SetVoltage(hopper_voltage);
    feed_motor->SetVoltage(feed_voltage);
}

void ShooterSystem::doLoaderUnload()
{
    hopper_motor->SetVoltage(-hopper_voltage);
    feed_motor->SetVoltage(-feed_voltage);
}

