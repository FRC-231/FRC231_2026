#include <ShooterSystem.h>
#include <cmath>
#include <iostream> 

ShooterSystem::ShooterSystem(std::shared_ptr<RobotContext> context):
    context(context)
{
    const auto& shooter_config = context->getConfig()->shooter_config;
    hopper_motor = std::make_unique<ctre::phoenix6::hardware::TalonFX>(shooter_config.hopper_motor_id, "secondary");
    hopper_motor2 = std::make_unique<ctre::phoenix6::hardware::TalonFX>(shooter_config.hopper_motor2_id, "secondary");
    feed_motor = std::make_unique<ctre::phoenix6::hardware::TalonFX>(shooter_config.feed_motor_id, "rio");
    left_shooter_motor = std::make_unique<ctre::phoenix6::hardware::TalonFX>(shooter_config.left_shooter_motor_id, "rio");
    right_shooter_motor = std::make_unique<ctre::phoenix6::hardware::TalonFX>(shooter_config.right_shooter_motor_id, "rio");

    // Apply current limits to hopper and feed motors to prevent brownouts
    ctre::phoenix6::configs::TalonFXConfiguration hopper_config{};
    hopper_config.CurrentLimits.SupplyCurrentLimitEnable = true;
    hopper_config.CurrentLimits.SupplyCurrentLimit = 30_A;
    hopper_config.MotorOutput.Inverted = ctre::phoenix6::signals::InvertedValue::CounterClockwise_Positive;
    hopper_config.MotorOutput.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Brake;
    hopper_motor->GetConfigurator().Apply(hopper_config);


    ctre::phoenix6::configs::TalonFXConfiguration hopper_config2{};
    hopper_config2.CurrentLimits.SupplyCurrentLimitEnable = true;
    hopper_config2.CurrentLimits.SupplyCurrentLimit = 30_A;
    hopper_config2.MotorOutput.Inverted = ctre::phoenix6::signals::InvertedValue::Clockwise_Positive;
    hopper_config2.MotorOutput.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Brake;
    hopper_motor2->GetConfigurator().Apply(hopper_config2);

    ctre::phoenix6::configs::TalonFXConfiguration feed_motor_config{};
    feed_motor_config.CurrentLimits.SupplyCurrentLimitEnable = true;
    feed_motor_config.CurrentLimits.SupplyCurrentLimit = 30_A;
    feed_motor->GetConfigurator().Apply(feed_motor_config);

    hopper_voltage = units::volt_t{shooter_config.hopper_voltage};
    hopper_wiggle_low_voltage = units::volt_t{shooter_config.hopper_wiggle_low_voltage};
    hopper_wiggle_period = shooter_config.hopper_wiggle_period;
    feed_voltage = units::volt_t{shooter_config.feed_voltage};
    shooter_gear_ratio = shooter_config.shooter_gear_ratio;
    max_shooter_voltage = units::volt_t{shooter_config.shooter_max_voltage};
    min_shooter_voltage = units::volt_t{shooter_config.shooter_min_voltage};
    servo_controller = std::make_unique<rev::servohub::ServoHub>(shooter_config.servo_controller_id);
    left_servo_id = shooter_config.left_servo_id;
    right_servo_id = shooter_config.right_servo_id;
    at_speed_tolerance = units::revolutions_per_minute_t{shooter_config.at_speed_tolerance};
    idle_mode_velocity = units::revolutions_per_minute_t{shooter_config.idle_mode_velocity};
    snow_blow_target_rpm = units::revolutions_per_minute_t{shooter_config.snow_blow_target_rpm};
    snow_blow_pulse_width = shooter_config.snow_blow_pulse_width;
    snow_blow_proportional = shooter_config.snow_blow_proportional;
    manual_shooter_adjustment_multiplier = units::revolutions_per_minute_t{shooter_config.manual_shooter_adjustment_multiplier};

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
    // Limit supply current to prevent brownouts
    shooter_left_config.CurrentLimits.SupplyCurrentLimitEnable = true;
    shooter_left_config.CurrentLimits.SupplyCurrentLimit = 40_A;
    left_shooter_motor->GetConfigurator().Apply(shooter_left_config);

    ctre::phoenix6::configs::TalonFXConfiguration shooter_right_config{};
    shooter_right_config.MotorOutput.Inverted = ctre::phoenix6::signals::InvertedValue::Clockwise_Positive;
    shooter_right_config.MotorOutput.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Coast;
    // Limit supply current to prevent brownouts
    shooter_right_config.CurrentLimits.SupplyCurrentLimitEnable = true;
    shooter_right_config.CurrentLimits.SupplyCurrentLimit = 40_A;
    right_shooter_motor->GetConfigurator().Apply(shooter_right_config);

    HighVoltage::applyMotionConfigToTalonFXFromRPM(shooter_config.shooter_pid, shooter_config.shooter_ff, *left_shooter_motor,  shooter_gear_ratio);
    HighVoltage::applyMotionConfigToTalonFXFromRPM(shooter_config.shooter_pid, shooter_config.shooter_ff, *right_shooter_motor, shooter_gear_ratio);

    // Seed the NT values with the config values so they display correctly on boot.
    nt_shooter_kP.setValue(shooter_config.shooter_pid.k_p);
    nt_shooter_kI.setValue(shooter_config.shooter_pid.k_i);
    nt_shooter_kD.setValue(shooter_config.shooter_pid.k_d);

    // Bind NT updates: writing kP/kI/kD from a dashboard will immediately
    // re-apply the scaled gains to both shooter motors.
    // NOTE: We bind manually instead of calling bindPIDNetworkTableToTalonFXFromRPM
    // twice, because setOnUpdate replaces the callback — a second call would
    // overwrite the first, leaving one motor with stale gains.
    auto* p_kP = &nt_shooter_kP;
    auto* p_kI = &nt_shooter_kI;
    auto* p_kD = &nt_shooter_kD;
    auto* p_left  = left_shooter_motor.get();
    auto* p_right = right_shooter_motor.get();
    auto ff = shooter_config.shooter_ff;
    double gr = shooter_gear_ratio;

    auto reapplyBoth = [p_kP, p_kI, p_kD, ff, p_left, p_right, gr]()
    {
        HighVoltage::PIDConfig pid{
            p_kP->getValue(0.0),
            p_kI->getValue(0.0),
            p_kD->getValue(0.0)
        };
        HighVoltage::applyMotionConfigToTalonFXFromRPM(pid, ff, *p_left,  gr);
        HighVoltage::applyMotionConfigToTalonFXFromRPM(pid, ff, *p_right, gr);
    };

    nt_shooter_kP.setOnUpdate([reapplyBoth, key = nt_shooter_kP.getKey()](const double& value)
    {
        std::cout << key << " set to " << value << std::endl;
        reapplyBoth();
    });
    nt_shooter_kI.setOnUpdate([reapplyBoth, key = nt_shooter_kI.getKey()](const double& value)
    {
        std::cout << key << " set to " << value << std::endl;
        reapplyBoth();
    });
    nt_shooter_kD.setOnUpdate([reapplyBoth, key = nt_shooter_kD.getKey()](const double& value)
    {
        std::cout << key << " set to " << value << std::endl;
        reapplyBoth();
    });

    ctre::phoenix6::configs::TalonFXConfiguration shooter_feed_config{};
    shooter_feed_config.MotorOutput.Inverted = ctre::phoenix6::signals::InvertedValue::Clockwise_Positive;
    shooter_feed_config.MotorOutput.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Brake;
    feed_motor->GetConfigurator().Apply(shooter_feed_config);
    backspin_amount = units::revolutions_per_minute_t{shooter_config.backspin_amount};

    //Load distance to velocity map
    for(const auto& pair : shooter_config.distance_to_shooter_velocity)
    {
        distance_to_shooter_velocity[units::meter_t{pair.first}] = units::revolutions_per_minute_t{pair.second};
    }
    //Load distance to pulsewidth map
    for(const auto& pair : shooter_config.distance_to_pulsewidth)
    {
        distance_to_pulsewidth[units::meter_t{pair.first}] = pair.second;
    }
    //Load distance to backspin map
    for(const auto& pair : shooter_config.distance_to_backspin)
    {
        distance_to_backspin[units::meter_t{pair.first}] = units::revolutions_per_minute_t{pair.second};
    }

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

    nt_servo_pulse_width.setOnUpdate([this](const int& value)
    {
        if(value != servo_pulse_width)
        {
            std::cout << "Set servo_pulse_width to " << value << std::endl;
            servo_pulse_width = value;
        }
    });

    nt_shooter_auto_adjustment.setOnUpdate([this](const double& value)
    {
        if(value != shooter_auto_adjustment.value())
        {
            std::cout << "Set shooter_auto_adjustment to " << value << std::endl;
            shooter_auto_adjustment = units::revolutions_per_minute_t{value};
        }
    });

    nt_backspin_adjustment.setOnUpdate([this](const double& value)
    {
        if(value != backspin_adjustment.value())
        {
            std::cout << "Set backspin_adjustment to " << value << std::endl;
            backspin_adjustment = units::revolutions_per_minute_t{value};
        }
    });

    nt_shooter_angle_adjustment.setOnUpdate([this](const double& value)
    {
        if(value != shooter_angle_adjustment)
        {
            std::cout << "Set shooter_angle_adjustment to " << value << std::endl;
            shooter_angle_adjustment = value;
        }
    });

    nt_shooter_manual_auto_speed.setOnUpdate([this](const double& value)
    {
        if(value != shooter_manual_auto_speed.value())
        {
            std::cout << "Set shooter_manual_auto_speed to " << value << std::endl;
            shooter_manual_auto_speed = units::revolutions_per_minute_t{value};
        }
    });

    nt_backspin_amount.setOnUpdate([this](const double& value)
    {
        if(value != backspin_amount.value())
        {
            std::cout << "Set backspin_amount to " << value << std::endl;
            backspin_amount = units::revolutions_per_minute_t{value};
        }
    });

    nt_snow_blow_target_rpm.setOnUpdate([this](const double& value)
    {
        if(value != snow_blow_target_rpm.value())
        {
            std::cout << "Set snow_blow_target_rpm to " << value << std::endl;
            snow_blow_target_rpm = units::revolutions_per_minute_t{value};
        }
    });

    nt_snow_blow_pulse_width.setOnUpdate([this](const int& value)
    {
        if(value != snow_blow_pulse_width)
        {
            std::cout << "Set snow_blow_pulse_width to " << value << std::endl;
            snow_blow_pulse_width = value;
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
        case ShooterMode::AUTO:
            doShooterAutoControl();
            break;
        case ShooterMode::MANUAL_AUTO:
            doShooterManualAutoControl();
            break;
        case ShooterMode::SNOW_BLOW:
            doShooterSnowBlowControl();
            break;
        case ShooterMode::IDLE:
            doShooterIdleControl();
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

    // Diagnostics: print servo status once every ~2 seconds (100 loops at 20ms)
    // Using a higher interval to avoid console I/O overhead


    updateNetworkTable();
}

void ShooterSystem::updateNetworkTable()
{
    // Throttle NT updates: 0 or 1 = every loop, N = every Nth loop
    if (shooter_nt_throttle_interval_ > 1)
    {
        shooter_nt_counter_++;
        if (shooter_nt_counter_ < shooter_nt_throttle_interval_) return;
        shooter_nt_counter_ = 0;
    }

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
    nt_backspin_amount.setValue(backspin_amount.value());
    nt_is_shooter_at_speed.setValue(isAtSpeed());
    nt_snow_blow_target_rpm.setValue(snow_blow_target_rpm.value());
    nt_snow_blow_pulse_width.setValue(snow_blow_pulse_width);
    nt_manual_shooter_adjustment.setValue(manual_shooter_adjustment.value());
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

        case ShooterMode::AUTO:
            return "AUTO";

        case ShooterMode::MANUAL_AUTO:
            return "MANUAL_AUTO";

        case ShooterMode::IDLE:
            return "IDLE";

        case ShooterMode::SNOW_BLOW:
            return "SNOW_BLOW";
    }
    return "UNKNOWN";
}

void ShooterSystem::setShooterMode(ShooterMode mode)
{
    shooter_mode = mode;
}

void ShooterSystem::stopShooter()
{
    setShooterMode(ShooterMode::STOP);
}

void ShooterSystem::autoShoot()
{
    setShooterMode(ShooterMode::AUTO);
}

void ShooterSystem::snowBlow()
{
    setShooterMode(ShooterMode::SNOW_BLOW);
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

void ShooterSystem::setDistanceUpdateCallback(std::function<units::meter_t()> callback)
{
    distance_update_callback = callback;
}

void ShooterSystem::setDistanceToWallUpdateCallback(std::function<units::meter_t()> callback)
{
    distance_to_wall_update_callback = callback;
}

void ShooterSystem::setShooterVoltages(units::volt_t left_voltage, units::volt_t right_voltage)
{
    left_shooter_voltage = left_voltage;
    right_shooter_voltage = right_voltage;
    shooter_mode = ShooterMode::VOLTAGE;
}

void ShooterSystem::setShooterVoltagesLua(double left_voltage, double right_voltage)
{
    setShooterVoltages(units::volt_t{left_voltage}, units::volt_t{right_voltage});
}

void ShooterSystem::setShooterTargetVelocities(units::revolutions_per_minute_t left_velocity, units::revolutions_per_minute_t right_velocity)
{
    right_shooter_target_velocity = left_velocity;
    left_shooter_target_velocity = right_velocity;
    shooter_mode = ShooterMode::SPEED;
}

void ShooterSystem::setShooterTargetVelocitiesLua(double left_velocity, double right_velocity)
{
    setShooterTargetVelocities(units::revolutions_per_minute_t{left_velocity}, units::revolutions_per_minute_t{right_velocity});
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

void ShooterSystem::doShooterAutoControl()
{
    if(!distance_update_callback)
    {
        std::cerr << "Error: ShooterSystem::doShooterAutoControl called but distance_update_callback is not set!" << std::endl;
        return;
    }
    distance_to_target = distance_update_callback();
    auto shooter_velocity = HighVoltage::cubicHermiteInterpolation<units::meter_t, units::revolutions_per_minute_t>(distance_to_shooter_velocity, distance_to_target);
    auto left_shooter_speed = shooter_velocity + shooter_auto_adjustment + manual_shooter_adjustment;
    auto right_shooter_speed = shooter_velocity + shooter_auto_adjustment + manual_shooter_adjustment;
    auto backspin_velocity = HighVoltage::cubicHermiteInterpolation<units::meter_t, units::revolutions_per_minute_t>(distance_to_backspin, distance_to_target);
    auto new_servo_pulse_width = HighVoltage::cubicHermiteInterpolation<units::meter_t, double>(distance_to_pulsewidth, distance_to_target);
    //Set the servo pulse width directly from the interpolation with adjustment
    servo_pulse_width = static_cast<int>(new_servo_pulse_width + shooter_angle_adjustment);
    auto left_channel_id  = static_cast<rev::servohub::ServoChannel::ChannelId>(left_servo_id);
    auto right_channel_id = static_cast<rev::servohub::ServoChannel::ChannelId>(right_servo_id);
    servo_controller->GetServoChannel(left_channel_id).SetPulseWidth(servo_pulse_width);
    servo_controller->GetServoChannel(right_channel_id).SetPulseWidth(servo_pulse_width);
    if(servo_pulse_width >= servo_center_pulse_width)
    {
        left_shooter_speed -= backspin_velocity/2;
        right_shooter_speed += backspin_velocity/2;
    }
    else
    {
        right_shooter_speed -= backspin_velocity/2;
        left_shooter_speed += backspin_velocity/2;
    }
    left_shooter_motor->SetControl(shooter_velocity_request.WithVelocity(units::turns_per_second_t{left_shooter_speed.value() / 60.0 * shooter_gear_ratio}));
    right_shooter_motor->SetControl(shooter_velocity_request.WithVelocity(units::turns_per_second_t{right_shooter_speed.value() / 60.0 * shooter_gear_ratio}));
    left_shooter_target_velocity = left_shooter_speed + manual_shooter_adjustment;
    right_shooter_target_velocity = right_shooter_speed + manual_shooter_adjustment;
}

void ShooterSystem::doShooterManualAutoControl()
{
    auto left_shooter_speed = shooter_manual_auto_speed + shooter_auto_adjustment;
    auto right_shooter_speed = shooter_manual_auto_speed + shooter_auto_adjustment;
    if(servo_pulse_width >= servo_center_pulse_width)
    {
        left_shooter_speed -= (backspin_amount + backspin_adjustment)/2;
        right_shooter_speed += (backspin_amount + backspin_adjustment)/2;
    }
    else
    {
        right_shooter_speed -= (backspin_amount + backspin_adjustment)/2;
        left_shooter_speed += (backspin_amount + backspin_adjustment)/2;
    }
    left_shooter_motor->SetControl(shooter_velocity_request.WithVelocity(units::turns_per_second_t{left_shooter_speed.value() / 60.0 * shooter_gear_ratio}));
    right_shooter_motor->SetControl(shooter_velocity_request.WithVelocity(units::turns_per_second_t{right_shooter_speed.value() / 60.0 * shooter_gear_ratio}));
    auto left_channel_id  = static_cast<rev::servohub::ServoChannel::ChannelId>(left_servo_id);
    auto right_channel_id = static_cast<rev::servohub::ServoChannel::ChannelId>(right_servo_id);
    servo_controller->GetServoChannel(left_channel_id).SetPulseWidth(servo_pulse_width);
    servo_controller->GetServoChannel(right_channel_id).SetPulseWidth(servo_pulse_width);
    left_shooter_target_velocity = left_shooter_speed + manual_shooter_adjustment;
    right_shooter_target_velocity = right_shooter_speed + manual_shooter_adjustment;
}

void ShooterSystem::doShooterIdleControl()
{
    //Set servos to servo_center_pulse_width
    auto left_channel_id  = static_cast<rev::servohub::ServoChannel::ChannelId>(left_servo_id);
    auto right_channel_id = static_cast<rev::servohub::ServoChannel::ChannelId>(right_servo_id);
    servo_controller->GetServoChannel(left_channel_id).SetPulseWidth(servo_center_pulse_width);
    servo_controller->GetServoChannel(right_channel_id).SetPulseWidth(servo_center_pulse_width);
    left_shooter_motor->SetControl(shooter_velocity_request.WithVelocity(units::turns_per_second_t{idle_mode_velocity.value() / 60.0 * shooter_gear_ratio}));
    right_shooter_motor->SetControl(shooter_velocity_request.WithVelocity(units::turns_per_second_t{idle_mode_velocity.value() / 60.0 * shooter_gear_ratio}));
}

void ShooterSystem::doShooterSnowBlowControl()
{
    if(distance_to_wall_update_callback)
    {
        distance_to_wall = distance_to_wall_update_callback();
    }

    units::meter_t hub_x_position = RobotContext::HUB_POSITION.X();
    auto snow_blow_velocity = snow_blow_target_rpm
        + units::revolutions_per_minute_t{(distance_to_wall.value()) * snow_blow_proportional}
        + manual_shooter_adjustment;
    units::turns_per_second_t target_rps{snow_blow_velocity.value() / 60.0 * shooter_gear_ratio};
    left_shooter_motor->SetControl(shooter_velocity_request.WithVelocity(target_rps));
    right_shooter_motor->SetControl(shooter_velocity_request.WithVelocity(target_rps));
    left_shooter_target_velocity = snow_blow_velocity;
    right_shooter_target_velocity = snow_blow_velocity;
    auto left_channel_id  = static_cast<rev::servohub::ServoChannel::ChannelId>(left_servo_id);
    auto right_channel_id = static_cast<rev::servohub::ServoChannel::ChannelId>(right_servo_id);
    servo_controller->GetServoChannel(left_channel_id).SetPulseWidth(snow_blow_pulse_width);
    servo_controller->GetServoChannel(right_channel_id).SetPulseWidth(snow_blow_pulse_width);
}

void ShooterSystem::setLoaderMode(LoaderMode mode)
{
    loader_mode = mode;
}

void ShooterSystem::loaderStop()
{
    setLoaderMode(LoaderMode::STOP);
}

void ShooterSystem::loaderLoad()
{
    setLoaderMode(LoaderMode::LOAD);
}

void ShooterSystem::loaderUnload()
{
    setLoaderMode(LoaderMode::UNLOAD);
}

bool ShooterSystem::isAtSpeed()
{
    auto current_left = getLeftShooterVelocity();
    auto current_right = getRightShooterVelocity();
    auto left_difference = current_left - left_shooter_target_velocity;
    if(left_difference < 0_rpm)
    {
        left_difference *= -1;
    }
    auto right_difference = current_right - right_shooter_target_velocity;
    if(right_difference < 0_rpm)
    {
        right_difference *= -1;
    }
    if(left_difference > at_speed_tolerance)
    {
        return false;
    }
    if(right_difference > at_speed_tolerance)
    {
        return false;
    }
    return true;
}

void ShooterSystem::setManualShooterAdjustmentValue(double adjustment)
{
    manual_shooter_adjustment = units::revolutions_per_minute_t{adjustment * manual_shooter_adjustment_multiplier.value()};
}

void ShooterSystem::doLoaderStop()
{
    hopper_motor->SetVoltage(0_V);
    feed_motor->SetVoltage(0_V);
}

void ShooterSystem::doLoaderLoad()
{
    double current_time = frc::Timer::GetFPGATimestamp().value();
    // Sine wave oscillates between hopper_wiggle_low_voltage and hopper_voltage.
    // sin() ranges from -1 to 1, so we map it to [0, 1] and lerp between the two voltages.
    double phase = (2.0 * M_PI / hopper_wiggle_period) * current_time;
    double t = (std::sin(phase) + 1.0) / 2.0; // [0, 1]
    units::volt_t current_hopper_voltage = hopper_wiggle_low_voltage + t * (hopper_voltage - hopper_wiggle_low_voltage);
    hopper_motor->SetVoltage(-current_hopper_voltage);
    feed_motor->SetVoltage(feed_voltage);
}

void ShooterSystem::doLoaderUnload()
{
    hopper_motor->SetVoltage(hopper_voltage);
    feed_motor->SetVoltage(-feed_voltage);
}

void ShooterSystem::addToLuaState(Keeko::ThreadedScript& script)
{
    luabridge::getGlobalNamespace(script)
        .beginClass<ShooterSystem> ("ShooterSystem")
            .addFunction("setShooterVoltages", &ShooterSystem::setShooterVoltagesLua)
            .addFunction("setShooterTargetVelocities", &ShooterSystem::setShooterTargetVelocitiesLua)
            .addFunction("stopShooter", &ShooterSystem::stopShooter)
            .addFunction("autoShoot", &ShooterSystem::autoShoot)
            .addFunction("snowBlow", &ShooterSystem::snowBlow)
            .addFunction("loaderLoad", &ShooterSystem::loaderLoad)
            .addFunction("loaderUnload", &ShooterSystem::loaderUnload)
            .addFunction("loaderStop", &ShooterSystem::loaderStop)
            .addFunction("isAtSpeed", &ShooterSystem::isAtSpeed)
        .endClass();
    script.addInstance("shooter_system", this);

}

