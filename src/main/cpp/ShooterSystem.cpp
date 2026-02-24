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
    shooter_pid = std::make_unique<frc::PIDController>(
        shooter_config.shooter_pid.k_p,
        shooter_config.shooter_pid.k_i,
        shooter_config.shooter_pid.k_d
    );
    shooter_ff = std::make_unique<frc::SimpleMotorFeedforward<units::meters>>(
        units::volt_t{shooter_config.shooter_ff.k_s},
        units::unit_t<frc::SimpleMotorFeedforward<units::meters>::kv_unit>{shooter_config.shooter_ff.k_v},
        units::unit_t<frc::SimpleMotorFeedforward<units::meters>::ka_unit>{shooter_config.shooter_ff.k_a}
    );

    ctre::phoenix6::configs::TalonFXConfiguration shooter_left_config{};
    shooter_left_config.MotorOutput.Inverted = ctre::phoenix6::signals::InvertedValue::Clockwise_Positive;
    shooter_left_config.MotorOutput.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Coast;
    left_shooter_motor->GetConfigurator().Apply(shooter_left_config);

    ctre::phoenix6::configs::TalonFXConfiguration shooter_right_config{};
    shooter_right_config.MotorOutput.Inverted = ctre::phoenix6::signals::InvertedValue::Clockwise_Positive;
    shooter_right_config.MotorOutput.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Coast;
    right_shooter_motor->GetConfigurator().Apply(shooter_right_config);

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

    nt_shooter_kP.setOnUpdate([this](const double& value)
    {
        if(value != shooter_pid->GetP())
        {
            std::cout << "Set shooter_kP to " << value << std::endl;
            shooter_pid->SetP(value);
        }
    });

    nt_shooter_kI.setOnUpdate([this](const double& value)
    {
        if(value != shooter_pid->GetI())
        {
            std::cout << "Set shooter_kI to " << value << std::endl;
            shooter_pid->SetI(value);
        }
    });

    nt_shooter_kD.setOnUpdate([this](const double& value)
    {
        if(value != shooter_pid->GetD())
        {
            std::cout << "Set shooter_kD to " << value << std::endl;
            shooter_pid->SetD(value);
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
    nt_shooter_kP.setValue(shooter_pid->GetP());
    nt_shooter_kI.setValue(shooter_pid->GetI());
    nt_shooter_kD.setValue(shooter_pid->GetD());
    nt_shooter_velocities.setValue({getLeftShooterVelocity().value(), getRightShooterVelocity().value()});
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

void ShooterSystem::doShooterSpeedControl()
{
    //Calculate feedforward voltage. The FF wants "meters" but will will just cast
    //the RPMs to MPS without any conversion since the feedforward constants will
    //just be tuned to match the RPMs instead of MPS
    units::meters_per_second_t left_velocity_as_mps = units::meters_per_second_t{left_shooter_target_velocity.value()};
    units::meters_per_second_t right_velocity_as_mps = units::meters_per_second_t{right_shooter_target_velocity.value()};
    units::volt_t left_ff_voltage = shooter_ff->Calculate(left_velocity_as_mps);
    units::volt_t right_ff_voltage = shooter_ff->Calculate(right_velocity_as_mps);

    //Calculate feedback voltage
    double left_velocity_error = left_shooter_target_velocity.value() - getLeftShooterVelocity().value();
    double right_velocity_error = right_shooter_target_velocity.value() - getRightShooterVelocity().value();
    units::volt_t left_fb_voltage = units::volt_t{shooter_pid->Calculate(left_velocity_error)};
    units::volt_t right_fb_voltage = units::volt_t{shooter_pid->Calculate(right_velocity_error)};

    //Set voltages
    left_shooter_motor->SetVoltage(left_ff_voltage + left_fb_voltage);
    right_shooter_motor->SetVoltage(right_ff_voltage + right_fb_voltage);
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

