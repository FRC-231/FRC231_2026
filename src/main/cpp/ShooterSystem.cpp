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
        HighVoltage::VoltsPerMeterPerSecond{shooter_config.shooter_ff.k_v},
        HighVoltage::VoltsPerMeterPerSecondSquared{shooter_config.shooter_ff.k_a}
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

    nt_ff_voltage.setOnUpdate([this](const double& value)
    {
        if(value != ff_voltage.value())
        {
            std::cout << "Set ff_voltage to " << value << std::endl;
            ff_voltage = units::volt_t{value};
        }
    });

    nt_shooter_voltage.setOnUpdate([this](const double& value)
    {
        if(value != shooter_voltage.value())
        {
            std::cout << "Set shooter_voltage to " << value << std::endl;
            shooter_voltage = units::volt_t{value};
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
        default:
            doShooterStop();
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
    nt_ff_voltage.setValue(ff_voltage.value());
    nt_shooter_velocity.setValue(left_shooter_motor->GetVelocity().GetValueAsDouble());
    nt_shooter_gear_ratio.setValue(shooter_gear_ratio);
    nt_left_shooter_velocity.setValue(getLeftShooterVelocity().value());
    nt_right_shooter_velocity.setValue(getRightShooterVelocity().value());
    //nt_shooter_voltage.setValue(shooter_voltage.value());
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

        case ShooterMode::FF:
            return "FF";
    }
    return "UNKNOWN";
}

void ShooterSystem::setShooterMode(ShooterMode mode)
{
    shooter_mode = mode;
}

void ShooterSystem::doShooterVoltageControl()
{
    left_shooter_motor->SetVoltage(shooter_voltage);
    right_shooter_motor->SetVoltage(shooter_voltage-2_V);
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
    return left_shooter_motor->GetVelocity().GetValue();
}

units::revolutions_per_minute_t ShooterSystem::getRightShooterVelocity()
{
    return right_shooter_motor->GetVelocity().GetValue();
}

