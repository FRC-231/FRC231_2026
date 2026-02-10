#include <IntakeSystem.h>
#include <iostream> 

IntakeSystem::IntakeSystem(std::shared_ptr<RobotContext> context):
    context(context)
{
    const auto& intake_config = context->getConfig()->intake_config;
    pivot_motor = std::make_unique<ctre::phoenix6::hardware::TalonFX>(intake_config.pivot_motor_id);
    feed_motor = std::make_unique<ctre::phoenix6::hardware::TalonFX>(intake_config.feed_motor_id);
    pivot_encoder = std::make_unique<ctre::phoenix6::hardware::CANcoder>(intake_config.pivot_encoder_id);
    feed_intake_voltage = units::volt_t{intake_config.feed_intake_voltage};
    feed_outtake_voltage = units::volt_t{intake_config.feed_outtake_voltage};
    pivot_max_voltage = units::volt_t{intake_config.pivot_max_voltage};
    pivot_min_voltage = units::volt_t{intake_config.pivot_min_voltage};
    pivot_max_angle = units::degree_t{intake_config.pivot_max_angle};
    pivot_min_angle = units::degree_t{intake_config.pivot_min_angle};

    nt_feed_intake_voltage.setOnUpdate([this](const double& value)
    {
        if(value != feed_intake_voltage.value())
        {
            std::cout << "Set feed_intake_voltage to " << value << std::endl;
            feed_intake_voltage = units::volt_t{value};
        }
    });

    nt_feed_outtake_voltage.setOnUpdate([this](const double& value)
    {
        if(value != feed_outtake_voltage.value())
        {
            std::cout << "Set feed_outtake_voltage to " << value << std::endl;
            feed_outtake_voltage = units::volt_t{value};
        }
    });

    nt_pivot_max_voltage.setOnUpdate([this](const double& value)
    {
        if(value != pivot_max_voltage.value())
        {
            std::cout << "Set pivot_max_voltage to " << value << std::endl;
            pivot_max_voltage = units::volt_t{value};
        }
    });

    nt_pivot_min_voltage.setOnUpdate([this](const double& value)
    {
        if(value != pivot_min_voltage.value())
        {
            std::cout << "Set pivot_min_voltage to " << value << std::endl;
            pivot_min_voltage = units::volt_t{value};
        }
    });

    nt_pivot_max_angle.setOnUpdate([this](const double& value)
    {
        if(value != pivot_max_angle.value())
        {
            std::cout << "Set pivot_max_angle to " << value << std::endl;
            pivot_max_angle = units::degree_t{value};
        }
    });

    nt_pivot_min_angle.setOnUpdate([this](const double& value)
    {
        if(value != pivot_min_angle.value())
        {
            std::cout << "Set pivot_min_angle to " << value << std::endl;
            pivot_min_angle = units::degree_t{value};
        }
    });
}

void IntakeSystem::Periodic()
{
    switch(feed_mode)
    {
        case FeedMode::STOP:
        {
            doFeedStop();
        }break;

        default:
        {
            doFeedStop();
        }break;
    }

    switch(pivot_mode)
    {
        case PivotMode::STOP:
        {
            doPivotStop();
        }break;

        default:
        {
            doPivotStop();
        }break;
    }
    updateNetworkTable();
}

void IntakeSystem::doFeedStop()
{
    feed_motor->SetVoltage(0_V);
}

void IntakeSystem::doPivotStop()
{
    pivot_motor->SetVoltage(0_V);
}

void IntakeSystem::updateNetworkTable()
{
    nt_feed_intake_voltage.setValue(feed_intake_voltage.value());
    nt_feed_outtake_voltage.setValue(feed_outtake_voltage.value());
    nt_pivot_max_voltage.setValue(pivot_max_voltage.value());
    nt_pivot_min_voltage.setValue(pivot_min_voltage.value());
    nt_pivot_max_angle.setValue(pivot_max_angle.value());
    nt_pivot_min_angle.setValue(pivot_min_angle.value());
    nt_feed_mode.setValue(feedModeToString(feed_mode));
    nt_pivot_mode.setValue(pivotModeToString(pivot_mode));
}

void IntakeSystem::doFeedIntake()
{
    feed_motor->SetVoltage(feed_intake_voltage);
}

void IntakeSystem::doFeedOuttake()
{
    feed_motor->SetVoltage(feed_outtake_voltage);
}

void IntakeSystem::feedStop()
{
    feed_mode = FeedMode::STOP;
}

void IntakeSystem::feedIntake()
{
    feed_mode = FeedMode::INTAKE;
}

void IntakeSystem::feedOuttake()
{
    feed_mode = FeedMode::OUTTAKE;
}

std::string IntakeSystem::feedModeToString(FeedMode mode)
{
    switch(mode)
    {
        case FeedMode::STOP:
            return "STOP";

        case FeedMode::INTAKE:
            return "INTAKE";

        case FeedMode::OUTTAKE:
            return "OUTTAKE";
    }
    return "UNKNOWN";
}

std::string IntakeSystem::pivotModeToString(PivotMode mode)
{
    switch(mode)
    {
        case PivotMode::STOP:
            return "STOP";

        case PivotMode::UP:
            return "UP";

        case PivotMode::DOWN:
            return "DOWN";
    }
    return "UNKNOWN";
}

