#include <IntakeSystem.h>
#include <iostream> 

// ==== Constructor ====

IntakeSystem::IntakeSystem(std::shared_ptr<RobotContext> context):
    context(context)
{
    const auto& intake_config = context->getConfig()->intake_config;
    pivot_motor = std::make_unique<ctre::phoenix6::hardware::TalonFX>(intake_config.pivot_motor_id);
    feed_motor = std::make_unique<ctre::phoenix6::hardware::TalonFX>(intake_config.feed_motor_id);
    pivot_encoder = std::make_unique<ctre::phoenix6::hardware::CANcoder>(intake_config.pivot_encoder_id);
    ctre::phoenix6::configs::CANcoderConfiguration encoder_config{};
    encoder_config.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5_tr;
    encoder_config.MagnetSensor.SensorDirection = ctre::phoenix6::signals::SensorDirectionValue::CounterClockwise_Positive;
    pivot_encoder->GetConfigurator().Apply(encoder_config);
    feed_intake_voltage = units::volt_t{intake_config.feed_intake_voltage};
    feed_outtake_voltage = units::volt_t{intake_config.feed_outtake_voltage};
    pivot_max_voltage = units::volt_t{intake_config.pivot_max_voltage};
    pivot_min_voltage = units::volt_t{intake_config.pivot_min_voltage};
    pivot_max_angle = units::degree_t{intake_config.pivot_max_angle};
    pivot_min_angle = units::degree_t{intake_config.pivot_min_angle};
    pivot_up_angle = units::degree_t{intake_config.pivot_up_angle};
    pivot_down_angle = units::degree_t{intake_config.pivot_down_angle};
    pivot_pid = std::make_unique<frc::PIDController>(
        intake_config.pivot_pid.k_p,
        intake_config.pivot_pid.k_i,
        intake_config.pivot_pid.k_d
    );
    pivot_encoder_offset = units::degree_t{intake_config.pivot_encoder_offset};


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

// ==== Subsystem Override ====

void IntakeSystem::Periodic()
{
    return;//remove
    switch(feed_mode)
    {
        case FeedMode::STOP:
        {
            doFeedStop();
        }break;

        case FeedMode::INTAKE:
        {
            doFeedIntake();
        }break;

        case FeedMode::OUTTAKE:
        {
            doFeedOuttake();
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
            doPivotMove();
        }break;
    }
    updateNetworkTable();
}

// ==== Feed Commands ====

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

// ==== Pivot Commands ====

void IntakeSystem::pivotStop()
{
    pivot_mode = PivotMode::STOP;
}

void IntakeSystem::pivotUp()
{
    pivot_mode = PivotMode::UP;
}

void IntakeSystem::pivotDown()
{
    pivot_mode = PivotMode::DOWN;
}

// ==== Getters ====

units::degree_t IntakeSystem::getPivotAngle()
{
    units::degree_t current_angle = (pivot_encoder->GetAbsolutePosition().GetValue() * 360.0) - pivot_encoder_offset;
    return current_angle;
}

// ==== Enum Helpers ====

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

// ==== Private: Mode Handlers ====

void IntakeSystem::doFeedStop()
{
    feed_motor->SetVoltage(0_V);
}

void IntakeSystem::doFeedIntake()
{
    feed_motor->SetVoltage(feed_intake_voltage);
}

void IntakeSystem::doFeedOuttake()
{
    feed_motor->SetVoltage(feed_outtake_voltage);
}

void IntakeSystem::doPivotStop()
{
    pivot_motor->SetVoltage(0_V);
}

void IntakeSystem::doPivotMove()
{
    units::degree_t current_angle = getPivotAngle();
    units::degree_t target_angle;
    if(pivot_mode == PivotMode::UP)
    {
        target_angle = pivot_up_angle;
    }
    else
    {
        target_angle = pivot_down_angle;
    }
    pivot_pid->SetSetpoint(target_angle.value());
    units::volt_t power = units::volt_t{pivot_pid->Calculate(current_angle.value())};
    power = HighVoltage::clamp(pivot_min_voltage, pivot_max_voltage, power);
    if(current_angle > pivot_max_angle)
    {
        power = std::min(power, 0_V);
    }
    else if(current_angle < pivot_min_angle)
    {
        power = std::max(power, 0_V);
    }
    
    pivot_motor->SetVoltage(power);
}

// ==== Private: Internal Helpers ====

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
    nt_pivot_encoder_offset.setValue(pivot_encoder_offset.value());
    nt_pivot_down_angle.setValue(pivot_down_angle.value());
    nt_pivot_up_angle.setValue(pivot_up_angle.value());
    nt_pivot_encoder_raw.setValue(pivot_encoder->GetAbsolutePosition().GetValueAsDouble() * 360);
}


void IntakeSystem::addToLuaState(Keeko::ThreadedScript& script)
{
    luabridge::getGlobalNamespace(script)
    .beginClass<IntakeSystem> ("IntakeSystem")
        .addFunction("feedStop", &IntakeSystem::feedStop)
        .addFunction("feedIntake", &IntakeSystem::feedIntake)
        .addFunction("feedOuttake", &IntakeSystem::feedOuttake)
        .addFunction("pivotStop", &IntakeSystem::pivotStop)
        .addFunction("pivotUp", &IntakeSystem::pivotUp)
        .addFunction("pivotDown", &IntakeSystem::pivotDown)
    .endClass();
    script.addInstance("intake_system", this);
}