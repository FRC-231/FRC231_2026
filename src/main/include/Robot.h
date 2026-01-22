// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <string>

#include <frc/TimedRobot.h>
#include <frc/smartdashboard/SendableChooser.h>
#include "SmartPID.h"
#include "NetworkTableValue.h"
#include <ctre/phoenix6/TalonFX.hpp>
#include <memory>
#include <ctre/phoenix6/CANcoder.hpp>
class Robot : public frc::TimedRobot
{
public:
    Robot();
    void RobotPeriodic() override;
    void AutonomousInit() override;
    void AutonomousPeriodic() override;
    void TeleopInit() override;
    void TeleopPeriodic() override;
    void DisabledInit() override;
    void DisabledPeriodic() override;
    void TestInit() override;
    void TestPeriodic() override;
    void SimulationInit() override;
    void SimulationPeriodic() override;

private:
    frc::SendableChooser<std::string> m_chooser;
    const std::string k_auto_name_default = "Default";
    const std::string k_auto_name_custom = "My Auto";
    std::string m_auto_selected;

    std::unique_ptr<ctre::phoenix6::hardware::TalonFX> test_motor;
    std::unique_ptr<ctre::phoenix6::hardware::CANcoder> test_encoder;

    HighVoltage::SmartPID test_pid{"test_pid", 0.08, 0.32, 0.0};

    double test_value = 0.0;
    NetworkTableValue<double> nt_velocity{"velocity", 0.0};
};
