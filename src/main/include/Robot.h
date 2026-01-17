// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <string>

#include <frc/TimedRobot.h>
#include <frc/smartdashboard/SendableChooser.h>
#include "SmartPID.h"
#include "NetworkTableValue.h"

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

    double test_value = 0.0;

    HighVoltage::SmartPID pid{ "ExamplePID", 0.1, 0.01, 0.001 };
    NetworkTableValue<double> nt_test{ "test/TestValue", 0.0 };
    NetworkTableValue<double> nt_test_2{ "test/TestValue2", 0.0 };
};
