// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <frc/smartdashboard/SmartDashboard.h>
#include <iostream>


Robot::Robot()
{
    m_chooser.SetDefaultOption(k_auto_name_default, k_auto_name_default);
    m_chooser.AddOption(k_auto_name_custom, k_auto_name_custom);
    frc::SmartDashboard::PutData("Auto Modes", &m_chooser);
    test_motor = std::make_unique<ctre::phoenix6::hardware::TalonFX>(2);
    test_encoder = std::make_unique<ctre::phoenix6::hardware::CANcoder>(23);

    test_motor = std::make_unique<ctre::phoenix6::hardware::TalonFX>(2);
}

/**
 * This function is called every 20 ms, no matter the mode. Use
 * this for items like diagnostics that you want ran during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
void Robot::RobotPeriodic()
{
    nt::NetworkTableInstance::GetDefault().Flush();
}

/**
 * This autonomous (along with the chooser code above) shows how to select
 * between different autonomous modes using the dashboard. The sendable chooser
 * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
 * remove all of the chooser code and uncomment the GetString line to get the
 * auto name from the text box below the Gyro.
 *
 * You can add additional auto modes by adding additional comparisons to the
 * if-else structure below with additional strings. If using the SendableChooser
 * make sure to add them to the chooser code above as well.
 */
void Robot::AutonomousInit()
{
    m_auto_selected = m_chooser.GetSelected();
    // m_auto_selected = SmartDashboard::GetString("Auto Selector",
    //     k_auto_name_default);
    std::cout << "Auto selected: " << m_auto_selected << std::endl;

    if (m_auto_selected == k_auto_name_custom)
    {
        // Custom Auto goes here
    }
    else
    {
        // Default Auto goes here
    }
}

void Robot::AutonomousPeriodic()
{
    if (m_auto_selected == k_auto_name_custom)
    {
        // Custom Auto goes here
    }
    else
    {
        // Default Auto goes here
    }
}

void Robot::TeleopInit()
{
    test_pid.SetSetpoint(90);
}

void Robot::TeleopPeriodic()
{
    test_motor -> SetVoltage(units::voltage::volt_t{2});
}
    

void Robot::DisabledInit()
{
    test_motor -> SetVoltage(units::voltage::volt_t{0});
}

void Robot::DisabledPeriodic()
{
}

void Robot::TestInit()
{
}

void Robot::TestPeriodic()
{
}

void Robot::SimulationInit()
{
}

void Robot::SimulationPeriodic()
{
}

#ifndef RUNNING_FRC_TESTS
int main()
{
    return frc::StartRobot<Robot>();
}
#endif
