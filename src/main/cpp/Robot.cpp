// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <frc/smartdashboard/SmartDashboard.h>
#include <iostream>


Robot::Robot()
{
    context = std::make_shared<RobotContext>();
    bool config_loaded = context->loadConfig(Paths::BASE_ROBOT_CONFIG);
    if (!config_loaded)
    {
        std::cerr << "Failed to load config file" << std::endl;
        throw std::runtime_error("Failed to load config file");
    } 

    drive_system = std::make_shared<DriveSystem>(context);

    m_chooser.SetDefaultOption(k_auto_name_default, k_auto_name_default);
    m_chooser.AddOption(k_auto_name_custom, k_auto_name_custom);
    frc::SmartDashboard::PutData("Auto Modes", &m_chooser);
    test_motor = std::make_unique<ctre::phoenix6::hardware::TalonFX>(2);
    test_encoder = std::make_unique<ctre::phoenix6::hardware::CANcoder>(23);

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
    applied_voltage = 7.0;
}

void Robot::TeleopPeriodic()
{   
    auto velocity_in_turns = test_encoder->GetVelocity().GetValue();
    std::vector<double> result = {applied_voltage, velocity_in_turns.value() * 360};
    nt_voltage_vs_velocity.setValue(result);
    //applied_voltage += 0.01;
    test_motor->SetVoltage(units::voltage::volt_t{applied_voltage});
}
    

void Robot::DisabledInit()
{
    test_motor->SetVoltage(units::voltage::volt_t{0});
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
