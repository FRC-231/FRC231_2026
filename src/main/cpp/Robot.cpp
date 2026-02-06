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

    context->setStartAngle(context->getPigeon().GetYaw().GetValue());

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
    frc2::CommandScheduler::GetInstance().Run();// Runs all subsystem's periodic functions
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
    frc::Pose2d drive_target
    {
        units::meter_t{6.46},
        units::meter_t{-2.10},
        frc::Rotation2d{units::degree_t{0.0}}
    };
    drive_system->driveTo(drive_target);
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
}

void Robot::TeleopPeriodic()
{
    double x = context->getPilotController().GetLeftX();
    double y = context->getPilotController().GetLeftY();
    double rotate = context->getPilotController().GetRightX();
    bool reset_field_orientation = context->getPilotController().GetR3Button();
    
    if(reset_field_orientation)
    {
        context->setStartAngle(HighVoltage::get180Range(context->getPigeon().GetYaw().GetValue()));
        //drive_system->resetTargetAngle();
    }

    drive_system->setDrive(x, y, rotate);
}
    

void Robot::DisabledInit()
{
}

void Robot::DisabledPeriodic()
{
    drive_system->stopDrive();
}

void Robot::TestInit()
{
    ff_voltage = 0_V;
}

void Robot::TestPeriodic()
{
    drive_system->setDriveFF(ff_voltage);
    ff_voltage -= 0.005_V;
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
