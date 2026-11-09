// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <frc/smartdashboard/SmartDashboard.h>
#include <iostream>
#include <frc/RobotController.h>
#include <units/math.h>


Robot::Robot()
{   
    frc::RobotController::SetBrownoutVoltage(4.6_V);
    context = std::make_shared<RobotContext>();
    bool config_loaded = context->loadConfig(Paths::BASE_ROBOT_CONFIG);
    if (!config_loaded)
    {
        std::cerr << "Failed to load config file" << std::endl;
        throw std::runtime_error("Failed to load config file");
    } 

    drive_system = std::make_shared<DriveSystem>(context);
    intake_system = std::make_shared<IntakeSystem>(context);
    shooter_system = std::make_shared<ShooterSystem>(context);
    shooter_system->setDistanceUpdateCallback([this]()
    {
        return drive_system->getDistanceFromTargetPosition(RobotContext::HUB_POSITION);
    });

    context->setStartAngle(context->getPigeon().GetYaw().GetValue());

    setupAutonSelector();

    //If we are in simulation, set up our network tables for the simulation dashboard
    if(frc::RobotBase::IsSimulation())
    {
        nt_simulation_data = std::make_unique<NTSimulationData::NTSimulationData>();
        std::cout << "Running in simulation, set up NT simulation data" << std::endl;
    }

}

Robot::~Robot()
{
    if(auton_loader_thread && auton_loader_thread->joinable())
    {
        auton_loader_thread->join();
    }
    if(auton_script)
    {
        auton_script->stop();
        auton_script->join();
    }
}

void Robot::setupAutonSelector()
{
    std::vector<std::string> auton_names;
    for(const auto& auton : AUTONS)
    {
        auton_names.push_back(auton.name);
    }
    nt_auto_selected = std::make_unique<HighVoltage::NetworkTable::StringSelector>("Auto_Selector", auton_names, AUTONS[0].name);
    nt_auto_selected->setOnUpdateCallback([this](const std::string& selected_auton)
    {
        std::cout << "Selected auton: " << selected_auton << std::endl;
        // You can add additional logic here to handle the selected auton, such as loading the corresponding file or setting a variable.
    });
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

    // Push the current NT throttle interval to all subsystems.
    // getNTThrottleInterval() returns 0 when not in competition mode (= full rate),
    // or the configured interval when in competition mode.
    int throttle = context->getNTThrottleInterval();
    drive_system->setNTThrottleInterval(throttle);
    shooter_system->setNTThrottleInterval(throttle);
    intake_system->setNTThrottleInterval(throttle);

    // Throttle NT flush by the same interval in competition mode.
    // In testing mode (interval <= 1), flush every loop for real-time telemetry.
    if (throttle > 1)
    {
        static int flush_counter = 0;
        if (++flush_counter >= throttle)
        {
            flush_counter = 0;
            nt::NetworkTableInstance::GetDefault().Flush();
        }
    }
    else
    {
        nt::NetworkTableInstance::GetDefault().Flush();
    }
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
    std::lock_guard<std::mutex> lock(mutex);
    shooter_system->setManualShooterAdjustmentValue(0.0);
    context->getVision().setMode(VisionMode::Auton);
    if(auton_script && !auton_script_loading)
    {
        std::cout << "Starting auton script: " << current_loaded_auton << std::endl;
        auton_script->runFunction("autonomous");
    }
}

void Robot::AutonomousPeriodic()
{
}

void Robot::TeleopInit()
{
    context->getVision().setMode(VisionMode::Teleop);
    if(auton_script && !auton_script_loading)
    {
        auton_script->stop();
    }
}

void Robot::TeleopPeriodic()
{
    drive_system->setMaxSpeed(6.7_mps);
    //Control collection
    auto& pilot = context->getPilotController();
    auto& pilotXbox = context->getPilotXboxController();
    auto& copilot = context->getCopilotController();

    double x = pilot.GetLeftX();
    double y = pilot.GetLeftY();
    double rotate = pilot.GetRightX();
    bool reset_field_orientation = pilot.GetR3Button();
    static bool auto_aim = false;
    bool auto_aim_toggle = pilot.GetL1ButtonPressed();
    if(auto_aim_toggle)
    {
        auto_aim = !auto_aim;
    }
    bool half_speed = pilot.GetTriangleButton();

    bool load_shooter = pilot.GetL2Axis() > 0.5;
    // bool do_shooter_voltage_control_toggle = pilot.GetCrossButtonPressed();

    static bool intake_pivot_up = true;
    bool toggle_intake_pivot = pilot.GetR1ButtonPressed();
    bool copilot_intake_up = copilot.GetPOV() == 0;
    bool copilot_intake_down = copilot.GetPOV() == 180;
    
    if(toggle_intake_pivot)
    {
        intake_pivot_up = !intake_pivot_up;
    }
    else if(copilot_intake_up)
    {
        intake_pivot_up = true;
    }
    else if(copilot_intake_down)
    {
        intake_pivot_up = false;
    }
    //bool pivot_wiggle = copilot.GetL1Button();
    bool run_intake = pilot.GetR2Axis() > 0.5;
    bool reverse_intake = pilot.GetSquareButton() || copilot.GetSquareButton();
    static bool use_snow_blower = false;
    // if(pilot.GetCircleButtonPressed() || copilot.GetCrossButtonPressed())
    // {
    //     use_snow_blower = !use_snow_blower;
    // }
    nt_use_snow_blower.setValue(use_snow_blower);
    bool is_shooter_at_speed = shooter_system->isAtSpeed();

    // double shooter_adjustment = copilot.GetL2Axis() - copilot.GetR2Axis();
    double climb_control = copilot.GetRightY();
    
    
    //Drive
    if(reset_field_orientation)
    {
        context->setStartAngle(HighVoltage::get180Range(context->getPigeon().GetYaw().GetValue()));
        //drive_system->setOdometryFromVision();
    }

    double sweet_spot_rumble = 0.0;
    auto distance_from_hub = drive_system->getDistanceFromTargetPosition(RobotContext::HUB_POSITION);
    auto distance_from_sweet_spot = units::math::abs(distance_from_hub - SWEET_STOP_DISTANCE);
    if(drive_system->getRobotPose().X() < RobotContext::HUB_POSITION.X())
    {
        if(distance_from_sweet_spot < SWEET_SPOT_ON_THRESHOLD)
        {
            sweet_spot_rumble = 1.0;
        }
        else if(distance_from_sweet_spot < SWEET_SPOT_NEAR_THRESHOLD)
        {
            sweet_spot_rumble = (1.0 - distance_from_sweet_spot.value() / SWEET_SPOT_NEAR_THRESHOLD.value()) * SWEET_SPOT_MAX_NEAR_RUMBLE;
        }
    }
    pilotXbox.SetRumble(SWEET_SPOT_RUMBLE_TYPE, sweet_spot_rumble);

    if(half_speed)
    {
        x *= 0.5;
        y *= 0.5;
        rotate *= 0.5;
    }
    if(auto_aim)
    {
        if(use_snow_blower)
        {
            drive_system->setManualFaceBack(-x, -y);
        }
        else
        {
            drive_system->setManualAA(-x, -y);
        }
    }
    else
    {
        drive_system->setDrive(-x, -y, rotate);
    }
    
    //Shooting
    static bool do_shooter_auto_control = true;
    // if(do_shooter_voltage_control_toggle)
    // {
    //     do_shooter_auto_control = !do_shooter_auto_control;
    //     std::cout << "Toggled shooter auto control to " << (do_shooter_auto_control ? "ON" : "OFF") << std::endl;
    // }
    nt_shooter_on.setValue(do_shooter_auto_control);
    if(do_shooter_auto_control)
    {
        if(is_shooter_at_speed)
        {
            pilotXbox.SetRumble(Robot::SHOOTER_AT_SPEED_RUMBLE_TYPE, 1.0);
        }
        else
        {
            pilotXbox.SetRumble(Robot::SHOOTER_AT_SPEED_RUMBLE_TYPE, 0.0);
        }
        
        if(use_snow_blower)
        {
            shooter_system->setShooterMode(ShooterSystem::ShooterMode::SNOW_BLOW);
        }
        else
        {
            shooter_system->setShooterMode(ShooterSystem::ShooterMode::AUTO);//Make sure its AUTO before we queue up 
        }
    }
    else
    {
        pilotXbox.SetRumble(Robot::SHOOTER_AT_SPEED_RUMBLE_TYPE, 0.0);
        shooter_system->setShooterMode(ShooterSystem::ShooterMode::IDLE);
    }

    if(load_shooter)
    {
        shooter_system->setLoaderMode(ShooterSystem::LoaderMode::LOAD);
        intake_system->feedIntake();
    }
    else
    {
        shooter_system->setLoaderMode(ShooterSystem::LoaderMode::STOP);
    }
    // shooter_system->setManualShooterAdjustmentValue(shooter_adjustment);

    //Intake
    if(!load_shooter)
    {
        if(run_intake)
        {
            if(do_shooter_auto_control && !use_snow_blower)
            {
                intake_system->feedHalfSpeed();
            }
            else
            {
                intake_system->feedIntake();
            }
        }
        else if (reverse_intake)
        {
            intake_system->feedOuttake();
            shooter_system->loaderUnload();
        }
        else
        {
            intake_system->feedStop();
        }
    }
    // if(pivot_wiggle)
    // {
    //     intake_system->pivotWiggle();
    // }
    // else if(intake_pivot_up)
    // {
    //     intake_system->pivotUp();
    // }
    // else
    // {
    //     intake_system->pivotDown();
    // }
    // intake_system->pivotDown();
}
    

void Robot::DisabledInit()
{
    context->getVision().setMode(VisionMode::Teleop);
}

void Robot::DisabledPeriodic()
{
    drive_system->stopDrive();
    if(auton_script)
    {
        auton_script->stop();
    }
    checkIfAutonScriptNeedsLoading();
    context->getPilotXboxController().SetRumble(frc::GenericHID::RumbleType::kBothRumble, 0.0);
}

void Robot::checkIfAutonScriptNeedsLoading()
{
    if(current_loaded_auton != nt_auto_selected->getSelected())
    {
        std::lock_guard<std::mutex> lock(mutex);
        std::string selected_auton_name = nt_auto_selected->getSelected();
        auto selected_auton = std::find_if(AUTONS.begin(), AUTONS.end(), [&selected_auton_name](const Auton& auton)
        {
            return auton.name == selected_auton_name;
        });
        if(selected_auton == AUTONS.end())
        {
            std::cerr << "Selected auton not found: " << selected_auton_name << std::endl;
            auton_script_loading = false;
            return;
        }
        if(auton_script_loading)
        {
            return;
        }
        if(auton_loader_thread && auton_loader_thread->joinable())
        {
            auton_loader_thread->join();
        }
        std::cout << "Loading auton script: " << selected_auton->name << std::endl;
        auton_loader_thread = std::make_unique<std::thread>([this, selected_auton]()
        {
            std::lock_guard<std::mutex> lock(mutex);
            auton_script_loading = true;
            auton_script = std::make_unique<Keeko::ThreadedScript>();
            auton_script->setOnFailCallback([this]()
            {
                std::cout << "Script failed! Stopping subsystems." << std::endl;
                drive_system->stopDrive();
            });
            auton_script->loadFile(selected_auton->path.string());
            addToLuaState(*auton_script);
            current_loaded_auton = selected_auton->name;
            auton_script_loading = false;
        });
    }
}

void Robot::addToLuaState(Keeko::ThreadedScript& script)
{
    drive_system->addToLuaState(script);
    intake_system->addToLuaState(script);
    shooter_system->addToLuaState(script);
}

void Robot::TestInit()
{
    ff_voltage = 7_V;
}

void Robot::TestPeriodic()
{
    shooter_system->setShooterVoltage(ff_voltage);
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
