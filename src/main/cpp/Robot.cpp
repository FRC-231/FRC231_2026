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
    double x = context->getPilotController().GetLeftX();
    double y = context->getPilotController().GetLeftY();
    double rotate = context->getPilotController().GetRightX();
    bool reset_field_orientation = context->getPilotController().GetR3Button();
    bool auto_aim = context->getPilotController().GetR1Button();
    bool intake_up = context->getPilotController().GetCircleButtonPressed();
    bool intake_down = context->getPilotController().GetTriangleButtonPressed();
    bool intake_balls = context->getPilotController().GetSquareButton();
    //Left trigger for loading shooter
    bool load_shooter = context->getPilotController().GetL2Axis() > 0.5;
    
    if(intake_up)
    {
        intake_system->pivotUp();
    }
    if(intake_down)
    {
        intake_system->pivotDown();
    }
    if(intake_balls)
    {
        intake_system->feedIntake();
    }
    else
    {
        intake_system->feedStop();
    }

    
    
    if(reset_field_orientation)
    {
        context->setStartAngle(HighVoltage::get180Range(context->getPigeon().GetYaw().GetValue()));
        //drive_system->setOdometryFromVision();
    }

    if(auto_aim)
    {
        drive_system->setManualAA(x, y);
    }
    else
    {
        drive_system->setDrive(x, y, rotate);
    }

    //Temp controls for setting shooter to voltage mode--------------------------

    bool do_shooter_voltage_control_toggle = context->getPilotController().GetCrossButtonPressed();
    static bool do_shooter_auto_control = false;
    if(do_shooter_voltage_control_toggle)
    {
        do_shooter_auto_control = !do_shooter_auto_control;
        std::cout << "Toggled shooter auto control to " << (do_shooter_auto_control ? "ON" : "OFF") << std::endl;
    }
    if(do_shooter_auto_control)
    {
        shooter_system->setShooterMode(ShooterSystem::ShooterMode::AUTO);
    }
    else
    {
        shooter_system->setShooterMode(ShooterSystem::ShooterMode::STOP);
    }

    if(load_shooter)
    {
        shooter_system->setLoaderMode(ShooterSystem::LoaderMode::LOAD);
    }
    else
    {
        shooter_system->setLoaderMode(ShooterSystem::LoaderMode::STOP);
    }


    //---------------------------------------------------------------------------

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
}

void Robot::TestInit()
{
    ff_voltage = 7_V;
}

void Robot::TestPeriodic()
{
    shooter_system->setShooterVoltages(ff_voltage, ff_voltage);
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
