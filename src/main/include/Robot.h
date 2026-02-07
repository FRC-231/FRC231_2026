// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <string>
#include <map>

#include <frc/TimedRobot.h>
#include <frc/smartdashboard/SendableChooser.h>
#include "SmartPID.h"
#include "NetworkTableValue.h"
#include <ctre/phoenix6/TalonFX.hpp>
#include <memory>
#include <ctre/phoenix6/CANcoder.hpp>
#include "RobotContext.h"
#include "DriveSystem.h"
#include "Paths.h"
#include "NetworkTable/StringSelector.h"
#include "Keeko/ThreadedScript.h"
#include <thread>
#include <mutex>


class Robot : public frc::TimedRobot
{
public:
    Robot();
    ~Robot() override;
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

    struct Auton
    {
        std::string name;
        std::filesystem::path path;
    };

    std::vector<Auton> AUTONS = {
        {"Test", Paths::TEST_AUTON},
        {"Dance", Paths::DANCE_AUTON}
    };

private:

    std::shared_ptr<RobotContext> context;
    std::shared_ptr<DriveSystem> drive_system;
    units::volt_t ff_voltage = 0_V;

    //Auton script stuff
    void setupAutonSelector();
    void checkIfAutonScriptNeedsLoading();
    void addToLuaState(Keeko::ThreadedScript& script);
    std::unique_ptr<std::thread> auton_loader_thread;
    std::mutex mutex;
    std::string current_loaded_auton;
    std::unique_ptr<Keeko::ThreadedScript> auton_script;
    bool auton_script_loading = false;
    std::unique_ptr<HighVoltage::NetworkTable::StringSelector> nt_auto_selected;
    
};
