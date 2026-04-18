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
#include "IntakeSystem.h"
#include "ShooterSystem.h"
#include "Paths.h"
#include "NetworkTable/StringSelector.h"
#include "Keeko/ThreadedScript.h"
#include <thread>
#include <mutex>
#include "NTSimulationData.h"


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
        {"Depot", Paths::QUICK_LOAD_AUTON},
        {"Lever", Paths::QUICK_LOAD_RIGHT_AUTON},
        {"CenterDepot", Paths::CENTER_DEPOT_AUTON},
        {"DoubleTakeLeft", Paths::DOUBLE_TAKE_LEFT_AUTON},
        {"DoubleTakeRight", Paths::DOUBLE_TAKE_RIGHT_AUTON},
        {"CenterBack", Paths::CENTER_BACK_AUTON},
        {"CurveTest", Paths::CURVE_TEST_AUTON},
        {"CurvedDoubleTakeLeft", Paths::CURVED_DOUBLE_TAKE_LEFT_AUTON},
        {"CurvedDoubleTakeRight", Paths::CURVED_DOUBLE_TAKE_RIGHT_AUTON},
        {"CurvedBumpLeft", Paths::CURVED_BUMP_LEFT_AUTON},
        {"CurvedBumpRight", Paths::CURVED_BUMP_RIGHT_AUTON}
    };

    static constexpr units::meter_t SWEET_STOP_DISTANCE{2.25_m}; //Distance is from Hub
    static constexpr units::meter_t SWEET_SPOT_ON_THRESHOLD{0.1_m};
    static constexpr units::meter_t SWEET_SPOT_NEAR_THRESHOLD{1.0_m};
    static constexpr double SWEET_SPOT_MAX_NEAR_RUMBLE = 0.6;
    static constexpr frc::GenericHID::RumbleType SHOOTER_AT_SPEED_RUMBLE_TYPE = frc::GenericHID::RumbleType::kLeftRumble;
    static constexpr frc::GenericHID::RumbleType SWEET_SPOT_RUMBLE_TYPE = frc::GenericHID::RumbleType::kRightRumble;


private:

    std::shared_ptr<RobotContext> context;
    std::shared_ptr<DriveSystem> drive_system;
    std::shared_ptr<IntakeSystem> intake_system;
    std::shared_ptr<ShooterSystem> shooter_system;
    units::volt_t ff_voltage = 0_V;
    units::meter_t test_distance = 0_m;

    std::unique_ptr<NTSimulationData::NTSimulationData> nt_simulation_data;

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
    HighVoltage::NetworkTableValue<bool> nt_use_snow_blower{"Robot/use_snow_blower", false};
    HighVoltage::NetworkTableValue<std::vector<double>> nt_test_output{"TestOutput", {0.0, 0.0, 0.0}};
    HighVoltage::NetworkTableValue<bool> nt_shooter_on{"Robot/shooter_on", false};
    
};
