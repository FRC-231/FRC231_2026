#pragma once
#include <memory>

#include <frc2/command/SubsystemBase.h>
#include <frc/controller/PIDController.h>
#include <ctre/phoenix6/CANcoder.hpp>
#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/signals/SpnEnums.hpp>
#include "ctre/phoenix6/StatusSignal.hpp"
#include "Keeko/ThreadedScript.h"

#include "RobotConfig.h"
#include "RobotContext.h"

#include "NetworkTableValue.h"

class ClimbSystem: public frc2::SubsystemBase
{
public:
    // ---- Constants ----
    static inline const std::string CLIMB_TABLE_NAME = "ClimbSystem";

    // ---- Constructor ----
    ClimbSystem(std::shared_ptr<RobotContext> context);
    ClimbSystem() = delete;

    // ---- Subsystem Override ----
    void Periodic() override;



private:
    // ---- Internal Helpers ----
    void updateNetworkTable();

    // ---- Core State ----
    std::shared_ptr<RobotContext> context;



};