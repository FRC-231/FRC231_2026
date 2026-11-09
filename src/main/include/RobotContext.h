#pragma once
#include <filesystem>
#include <optional>
#include "RobotConfig.h"
#include "NetworkTableValue.h"
#include <frc/PS5Controller.h>
#include <frc/XboxController.h>
#include <ctre/phoenix6/Pigeon2.hpp>
#include <Vision.h>

class RobotContext
{
public:
    // ── Camera configuration ──
    // Add or remove Limelight names here when cameras are added to the robot.
    // inline static const std::vector<std::string> CAMERA_NAMES{
    //     "limelight",
    //     "limelight-front"
    // };

    inline static const std::vector<std::string> CAMERA_NAMES{
        "limelight",
        "limelight-left",
        "limelight-right"
    };

    RobotContext();
    bool loadConfig(const std::filesystem::path& path);
    const std::optional<RobotConfig::RobotConfig>& getConfig() const;
    frc::PS5Controller& getPilotController();
    frc::PS5Controller& getCopilotController();
    frc::XboxController& getPilotXboxController();
    ctre::phoenix6::hardware::Pigeon2& getPigeon();
    void setStartAngle(units::angle::degree_t angle);
    units::angle::degree_t getStartAngle();
    Vision& getVision();

    /// Returns true when NT telemetry should be throttled (competition mode).
    /// Toggleable live via the "Robot/competition_mode" NT entry.
    bool isCompetitionMode() const { return competition_mode; }

    /// Returns the current NT throttle interval.
    /// 0 or 1 = no throttle, N = publish every Nth loop.
    /// In competition mode, returns the configured interval. Otherwise returns 0.
    int getNTThrottleInterval() const { return competition_mode ? nt_throttle_interval : 0; }

    inline static const frc::Pose2d HUB_POSITION{units::inch_t{182.11}, units::inch_t{0.0}, frc::Rotation2d{units::degree_t{0.0}}};


private:
    std::optional<RobotConfig::RobotConfig> config;
    frc::PS5Controller pilot_controller{0};
    frc::PS5Controller copilot_controller{1};
    frc::XboxController pilot_as_xbox_controller{3};
    ctre::phoenix6::hardware::Pigeon2 pigeon{30, "Drivebase"};
    //ctre::phoenix6::hardware::Pigeon2 pigeon{30, "rio"};
    units::angle::degree_t start_angle{0.0};
    Vision vision{CAMERA_NAMES};

    bool competition_mode{false};
    int nt_throttle_interval{5};
    HighVoltage::NetworkTableValue<bool> nt_competition_mode{"Robot/competition_mode", false};
    HighVoltage::NetworkTableValue<double> nt_throttle_interval_value{"Robot/nt_throttle_interval", 5};
};
