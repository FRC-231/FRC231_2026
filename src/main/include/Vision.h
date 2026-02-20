#pragma once

#include <vector>
#include <optional>
#include <array>
#include <frc/DriverStation.h>
#include <frc/estimator/SwerveDrivePoseEstimator.h>
#include <nlohmann/json.hpp>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>
#include <frc/Timer.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include "Utility.h"



class Vision
{
public:
    Vision();
    void applyVisionData(frc::SwerveDrivePoseEstimator<4>& estimator);
    std::optional<std::vector<double>> filterData(std::vector<double> raw);

    static constexpr double Y_OFFSET = 4.105148;
    inline static const std::string LIMELIGHT_TABLE_NAME{"limelight"};
    inline static const std::string LIMELIGHT_FRONT_TABLE_NAME{"limelight-front"};
    inline static const std::string RED_BOT_POSE_NAME{"botpose_orb_wpired"};
    inline static const std::string BLUE_BOT_POSE_NAME{"botpose_orb_wpiblue"};
    inline static const std::string PIPELINE_LATENCY_NAME{"tl"};
    inline static const std::string CAPTURE_LATENCY_NAME{"cl"};
    inline static const std::string JSON_ELEMENT_NAME{"json"};
    inline static const std::string JSON_ROOT_NAME{"Results"};
    inline static const std::string JSON_APRIL_SECTION_NAME{"Fiducial"};
    inline static const std::string TARGET_AREA_NAME{"ta"};
    inline static const std::string JSON_APRIL_TAG_ID_NAME{"fID"};
    inline static const std::string JSON_APRIL_TAG_TX_NAME{"tx"};
    inline static const std::string RING_OFFSET_NAME{"tx"};
    inline static const std::string RING_IN_VIEW_NAME{"tv"};
    inline static const std::string TARGET_SPACE_ROBOT_POSE_NAME{"t6r_ts"};

    frc::Pose2d getCameraPose();
    void setRobotOrientation(
        double yaw,
        double yaw_rate,
        double pitch,
        double pitch_rate,
        double roll,
        double roll_rate
    );


private:
    std::array<double, 3> last_reading{{0.0, 0.0, 0.0}};
    frc::Pose2d last_position;
    
};