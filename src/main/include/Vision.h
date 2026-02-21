#pragma once

#include <vector>
#include <optional>
#include <array>
#include <string>
#include <frc/DriverStation.h>
#include <frc/estimator/SwerveDrivePoseEstimator.h>
#include <nlohmann/json.hpp>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>
#include <frc/Timer.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include "Utility.h"

/**
 * Per-camera state for a single Limelight camera.
 * Each camera tracks its own last reading and last position
 * so that duplicate/stale data is filtered independently.
 */
struct LimelightCamera
{
    std::string table_name;
    std::array<double, 3> last_reading{{0.0, 0.0, 0.0}};
    frc::Pose2d last_position;
};

class Vision
{
public:
    /**
     * Construct with any number of Limelight NetworkTable names.
     * Example: Vision({"limelight", "limelight-front", "limelight-back"})
     */
    explicit Vision(std::vector<std::string> camera_names = {"limelight", "limelight-front"});

    /** Process every camera and add each valid measurement to the estimator. */
    void applyVisionData(frc::SwerveDrivePoseEstimator<4>& estimator);

    std::optional<std::vector<double>> filterData(std::vector<double> raw, LimelightCamera& camera);

    static constexpr double Y_OFFSET = 4.105148;
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

    /** Add a new camera at runtime (e.g. when a 3rd Limelight is installed). */
    void addCamera(const std::string& table_name);

    /** Get the number of cameras currently configured. */
    size_t getCameraCount() const { return cameras.size(); }

private:
    /** Try to apply vision data from a single camera. */
    void applyCameraData(LimelightCamera& camera, frc::SwerveDrivePoseEstimator<4>& estimator);

    std::vector<LimelightCamera> cameras;
};