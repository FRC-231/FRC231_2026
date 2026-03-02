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
/**
 * Vision operates in two modes:
 *  - Auton:  strict distance rejection to prevent bad jumps during autonomous.
 *  - Teleop: much more trusting of the camera because defence from other robots
 *            can cause large odometry drift that vision needs to correct.
 *
 * Disabled uses Teleop mode so the pose stays accurate before a match starts.
 */
enum class VisionMode
{
    Auton,
    Teleop
};

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
     * Camera names are defined in RobotContext::CAMERA_NAMES.
     */
    Vision(std::vector<std::string> camera_names);

    /** Process every camera and add each valid measurement to the estimator. */
    void applyVisionData(frc::SwerveDrivePoseEstimator<4>& estimator);

    std::optional<std::vector<double>> filterData(std::vector<double> raw, LimelightCamera& camera);
    static constexpr double FIELD_HEIGHT = 8.069326;
    static constexpr double Y_OFFSET = FIELD_HEIGHT / 2.0;

    // ── Tuning constants for standard deviations ──
    // The pose estimator is a Kalman filter. Standard deviations (std devs)
    // control how much we trust each vision measurement:
    //   - LOWER std dev = we trust vision MORE  (vision pulls the estimate harder)
    //   - HIGHER std dev = we trust vision LESS  (odometry dominates)
    //
    // Formula:  std_dev = BASE + (PER_METER * average_tag_distance)
    // So a tag 1m away gives a low std dev (high trust), and a tag 5m away
    // gives a higher std dev (low trust), scaling linearly.
    //
    // These values are tuned for MegaTag 2, which uses the gyro-fed robot
    // orientation to constrain the solve. This makes both multi-tag and
    // single-tag measurements significantly more accurate than standard solves.

    // ═══════════════════════════════════════════════════════════
    //  AUTON constants — strict filtering to prevent bad jumps
    // ═══════════════════════════════════════════════════════════

    // Multi-tag (MegaTag 2): gyro-constrained + triangulation → high confidence
    // Old working values were 0.7 / 6 — these are slightly better for MegaTag 2
    static constexpr double MULTI_TAG_XY_BASE      = 0.5;   // meters — base trust for xy
    static constexpr double MULTI_TAG_XY_PER_METER  = 0.1;   // additional uncertainty per meter of distance
    static constexpr double MULTI_TAG_DEG_BASE      = 6.0;   // degrees — base trust for rotation
    static constexpr double MULTI_TAG_DEG_PER_METER = 1.5;   // additional uncertainty per meter of distance

    // Single-tag: keep std devs high so a bad reading can't cause a sudden jump.
    static constexpr double SINGLE_TAG_XY_BASE      = 1.5;
    static constexpr double SINGLE_TAG_XY_PER_METER  = 0.8;
    static constexpr double SINGLE_TAG_DEG_BASE      = 30.0;
    static constexpr double SINGLE_TAG_DEG_PER_METER = 8.0;

    // Reject a single-tag measurement if the tag is farther than this (meters)
    static constexpr double MAX_SINGLE_TAG_DISTANCE = 3.5;

    // Reject a SINGLE-tag measurement if it disagrees with odometry by more
    // than this distance (meters). This was the key filter in the old working
    // code — single-tag solves that are far from the current estimate are
    // almost always wrong. The old code used 1.0m.
    static constexpr double MAX_SINGLE_TAG_VISION_DISTANCE = 1.0;

    // Reject any vision measurement (including multi-tag) if it disagrees
    // with odometry by more than this distance (meters).
    static constexpr double MAX_ACCEPTED_VISION_DISTANCE = 4.0;

    // ═══════════════════════════════════════════════════════════
    //  TELEOP constants — trust vision much more because defence
    //  from other robots can cause large odometry drift
    // ═══════════════════════════════════════════════════════════

    // Multi-tag in teleop: very high trust
    static constexpr double TELEOP_MULTI_TAG_XY_BASE      = 0.3;
    static constexpr double TELEOP_MULTI_TAG_XY_PER_METER  = 0.05;
    static constexpr double TELEOP_MULTI_TAG_DEG_BASE      = 4.0;
    static constexpr double TELEOP_MULTI_TAG_DEG_PER_METER = 1.0;

    // Single-tag in teleop: still trust quite a bit
    static constexpr double TELEOP_SINGLE_TAG_XY_BASE      = 0.8;
    static constexpr double TELEOP_SINGLE_TAG_XY_PER_METER  = 0.4;
    static constexpr double TELEOP_SINGLE_TAG_DEG_BASE      = 15.0;
    static constexpr double TELEOP_SINGLE_TAG_DEG_PER_METER = 4.0;

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

    /** Set the vision mode (Auton = strict filtering, Teleop = trust camera more). */
    void setMode(VisionMode new_mode) { mode = new_mode; }

    /** Get the current vision mode. */
    VisionMode getMode() const { return mode; }

private:
    /** Try to apply vision data from a single camera. */
    void applyCameraData(LimelightCamera& camera, frc::SwerveDrivePoseEstimator<4>& estimator);

    std::vector<LimelightCamera> cameras;
    VisionMode mode = VisionMode::Teleop;  // default to Teleop so disabled-mode trusts camera
};