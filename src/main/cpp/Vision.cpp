#include "Vision.h"
#include <iostream>
#include <cmath>

Vision::Vision(std::vector<std::string> camera_names)
{
    for (const auto& name : camera_names)
    {
        cameras.push_back(LimelightCamera{name, {0.0, 0.0, 0.0}, frc::Pose2d{}});
    }
    frc::SmartDashboard::SetDefaultNumber("xy_std", 0.0);
    frc::SmartDashboard::SetDefaultNumber("deg_std", 0.0);
    frc::SmartDashboard::SetDefaultNumber("cam_offset", 0.0);
    frc::SmartDashboard::SetDefaultNumber("target_distance", 0.0);
}

void Vision::addCamera(const std::string& table_name)
{
    cameras.push_back(LimelightCamera{table_name, {0.0, 0.0, 0.0}, frc::Pose2d{}});
}

std::optional<std::vector<double>> Vision::filterData(std::vector<double> raw, LimelightCamera& camera)
{
    if(raw[0] == 0.0 && raw[1] == 0.0 && raw[2] == 0.0)
    {
        return std::optional<std::vector<double>>{};
    }
    if(raw[0] == camera.last_reading[0] &&
        raw[1] == camera.last_reading[1] &&
        raw[5] == camera.last_reading[2])
    {
        return std::optional<std::vector<double>>{};
    }
    camera.last_reading = {raw[0], raw[1], raw[5]};

    return std::optional<std::vector<double>>{{ raw[0] , raw[1] - Y_OFFSET, raw[5]}};
}

void Vision::setRobotOrientation(
    double yaw,
    double yaw_rate,
    double pitch,
    double pitch_rate,
    double roll,
    double roll_rate
)
{
    std::vector<double> entries = {yaw, yaw_rate, pitch, pitch_rate, roll, roll_rate};
    // Broadcast orientation to every camera
    for (const auto& camera : cameras)
    {
        nt::NetworkTableInstance::GetDefault().GetTable(camera.table_name)->GetEntry("robot_orientation_set").SetDoubleArray(entries);
    }
}

void Vision::applyVisionData(frc::SwerveDrivePoseEstimator<4>& estimator)
{
    // Process each camera independently — every valid measurement
    // is fed into the pose estimator's Kalman filter, which naturally
    // fuses them weighted by the standard deviations we provide.
    for (auto& camera : cameras)
    {
        applyCameraData(camera, estimator);
    }
}

void Vision::applyCameraData(LimelightCamera& camera, frc::SwerveDrivePoseEstimator<4>& estimator)
{
    auto current_alliance = frc::DriverStation::GetAlliance();
    if (!current_alliance.has_value())
    {
        return;
    }
    try
    {
        auto table = nt::NetworkTableInstance::GetDefault().GetTable(camera.table_name);

        // ── 1. Read the bot-pose array from the correct alliance key ──
        // The Limelight botpose arrays contain more than 6 elements:
        //   [0-2] = x, y, z   [3-5] = roll, pitch, yaw
        //   [6]   = total latency (ms)
        //   [7]   = tag count
        //   [8]   = tag span (m)
        //   [9]   = average tag distance from camera (m)
        //   [10]  = average tag area (% of image)
        // Using these avoids expensive JSON parsing every loop.
        std::vector<double> raw_position;
        if (current_alliance == frc::DriverStation::kBlue)
        {
            raw_position = table->GetNumberArray(BLUE_BOT_POSE_NAME, std::vector<double>{});
        }
        else
        {
            raw_position = table->GetNumberArray(RED_BOT_POSE_NAME, std::vector<double>{});
        }

        // Need at least 10 elements to get tag count and avg distance
        if (raw_position.size() < 10)
        {
            return;
        }

        int number_of_targets = static_cast<int>(raw_position[7]);
        double avg_tag_distance = raw_position[9];

        if (number_of_targets < 1)
        {
            return;
        }

        // ── 2. Compute latency-corrected timestamp ──
        // Use the total latency from the botpose array (element [6]) which
        // already combines pipeline + capture latency.
        double latency = raw_position[6] / 1000.0;

        // ── 3. Build the vision pose ──
        frc::Pose2d vision_position(
            units::meter_t{raw_position[0]},
            units::meter_t{raw_position[1] - Y_OFFSET},
            frc::Rotation2d(units::degree_t{raw_position[5]})
        );

        // ── 4. Reject obviously bad data ──
        // Zero pose means the Limelight couldn't solve
        if (vision_position.X().value() == 0.0)
        {
            return;
        }
        // Identical to the last reading means we already processed this frame
        if (vision_position == camera.last_position)
        {
            return;
        }
        camera.last_position = vision_position;

        // If vision says we're very far from where odometry thinks we are,
        // something is probably wrong — reject this measurement.
        //
        // In AUTON mode we apply strict distance checks:
        //   - Single-tag gets a STRICT distance check (1.0m) to prevent
        //     bad single-tag solves from jumping the position forward.
        //   - Multi-tag gets a generous limit since those solves are reliable.
        //
        // In TELEOP mode we skip these distance checks entirely because
        // defence from other robots can cause large odometry drift, and we
        // need vision to pull us back to the correct position.
        auto robot_estimate = estimator.GetEstimatedPosition();
        double odometry_vs_vision_distance =
            robot_estimate.Translation().Distance(vision_position.Translation()).value();

        if (mode == VisionMode::Auton)
        {
            double max_distance = (number_of_targets >= 2)
                ? MAX_ACCEPTED_VISION_DISTANCE
                : MAX_SINGLE_TAG_VISION_DISTANCE;
            if (odometry_vs_vision_distance > max_distance)
            {
                return;
            }
        }

        // ── 5. Calculate standard deviations (trust level) ──
        // The key insight: the Kalman filter uses std devs to weight
        // measurements. LOWER std devs = we trust this measurement MORE.
        //
        // Two factors determine trust:
        //   a) number_of_targets — more tags = better triangulation
        //   b) avg_tag_distance  — closer tags = more accurate pixel solve
        //
        // We use a simple linear scale: trust decreases as distance grows.
        // With multiple tags the Limelight can triangulate, so we start
        // with a much lower (better) base.

        double xy_stds;
        double deg_stds;
        if (number_of_targets >= 2)
        {
            if (mode == VisionMode::Auton)
            {
                // Multi-tag auton: reliable but conservative.
                xy_stds = MULTI_TAG_XY_BASE + (MULTI_TAG_XY_PER_METER * avg_tag_distance);
                deg_stds = MULTI_TAG_DEG_BASE + (MULTI_TAG_DEG_PER_METER * avg_tag_distance);
            }
            else
            {
                // Multi-tag teleop: very high trust to counteract defence-induced drift.
                xy_stds = TELEOP_MULTI_TAG_XY_BASE + (TELEOP_MULTI_TAG_XY_PER_METER * avg_tag_distance);
                deg_stds = TELEOP_MULTI_TAG_DEG_BASE + (TELEOP_MULTI_TAG_DEG_PER_METER * avg_tag_distance);
            }
        }
        else
        {
            if (mode == VisionMode::Auton)
            {
                // Single tag auton: less reliable, scale aggressively.
                xy_stds = SINGLE_TAG_XY_BASE + (SINGLE_TAG_XY_PER_METER * avg_tag_distance);
                deg_stds = SINGLE_TAG_DEG_BASE + (SINGLE_TAG_DEG_PER_METER * avg_tag_distance);
            }
            else
            {
                // Single tag teleop: still trust more than auton.
                xy_stds = TELEOP_SINGLE_TAG_XY_BASE + (TELEOP_SINGLE_TAG_XY_PER_METER * avg_tag_distance);
                deg_stds = TELEOP_SINGLE_TAG_DEG_BASE + (TELEOP_SINGLE_TAG_DEG_PER_METER * avg_tag_distance);
            }
        }

        // In auton, reject single-tag measurements that are too far away
        if (mode == VisionMode::Auton && number_of_targets < 2 && avg_tag_distance > MAX_SINGLE_TAG_DISTANCE)
        {
            return;
        }

        // ── 6. Publish telemetry for debugging (throttled) ──
        // Only update SmartDashboard every ~10 valid measurements to reduce overhead.
        camera.debug_counter++;
        if (camera.debug_counter >= 10)
        {
            camera.debug_counter = 0;
            frc::SmartDashboard::PutNumber("Vision/" + camera.table_name + "/xy_std", xy_stds);
            frc::SmartDashboard::PutNumber("Vision/" + camera.table_name + "/deg_std", deg_stds);
            frc::SmartDashboard::PutNumber("Vision/" + camera.table_name + "/num_targets", number_of_targets);
            frc::SmartDashboard::PutNumber("Vision/" + camera.table_name + "/avg_tag_dist", avg_tag_distance);
            frc::SmartDashboard::PutString("Vision/mode", (mode == VisionMode::Auton) ? "Auton" : "Teleop");
        }

        // ── 7. Feed the measurement into the Kalman filter ──
        estimator.SetVisionMeasurementStdDevs({xy_stds, xy_stds, HighVoltage::toRadians(deg_stds)});
        estimator.AddVisionMeasurement(vision_position, frc::Timer::GetFPGATimestamp() - units::second_t{latency});
    }
    catch (const std::exception& e)
    {
        std::cout << "Error getting info from limelight (" << camera.table_name << ")!" << std::endl;
        std::cerr << e.what() << '\n';
        return;
    }
}

frc::Pose2d Vision::getCameraPose()
{
    // Use the first camera for getCameraPose (backwards compatibility)
    if (cameras.empty())
    {
        return frc::Pose2d{};
    }
    auto& camera = cameras[0];
    try
    {
        auto table = nt::NetworkTableInstance::GetDefault().GetTable(camera.table_name);
        std::string limelight_info = table->GetString(JSON_ELEMENT_NAME, "");
        if (limelight_info.empty())
        {
            return camera.last_position;
        }
        nlohmann::json limelight_info_json = nlohmann::json::parse(limelight_info);
        frc::Pose2d vision_position;
        auto current_alliance = frc::DriverStation::GetAlliance();
        if (current_alliance == frc::DriverStation::kBlue)
        {
            auto raw_position = table->GetNumberArray(BLUE_BOT_POSE_NAME, std::vector<double>(6));
            vision_position = frc::Pose2d(
                units::meter_t{raw_position[0]},
                units::meter_t{raw_position[1] - Y_OFFSET},
                frc::Rotation2d(units::degree_t{raw_position[5]})
            );
        }
        else
        {
            auto raw_position = table->GetNumberArray(RED_BOT_POSE_NAME, std::vector<double>(6));
            vision_position = frc::Pose2d(
                units::meter_t{raw_position[0]},
                units::meter_t{raw_position[1] - Y_OFFSET},
                frc::Rotation2d(units::degree_t{raw_position[5]})
            );
        }
        if (vision_position.X().value() == 0.0)
        {
            return camera.last_position;
        }
        camera.last_position = vision_position;
    }
    catch (const std::exception& e)
    {
        std::cout << "Error getting info from limelight (" << camera.table_name << ")!" << std::endl;
    }
    return camera.last_position;
}