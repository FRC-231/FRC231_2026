#include "Vision.h"
#include <iostream>

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
    // Get alliance color
    auto current_alliance = frc::DriverStation::GetAlliance();
    if (!current_alliance.has_value())
    {
        return;
    }
    try
    {
        auto table = nt::NetworkTableInstance::GetDefault().GetTable(camera.table_name);

        std::string limelight_info = table->GetString(JSON_ELEMENT_NAME, "");
        if (limelight_info.empty())
        {
            return;
        }

        nlohmann::json limelight_info_json = nlohmann::json::parse(limelight_info);

        int number_of_targets = limelight_info_json[JSON_APRIL_SECTION_NAME].size();
        if (number_of_targets < 1)
        {
            return;
        }

        double target_area = 0.1;
        auto robot_estimate_position_raw = estimator.GetEstimatedPosition();

        double latency = (
            table->GetNumber(PIPELINE_LATENCY_NAME, 0) +
            table->GetNumber(CAPTURE_LATENCY_NAME, 0)
        ) / 1000;

        frc::Pose2d vision_position;
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
            return;
        }
        if (vision_position == camera.last_position)
        {
            return;
        }
        camera.last_position = vision_position;

        double distance_between_cord_and_vision =
            robot_estimate_position_raw.Translation().Distance(vision_position.Translation()).value();

        // Determine standard deviations based on quality of measurement
        double xy_stds;
        double deg_stds;
        if (number_of_targets >= 2) // Multiple targets detected
        {
            xy_stds = 0.7;
            deg_stds = 6;
        }
        else if (target_area > 0.4 && distance_between_cord_and_vision < 1.0) // Close to target
        {
            xy_stds = 1.0;
            deg_stds = 12;
        }
        else if (target_area > 0.05 && distance_between_cord_and_vision < 1.0) // Far from target
        {
            xy_stds = 1.5;
            deg_stds = 30;
        }
        else
        {
            return;
        }

        frc::SmartDashboard::PutNumber("Vision/" + camera.table_name + "/xy_std", xy_stds);
        frc::SmartDashboard::PutNumber("Vision/" + camera.table_name + "/deg_std", deg_stds);
        frc::SmartDashboard::PutNumber("Vision/" + camera.table_name + "/num_targets", number_of_targets);

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