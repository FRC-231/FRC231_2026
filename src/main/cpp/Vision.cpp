#include "Vision.h"
#include <iostream>

Vision::Vision()
{
    frc::SmartDashboard::SetDefaultNumber("xy_std", 0.0);
    frc::SmartDashboard::SetDefaultNumber("deg_std", 0.0);
    frc::SmartDashboard::SetDefaultNumber("cam_offset", 0.0);
    frc::SmartDashboard::SetDefaultNumber("target_distance", 0.0);
}

std::optional<std::vector<double>> Vision::filterData(std::vector<double> raw)
{
    if(raw[0] == 0.0 && raw[1] == 0.0 && raw[2] == 0.0)
    {
        return std::optional<std::vector<double>>{};
    }
    if(raw[0] == last_reading[0] &&
        raw[1] == last_reading[1] &&
        raw[5] == last_reading[2])
    {
        return std::optional<std::vector<double>>{};
    }
    last_reading = {raw[0], raw[1], raw[5]};

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
    nt::NetworkTableInstance::GetDefault().GetTable(LIMELIGHT_TABLE_NAME)->GetEntry("robot_orientation_set").SetDoubleArray(entries);
    nt::NetworkTableInstance::GetDefault().GetTable(LIMELIGHT_FRONT_TABLE_NAME)->GetEntry("robot_orientation_set").SetDoubleArray(entries);
}

void Vision::applyVisionData(frc::SwerveDrivePoseEstimator<4>& estimator)
{
    //Get alliance color
    auto current_alliance = frc::DriverStation::GetAlliance();
    if(!current_alliance.has_value())
    {
        frc::SmartDashboard::PutNumber("xy_std", -1);
        frc::SmartDashboard::PutNumber("deg_std", -1);
        return;
    }
    try
    {
       // std::cout << nt::NetworkTableInstance::GetDefault().GetTable(LIMELIGHT_TABLE_NAME)->GetString(JSON_ELEMENT_NAME,"")               << std::endl;
        std::string limelight_info = nt::NetworkTableInstance::GetDefault().GetTable(LIMELIGHT_TABLE_NAME)->GetString(JSON_ELEMENT_NAME,"");
        if(limelight_info.empty())
        {
            frc::SmartDashboard::PutNumber("xy_std", -2);
            frc::SmartDashboard::PutNumber("deg_std", -2);
            return;
        }
        nlohmann::json limelight_info_json;
        limelight_info_json = nlohmann::json::parse(limelight_info);
    
        int number_of_targets = limelight_info_json[JSON_APRIL_SECTION_NAME].size();
        if(number_of_targets < 1)
        {
            frc::SmartDashboard::PutNumber("xy_std", -3);
            frc::SmartDashboard::PutNumber("deg_std", -3);
            return;
        }
        double target_area = 0.1;// limelight_info_json[JSON_ROOT_NAME][JSON_APRIL_SECTION_NAME][0][TARGET_AREA_NAME];
        auto robot_estimate_position_raw = estimator.GetEstimatedPosition();
        double latency = (
            nt::NetworkTableInstance::GetDefault().GetTable(LIMELIGHT_TABLE_NAME)->GetNumber(PIPELINE_LATENCY_NAME, 0) +
            nt::NetworkTableInstance::GetDefault().GetTable(LIMELIGHT_TABLE_NAME)->GetNumber(CAPTURE_LATENCY_NAME, 0)
        )/1000;
        frc::Pose2d vision_position;
        if(current_alliance == frc::DriverStation::kBlue)
        {
            auto raw_position = nt::NetworkTableInstance::GetDefault().GetTable(LIMELIGHT_TABLE_NAME)->GetNumberArray(BLUE_BOT_POSE_NAME, std::vector<double>(6));
            vision_position = frc::Pose2d(units::meter_t{raw_position[0]}, units::meter_t{raw_position[1] - Y_OFFSET}, frc::Rotation2d(units::degree_t{raw_position[5]}));
        }
        else
        {
            auto raw_position = nt::NetworkTableInstance::GetDefault().GetTable(LIMELIGHT_TABLE_NAME)->GetNumberArray(RED_BOT_POSE_NAME, std::vector<double>(6));
            vision_position = frc::Pose2d(units::meter_t{raw_position[0]}, units::meter_t{raw_position[1] - Y_OFFSET}, frc::Rotation2d(units::degree_t{raw_position[5]}));
        }
        if(vision_position.X().value() == 0.0)
        {
            frc::SmartDashboard::PutNumber("xy_std", -4);
            frc::SmartDashboard::PutNumber("deg_std", -4);
            return;
        }
        if(vision_position == last_position)
        {
            frc::SmartDashboard::PutNumber("xy_std", -5);
            frc::SmartDashboard::PutNumber("deg_std", -5);
            return;
        }
        last_position = vision_position;
        double distance_between_cord_and_vision = robot_estimate_position_raw.Translation().Distance(vision_position.Translation()).value();
        //Get std div
        double xy_stds;
        double deg_stds;
        if(number_of_targets >= 2)//Multiple targets detected
        {
            xy_stds = 0.7;
            deg_stds = 6;
        }
        else if(target_area > 0.4 && distance_between_cord_and_vision < 1.0)//Close to target
        {
            xy_stds = 1.0;
            deg_stds = 12;
        }
        else if(target_area > 0.05 && distance_between_cord_and_vision < 1.0)//Far from target
        {
            xy_stds = 1.5;
            deg_stds = 30;
        }
        else
        {
            frc::SmartDashboard::PutNumber("xy_std", -6);
            frc::SmartDashboard::PutNumber("deg_std", -6);
            return;
        }
        frc::SmartDashboard::PutNumber("xy_std", xy_stds);
        frc::SmartDashboard::PutNumber("deg_std", deg_stds);
        estimator.SetVisionMeasurementStdDevs({xy_stds, xy_stds, HighVoltage::toRadians(deg_stds)});
        estimator.AddVisionMeasurement(vision_position, frc::Timer::GetFPGATimestamp() - units::second_t{latency});
    }
    catch(const std::exception& e)
    {
        std::cout << "Error getting info from limelight!" << std::endl;
        std::cerr << e.what() << '\n';
        return;
    }

    
}

frc::Pose2d Vision::getCameraPose()
{
    try
    {
        std::string limelight_info = nt::NetworkTableInstance::GetDefault().GetTable(LIMELIGHT_TABLE_NAME)->GetString(JSON_ELEMENT_NAME,"");
        if(limelight_info.empty())
        {
            frc::SmartDashboard::PutNumber("xy_std", -118);
            frc::SmartDashboard::PutNumber("deg_std", -118);
            return last_position;
        }
        nlohmann::json limelight_info_json;
        limelight_info_json = nlohmann::json::parse(limelight_info);
        frc::Pose2d vision_position;
        auto current_alliance = frc::DriverStation::GetAlliance();
        if(current_alliance == frc::DriverStation::kBlue)
        {
            auto raw_position = nt::NetworkTableInstance::GetDefault().GetTable(LIMELIGHT_TABLE_NAME)->GetNumberArray(BLUE_BOT_POSE_NAME, std::vector<double>(6));
            vision_position = frc::Pose2d(units::meter_t{raw_position[0]}, units::meter_t{raw_position[1] - Y_OFFSET}, frc::Rotation2d(units::degree_t{raw_position[5]}));
        }
        else
        {
            auto raw_position = nt::NetworkTableInstance::GetDefault().GetTable(LIMELIGHT_TABLE_NAME)->GetNumberArray(RED_BOT_POSE_NAME, std::vector<double>(6));
            vision_position = frc::Pose2d(units::meter_t{raw_position[0]}, units::meter_t{raw_position[1] - Y_OFFSET}, frc::Rotation2d(units::degree_t{raw_position[5]}));
        }
        if(vision_position.X().value() == 0.0)
        {
            return last_position;
        }
        last_position = vision_position;
    }
    catch(const std::exception& e)
    {
        std::cout << "Error getting info from limelight!" << std::endl;
    }
    return last_position;
}