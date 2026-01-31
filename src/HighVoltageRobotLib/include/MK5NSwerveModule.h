#pragma once

#include <ctre/phoenix6/CANcoder.hpp>
#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/signals/SpnEnums.hpp>
#include <memory>

#include <frc/controller/SimpleMotorFeedforward.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/controller/PIDController.h>
#include <frc/trajectory/TrapezoidProfile.h>
#include <frc/kinematics/SwerveModulePosition.h>
#include <networktables/NetworkTableInstance.h>
#include <networktables/DoubleTopic.h>
#include <networktables/NetworkTable.h>
#include <networktables/DoubleArrayTopic.h>

#include "SwerveModule.h"
#include "CommonConfig.h"
#include "Utility.h"
#include "NetworkTableValue.h"


namespace HighVoltage
{

class MK5NSwerveModule : public SwerveModule
{
public:

    struct Mk5NConfig
    {
        static inline const std::string DRIVE_ID = "drive_id";
        static inline const std::string ROTATE_ID = "rotate_id";
        static inline const std::string ENCODER_ID = "encoder_id";
        static inline const std::string ENCODER_OFFSET_DEGREES = "encoder_offset";
        static inline const std::string ROTATE_FF = "rotate_feed_forward";
        static inline const std::string ROTATE_VELOCITY_PID = "rotate_velocity_pid";
        static inline const std::string ROTATE_ANGLE_PID = "rotate_angle_pid";
        static inline const std::string DRIVE_TURNS_PER_METER = "drive_turns_per_meter";
        static inline const std::string DRIVE_FF = "drive_feed_forward";


        uint8_t drive_id; //CAN ID for the drive motor
        uint8_t rotate_id; //CAN ID for the rotation motor
        uint8_t encoder_id; //CAN ID for the encoder
        units::angle::degree_t encoder_offset; //The magnets in the encoders are not alligned with the wheels. This offset corrects for that.

        FeedForwardConfig rotate_ff; //The feedforward for velocity of the rotation motor
        PIDConfig rotate_velocity_pid; //Helps the feed forward control velocity of the rotation motor
        PIDConfig rotate_angle_pid; //Calculates the velocity setpoint for the feedforward + velocity pid to reach the desired angle
        double drive_turns_per_meter = 0.0;
        FeedForwardConfig drive_ff;


        friend void from_json(const nlohmann::json& j, Mk5NConfig& config); //Friend function to convert from json
        /* example json:
        {
            "drive_id":5,
            "rotate_id":6,
            "encoder_id":7,
            "encoder_offset_degrees":45.0,
            "rotate_feed_forward":{
                "k_s":0.22,
                "k_v":0.00694647141201765,
                "k_a":0.0
            },
            "rotate_velocity_pid":{
                "k_p":0.1,
                "k_i":0.0,
                "k_d":0.0
            },
            "rotate_angle_pid":{
                "k_p":1.0,
                "k_i":0.0,
                "k_d":0.0
            }
        }
        */
    };

    MK5NSwerveModule() = delete;
    MK5NSwerveModule(std::string name,
        Mk5NConfig config);

    void update(std::chrono::duration<double> seconds_passed) override; //Update the module control loop
    frc::SwerveModulePosition getModulePosition() override; //Get the position of the module
    units::angle::degree_t getRawAngle() override; //Angle without offset correction
    units::angle::degree_t getAngle() override; //Angle with offset correction
    units::length::meter_t getEncoderPosition() override; //Get the distance the drive motor has traveled
    double getEncoderRawPosition();
    double getDriveTurnsPerMeter() {return drive_turns_per_meter;}

    void resetError() override;
    void updateTurnMotor(units::angle::degree_t degrees_off);
    void updateDriveMotor(bool is_forward, units::degree_t degrees_off = 0_deg);
    void updateDriveMotorVoltage(bool is_forward, units::volt_t voltage);
    void updateDriveMotorVoltage(units::volt_t voltage);
    void pointTurnMotor(units::angle::degree_t target_angle);

    std::shared_ptr<ctre::phoenix6::hardware::TalonFX> getDriveMotor() { return drive_motor; }
    std::shared_ptr<ctre::phoenix6::hardware::TalonFX> getRotateMotor() { return rotate_motor; }
    std::shared_ptr<ctre::phoenix6::hardware::CANcoder> getEncoder() { return encoder; }

private:

    void updateNetworkTables();
    

    units::angle::degree_t offset{0};

    Mk5NConfig config;

    std::shared_ptr<ctre::phoenix6::hardware::TalonFX> drive_motor;  // The motor that moves the robot
    std::shared_ptr<ctre::phoenix6::hardware::TalonFX> rotate_motor; // The motor that rotates the wheel module
    std::shared_ptr<ctre::phoenix6::hardware::CANcoder> encoder; //The encoder that measures the angle of the wheel module
    //TODO: Fix this hacky typecasting by making a custom feedforward class
    std::unique_ptr<frc::SimpleMotorFeedforward<units::meters>> rotate_feedforward; //Using meters for angle because the SMFF class requires a length. We will still be using degree but typecasted.
    std::unique_ptr<frc::PIDController> rotate_velocity_pid; //PID to help the feedforward control velocity
    std::unique_ptr<frc::PIDController> rotate_angle_pid; //PID to calculate the velocity setpoint for the feedforward + velocity pid to reach the desired angle
    std::unique_ptr<frc::SimpleMotorFeedforward<units::meters>> drive_feedforward; //Using meters for angle because the SMFF class requires a length. We will still be using degree but typecasted.

    double drive_turns_per_meter = 0.0;

    std::unique_ptr<HighVoltage::NetworkTableValue<double>> nt_target_angle;
    std::unique_ptr<HighVoltage::NetworkTableValue<double>> nt_raw_angle;
    std::unique_ptr<HighVoltage::NetworkTableValue<double>> nt_angle;
    std::unique_ptr<HighVoltage::NetworkTableValue<double>> nt_rotate_velocity;
    std::unique_ptr<HighVoltage::NetworkTableValue<double>> nt_rotate_pid_voltage;
    std::unique_ptr<HighVoltage::NetworkTableValue<double>> nt_rotate_ff_voltage;
    std::unique_ptr<HighVoltage::NetworkTableValue<double>> nt_encoder_position;
    std::unique_ptr<HighVoltage::NetworkTableValue<double>> nt_encoder_raw_position;
};
} // namespace HighVoltage
