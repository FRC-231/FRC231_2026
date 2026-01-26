#include "MK5NSwerveModule.h"

namespace HighVoltage
{

MK5NSwerveModule::MK5NSwerveModule(std::string name,
    MK5NSwerveModule::Mk5NConfig config) : SwerveModule{name}, config{config}
{
    //Constructor body can be empty for now
}

void from_json(const nlohmann::json& j, MK5NSwerveModule::Mk5NConfig& config)
{
    j.at("drive_id").get_to(config.drive_id);
    j.at("rotate_id").get_to(config.rotate_id);
    j.at("encoder_id").get_to(config.encoder_id);
    double offset_degrees = j.at("encoder_offset_degrees").get<double>(); //Grab the value as a double
    config.encoder_offset = units::angle::degree_t{offset_degrees}; //Covert that double to degree_t and assign
}

} // namespace HighVoltage



