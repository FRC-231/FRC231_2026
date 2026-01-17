#include "AudioPlayer.h"


namespace HighVoltage
{

void to_json(nlohmann::json &j, const PlayAudioMessage &msg)
{
    j[PlayAudioMessage::EVENT_TYPE_KEY] = PlayAudioMessage::eventTypeToString(msg.event_type);
    if(msg.sound_name)
    {
        j[PlayAudioMessage::SOUND_NAME_KEY] = *msg.sound_name;
    }
    j[PlayAudioMessage::TIME_KEY] = frc::Timer::GetFPGATimestamp().value();
}

AudioPlayer::AudioPlayer()
{
    nt_instance = nt::NetworkTableInstance::GetDefault();
    audio_table = nt_instance.GetTable(AUDIO_TABLE_NAME);
    audio_message_topic = audio_table->GetStringTopic(AUDIO_TOPIC_NAME);
    audio_message_publisher = audio_message_topic.Publish();
    audio_message_publisher.SetDefault(nlohmann::json(DEFAULT_MESSAGE).dump());
}

void AudioPlayer::playSound(const std::string& sound_name)
{
    PlayAudioMessage msg;
    msg.event_type = PlayAudioMessage::EventType::PLAY_SOUND;
    msg.sound_name = sound_name;
    audio_message_publisher.Set(nlohmann::json(msg).dump());
}

void AudioPlayer::stopAllSounds()
{
    PlayAudioMessage msg;
    msg.event_type = PlayAudioMessage::EventType::STOP_ALL_SOUNDS;
    audio_message_publisher.Set(nlohmann::json(msg).dump());
}

void AudioPlayer::loopSound(const std::string& sound_name)
{
    PlayAudioMessage msg;
    msg.event_type = PlayAudioMessage::EventType::LOOP_SOUND;
    msg.sound_name = sound_name;
    audio_message_publisher.Set(nlohmann::json(msg).dump());
}

void AudioPlayer::stopSound(const std::string& sound_name)
{
    PlayAudioMessage msg;
    msg.event_type = PlayAudioMessage::EventType::STOP_SOUND;
    msg.sound_name = sound_name;
    audio_message_publisher.Set(nlohmann::json(msg).dump());
}


} // namespace HighVoltage