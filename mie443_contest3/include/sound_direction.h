#ifndef SOUND_DIRECTION_H
#define SOUND_DIRECTION_H

#include <ros/ros.h>
#include <portaudio.h>
#include <vector>
#include <string>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>

class SoundDirectionDetector {
public:
    SoundDirectionDetector();
    ~SoundDirectionDetector();
    
    bool initialize();
    void run();

private:
    static int audioCallback(const void* inputBuffer,
                            void* outputBuffer,
                            unsigned long framesPerBuffer,
                            const PaStreamCallbackTimeInfo* timeInfo,
                            PaStreamCallbackFlags statusFlags,
                            void* userData);
    
    float getSoundDirection();
    float calculateVolumeDB(int channel);
    
    PaStream* stream_;
    std::vector<std::vector<float>> audioBuffers_;
    unsigned int sampleRate_;
    unsigned int channels_;
    
    // Configuration
    static constexpr float DB_THRESHOLD = -30.0f;  // -30dB threshold
    static constexpr float MIC_SPACING = 0.1f;      // 10cm between mics
    static constexpr float SOUND_SPEED = 343.0f;    // m/s
};

#endif