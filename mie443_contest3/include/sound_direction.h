#ifndef SOUND_DIRECTION_H
#define SOUND_DIRECTION_H

#include <ros/ros.h>
#include <portaudio.h>
#include <vector>
#include <string>
#include <array>
#include <algorithm>
#include <std_msgs/String.h>

class SoundDirectionDetector {
public:
    SoundDirectionDetector();
    ~SoundDirectionDetector();
    
    bool initialize();
    void run();

private:
    // Configuration
    static constexpr float DB_THRESHOLD = -25.0f;   // Adjust based on environment
    static constexpr float EVENT_WINDOW = 0.4f;      // 400ms analysis window
    static constexpr int MIN_DOMINANCE = 3;          // Required direction difference
    static constexpr float MIC_SPACING = 0.1f;       // 10cm between mics
    static constexpr float SOUND_SPEED = 343.0f;     // m/s
    static constexpr int HISTORY_SIZE = 5;           // Smoothing samples
    
    // Audio processing
    static int audioCallback(const void* inputBuffer, void* outputBuffer,
                            unsigned long framesPerBuffer,
                            const PaStreamCallbackTimeInfo* timeInfo,
                            PaStreamCallbackFlags statusFlags,
                            void* userData);
    float calculateVolumeDB(int channel);
    float getSoundDirection();
    void calibrateChannels();
    
    // State management
    PaStream* stream_;
    std::vector<std::vector<float>> audioBuffers_;
    std::array<float, HISTORY_SIZE> angleHistory;
    int historyIndex = 0;
    float channelBalance = 1.0f;
    
    // Event tracking
    bool soundActive = false;
    ros::Time eventStart;
    int leftCount = 0;
    int rightCount = 0;
    
    // Hardware params
    unsigned int sampleRate_;
    unsigned int channels_;
};

#endif