#ifndef SOUND_DIRECTION_H
#define SOUND_DIRECTION_H

#include <ros/ros.h>
#include <portaudio.h>
#include <vector>
#include <string>
#include <array>
#include <deque>
#include <atomic>
#include <thread>
#include <mutex>       // Added
#include <memory>      // Added
#include <std_msgs/String.h>




class SoundDirectionDetector {
public:
    SoundDirectionDetector();
    ~SoundDirectionDetector();
    
    bool initialize();
    void run();

private:
    // Configuration
    static constexpr float DB_THRESHOLD = -25.0f;
    static constexpr float EVENT_WINDOW = 0.4f;
    static constexpr int MIN_DOMINANCE = 3;
    static constexpr float MIC_SPACING = 0.1f;
    static constexpr float SOUND_SPEED = 343.0f;
    static constexpr int HISTORY_SIZE = 5;
    static constexpr int PROCESS_STEP = 2;
    static constexpr int MAX_LAG = 50;

    // Precomputed Hann window
    static const std::array<float, 1024> HANN_WINDOW;

    // Audio processing
    static int audioCallback(const void* inputBuffer, void* outputBuffer,
                            unsigned long framesPerBuffer,
                            const PaStreamCallbackTimeInfo* timeInfo,
                            PaStreamCallbackFlags statusFlags,
                            void* userData);
    float calculateVolumeDB(int channel);
    float getSoundDirection();
    void processingLoop();
    void calibrateChannels();
    
    // Thread-safe buffers
    std::vector<std::vector<float>> audioBuffers_;
    std::mutex bufferMutex_;
    std::array<float, HISTORY_SIZE> angleHistory_;
    int historyIndex_ = 0;
    float channelBalance_ = 1.0f;
    
    // State management
    std::atomic<bool> running_{false};
    std::atomic<int> leftCount_{0};
    std::atomic<int> rightCount_{0};
    std::atomic<bool> soundActive_{false};
    ros::Time eventStart_;

    // Audio stream
    PaStream* stream_ = nullptr;
    unsigned int sampleRate_;
    unsigned int channels_;
};

#endif