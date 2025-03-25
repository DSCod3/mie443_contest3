#ifndef SOUND_DETECTION_H
#define SOUND_DETECTION_H

#include <ros/ros.h>
#include <portaudio.h>
#include <vector>
#include <mutex>
#include <thread>
#include <functional>
#include <std_msgs/Bool.h>

class LoudSoundDetector {
public:
    LoudSoundDetector();
    ~LoudSoundDetector();

    bool initialize();
    void run();

private:
    static int audioCallback(const void* inputBuffer, void* outputBuffer,
                            unsigned long framesPerBuffer,
                            const PaStreamCallbackTimeInfo* timeInfo,
                            PaStreamCallbackFlags statusFlags,
                            void* userData);

    float calculateVolumeDB();
    void processingLoop(ros::Publisher& pub);

    const float DB_THRESHOLD = -40.0;
    const int sampleRate_ = 44100;
    const int channels_ = 1;
    std::vector<std::vector<float>> audioBuffers_;
    std::mutex bufferMutex_;
    PaStream* stream_;
    bool running_;
    bool soundActive_;
};

#endif // SOUND_DETECTION_H