#ifndef SOUND_DIRECTION_H
#define SOUND_DIRECTION_H

#include <ros/ros.h>
#include <portaudio.h>
#include <vector>

class SoundDirectionDetector {
public:
    SoundDirectionDetector();
    ~SoundDirectionDetector();
    
    bool initialize();
    float getSoundDirection();
    void run();

private:
    static int audioCallback(const void* inputBuffer,
                            void* outputBuffer,
                            unsigned long framesPerBuffer,
                            const PaStreamCallbackTimeInfo* timeInfo,
                            PaStreamCallbackFlags statusFlags,
                            void* userData);
    
    PaStream* stream_;
    std::vector<std::vector<float>> audioBuffers_;
    unsigned int sampleRate_;
    unsigned int channels_;
};

#endif // SOUND_DIRECTION_H