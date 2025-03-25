#include "sound_direction.h"
#include <std_msgs/Float32.h>
#include <iostream>
#include <cmath>

SoundDirectionDetector::SoundDirectionDetector() :
    stream_(nullptr),
    sampleRate_(44100),
    channels_(4)  // Assuming 4-microphone array
{
    audioBuffers_.resize(channels_);
}

SoundDirectionDetector::~SoundDirectionDetector() {
    if(stream_) {
        Pa_CloseStream(stream_);
        Pa_Terminate();
    }
}

bool SoundDirectionDetector::initialize() {
    PaError err = Pa_Initialize();
    if(err != paNoError) {
        ROS_ERROR("PortAudio error: %s", Pa_GetErrorText(err));
        return false;
    }

    PaStreamParameters inputParams;
    inputParams.device = Pa_GetDefaultInputDevice();
    if(inputParams.device == paNoDevice) {
        ROS_ERROR("No audio input device found!");
        return false;
    }
    
    inputParams.channelCount = channels_;
    inputParams.sampleFormat = paFloat32;
    inputParams.suggestedLatency = Pa_GetDeviceInfo(inputParams.device)->defaultLowInputLatency;
    inputParams.hostApiSpecificStreamInfo = nullptr;

    err = Pa_OpenStream(&stream_,
                       &inputParams,
                       nullptr,
                       sampleRate_,
                       1024,  // framesPerBuffer
                       paClipOff,
                       &SoundDirectionDetector::audioCallback,
                       this);

    if(err != paNoError) {
        ROS_ERROR("Failed to open audio stream: %s", Pa_GetErrorText(err));
        return false;
    }

    return true;
}

int SoundDirectionDetector::audioCallback(const void* inputBuffer,
                                        void* /*outputBuffer*/,
                                        unsigned long framesPerBuffer,
                                        const PaStreamCallbackTimeInfo* /*timeInfo*/,
                                        PaStreamCallbackFlags /*statusFlags*/,
                                        void* userData) {
    SoundDirectionDetector* detector = static_cast<SoundDirectionDetector*>(userData);
    const float* input = static_cast<const float*>(inputBuffer);

    // Store audio data in circular buffers
    for(unsigned i=0; i<framesPerBuffer; i++) {
        for(unsigned ch=0; ch<detector->channels_; ch++) {
            if(detector->audioBuffers_[ch].size() >= detector->sampleRate_) {
                detector->audioBuffers_[ch].erase(detector->audioBuffers_[ch].begin());
            }
            detector->audioBuffers_[ch].push_back(input[i*detector->channels_ + ch]);
        }
    }
    
    return paContinue;
}

float SoundDirectionDetector::getSoundDirection() {
    // Simple cross-correlation between first two channels
    if(audioBuffers_[0].size() < 1024 || audioBuffers_[1].size() < 1024)
        return 0.0f;

    float maxCorr = -1.0f;
    int bestLag = 0;
    const int maxLag = 100;  // Corresponds to ~0.7ms at 44.1kHz

    for(int lag = -maxLag; lag <= maxLag; lag++) {
        float corr = 0.0f;
        for(int i = maxLag; i < 1024 - maxLag; i++) {
            corr += audioBuffers_[0][i] * audioBuffers_[1][i + lag];
        }
        if(corr > maxCorr) {
            maxCorr = corr;
            bestLag = lag;
        }
    }

    // Convert lag to angle (simplified)
    const float micSpacing = 0.1f;  // 10cm between mics
    const float soundSpeed = 343.0f; // m/s
    float timeDiff = static_cast<float>(bestLag)/sampleRate_;
    float angle = asin(timeDiff * soundSpeed / micSpacing);
    
    return angle;
}

void SoundDirectionDetector::run() {
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<std_msgs::Float32>("/sound_direction", 10);
    std_msgs::Float32 angleMsg;

    Pa_StartStream(stream_);
    ROS_INFO("Sound direction detection started");

    while(ros::ok()) {
        angleMsg.data = getSoundDirection();
        pub.publish(angleMsg);
        ros::spinOnce();
        usleep(10000);  // 10ms update rate
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "sound_direction_node");
    SoundDirectionDetector detector;
    
    if(!detector.initialize()) {
        ROS_ERROR("Failed to initialize sound direction detector");
        return 1;
    }

    detector.run();
    return 0;
}