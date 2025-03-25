#include "sound_detection.h"
#include <thread>
#include <cmath>

// Constructor
LoudSoundDetector::LoudSoundDetector() : 
    sampleRate_(44100),
    channels_(1),
    stream_(nullptr),
    running_(false),
    soundActive_(false)
{
    audioBuffers_.resize(channels_);
}

// Destructor
LoudSoundDetector::~LoudSoundDetector() {
    running_ = false;
    if (stream_) {
        Pa_CloseStream(stream_);
        Pa_Terminate();
    }
}

// Initialization method
bool LoudSoundDetector::initialize() {
    PaError err = Pa_Initialize();
    if (err != paNoError) {
        ROS_ERROR("PortAudio error: %s", Pa_GetErrorText(err));
        return false;
    }

    PaStreamParameters inputParams;
    inputParams.device = Pa_GetDefaultInputDevice();
    if (inputParams.device == paNoDevice) {
        ROS_ERROR("No audio input device found!");
        return false;
    }

    inputParams.channelCount = channels_;
    inputParams.sampleFormat = paFloat32;
    inputParams.suggestedLatency = 0.02;
    inputParams.hostApiSpecificStreamInfo = nullptr;

    err = Pa_OpenStream(&stream_,
                       &inputParams,
                       nullptr,
                       sampleRate_,
                       512,
                       paClipOff,
                       &LoudSoundDetector::audioCallback,
                       this);

    if (err != paNoError) {
        ROS_ERROR("Failed to open audio stream: %s", Pa_GetErrorText(err));
        return false;
    }

    return true;
}

// Audio callback implementation
int LoudSoundDetector::audioCallback(const void* inputBuffer, void* /*outputBuffer*/,
                         unsigned long framesPerBuffer,
                         const PaStreamCallbackTimeInfo* /*timeInfo*/,
                         PaStreamCallbackFlags /*statusFlags*/,
                         void* userData) {
    LoudSoundDetector* detector = static_cast<LoudSoundDetector*>(userData);
    const float* input = static_cast<const float*>(inputBuffer);
    std::lock_guard<std::mutex> lock(detector->bufferMutex_);

    auto& buffer = detector->audioBuffers_[0];
    buffer.insert(buffer.end(), input, input + framesPerBuffer);

    if (buffer.size() > detector->sampleRate_) {
        buffer.erase(buffer.begin(), buffer.begin() + (buffer.size() - detector->sampleRate_));
    }

    return paContinue;
}

// Volume calculation
float LoudSoundDetector::calculateVolumeDB() {
    std::lock_guard<std::mutex> lock(bufferMutex_);
    auto& buffer = audioBuffers_[0];
    if (buffer.empty()) return -INFINITY;

    float sum = 0.0f;
    for (float sample : buffer) {
        sum += sample * sample;
    }
    float rms = sqrt(sum / buffer.size());
    return 20.0f * log10(rms + 1e-9f);
}

// Processing loop
void LoudSoundDetector::processingLoop(ros::Publisher& pub) {
    ros::Rate rate(100); // 100Hz
    while (running_) {
        float volume_db = calculateVolumeDB();
        if (volume_db > DB_THRESHOLD) {
            if (!soundActive_) {
                std_msgs::Bool msg;
                msg.data = true;
                pub.publish(msg);
                ROS_INFO("Loud sound detected: %.1f dB", volume_db);
                soundActive_ = true;
            }
        } else {
            if (soundActive_) {
                soundActive_ = false;
            }
        }
        rate.sleep();
    }
}

// Main run method
void LoudSoundDetector::run() {
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<std_msgs::Bool>("/loud_sound", 10);
    running_ = true;
    std::thread processor(&LoudSoundDetector::processingLoop, this, std::ref(pub));

    Pa_StartStream(stream_);
    ROS_INFO("Loud sound detection ready. Threshold: %.1f dB", DB_THRESHOLD);

    while (ros::ok() && running_) {
        ros::spinOnce();
        ros::Rate(50).sleep();
    }

    running_ = false;
    processor.join();
    Pa_StopStream(stream_);
}

// Main function
int main(int argc, char** argv) {
    ros::init(argc, argv, "loud_sound_detector");
    LoudSoundDetector detector;
    if (!detector.initialize()) {
        ROS_ERROR("Failed to initialize loud sound detector");
        return 1;
    }
    detector.run();
    return 0;
}