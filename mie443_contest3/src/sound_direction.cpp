#include "sound_direction.h"
#include <cmath>
#include <numeric>
#include <chrono>

// Add these declarations if not already present
#ifdef __linux__
extern "C" {
    int PaAlsa_EnableRealtimeScheduling(PaStream* s, int enable);
    int PaAlsa_SetNumPeriods(PaStream* s, int numPeriods);
}
#endif

// Precomputed Hann window
const std::array<float, 1024> SoundDirectionDetector::HANN_WINDOW = [](){
    std::array<float, 1024> window;
    for(int i=0; i<1024; i++) {
        window[i] = 0.5f * (1 - cos(2*M_PI*i/(1024-1)));
    }
    return window;
}();

SoundDirectionDetector::SoundDirectionDetector() :
    sampleRate_(44100),
    channels_(4)
{
    audioBuffers_.resize(channels_);
    angleHistory_.fill(0.0f);
}

SoundDirectionDetector::~SoundDirectionDetector() {
    running_ = false;
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
    inputParams.suggestedLatency = 0.02;  // 20ms latency
    inputParams.hostApiSpecificStreamInfo = nullptr;

    err = Pa_OpenStream(&stream_,
                       &inputParams,
                       nullptr,
                       sampleRate_,
                       512,  // Smaller buffer size
                       paClipOff,
                       &SoundDirectionDetector::audioCallback,
                       this);

    if(err != paNoError) {
        ROS_ERROR("Failed to open audio stream: %s", Pa_GetErrorText(err));
        return false;
    }

    #ifdef __linux__
    // ALSA-specific optimizations (Linux only)
    if(PaAlsa_EnableRealtimeScheduling) { // Check if function exists
        PaAlsa_EnableRealtimeScheduling(stream_, 1);
    }
    if(PaAlsa_SetNumPeriods) { // Check if function exists
        PaAlsa_SetNumPeriods(stream_, 3);
    }
    #endif

    calibrateChannels();
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

std::lock_guard<std::mutex> lock(detector->bufferMutex_);

for(unsigned ch=0; ch<detector->channels_; ch++) {
auto& buffer = detector->audioBuffers_[ch];
const float* channelData = input + ch;

// Maintain buffer size
if(buffer.size() + framesPerBuffer > detector->sampleRate_) {
buffer.erase(buffer.begin(), buffer.begin() + (buffer.size() + framesPerBuffer - detector->sampleRate_));
}

// Bulk insert
buffer.insert(buffer.end(), channelData, channelData + framesPerBuffer * detector->channels_);
}

return paContinue;
}

void SoundDirectionDetector::calibrateChannels() {
    ros::WallDuration(0.5).sleep();
    
    std::lock_guard<std::mutex> lock(bufferMutex_);
    float sum0 = std::accumulate(audioBuffers_[0].begin(), audioBuffers_[0].end(), 0.0f);
    float sum1 = std::accumulate(audioBuffers_[1].begin(), audioBuffers_[1].end(), 0.0f);
    channelBalance_ = (sum1 != 0) ? sum0/sum1 : 1.0f;
    ROS_INFO("Channel balance calibrated: %.2f", channelBalance_);
}

float SoundDirectionDetector::calculateVolumeDB(int channel) {
    std::lock_guard<std::mutex> lock(bufferMutex_);
    
    if(audioBuffers_[channel].empty()) return -INFINITY;
    
    float sum = 0.0f;
    for(float sample : audioBuffers_[channel]) {
        sum += sample * sample;
    }
    float rms = sqrt(sum / audioBuffers_[channel].size());
    return 20.0f * log10(rms + 1e-9f);
}

float SoundDirectionDetector::getSoundDirection() {
    std::lock_guard<std::mutex> lock(bufferMutex_);
    
    if(audioBuffers_[0].size() < 1024 || audioBuffers_[1].size() < 1024)
        return 0.0f;

    float maxCorr = -1.0f;
    int bestLag = 0;
    // Changed from .data() to direct vector access
    const std::vector<float>& buf0 = audioBuffers_[0];
    const std::vector<float>& buf1 = audioBuffers_[1];
    
    // Add bounds check
    if(buf0.size() < 1024 || buf1.size() < 1024) return 0.0f;

    // Optimized correlation with step size and reduced lag range
    for(int lag = -MAX_LAG; lag <= MAX_LAG; lag += 1) {
        float corr = 0.0f;
        const int start = std::max(MAX_LAG, -lag);
        const int end = 1024 - MAX_LAG - std::abs(lag);
        
        for(int i = start; i < end; i += PROCESS_STEP) {
            const int j = i + lag;
            const float windowed0 = buf0[i] * HANN_WINDOW[i] * channelBalance_;
            const float windowed1 = buf1[j] * HANN_WINDOW[j];
            corr += windowed0 * windowed1;
        }
        
        if(corr > maxCorr) {
            maxCorr = corr;
            bestLag = lag;
        }
    }

    float timeDiff = static_cast<float>(bestLag)/sampleRate_;
    float sin_theta = (timeDiff * SOUND_SPEED) / MIC_SPACING;
    sin_theta = std::max(-1.0f, std::min(sin_theta, 1.0f));
    float angle = asin(sin_theta);

    // Update smoothed angle history
    angleHistory_[historyIndex_++ % HISTORY_SIZE] = angle;
    return std::accumulate(angleHistory_.begin(), angleHistory_.end(), 0.0f) / HISTORY_SIZE;
}

void SoundDirectionDetector::processingLoop() {
    ros::Rate rate(100);  // 100Hz processing
    
    while(running_) {
        float volume_db = calculateVolumeDB(0);
        
        if(soundActive_) {
            if((ros::Time::now() - eventStart_).toSec() > EVENT_WINDOW) {
                soundActive_ = false;
                
                const int left = leftCount_.load();
                const int right = rightCount_.load();
                
                if(abs(left - right) >= MIN_DOMINANCE) {
                    std_msgs::String msg;
                    msg.data = (left > right) ? "LEFT" : "RIGHT";
                    ROS_INFO("Direction: %s (L:%d R:%d)", msg.data.c_str(), left, right);
                }
                
                leftCount_ = 0;
                rightCount_ = 0;
            }
            else {
                float angle = getSoundDirection();
                if(angle < -0.087f) leftCount_++;
                else if(angle > 0.087f) rightCount_++;
            }
        }
        else if(volume_db > DB_THRESHOLD) {
            soundActive_ = true;
            eventStart_ = ros::Time::now();
        }
        
        rate.sleep();
    }
}

void SoundDirectionDetector::run() {
    ros::NodeHandle nh;
    ros::Publisher dir_pub = nh.advertise<std_msgs::String>("/sound_direction", 10);
    
    running_ = true;
    std::thread processor(&SoundDirectionDetector::processingLoop, this);
    
    Pa_StartStream(stream_);
    ROS_INFO("Sound direction detection ready (Threshold: %.1f dB)", DB_THRESHOLD);

    while(ros::ok() && running_) {
        ros::spinOnce();
        ros::Rate(50).sleep();  // Main loop at 50Hz
    }
    
    running_ = false;
    processor.join();
    Pa_StopStream(stream_);
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