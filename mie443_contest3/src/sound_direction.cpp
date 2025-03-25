#include "sound_direction.h"
#include <cmath>
#include <numeric>

SoundDirectionDetector::SoundDirectionDetector() :
    stream_(nullptr),
    sampleRate_(44100),
    channels_(4)
{
    audioBuffers_.resize(channels_);
    angleHistory.fill(0.0f);
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
                       1024,
                       paClipOff,
                       &SoundDirectionDetector::audioCallback,
                       this);

    if(err != paNoError) {
        ROS_ERROR("Failed to open audio stream: %s", Pa_GetErrorText(err));
        return false;
    }

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

void SoundDirectionDetector::calibrateChannels() {
    // Collect some silent samples first
    ros::Duration(0.5).sleep();
    
    float sum0 = std::accumulate(audioBuffers_[0].begin(), audioBuffers_[0].end(), 0.0f);
    float sum1 = std::accumulate(audioBuffers_[1].begin(), audioBuffers_[1].end(), 0.0f);
    channelBalance = (sum1 != 0) ? sum0/sum1 : 1.0f;
    ROS_INFO("Channel balance calibrated: %.2f", channelBalance);
}

float SoundDirectionDetector::calculateVolumeDB(int channel) {
    if(audioBuffers_[channel].empty()) return -INFINITY;
    
    float sum = 0.0f;
    for(float sample : audioBuffers_[channel]) {
        sum += sample * sample;
    }
    float rms = sqrt(sum / audioBuffers_[channel].size());
    return 20.0f * log10(rms + 1e-9f);
}

float SoundDirectionDetector::getSoundDirection() {
    if(audioBuffers_[0].size() < 1024 || audioBuffers_[1].size() < 1024)
        return 0.0f;

    float maxCorr = -1.0f;
    int bestLag = 0;
    const int maxLag = 100;

    for(int lag = -maxLag; lag <= maxLag; lag++) {
        float corr = 0.0f;
        for(int i = maxLag; i < 1024 - maxLag; i++) {
            // Apply Hann window and calibration
            float window = 0.5f * (1 - cos(2*M_PI*i/(1024-1)));
            float sample0 = audioBuffers_[0][i] * window * channelBalance;
            float sample1 = audioBuffers_[1][i + lag] * window;
            corr += sample0 * sample1;
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

    // Apply smoothing
    angleHistory[historyIndex++ % HISTORY_SIZE] = angle;
    return std::accumulate(angleHistory.begin(), angleHistory.end(), 0.0f) / HISTORY_SIZE;
}

void SoundDirectionDetector::run() {
    ros::NodeHandle nh;
    ros::Publisher dir_pub = nh.advertise<std_msgs::String>("/sound_direction", 10);
    std_msgs::String dir_msg;

    Pa_StartStream(stream_);
    ROS_INFO("Sound direction detection ready (Threshold: %.1f dB)", DB_THRESHOLD);

    while(ros::ok()) {
        ros::spinOnce();
        
        float volume_db = calculateVolumeDB(0);
        
        if(soundActive) {
            if((ros::Time::now() - eventStart).toSec() > EVENT_WINDOW) {
                soundActive = false;
                
                if(abs(leftCount - rightCount) >= MIN_DOMINANCE) {
                    dir_msg.data = (leftCount > rightCount) ? "LEFT" : "RIGHT";
                    dir_pub.publish(dir_msg);
                    ROS_INFO("Direction: %s (L:%d R:%d)", 
                            dir_msg.data.c_str(), leftCount, rightCount);
                }
                else {
                    ROS_DEBUG("Ambiguous sound ignored (L:%d R:%d)", leftCount, rightCount);
                }
                
                leftCount = 0;
                rightCount = 0;
            }
            else {
                float angle = getSoundDirection();
                if(angle < -0.087f) leftCount++;  // -5 degrees threshold
                else if(angle > 0.087f) rightCount++;
            }
        }
        else if(volume_db > DB_THRESHOLD) {
            soundActive = true;
            eventStart = ros::Time::now();
            ROS_DEBUG("New sound event started");
        }
        
        usleep(10000);
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