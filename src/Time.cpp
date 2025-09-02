#include "ChargeVideo.hpp"
#include <Magnum/Timeline.h>
#include <vector>

using namespace ChargeVideo;

// ================== Video Timing ==================
float Time::DeltaTime = 0.0f, Time::AverageDeltaTime = 0.0f,
      Time::rollingSum = 0.0f;
uint16_t Time::ADTMaxSample = 90, Time::ADTIndex = 0, Time::videoIDCounter = 0;
bool Time::ADTFirstCycle = true;
std::vector<float> Time::deltaAverage;
std::unordered_map<uint16_t, std::function<void()>> Time::videoPlayMethods;
std::vector<uint16_t> Time::toUnhook;

Timeline Time::time{};

void Time::AdvanceTime() {
  if (deltaAverage.size() != ADTMaxSample) {
    deltaAverage.resize(ADTMaxSample, 0.0f);
  }

  if (time.currentFrameTime() == 0.0f) {
    time.start();
  }

  // We are giving average delta for frame timing stablisation
  DeltaTime = time.currentFrameDuration();
  rollingSum += DeltaTime - deltaAverage[ADTIndex];
  deltaAverage[ADTIndex] = DeltaTime;

  // First cycle would be ruined if we use MaxSample since not all the slots
  // would be filled yet
  if (ADTFirstCycle && ADTIndex == ADTMaxSample - 1) {
    ADTFirstCycle = false;
  }
  AverageDeltaTime = rollingSum / (ADTFirstCycle ? ADTIndex + 1 : ADTMaxSample);
  ADTIndex = (ADTIndex + 1) % ADTMaxSample;

  for (auto processVideo : videoPlayMethods) {
    processVideo.second();
  }

  for (uint16_t id : toUnhook) {
    videoPlayMethods.erase(id);
  }

  time.nextFrame();
}

uint16_t Time::hookVideo(std::function<void()> videoPlay) {
  videoPlayMethods.insert({++videoIDCounter, videoPlay});
  return videoIDCounter;
}

void Time::unhookVideo(uint16_t ID) { toUnhook.push_back(ID); }
