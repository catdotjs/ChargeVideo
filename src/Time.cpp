#include "ChargeVideo.hpp"
#include <Magnum/Timeline.h>
#include <vector>

using namespace ChargeVideo;

// ================== Video Timing ==================
float Time::DeltaTime = 0.0f;
uint16_t Time::videoIDCounter = 0;

std::unordered_map<uint16_t, std::function<void()>> Time::videoPlayMethods;
std::vector<uint16_t> Time::toUnhook;

Timeline Time::time{};

void Time::AdvanceTime() {
  if (time.currentFrameTime() == 0.0f) {
    time.start();
  }

  // We are giving average delta for frame timing stablisation
  DeltaTime = time.currentFrameDuration();

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
