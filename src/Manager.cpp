#include "ChargeVideo.hpp"
#include <Magnum/Timeline.h>
#include <vector>

using namespace ChargeVideo;

// ================== Video Managing ==================
float Manager::DeltaTime = 0.0f;
uint16_t Manager::videoIDCounter = 0;

std::unordered_map<uint16_t, std::function<void()>> Manager::videoPlayMethods;
std::vector<uint16_t> Manager::toUnhook;

Timeline Manager::time{};

void Manager::AdvanceTime() {
  if (time.currentFrameTime() == 0.0f) {
    time.start();
  }
  DeltaTime = time.currentFrameDuration();

  for (auto processVideo : videoPlayMethods) {
    processVideo.second();
  }

  for (uint16_t id : toUnhook) {
    videoPlayMethods.erase(id);
  }

  time.nextFrame();
}

uint16_t Manager::hookVideo(std::function<void()> videoPlay) {
  videoPlayMethods.insert({++videoIDCounter, videoPlay});
  return videoIDCounter;
}

void Manager::unhookVideo(uint16_t ID) { toUnhook.push_back(ID); }
