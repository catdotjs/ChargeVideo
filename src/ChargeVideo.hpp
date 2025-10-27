#ifndef CHARGE_VIDEO_BASE_H
#define CHARGE_VIDEO_BASE_H

#include <cstdint>
#include <cstdlib>
#include <functional>
#include <map>
#include <type_traits>
#include <vector>

#include <Corrade/Containers/Array.h>
#include <Magnum/GL/Texture.h>
#include <Magnum/Image.h>
#include <Magnum/ImageView.h>
#include <Magnum/Magnum.h>
#include <Magnum/Timeline.h>

#include <Charge/ChargeAudio.hpp>

namespace ChargeVideo {
namespace _ffmpeg {
extern "C" {
#include <libavcodec/avcodec.h>
#include <libavcodec/codec.h>
#include <libavcodec/packet.h>
#include <libavformat/avformat.h>
#include <libavutil/avutil.h>
#include <libavutil/frame.h>
#include <libavutil/imgutils.h>
#include <libavutil/pixdesc.h>
#include <libavutil/pixfmt.h>
#include <libavutil/samplefmt.h>
#include <libswresample/swresample.h>
#include <libswscale/swscale.h>
}
} // namespace _ffmpeg

using namespace Corrade;
using namespace Magnum;
using namespace Math::Literals;
// ======================== CLASSES ========================
class Manager {
public:
  static void Advance();
  static float DeltaTime;

private:
  static Timeline time;
  static uint16_t videoIDCounter;
  static std::unordered_map<uint16_t, std::function<void()>> videoPlayMethods;
  static std::vector<uint16_t> toUnhook;

  // Specific for internal controls
  static uint16_t hookVideo(std::function<void()> videoPlay);
  static void unhookVideo(uint16_t ID);
  friend class Video;
};

class Video {
public:
  // Enums & Flags
  enum class State { Playing, Finished, Paused, Idle };
  enum class Flags : uint32_t { None = 0, Looping = 1 << 0 };

  Video(std::string path, ChargeAudio::Engine *audioEngine = nullptr,
        Flags videoFlags = Flags::None, float bufferSizeInSeconds = 1.0f);
  ~Video();

  // Manual Control
  void AdvanceToNextFrame();

  // Automatic play
  void Play();
  void Pause();
  void SwitchLooping();
  void Restart();

  // Info
  const double GetDuration();
  const double GetPlaybackTime();
  const Vector2i GetDimensions();
  State GetState();
  Flags GetFlags();

  // Frame and buffer
  GL::Texture2D CurrentFrame;

  // Audio
  ChargeAudio::SoundContainer Sound;

private:
  // Contextes
  _ffmpeg::AVFormatContext *ctx;
  const _ffmpeg::AVCodec *vCodec, *aCodec;
  _ffmpeg::AVCodecContext *vCodecCtx, *aCodecCtx;
  _ffmpeg::AVStream *videoStream, *audioStream;
  struct _ffmpeg::SwsContext *swsCtx = NULL; // Visual
  struct _ffmpeg::SwrContext *swrCtx = NULL; // Audio

  // State
  int8_t videoStreamNum = -1, audioStreamNum = -1;
  State videoState;
  Flags videoFlags;
  uint16_t ID = 0;

  // Frames
  _ffmpeg::AVFrame *frame = _ffmpeg::av_frame_alloc(),
                   *convertedFrame = _ffmpeg::av_frame_alloc(),
                   *audioFrame = _ffmpeg::av_frame_alloc(),
                   *convertedAudioFrame = _ffmpeg::av_frame_alloc();
  _ffmpeg::AVPacket *packet = _ffmpeg::av_packet_alloc();

  // Time specific
  uint32_t currentFrameNumber = 0;
  double timeBase = 0, clock = 0;

  // Audio
  ChargeAudio::Engine *audioEngine;
  _ffmpeg::AVChannelLayout outLayout;
  _ffmpeg::AVSampleFormat sampleFormat = _ffmpeg::AV_SAMPLE_FMT_FLT;

  // Image
  std::map<double, Image2D> frameBuffer;
  Vector2i Dimensions{0, 0};
  uint32_t bufferMaxFrames = 0, scaleFactor = 1;
  float bufferLenghtInSeconds = 0.0f;
  bool frameSet = false;

  // Methods
  void continueVideo();
  std::pair<double, Containers::Array<char>> loadNextFrame();
  inline void frameDebug(_ffmpeg::AVFrame *frame);
  inline void frameSetScaleSAR(_ffmpeg::AVFrame *frame);
  inline void frameConvert();
  inline void frameFlip(_ffmpeg::AVFrame *frame);

  void restartVideo();
  void dumpAndRefillBuffer();

  void loadTexture(Containers::Array<char> data);
  void loadTexture(ImageView2D image);
  Image2D loadImage(Containers::Array<char> data);

  void reinitSound();
};

inline Video::Flags operator|(Video::Flags x, Video::Flags y) {
  return static_cast<Video::Flags>(
      static_cast<std::underlying_type_t<Video::Flags>>(x) |
      static_cast<std::underlying_type_t<Video::Flags>>(y));
}

inline Video::Flags operator&(Video::Flags x, Video::Flags y) {
  return static_cast<Video::Flags>(
      static_cast<std::underlying_type_t<Video::Flags>>(x) &
      static_cast<std::underlying_type_t<Video::Flags>>(y));
}

inline Video::Flags operator^(Video::Flags x, Video::Flags y) {
  return static_cast<Video::Flags>(
      static_cast<std::underlying_type_t<Video::Flags>>(x) ^
      static_cast<std::underlying_type_t<Video::Flags>>(y));
}
} // namespace ChargeVideo
#endif
