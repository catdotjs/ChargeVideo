#ifndef CHARGE_VIDEO_BASE_H
#define CHARGE_VIDEO_BASE_H

#include <cstdint>
#include <cstdlib>
#include <functional>
#include <map>
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
  Video(std::string path, ChargeAudio::Engine *audioEngine = nullptr,
        bool ShouldVideoLoop = true, float BufferSizeInSeconds = 1.0f);
  ~Video();

  // Manual Control
  void AdvanceToNextFrame();

  // Automatic play
  void Play();
  void Pause();
  void StopLooping();
  void StartLooping();
  void Restart();

  // Frame and buffer
  GL::Texture2D CurrentFrame;

  float BufferLenghtInSeconds = 1;
  bool isVideoLooping = true, isVideoOver = false, isVideoPaused = false;

  // SAR and Scaling
  Vector2i Dimensions{0, 0};

  // Audio
  ChargeAudio::SoundContainer Sound;

private:
  // Contextes
  _ffmpeg::AVFormatContext *ctx;
  const _ffmpeg::AVCodec *vCodec;
  const _ffmpeg::AVCodec *aCodec;
  _ffmpeg::AVCodecContext *vCodecCtx, *aCodecCtx;
  _ffmpeg::AVStream *videoStream, *audioStream;
  struct _ffmpeg::SwsContext *swsCtx = NULL; // Visual
  struct _ffmpeg::SwrContext *swrCtx = NULL; // Audio
  int8_t videoStreamNum = -1, audioStreamNum = -1;
  uint16_t ID = 0;

  // Time specific
  uint32_t currentFrameNumber = 0;
  double timeBase = 0, clock = 0;

  // Audio
  ChargeAudio::Engine *audioEngine;

  // Audio Channel data
  _ffmpeg::AVChannelLayout outLayout;
  _ffmpeg::AVSampleFormat sampleFormat = _ffmpeg::AV_SAMPLE_FMT_FLT;

  // Image Buffering
  std::map<double, Image2D> frameBuffer;
  uint32_t bufferMaxFrames = 0;

  // SAR / Sizing
  uint32_t scaleFactor = 1;

  // Frame handling
  bool frameSet = false;

  // Methods
  void continueVideo();
  std::pair<double, Containers::Array<char>> loadNextFrame();
  inline void frameDebug(_ffmpeg::AVFrame *frame);
  inline void frameSetScaleSAR(_ffmpeg::AVFrame *frame);
  inline void frameConvert(_ffmpeg::AVFrame *sourceFrame,
                           _ffmpeg::AVFrame *convertedFrame);
  inline void frameFlip(_ffmpeg::AVFrame *frame);

  inline void restartVideo();
  void dumpAndRefillBuffer();

  void loadTexture(Containers::Array<char> data);
  void loadTexture(ImageView2D image);
  Image2D loadImage(Containers::Array<char> data);
};
} // namespace ChargeVideo
#endif
