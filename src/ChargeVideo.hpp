#ifndef CHARGE_VIDEO_BASE_H
#define CHARGE_VIDEO_BASE_H

#include <Corrade/Containers/Array.h>
#include <Magnum/GL/Texture.h>
#include <Magnum/Image.h>
#include <Magnum/ImageView.h>
#include <Magnum/Magnum.h>

#include <cstdint>
#include <cstdlib>
#include <functional>
#include <queue>
#include <vector>

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
class Time {
public:
  static void AdvanceTime();
  static float DeltaTime;
  static float AverageDeltaTime;
  static uint16_t ADTMaxSample;

private:
  static Timeline time;
  static float rollingSum;
  static uint16_t ADTIndex, videoIDCounter;
  static bool ADTFirstCycle;
  static std::unordered_map<uint16_t, std::function<void()>> videoPlayMethods;
  static std::vector<float> deltaAverage;
  static std::vector<uint16_t> toUnhook;

  // Specific for internal controls
  static uint16_t hookVideo(std::function<void()> videoPlay);
  static void unhookVideo(uint16_t ID);
  friend class Video; // friend allows other classes to use private methods of a
                      // class without having to make it public for all
};

class Video {
public:
  Video(std::string path, bool ShouldVideoLoop = true,
        float BufferSizeInSeconds = 1.0f);
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

private:
  // Contextes
  _ffmpeg::AVFormatContext *ctx;
  const _ffmpeg::AVCodec *vCodec;
  const _ffmpeg::AVCodec *aCodec;
  _ffmpeg::AVCodecContext *vCodecCtx, *aCodecCtx;
  _ffmpeg::AVStream *videoStream, *audioStream;
  struct _ffmpeg::SwsContext *swsCtx = NULL; // Visual
  struct _ffmpeg::SwrContext *swrCtx = NULL; // Audio
  uint16_t ID = 0;

  // Time specific
  int8_t videoStreamNum = -1, audioStreamNum = -1;
  uint32_t currentFrameNumber = 0;
  float timeSink = 0.0f, frameTime = 0.0f;

  // Buffering
  std::queue<Image2D> frameBuffer;
  uint32_t bufferMaxFrames = 0;

  // SAR / Sizing
  uint32_t scaleFactor = 1;

  // Frame handling
  bool frameSet = false;
  void continueVideo();
  Containers::Array<char> loadNextFrame();
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
