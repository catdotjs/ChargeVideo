#include <Corrade/Containers/GrowableArray.h>

#include "ChargeVideo.hpp"

#include <Charge/ChargeAudio.hpp>
#include <Corrade/Containers/Pair.h>
#include <Corrade/Utility/Debug.h>
#include <Corrade/Utility/Utility.h>

#include <Magnum/GL/TextureFormat.h>
#include <Magnum/Image.h>
#include <Magnum/Math/Functions.h>
#include <Magnum/PixelFormat.h>
#include <libavcodec/avcodec.h>
#include <libavcodec/packet.h>
#include <libavformat/avformat.h>
#include <libavutil/channel_layout.h>
#include <libavutil/frame.h>
#include <libavutil/pixfmt.h>
#include <libavutil/rational.h>
#include <libavutil/samplefmt.h>
#include <libswresample/swresample.h>
#include <libswscale/swscale.h>
#include <map>
#include <utility>

using namespace ChargeVideo;
using namespace _ffmpeg;
#include <cstring>
#include <string>

// ================== Video Construct/Destruct ==================
Video::Video(std::string path, ChargeAudio::Engine *engine, Flags videoF,
             float bufferS)
    : audioEngine(engine) {
  // Have to do it here since ordering of init in the header class
  bufferLenghtInSeconds = bufferS;
  videoFlags = videoF;

  // Context to hold our data
  ctx = avformat_alloc_context();
  if (!ctx) {
    Utility::Error{} << "Could not allocate space for " << path.c_str();
    return;
  }

  if (avformat_open_input(&ctx, path.c_str(), NULL, NULL) != 0) {
    Utility::Error{} << "Could not open file " << path.c_str();
    avformat_close_input(&ctx);
    avformat_free_context(ctx);
    return;
  }

  for (uint8_t x = 0; x < ctx->nb_streams; x++) {
    if (ctx->streams[x]->codecpar->codec_type == AVMEDIA_TYPE_VIDEO) {
      videoStreamNum = x;
      videoStream = ctx->streams[x];
      continue;
    }
    if (ctx->streams[x]->codecpar->codec_type == AVMEDIA_TYPE_AUDIO) {
      audioStreamNum = x;
      audioStream = ctx->streams[x];
      continue;
    }
  }

  if (videoStreamNum == -1) {
    Utility::Error{} << "Could not find a video stream! " << path.c_str();
    avformat_free_context(ctx);
    return;
  }

  if (audioStreamNum == -1) {
    Utility::Debug{} << "No audio stream was found! Continuing anyway";
  }

  // Actual stream
  // Video Codec
  vCodec = avcodec_find_decoder(videoStream->codecpar->codec_id);
  vCodecCtx = avcodec_alloc_context3(vCodec);
  avcodec_parameters_to_context(vCodecCtx, videoStream->codecpar);

  vCodecCtx->thread_count = 0;
  vCodecCtx->thread_type = FF_THREAD_SLICE;
  avcodec_open2(vCodecCtx, vCodec,
                NULL); // open2 is such a stupid name

  // Some videos do not have audio streams
  // Audio Codec
  if (audioStreamNum != -1 && audioEngine) {
    aCodec = avcodec_find_decoder(audioStream->codecpar->codec_id);
    aCodecCtx = avcodec_alloc_context3(aCodec);
    avcodec_parameters_to_context(aCodecCtx, audioStream->codecpar);

    avcodec_open2(aCodecCtx, aCodec, NULL);

    // Hoo boy I love bunch of different ways to do the same thing!!!!!!
    // Now we have to deal with all of the ways to do that thing!!!!!
    if (audioEngine->GetChannelCount() == 2) {
      outLayout = AV_CHANNEL_LAYOUT_STEREO;
    } else {
      outLayout = AV_CHANNEL_LAYOUT_MONO;
    }

    // Resampling
    swr_alloc_set_opts2(&swrCtx, &outLayout, sampleFormat,
                        audioEngine->GetSampleRate(), &aCodecCtx->ch_layout,
                        aCodecCtx->sample_fmt, aCodecCtx->sample_rate, 0, NULL);
    swr_init(swrCtx);

    Sound = audioEngine->CreateSound(10);

    // Frame init
  }

  bufferMaxFrames = av_q2d(videoStream->avg_frame_rate) * bufferLenghtInSeconds;
  timeBase = av_q2d(videoStream->time_base);
}

Video::~Video() {
  sws_freeContext(swsCtx);
  if (audioStreamNum != -1) {
    swr_free(&swrCtx);
    av_frame_free(&audioFrame);
    av_frame_free(&convertedAudioFrame);
  }
  av_frame_free(&frame);
  av_frame_free(&convertedFrame);
  av_packet_free(&packet);

  avformat_close_input(&ctx);
  avformat_free_context(ctx);
  avcodec_free_context(&vCodecCtx);
  avcodec_free_context(&aCodecCtx);
}

// ================== Public Video Controls ==================
void Video::AdvanceToNextFrame() { loadTexture(loadNextFrame().second); }

void Video::Play() {
  if (ID != 0) {
    return;
  }
  ID = Manager::hookVideo(std::bind(&Video::continueVideo, this));
  reinitSound();
  if (audioStreamNum != -1) {
    Sound->Play();
  }
  videoState = State::Playing;
}

void Video::Pause() {
  if (ID == 0) {
    return;
  }
  Manager::unhookVideo(ID);
  reinitSound();
  ID = 0;
  videoState = State::Paused;
}

void Video::Restart() {
  if (ID == 0) {
    Play();
  }
  restartVideo();
}

const double Video::GetDuration() { return timeBase * videoStream->duration; }
const double Video::GetPlaybackTime() { return clock; }
const Vector2i Video::GetDimensions() { return Dimensions; }
Video::State Video::GetState() { return videoState; }
Video::Flags Video::GetFlags() { return videoFlags; }
void Video::SwitchLooping() { videoFlags = videoFlags ^ Flags::Looping; }

// ================== Private Video Controls ==================
void Video::continueVideo() {
  bool finishedDecoding = currentFrameNumber >= videoStream->nb_frames - 2,
       bufferEmpty = frameBuffer.empty(),
       isNotLooping = (videoFlags & Flags::Looping) != Flags::Looping;
  // Looping handling
  if (finishedDecoding && bufferEmpty) {
    if (isNotLooping) {
      Pause();
      videoState = State::Finished;
      return;
    }
    restartVideo();
  }

  // Timing
  // Audio Synced
  if (audioStreamNum != -1) {
    clock =
        (double)Sound->GetPlayedSampleCount() / audioEngine->GetSampleRate();
  } else {
    clock += Manager::DeltaTime;
  }

  // Load frame
  auto nextFrame = frameBuffer.begin();
  if (!bufferEmpty && nextFrame->first <= clock) {
    loadTexture(nextFrame->second);
    frameBuffer.erase(nextFrame);
  }

  if (!finishedDecoding && frameBuffer.size() < bufferMaxFrames) {
    auto frameData = loadNextFrame();
    frameBuffer.insert_or_assign(frameData.first,
                                 loadImage(std::move(frameData.second)));
  }

  if (audioStreamNum != -1 &&
      Sound->GetState() != ChargeAudio::Sound::State::Playing)
    Sound->Play();
}

// ======================== HELPERS ========================
std::pair<double, Containers::Array<char>> Video::loadNextFrame() {
  av_frame_unref(convertedFrame);
  av_frame_unref(frame);

  // A hard stop if we are out of frames to read
  while (av_read_frame(ctx, packet) >= 0) {
    if (audioEngine &&
        static_cast<int8_t>(packet->stream_index) == audioStreamNum) {
      avcodec_send_packet(aCodecCtx, packet);
      avcodec_receive_frame(aCodecCtx, audioFrame);

      if (audioFrame->format != -1 && audioEngine) {
        // Putting a pin on here, shouldn't we be able to rewrite on top of the
        // same buffer?
        //
        // At least with image frames the size is always constant
        convertedAudioFrame->format = sampleFormat;
        convertedAudioFrame->sample_rate = audioEngine->GetSampleRate();
        convertedAudioFrame->ch_layout = outLayout;
        convertedAudioFrame->nb_samples =
            swr_get_out_samples(swrCtx, audioFrame->nb_samples);
        av_frame_get_buffer(convertedAudioFrame, 2);

        swr_convert_frame(swrCtx, convertedAudioFrame, audioFrame);

        Sound->WriteToRingBuffer(convertedAudioFrame->data[0],
                                 convertedAudioFrame->linesize[0]);

        // Just due to a small memory leak this needs to be here
        av_frame_unref(convertedAudioFrame);
        av_frame_unref(audioFrame);
      }
    }

    if (static_cast<int8_t>(packet->stream_index) == videoStreamNum) {
      avcodec_send_packet(vCodecCtx, packet);
      avcodec_receive_frame(vCodecCtx, frame);
      av_packet_unref(packet);

      if (frame->format != AV_PIX_FMT_NONE) {
        frameSetScaleSAR(frame);
        frameFlip(frame);
        frameConvert();
        break;
      }
    }

    // You don't know what you are doing, do not touch this
    av_packet_unref(packet);
  }

  // Definetly need to calculate this once and then make a list of
  // preallocated frames in the framebuffer that we can constantly
  // cycle
  size_t dataSize = av_image_get_buffer_size(
      static_cast<AVPixelFormat>(convertedFrame->format), Dimensions.x(),
      Dimensions.y(), 3);
  Containers::Array<char> data = Containers::Array<char>{NoInit, dataSize};
  std::memcpy(data.data(), convertedFrame->data[0], dataSize);
  currentFrameNumber++;

  double ptsInSeconds = timeBase * frame->pts;

  // Cleanup time cus this is a C library yay (ironic)

  return {ptsInSeconds, std::move(data)};
}

Image2D Video::loadImage(Containers::Array<char> data) {
  Image2D image{PixelFormat::RGB8Unorm, Dimensions, std::move(data)};
  return image;
}

void Video::loadTexture(Containers::Array<char> data) {
  ImageView2D image{PixelFormat::RGB8Unorm, Dimensions, data};
  loadTexture(image);
}

void Video::loadTexture(ImageView2D image) {
  if (!frameSet) {
    CurrentFrame.setWrapping(GL::SamplerWrapping::ClampToEdge)
        .setMagnificationFilter(GL::SamplerFilter::Nearest)
        .setMinificationFilter(GL::SamplerFilter::Nearest)
        .setStorage(1, GL::textureFormat(image.format()), image.size());
    frameSet = true;
  }
  CurrentFrame.setSubImage(0, {}, image).generateMipmap();
}

// ======================== INLINES ========================
void Video::frameDebug(AVFrame *frame) {
  Utility::Debug{} << "Frame" << currentFrameNumber << "/"
                   << videoStream->nb_frames - 2
                   << "codec:" << avcodec_get_name(vCodecCtx->codec_id)
                   << "colourspace:"
                   << av_get_pix_fmt_name(
                          static_cast<AVPixelFormat>(frame->format))
                   << "SAR:" << frame->sample_aspect_ratio.num << ":"
                   << frame->sample_aspect_ratio.den
                   << "strides:" << frame->linesize[0] << frame->linesize[1]
                   << frame->linesize[2] << frame->linesize[3]
                   << "Ratio:" << frame->width << "x" << frame->height;
}

void Video::frameFlip(AVFrame *frame) {
  // Thank you so much to
  // https://ffmpeg-user.ffmpeg.narkive.com/t6y9mIOC/flip-in-sws-scale#post10
  //
  // Flips image 180 deg due to origin points of YUV420p and RGB24 being
  // different cus of course it is.
  // I had to figure out that U and V channels also need to be flipped but
  // we know that U and V are half of the size of Y so height/2
  frame->data[0] += frame->linesize[0] * (frame->height - 1);
  frame->data[1] += frame->linesize[1] * (frame->height / 2 - 1);
  frame->data[2] += frame->linesize[2] * (frame->height / 2 - 1);
  frame->linesize[0] = -frame->linesize[0];
  frame->linesize[1] = -frame->linesize[1];
  frame->linesize[2] = -frame->linesize[2];
}

void Video::frameConvert() {
  // Converting YUV420p to RGB24
  convertedFrame->format = AV_PIX_FMT_RGB24;
  convertedFrame->colorspace = AVCOL_SPC_BT709;
  convertedFrame->color_range = AVCOL_RANGE_JPEG;
  convertedFrame->width = Dimensions.x();
  convertedFrame->height = Dimensions.y();
  av_frame_get_buffer(convertedFrame,
                      3); // Proper way to allocate space for data

  if (swsCtx == NULL) {
    swsCtx = sws_getContext(Dimensions.x(), Dimensions.y(), vCodecCtx->pix_fmt,
                            Dimensions.x(), Dimensions.y(), AV_PIX_FMT_RGB24,
                            SWS_BILINEAR, NULL, NULL, NULL);
  }
  sws_setColorspaceDetails(swsCtx, sws_getCoefficients(vCodecCtx->colorspace),
                           frame->color_range,
                           sws_getCoefficients(convertedFrame->colorspace),
                           convertedFrame->color_range, 0, 1 << 16, 1 << 16);

  sws_scale(swsCtx, frame->data, frame->linesize, 0, Dimensions.y(),
            convertedFrame->data, convertedFrame->linesize);
}

void Video::frameSetScaleSAR(AVFrame *frame) {
  // SAR calculations
  if (Dimensions.x() == 0) {
    Dimensions.x() = frame->width;
    Dimensions.y() = frame->height;
    if (vCodecCtx->sample_aspect_ratio.num != 0) {
      AVRational SAR = vCodecCtx->sample_aspect_ratio, DAR;
      av_reduce(&DAR.num, &DAR.den, SAR.num * Dimensions.x(),
                SAR.den * Dimensions.y(), INT64_MAX);
      // Just to let the programmer know we have scaling happening due to
      // SAR
      scaleFactor = Math::min(Math::floor(Dimensions.x() / DAR.num),
                              Math::floor(Dimensions.y() / DAR.den));
      Dimensions.x() = DAR.num * scaleFactor;
      Dimensions.y() = DAR.den * scaleFactor;
    }
  }
}

void Video::restartVideo() {
  av_seek_frame(ctx, videoStreamNum, 0, AVSEEK_FLAG_BACKWARD);
  avcodec_flush_buffers(vCodecCtx);
  if (audioStreamNum != -1) {
    avcodec_flush_buffers(aCodecCtx);
  }
  currentFrameNumber = 0;
  clock = 0;
  dumpAndRefillBuffer();
}

void Video::dumpAndRefillBuffer() {
  std::map<double, Image2D>().swap(frameBuffer);
  reinitSound();
  loadTexture(loadNextFrame().second);
}

void Video::reinitSound() {
  if (audioStreamNum != -1) {
    // TO DO: we need a way to easily pass sound configs between two instances
    auto oldSound = Sound.release();
    oldSound->Pause();
    Sound = std::move(audioEngine->CreateSound(10));

    // This is very much temparory
    Sound->SetVolume(oldSound->GetVolume());
    Sound->SetPosition(oldSound->GetPosition());

    delete oldSound;
  }
}
