#include "ChargeVideo.hpp"

#include <Corrade/Utility/Utility.h>

#include <Magnum/GL/TextureFormat.h>
#include <Magnum/Math/Functions.h>
#include <Magnum/PixelFormat.h>

using namespace ChargeVideo;
using namespace _ffmpeg;
#include <cstring>
#include <string>

// ================== Video Construct/Destruct ==================
// ShouldVideoLoop default is true
Video::Video(std::string path, bool ShouldVideoLoop, float BufferSizeInSeconds)
    : BufferLenghtInSeconds(BufferSizeInSeconds),
      isVideoLooping(ShouldVideoLoop) {
  // Context to hold our data
  ctx = avformat_alloc_context();
  if (!ctx) {
    Utility::Error{} << "Could not allocate space for " << path.c_str();
    return;
  }

  if (avformat_open_input(&ctx, path.c_str(), NULL, NULL) != 0) {
    Utility::Error{} << "Could not open file " << path.c_str();
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
  vCodec = avcodec_find_decoder(videoStream->codecpar->codec_id);
  vCodecCtx = avcodec_alloc_context3(vCodec);
  avcodec_parameters_to_context(vCodecCtx, videoStream->codecpar);

  vCodecCtx->thread_count = 0;
  vCodecCtx->thread_type = FF_THREAD_SLICE;
  avcodec_open2(vCodecCtx, vCodec,
                NULL); // open2 is such a stupid name

  // Some videos do not have audio streams
  if (audioStreamNum != -1) {
    aCodec = avcodec_find_decoder(audioStream->codecpar->codec_id);
    aCodecCtx = avcodec_alloc_context3(aCodec);
    avcodec_parameters_to_context(aCodecCtx, audioStream->codecpar);

    avcodec_open2(aCodecCtx, aCodec, NULL);

    // Hoo boy I love bunch of different ways to do the same thing!!!!!!
    // Now we have to deal with all of the ways to do that thing!!!!!
    AVChannelLayout outLayout = AV_CHANNEL_LAYOUT_STEREO;
    swr_alloc_set_opts2(&swrCtx, &outLayout, AV_SAMPLE_FMT_S16, 44100,
                        &aCodecCtx->ch_layout, aCodecCtx->sample_fmt,
                        aCodecCtx->sample_rate, 0, NULL);
    swr_init(swrCtx);
  }

  // Timing stuff
  frameTime = 1 / av_q2d(videoStream->avg_frame_rate);
  bufferMaxFrames = (1 / frameTime) * BufferLenghtInSeconds;
}

Video::~Video() {
  sws_freeContext(swsCtx);
  swr_free(&swrCtx);
  avformat_free_context(ctx);
  avcodec_free_context(&vCodecCtx);
  avcodec_free_context(&aCodecCtx);
}

// ================== Public Video Controls ==================
void Video::AdvanceToNextFrame() { loadTexture(loadNextFrame()); }

void Video::Play() {
  if (ID != 0) {
    return;
  }
  ID = Time::hookVideo(std::bind(&Video::continueVideo, this));
  isVideoPaused = false;
  isVideoOver = false;
}

void Video::Pause() {
  if (ID == 0) {
    return;
  }
  Time::unhookVideo(ID);
  ID = 0;
  isVideoPaused = true;
}

void Video::Restart() {
  if (ID == 0) {
    Play();
  }
  restartVideo();
  dumpAndRefillBuffer();
}

void Video::StopLooping() { isVideoLooping = false; }

void Video::StartLooping() { isVideoLooping = true; }

// ================== Private Video Controls ==================
void Video::continueVideo() {
  // Looping handling
  if (currentFrameNumber >= videoStream->nb_frames - 2) {
    if (!isVideoLooping) {
      isVideoOver = true;
      Pause(); // Here we did that (check comment below)
      return;  // We remove what we are returning TO
    }
    restartVideo();
  }

  // Timing
  float variableFrameTime = frameTime - Time::AverageDeltaTime;
  if (timeSink < variableFrameTime) {
    timeSink += Time::DeltaTime;

    if (!isVideoOver && frameBuffer.size() < bufferMaxFrames) {
      frameBuffer.push(loadImage(loadNextFrame()));
    }
    return;
  }
  // This allows the lag to not accumillate
  timeSink -= variableFrameTime;

  if (frameBuffer.size() == 0) {
    frameBuffer.push(loadImage(loadNextFrame()));
  }

  loadTexture(frameBuffer.front());
  frameBuffer.pop();
}

// ======================== HELPERS ========================
Containers::Array<char> Video::loadNextFrame() {
  AVFrame *frame = av_frame_alloc(), *convertedFrame = av_frame_alloc(),
          *audioFrame = av_frame_alloc(),
          *convertedAudioFrame = av_frame_alloc();
  AVPacket *packet = av_packet_alloc();

  // A hard stop if we are out of frames to read
  while (av_read_frame(ctx, packet) >= 0) {
    if (static_cast<int8_t>(packet->stream_index) == audioStreamNum) {
      avcodec_send_packet(aCodecCtx, packet);
      avcodec_receive_frame(aCodecCtx, audioFrame);
      if (audioFrame->format != -1) {
        convertedAudioFrame->format = AV_SAMPLE_FMT_S16;
        convertedAudioFrame->nb_samples =
            swr_get_out_samples(swrCtx, audioFrame->nb_samples);
        convertedAudioFrame->ch_layout = AV_CHANNEL_LAYOUT_STEREO;
        av_frame_get_buffer(convertedAudioFrame,
                            2); // since it is LRLRLRLRLRLRLR

        swr_convert(swrCtx, convertedAudioFrame->data,
                    convertedAudioFrame->nb_samples, audioFrame->data,
                    audioFrame->nb_samples);
      }
    }

    if (static_cast<int8_t>(packet->stream_index) == videoStreamNum) {
      // Requests a frame from the decoder
      avcodec_send_packet(vCodecCtx, packet);
      avcodec_receive_frame(vCodecCtx, frame);
      av_packet_unref(packet);

      if (frame->format != -1) {
        // FrameDebug(frame);
        frameSetScaleSAR(frame);
        frameFlip(frame);

        frameConvert(frame, convertedFrame);
        // FrameDebug(convertedFrame);
        break;
      }
    }
    av_packet_unref(packet);
  }
  // You cannot use strlen(data) it does not work
  size_t dataSize = av_image_get_buffer_size(
      static_cast<AVPixelFormat>(convertedFrame->format), Dimensions.x(),
      Dimensions.y(), 3);
  Containers::Array<char> data = Containers::Array<char>{NoInit, dataSize};
  std::memcpy(data.data(), convertedFrame->data[0], dataSize);
  currentFrameNumber++;

  // Cleanup time cus this is a C library yay (ironic)
  av_frame_free(
      &convertedFrame); // Data[0] from here needs to be owned by someone else
  av_frame_free(&convertedAudioFrame);
  av_frame_free(&frame);
  av_frame_free(&audioFrame);
  av_packet_free(&packet);

  return data;
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

void Video::frameConvert(AVFrame *sourceFrame, AVFrame *convertedFrame) {
  // Converting YUV420p to RGB24
  convertedFrame->format = AV_PIX_FMT_RGB24;
  convertedFrame->colorspace = AVCOL_SPC_BT709;
  convertedFrame->color_range = AVCOL_RANGE_JPEG;
  convertedFrame->width = Dimensions.x();
  convertedFrame->height = Dimensions.y();
  av_frame_get_buffer(convertedFrame,
                      3); // Proper way to allocate space for data

  if (swsCtx == NULL) {
    swsCtx = sws_getContext(Dimensions.x(), Dimensions.y(),
                            static_cast<AVPixelFormat>(sourceFrame->format),
                            Dimensions.x(), Dimensions.y(),
                            static_cast<AVPixelFormat>(convertedFrame->format),
                            SWS_BICUBIC, NULL, NULL, NULL);
  }
  // TO DO: DO THIS PROPERLY
  sws_setColorspaceDetails(swsCtx, sws_getCoefficients(SWS_CS_ITU709),
                           sourceFrame->color_range,
                           sws_getCoefficients(SWS_CS_ITU709),
                           convertedFrame->color_range, 0, 1 << 16, 1 << 16);
  // -----------------------------

  sws_scale(swsCtx, sourceFrame->data, sourceFrame->linesize, 0, Dimensions.y(),
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
  avcodec_flush_buffers(aCodecCtx);
  currentFrameNumber = 0;
}

void Video::dumpAndRefillBuffer() {
  std::queue<Image2D>().swap(frameBuffer);
  loadTexture(loadNextFrame());
}
