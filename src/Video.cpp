#include "ChargeVideo.hpp"
#include <utility>

using namespace ChargeVideo;
using namespace _ffmpeg;

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
