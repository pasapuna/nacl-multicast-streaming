// Copyright 2016 Intel Corporation. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "sender/audio_encoder.h"

#include "base/logger.h"
#include "net/sharer_transport_config.h"
#include "sharer_defines.h"

#include "ppapi/cpp/instance.h"

namespace sharer {

AudioEncoder::AudioEncoder(pp::Instance* instance, const SenderConfig& config,
                           AudioEncoderInitializedCb cb)
    : instance_(instance),
      factory_(this),
      initialized_cb_(cb),
      config_(config),
      last_encoded_frame_id_(kStartFrameId),
      last_timestamp_(0),
      is_initialized_(false) {
  INF() << "Starting audio encoder";
  Initialize();
}

void AudioEncoder::Initialize() {
  thread_loop_ = pp::MessageLoop(instance_);
  encoder_thread_ = std::thread(&AudioEncoder::ThreadInitialize, this);
}

void AudioEncoder::InitializedThread(int32_t result) {
  if (!is_initialized_) {
    is_initialized_ = true;

    // Inform successful initialization only once
    if (initialized_cb_) {
      initialized_cb_(result == PP_OK);
      initialized_cb_ = nullptr;
    }
  }

  EncodeOneFrame();
}

void AudioEncoder::EncodeBuffer(pp::AudioBuffer audio_buffer,
                                const base::TimeTicks& reference_time,
                                EncoderReleaseCb cb) {
  Request req;
  req.audio_buffer = audio_buffer;
  req.callback = cb;
  req.reference_time = reference_time;

  requests_.push(req);

  EncodeOneFrame();
}

void AudioEncoder::GetEncodedFrame(EncoderEncodedCb cb) {
  if (encoded_cb_) {
    WRN() << "EncodedFrame already requested, ignoring.";
    return;
  }

  encoded_cb_ = cb;

  if (is_initialized_) {
    auto cc = factory_.NewCallback(&AudioEncoder::EmitOneFrame);
    pp::Module::Get()->core()->CallOnMainThread(0, cc);
  } else {
    Initialize();
  }
}

void AudioEncoder::EncoderPauseDestructor() {
  uint32_t ret = thread_loop_.PostQuit(PP_TRUE);
  encoder_thread_.join();
  audio_encoder_.Close();
  DINF() << "Pausing encoder thread: " << ret;
  is_initialized_ = false;
}

void AudioEncoder::Stop() { EncoderPauseDestructor(); }

void AudioEncoder::EmitOneFrame(int32_t result) {
  if (!encoded_cb_ || encoded_frames_.empty()) {
    return;
  }

  EncoderEncodedCb cb = encoded_cb_;
  encoded_cb_ = nullptr;

  auto encoded = encoded_frames_.front();
  encoded_frames_.pop();

  cb(true, encoded);
}

void AudioEncoder::EncodeOneFrame() {
  if (!is_initialized_) return;

  if (requests_.empty()) return;

  // Already encoding a frame
  if (current_request_.callback) return;

  current_request_ = requests_.front();
  requests_.pop();

  auto cc = factory_.NewCallback(&AudioEncoder::ThreadEncode);
  thread_loop_.PostWork(cc);
}

void AudioEncoder::OnFrameReleased(int32_t result) {
  if (current_request_.callback) {
    current_request_.callback(current_request_.audio_buffer);
  }
  current_request_ = Request();

  EncodeOneFrame();
}

void AudioEncoder::OnEncodedFrame(int32_t result,
                                  std::shared_ptr<EncodedFrame> frame) {
  encoded_frames_.push(frame);
  EmitOneFrame(PP_OK);
}

void AudioEncoder::FlushEncodedFrames() {
  while (!encoded_frames_.empty()) {
    encoded_frames_.pop();
  }
  while (!requests_.empty()) {
    requests_.pop();
  }

  encoded_cb_ = nullptr;
}

void AudioEncoder::ThreadInitialize() {
  DINF() << "Thread starting.";
  thread_loop_.AttachToCurrentThread();

  auto cc = factory_.NewCallback(&AudioEncoder::ThreadInitialized);

  audio_encoder_ = pp::AudioEncoder(instance_);

  audio_encoder_.Initialize(2, PP_AUDIOBUFFER_SAMPLERATE_48000,
    PP_AUDIOBUFFER_SAMPLESIZE_16_BITS, PP_AUDIOPROFILE_OPUS,
    config_.initial_bitrate * 1000, PP_HARDWAREACCELERATION_WITHFALLBACK, cc);
  );

  thread_loop_.Run();

  DINF() << "Thread finalizing.";
}

void AudioEncoder::ThreadInitialized(int32_t result) {
  auto cc = factory_.NewCallback(&AudioEncoder::InitializedThread);

  if (result != PP_OK) {
    ERR() << "Could not initialize AudioEncoder:" << result;
    pp::Module::Get()->core()->CallOnMainThread(0, cc, PP_ERROR_FAILED);
    return;
  }

  if (audio_encoder_.GetFrameCodedSize(&encoder_size_) != PP_OK) {
    ERR() << "Could not get Frame Coded Size.";
    pp::Module::Get()->core()->CallOnMainThread(0, cc, PP_ERROR_FAILED);
    return;
  }

  DINF() << "Audio encoder thread initialized.";
  pp::Module::Get()->core()->CallOnMainThread(0, cc, PP_OK);

  auto bitstream_cb = factory_.NewCallbackWithOutput(
      &AudioEncoder::ThreadOnBitstreamBufferReceived);
  audio_encoder_.GetBitstreamBuffer(bitstream_cb);
}

void AudioEncoder::ThreadOnBitstreamBufferReceived(int32_t result,
                                                   PP_BitstreamBuffer buffer) {
  if (result == PP_ERROR_ABORTED) return;

  if (result != PP_OK) {
    ERR() << "Could not get bitstream buffer: " << result;
    return;
  }

  auto encoded_frame = ThreadBitstreamToEncodedFrame(buffer);
  if (encoded_frame) {
    auto encoded_main_cb =
        factory_.NewCallback(&AudioEncoder::OnEncodedFrame, encoded_frame);
    pp::Module::Get()->core()->CallOnMainThread(0, encoded_main_cb, PP_OK);
  }

  audio_encoder_.RecycleBitstreamBuffer(buffer);

  auto bitstream_cb = factory_.NewCallbackWithOutput(
      &AudioEncoder::ThreadOnBitstreamBufferReceived);
  audio_encoder_.GetBitstreamBuffer(bitstream_cb);
}

std::shared_ptr<EncodedFrame> AudioEncoder::ThreadBitstreamToEncodedFrame(
    PP_BitstreamBuffer buffer) {
  auto frame = std::make_shared<EncodedFrame>();
  frame->frame_id = ++last_encoded_frame_id_;
  if (buffer.key_frame) {
    frame->dependency = EncodedFrame::KEY;
    frame->referenced_frame_id = frame->frame_id;
  } else {
    frame->dependency = EncodedFrame::DEPENDENT;
    frame->referenced_frame_id = frame->frame_id - 1;
  }

  frame->rtp_timestamp =
      PP_TimeDeltaToRtpDelta(last_timestamp_, kVideoFrequency);
  frame->reference_time = last_reference_time_;
  frame->data.clear();
  frame->data.reserve(buffer.size);
  frame->data.insert(0, static_cast<char*>(buffer.buffer), buffer.size);

  return frame;
}

void AudioEncoder::ThreadEncode(int32_t result) {
  /* DINF() << "Request to encode frame."; */
  auto cc = factory_.NewCallbackWithOutput(&AudioEncoder::ThreadOnEncoderFrame,
                                           current_request_);
  audio_encoder_.GetBuffer(cc);
}

void AudioEncoder::ThreadInformFrameRelease(int32_t result) {
  auto cc = factory_.NewCallback(&AudioEncoder::OnFrameReleased);
  pp::Module::Get()->core()->CallOnMainThread(0, cc, result);
}

void AudioEncoder::ThreadOnEncoderFrame(int32_t result,
                                        pp::AudioBuffer encoder_buffer,
                                        Request req) {
  if (result == PP_ERROR_ABORTED) {
    ThreadInformFrameRelease(result);
    return;
  }

  if (result != PP_OK) {
    ERR() << "Could not get frame from encoder: " << result;
    ThreadInformFrameRelease(result);
    return;
  }

  // TODO: Check for frame size

  if (ThreadCopyAudioBuffer(encoder_buffer, req.audio_buffer) == PP_OK) {
    PP_TimeDelta timestamp = req.audio_buffer.GetTimestamp();

    last_timestamp_ = timestamp;
    last_reference_time_ = req.reference_time;
    auto cc =
        factory_.NewCallback(&AudioEncoder::ThreadOnEncodeDone, timestamp, req);
    audio_encoder_.Encode(encoder_buffer, PP_FALSE, cc);
  }

  ThreadInformFrameRelease(PP_OK);
}

int32_t AudioEncoder::ThreadCopyAudioBuffer(pp::AudioBuffer dst,
                                            pp::AudioBuffer src) {
  if (dst.GetDataBufferSize() < src.GetDataBufferSize()) {
    ERR() << "Incorrect destination audio buffer size: "
          << dst.GetDataBufferSize() << " < " << src.GetDataBufferSize();
    return PP_ERROR_FAILED;
  }

  dst.SetTimestamp(src.GetTimestamp());
  memcpy(dst.GetDataBuffer(), src.GetDataBuffer(), src.GetDataBufferSize());
  return PP_OK;
}

void AudioEncoder::ThreadOnEncodeDone(int32_t result, PP_TimeDelta timestamp,
                                      Request req) {
  if (result == PP_ERROR_ABORTED) return;

  if (result != PP_OK) {
    ERR() << "Encode failed: " << result;
    return;
  }
}

}  // namespace sharer
