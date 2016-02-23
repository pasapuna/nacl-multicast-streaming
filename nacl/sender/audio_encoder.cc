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

void AudioEncoder::EncodeFrame(pp::AudioBuffer audio_buffer,
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
    config_.initial_bitrate, PP_HARDWAREACCELERATION_WITHFALLBACK, cc);
  );

  thread_loop_.Run();

  DINF() << "Thread finalizing.";
}
