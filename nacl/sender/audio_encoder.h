// Copyright 2016 Intel Corporation. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef SENDER_AUDIO_ENCODER_H_
#define SENDER_AUDIO_ENCODER_H_

#include "base/macros.h"
#include "base/time/time.h"
#include "sharer_config.h"

#include "ppapi/c/pp_time.h"
#include "ppapi/cpp/instance.h"
#include "ppapi/cpp/message_loop.h"
#include "ppapi/cpp/audio_encoder.h"
#include "ppapi/utility/completion_callback_factory.h"

namespace sharer {

struct SenderConfig;

class AudioEncoder {
 public:
  using AudioEncoderInitializedCb = std::function<void(bool result)>;
  using EncoderReleaseCb = std::function<void(pp::AudioBuffer audio_buffer)>;
  using EncoderEncodedCb =
      std::function<void(bool success, std::shared_ptr<EncodedFrame> frame)>;
  explicit AudioEncoder(pp:Instance* instance, const SenderConfig& config,
                        AudioEncoderInitializedCb cb);

  void EncodeFrame(pp::AudioBuffer audio_buffer,
    const base::TimeTicks& timestamp, EncoderReleaseCb cb);
  void GetEncodedFrame(EncoderEncodedCb cb);
  void FlushEncodedFrames();
  void Stop();
  void ChangeEncoding(const SenderConfig& config);
 private:
  struct Request {
    pp::AudioBuffer audio_buffer;
    EncoderReleaseCb callback;
    base::TimeTicks reference_time;
  };

  void Initialize();
  void InitializedThread(int32_t result);
  void EncodeOneFrame();
  void OnFrameReleased(int32_t result);
  void OnEncodedFrame(int32_t result, std::shared_ptr<EncodedFrame> frame);
  void EmitOneFrame(int32_t result);
  void EncoderPauseDestructor();

  void ThreadInitialize();
  void ThreadInitialized(int32_t result);
  void ThreadEncode(int32_t result);

  pp::Instance* instance;
  pp::CompletionCallbackFactory<AudioEncoder> factory_;
  AudioEncoderInitializedCb initialized_cb_;
  SenderConfig config_;

  std::queue<Request> requests_;
  Request current_request_;

  std::queue<std::shared_ptr<EncodedFrame>> encoded_audio_buffers_;

  pp::MessageLoop thread_loop_;
  std::thread encoder_thread_;

  pp::AudioEncoder audio_encoder_;

  uint32_t last_encoded_frame_id_;
  PP_TimeDelta last_timestamp_;
  base::TimeTicks last_reference_time_;
  bool is_initialized_;

  DISALLOW_COPY_AND_ASSIGN(AudioEncoder);
};

} // namespace sharer

#endif  // SENDER_AUDIO_ENCODER_H_
