// Copyright 2016 Intel Corporation. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef SENDER_AUDIO_SENDER_H_
#define SENDER_AUDIO_SENDER_H_

#include "base/macros.h"
#include "sender/frame_sender.h"
#include "sender/audio_encoder.h"
#include "sharer_environment.h"

#include "ppapi/cpp/media_stream_audio_track.h"

#include <memory>

namespace sharer {

struct SenderConfig;

class AudioSender : public FrameSender {
 public:
  using PlayoutDelayChangeCb = std::function<void(base::TimeDelta)>;
  using SharerSuccessCb = std::function<void(bool success)>;

  explicit AudioSender(SharerEnvironment* env,
                       TransportSender* const transport_sender,
                       const SenderConfig& config, SharerSuccessCb cb,
                       PlayoutDelayChangeCb playout_delay_change_cb);
  ~AudioSender();

  void SetSize(const pp::Size& size);
  void StartSending(const pp::MediaStreamAudioTrack& audio_track,
                    const SharerSuccessCb& cb);
  void StopSending(const SharerSuccessCb& cb);
  void ChangeEncoding(const SenderConfig& config);

 protected:
  int GetNumberOfFramesInEncoder() const final;
  base::TimeDelta GetInFlightMediaDuration() const final;
  void OnAck(uint32_t frame_id) final;

 private:
  void Initialized(bool result);
  void ConfigureForFirstBuffer();
  void OnConfiguredForFirstBuffer(int32_t result);
  void OnFirstBuffer(int32_t result, pp::AudioBuffer buffer);
  void OnEncoderResized(bool success);
  void ConfigureTrack();
  void OnConfiguredTrack(int32_t result);
  void StartTrackBuffers();
  void StopTrackBuffers();
  void OnTrackBuffer(int32_t result, pp::AudioBuffer buffer);
  void ScheduleNextEncode();
  void GetEncoderFrameTick(int32_t result);
  void RecycleBuffer(pp::AudioBuffer buffer);
  void RequestEncodedFrame();
  void OnEncodedFrame(bool success, std::shared_ptr<EncodedFrame> frame);
  bool InsertRawAudioBuffer(const pp::AudioBuffer& buffer);

  SharerEnvironment* env_;

  bool initialized_;
  SharerSuccessCb initialized_cb_;
  PlayoutDelayChangeCb playout_delay_change_cb_;

  pp::CompletionCallbackFactory<AudioSender> factory_;

  std::unique_ptr<AudioEncoder> encoder_;

  double frame_rate_;
  int frames_in_encoder_;

  base::TimeDelta duration_in_encoder_;
  base::TimeTicks last_reference_time_;
  PP_TimeTicks pause_delta_;
  RtpTimestamp last_enqueued_frame_rtp_timestamp_;

  pp::Size requested_size_;
  pp::Size stream_size_;
  bool querying_size_;
  bool skip_resize_;

  bool is_receiving_track_frames_;
  bool is_sending_;
  pp::MediaStreamAudioTrack audio_track_;
  pp::AudioBuffer current_track_buffer_;

  SharerSuccessCb start_sending_cb_;

  DISALLOW_COPY_AND_ASSIGN(AudioSender);
};

}  // namespace sharer

#endif  // SENDER_AUDIO_SENDER_H_
