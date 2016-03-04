// Copyright 2016 Intel Corporation. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "sender/audio_sender.h"

#include "base/logger.h"
#include "base/ptr_utils.h"
#include "net/sharer_transport_config.h"
#include "net/transport_sender.h"
#include "sender/congestion_control.h"
#include "sharer_defines.h"

static int32_t roundTo4(int32_t value) {
  int32_t rest = value % 4;
  return value - rest;
}

namespace sharer {

const int kRoundTripsNeeded = 4;
const int kConstantTimeMs = 75;

AudioSender::AudioSender(SharerEnvironment* env,
                         TransportSender* const transport_sender,
                         const SenderConfig& config, SharerSuccessCb cb,
                         PlayoutDelayChangeCb playout_delay_change_cb)
    : FrameSender(env->clock(), true, transport_sender, kVideoFrequency,
                  11, /* config.ssrc, */
                  config.frame_rate,
                  base::TimeDelta(), /* config.min_playout_delay, */
                  base::TimeDelta::FromMilliseconds(
                      kDefaultRtpMaxDelayMs), /* config.max_playout_delay, */
                  NewFixedCongestionControl(2000000)),
      env_(env),
      initialized_(false),
      initialized_cb_(cb),
      playout_delay_change_cb_(playout_delay_change_cb),
      factory_(this),
      frame_rate_(config.frame_rate),
      frames_in_encoder_(0),
      pause_delta_(0.1),
      querying_size_(false),
      skip_resize_(true),
      is_receiving_track_buffers_(false),
      is_sending_(false) {
  encoder_ = make_unique<AudioEncoder>(env->instance(), config);

  auto sharer_feedback_cb =
      [this](const std::string& addr, const RtcpSharerMessage& sharer_message) {
    this->OnReceivedSharerFeedback(sharer_message);
  };

  auto rtt_cb =
      [this](base::TimeDelta rtt) { this->OnMeasuredRoundTripTime(rtt); };

  SharerTransportRtpConfig transport_config;
  transport_config.ssrc = 11;
  transport_config.feedback_ssrc = 12;
  transport_config.rtp_payload_type = 96;
  transport_sender->InitializeVideo(transport_config, sharer_feedback_cb,
                                    rtt_cb);

  initialized_ = true;
  cb(true);
}

AudioSender::~AudioSender() { DINF() << "Destroying AudioSender."; }

int AudioSender::GetNumberOfFramesInEncoder() const {
  return frames_in_encoder_;
}

base::TimeDelta AudioSender::GetInFlightMediaDuration() const {
  /* if (GetUnacknowledgedFrameCount() > 0) { */
  /*   const uint32_t oldest_unacked_frame_id = latest_acked_frame_id_ + 1; */
  /*   return last_reference_time_ - */
  /*       GetRecordedReferenceTime(oldest_unacked_frame_id); */
  /* } else { */
  return duration_in_encoder_;
  /* } */
}

void AudioSender::OnAck(uint32_t frame_id) {}

void AudioSender::StartSending(const pp::MediaStreamAudioTrack& audio_track,
                               const SharerSuccessCb& cb) {
  if (!audio_track_.is_null()) {
    ERR() << "Already sending or trying to send track.";
    cb(false);
    return;
  }

  if (!initialized_) {
    ERR() << "Did not initialize audio sender yet. Can't start sending.";
    cb(false);
    return;
  }
  if (!initialized_) {
    ERR() << "Did not initialize audio sender yet. There is nothing to stop";
    return;
  }

  StopTrackFrames();
  encoder_->Stop();

  audio_track_.Close();
  audio_track_ = pp::MediaStreamAudioTrack();
  frames_in_encoder_ = 0;
  encoder_->FlushEncodedFrames();

  last_reference_time_ = base::TimeTicks();
  duration_in_encoder_ = duration_in_encoder_.FromInternalValue(0);
  DINF() << "Stopped sending frames.\n";
  is_sending_ = false;
  cb(true);
}

void AudioSender::ChangeEncoding(const SenderConfig& config) {
  DINF() << "Changing encoding";
  encoder_->ChangeEncoding(config);
}

void AudioSender::ConfigureForFirstBuffer() {
  int32_t attrib_list[]{
      PP_MediaStreamAudioTrack_ATTRIB_FORMAT, encoder_->format(),
      PP_MediaStreamAudioTrack_ATTRIB_NONE};

  auto cc = factory_.NewCallback(&AudioSender::OnConfiguredForFirstFrame);
  audio_track_.Configure(attrib_list, cc);
}

void AudioSender::OnConfiguredForFirstBuffer(int32_t result) {
  if (result != PP_OK) {
    ERR() << "Could not configure audio track: " << result;
    if (start_sending_cb_) start_sending_cb_(false);
    start_sending_cb_ = nullptr;
    return;
  }

  auto cc = factory_.NewCallbackWithOutput(&AudioSender::OnFirstBuffer);
  audio_track_.GetBuffer(cc);
}

void AudioSender::OnFirstBuffer(int32_t result, pp::AudioBuffer buffer) {
  // TODO: on aborted, stop everything related to encoding and sending frames
  if (result == PP_ERROR_ABORTED) return;

  if (result != PP_OK) {
    ERR() << "Cannot get buffer from audio track: " << result;
    return;
  }

  audio_track_.RecycleBuffer(buffer);
}

void AudioSender::OnEncoderResized(bool success) {
  if (!success) {
    ERR() << "Could not resize encoder.";
    return;
  }

  if (skip_resize_)
    OnConfiguredTrack(PP_OK);
  else
    // Reconfigure stream
    ConfigureTrack();
}

void AudioSender::ConfigureTrack() {
  int32_t attrib_list[]{
      PP_MEDIASTREAMAUDIOTRACK_ATTRIB_BUFFERS, encoder_->buffers(),
      PP_MEDIASTREAMAUDIOTRACK_ATTRIB_SAMPLE_RATE, encoder_->sampleRate(),
      PP_MEDIASTREAMAUDIOTRACK_ATTRIB_SAMPLE_SIZE, encoder_->sampleSize(),
      PP_MEDIASTREAMAUDIOTRACK_ATTRIB_CHANNELS, encoder_->channels(),
      PP_MEDIASTREAMAUDIOTRACK_ATTRIB_DURATION, encoder_->duration(),
      PP_MEDIASTREAMAUDIOTRACK_ATTRIB_NONE};

  DINF() << "Configuring track to: " << encoder_->buffers() << " buffers, "
         << encoder_->sampleRate() <<  " sample rate, "
         << encoder_->sampleSize() << " sample size, " << encoder_->channels()
         << " channels, " << encoder_->duration() << "duration"
         

  auto cc = factory_.NewCallback(&AudioSender::OnConfiguredTrack);
  audio_track_.Configure(attrib_list, cc);
}

void AudioSender::OnConfiguredTrack(int32_t result) {
  if (result != PP_OK) {
    ERR() << "Could not configure audio track: " << result;
    if (start_sending_cb_)
      start_sending_cb_(false);
    
    start_sending_cb_ = nullptr;
    
    return;
  }

  RequestEncodedFrame();
  StartTrackFrames();
  ScheduleNextEncode();
  if (start_sending_cb_)
    start_sending_cb_(true);
  start_sending_cb_ = nullptr;
}

void AudioSender::StartTrackBuffers() {
  DINF() << "Starting to track buffers.";
  is_receiving_track_buffers_ = true;
  auto cc = factory_.NewCallbackWithOutput(&AudioSender::OnTrackBuffer);
  audio_track_.GetBuffer(cc);
}

void AudioSender::StopTrackBuffers() {
  is_receiving_track_buffers_ = false;
  if (!current_track_buffer_.is_null()) {
    audio_track_.RecycleBuffer(current_track_buffer_);
    current_track_buffer_.detach();
  }
}

void AudioSender::OnTrackBuffer(int32_t result, pp::AudioBuffer buffer) {
  if (result == PP_ERROR_ABORTED) return;

  if (!current_track_buffer_.is_null()) {
    audio_track_.RecycleBuffer(current_track_buffer_);
    current_track_buffer_.detach();
  }

  if (result != PP_OK) {
    ERR() << "Cannot get buffer from audio track: " << result;
    return;
  }

  if (is_receiving_track_buffers_) {
    current_track_buffer_ = buffer;
    auto cc = factory_.NewCallbackWithOutput(&AudioSender::OnTrackBuffer);
    audio_track_.GetBuffer(cc);
  }
}

void AudioSender::ScheduleNextEncode() {
  auto cc = factory_.NewCallback(&AudioSender::GetEncoderFrameTick);
  pp::Module::Get()->core()->CallOnMainThread(1000 / frame_rate_, cc, 0);
}

void AudioSender::GetEncoderFrameTick(int32_t result) {
  if (!current_track_buffer_.is_null()) {
    pp::AudioBuffer buffer = current_track_buffer_;
    current_track_buffer_.detach();

    if (!InsertRawAudioBuffer(buffer)) {
      RecycleBuffer(buffer);
    }
  }

  ScheduleNextEncode();
}

bool AudioSender::InsertRawAudioBuffer(const pp::AudioBuffer& buffer) {
  if (!encoder_) {
    PP_NOTREACHED();
    return false;
  }
  PP_TimeTicks time_sticks = buffer.GetTimestamp();

  const base::TimeTicks reference_time = env_->clock()->NowTicks();

  const RtpTimestamp rtp_timestamp =
      PP_TimeDeltaToRtpDelta(time_sticks, kVideoFrequency);

  if (!last_reference_time_.is_null() &&
      (!IsNewerRtpTimestamp(rtp_timestamp,
                            last_enqueued_frame_rtp_timestamp_) ||
       reference_time < last_reference_time_)) {
    DWRN() << "Dropping audio buffer: RTP or reference time did not increase.";
    return false;
  }
  const base::TimeDelta duration_added_by_next_frame = reference_time -
                                                       last_reference_time_;
                             

  if (ShouldDropNextFrame(duration_added_by_next_frame)) {
    base::TimeDelta new_target_delay =
        std::min(current_round_trip_time_ * kRoundTripsNeeded +
                     base::TimeDelta::FromMilliseconds(kConstantTimeMs),
                 max_playout_delay_);
    if (new_target_delay > target_playout_delay_) {
      DWRN() << "New target delay: " << new_target_delay.InMilliseconds();
      playout_delay_change_cb_(new_target_delay);
    }

    return false;
  }

  // Send buffer to encoder, and add a callback to when it can be released.
  auto release_cb = [this](pp::AudioBuffer buffer) { this->RecycleBuffer(buffer); };
  frames_in_encoder_++;
  duration_in_encoder_ += duration_added_by_next_frame;
  last_reference_time_ = reference_time;
  last_enqueued_frame_rtp_timestamp_ = rtp_timestamp;
  pause_delta_ = time_sticks + 0.1;
  encoder_->EncodeBuffer(buffer, reference_time, release_cb);
  return true;
}

void AudioSender::RecycleBuffer(pp::AudioBuffer buffer) {
  audio_track_.RecycleBuffer(buffer);
}

void AudioSender::RequestEncodedFrame() {
  auto encoded_cb = [this](bool success, std::shared_ptr<EncodedFrame> frame) {
    this->OnEncodedFrame(success, frame);
  };
  encoder_->GetEncodedFrame(encoded_cb);
}

void AudioSender::OnEncodedFrame(bool success,
                                 std::shared_ptr<EncodedFrame> frame) {
  /* DINF() << "Encoded frame received: " << frame->frame_id; */
  duration_in_encoder_ = last_reference_time_ - frame->reference_time;
  frames_in_encoder_--;

  SendEncodedFrame(frame);

  RequestEncodedFrame();
}

}  // namespace sharer
