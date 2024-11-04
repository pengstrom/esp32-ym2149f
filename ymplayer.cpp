#include "ymplayer.h"
#include <esp_timer.h>
#include <cstring>
#include <rom/ets_sys.h>

const YmPlayer::msg_t YmPlayer::MSG_DEFAULT = {};

YmPlayer::YmPlayer(config_t cfg)
    : Ym2149(cfg)
{
  _msg_queue = xQueueCreate(QUEUE_SIZE, QUEUE_ITEM_SIZE);

  // esp_timer_create_args_t timer_cfg =
  //     {
  //         .callback = msg_handler,
  //         .arg = this,
  //         .name = "msg_timer",
  //         .skip_unhandled_events = true};
  // esp_timer_create(&timer_cfg, &_msg_timer);
  // esp_timer_start_periodic(_msg_timer, MSG_PERIOD_US);

  xTaskCreate(msg_handler, "msg_handler", 4096, this, configMAX_PRIORITIES - 1, &_msg_task);
}

YmPlayer::~YmPlayer()
{
  vQueueDelete(_msg_queue);
}

void YmPlayer::enqueueReset()
{
  enqueueMsg(MsgReset);
}

void YmPlayer::enqueueChanFreqFull(Channel chan, float fq)
{
  enqueueMsg(MsgChanFreqFull, {.fq = fq, .chan = chan});
}

void YmPlayer::enqueueChanFreqFine(Channel chan, float fq)
{
  enqueueMsg(MsgChanFreqFine, {.fq = fq, .chan = chan});
}

void YmPlayer::enqueueChanFreqRough(Channel chan, float fq)
{
  enqueueMsg(MsgChanFreqRough, {.fq = fq, .chan = chan});
}

void YmPlayer::enqueueNoiseFreq(float fq)
{
  enqueueMsg(MsgNoiseFreq, {.fq = fq});
}

void YmPlayer::enqueueMixerSetting(mixer_t mix)
{
  enqueueMsg(MsgMixer, {.mixer_settings = mix});
}

void YmPlayer::enqueueLevel(Channel chan, LevelMode mode, uint8_t level)
{
  enqueueMsg(MsgLevel, {.level = level, .level_mode = mode, .chan = chan});
}

void YmPlayer::enqueueEnvelopeFreqFull(float fq)
{
  enqueueMsg(MsgEnvFreqFull, {.fq = fq});
}

void YmPlayer::enqueueEnvelopeFreqFine(float fq)
{
  enqueueMsg(MsgEnvFreqFine, {.fq = fq});
}

void YmPlayer::enqueueEnvelopeFreqRough(float fq)
{
  enqueueMsg(MsgEnvFreqRough, {.fq = fq});
}

void YmPlayer::enqueueEnvelopeShape(envelope_t cfg)
{
  enqueueMsg(MsgEnvShape, {.env_settings = cfg});
}

void YmPlayer::dumpRegisters()
{
  for (size_t i = 0; i < 16; i++)
  {
    dumpRegister(i);
  }
}

void YmPlayer::dumpRegister(uint8_t reg)
{
  uint8_t data = readRegister((Register)reg);
  printf("Reg:    0x%02X\n", reg);
  printf(" Data:  0x%02X\n\n", data);
}

// Play snare with noise and single envelope fall
void YmPlayer::playSnare(Channel chan, uint32_t dur_ms)
{
  float f = 1000.0 / dur_ms;
  enqueueNoiseFreq(f);

  enqueueEnvelopeShape({});

  enqueueEnvelopeFreqFull(f);

  enqueueLevel(chan, LevelEnvelope, _state.level[chan]);

  mixer_t new_mix = _state.mixer;
  new_mix.noise[chan] = true;
  new_mix.chan[chan] = false;
  enqueueMixerSetting(new_mix);
}

void YmPlayer::msg_handler(void *arg)
{
  YmPlayer *self = (YmPlayer *)arg;
  while (1)
  {
    self->msgHandler();
  }
}

void YmPlayer::msgHandler()
{
  // Runs in task msg_handler
  msg_t msg;
  BaseType_t recv = xQueueReceive(_msg_queue, &msg, portMAX_DELAY);
  if (recv == pdFALSE)
  {
    // printf("Nothing in queue...\n");
    return;
  }

  // printf("Qeueu item: %u\n", msg.type);

  switch (msg.type)
  {
  case MsgReset:
    reset();
    std::memset(&_state, 0, sizeof(State)); // Nuke local state
    return;
  case MsgChanFreqFull:
    writeChanFreqFull(msg.chan, msg.fq);
    _state.freq[msg.chan] = msg.fq;
    return;
  case MsgChanFreqFine:
    writeChanFreqFine(msg.chan, msg.fq);
    _state.freq[msg.chan] = msg.fq;
    return;
  case MsgChanFreqRough:
    writeChanFreqRough(msg.chan, msg.fq);
    _state.freq[msg.chan] = msg.fq;
    return;
  case MsgNoiseFreq:
    writeNoiseFreq(msg.fq);
    _state.freq_noise = msg.fq;
    return;
  case MsgMixer:
    writeMixerSetting(msg.mixer_settings);
    _state.mixer = msg.mixer_settings;
    return;
  case MsgLevel:
    writeLevel(msg.chan, msg.level_mode, msg.level);
    _state.level[msg.chan] = msg.level;
    _state.level_mode[msg.chan] = msg.level_mode;
    return;
  case MsgEnvFreqFull:
    writeEnvelopeFreqFull(msg.fq);
    _state.freq_env = msg.fq;
    return;
  case MsgEnvFreqFine:
    writeEnvelopeFreqFine(msg.fq);
    _state.freq_env = msg.fq;
    return;
  case MsgEnvFreqRough:
    writeEnvelopeFreqRough(msg.fq);
    _state.freq_env = msg.fq;
    return;
  case MsgEnvShape:
    writeEnvelopeShape(msg.env_settings);
    _state.env = msg.env_settings;
    return;
  }
}

void YmPlayer::enqueueMsg(Message type, msg_t msg)
{
  msg.type = type;
  // printf("Enqueuing msg type: %u\n", type);
  xQueueSendToBack(_msg_queue, &msg, portMAX_DELAY);
}
