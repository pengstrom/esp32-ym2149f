#if !defined(_YMPLAYER_H)
#define _YMPLAYER_H

#include <ym2149f.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <esp_timer.h>

class YmPlayer : private Ym2149
{
public:
  using Ym2149::Channel;
  using Ym2149::config_t;
  using Ym2149::LevelMode;

  YmPlayer(config_t cfg);
  ~YmPlayer();

  void enqueueReset();

  void enqueueChanFreqFull(Channel chan, float fq);
  void enqueueChanFreqFine(Channel chan, float fq);
  void enqueueChanFreqRough(Channel chan, float fq);
  void enqueueNoiseFreq(float fq);

  void enqueueMixerSetting(mixer_t mix);

  void enqueueLevel(Channel chan, LevelMode mode, uint8_t level);

  void enqueueEnvelopeFreqFull(float fq);
  void enqueueEnvelopeFreqFine(float fq);
  void enqueueEnvelopeFreqRough(float fq);

  /// @brief The Envelope Generator further counts down the envelope frequency by 16, producing a 16-state per cycle envelope pattern as defined by its 4-bit counter output, E3 E2 El E0. The particular shape and cycle pattern of any desired envelope is accomplished by controlling the count pattern (count up/count down) of the 4-bit counter and by defining a single-cycle or repeat-cycle pattern
  /// @param cfg.repeat when set to logic “1”, the cycle pattern will be as defined by the Hold bit; when set to logic “0”, the envelope generator will reset to 0000 after one cycle and hold at that count.
  /// @param cfg.attack when set to logic “1": the envelope counter will count up (attack) from E3 E2 E1 E0=0000 to E3 E2 E1 E0=1111; when set to logic “0”, the envelope counter will count down (decay) from 1111 to 0000.
  /// @param cfg.alternate when set to logic “1”, the envelope counter reverses count direction (up-down) after each cycle.
  /// @param cfg.hold when set to logic “1”, limits the envelope to one cycle, holding the last count of the envelope counter (E3--E0=0000 or 1111, depending on whether the envelope counter was in a count-down or count-up mode, respectively).
  void enqueueEnvelopeShape(envelope_t cfg);

  void dumpRegisters();
  void dumpRegister(uint8_t reg);

  void playSnare(Channel chan, uint32_t dur_ms);

private:
  struct State
  {
    mixer_t mixer;
    envelope_t env;
    float freq[3];
    float freq_noise;
    float freq_env;
    uint8_t level[3];
    LevelMode level_mode[3];
  };

  enum Message : uint8_t
  {
    MsgReset,
    MsgChanFreqFull,
    MsgChanFreqFine,
    MsgChanFreqRough,
    MsgNoiseFreq,
    MsgMixer,
    MsgLevel,
    MsgEnvFreqFull,
    MsgEnvFreqFine,
    MsgEnvFreqRough,
    MsgEnvShape
  };

  struct msg_t
  {
    mixer_t mixer_settings = {};
    envelope_t env_settings = {};
    float fq = 0;
    uint8_t level = 0;
    LevelMode level_mode = LevelFixed;
    Message type = MsgReset;
    Channel chan = ChanA;
  };

  State _state;

  static const msg_t MSG_DEFAULT;

  static const size_t QUEUE_SIZE = 32;
  static const size_t QUEUE_ITEM_SIZE = sizeof(msg_t);
  QueueHandle_t _msg_queue;

  static const uint64_t MSG_PERIOD_US = 1000; // 1 ms
  TaskHandle_t _msg_task;
  esp_timer_handle_t _msg_timer;
  static void msg_handler(void *arg);
  void msgHandler();

  void enqueueMsg(Message type, msg_t msg = MSG_DEFAULT);
};

#endif // _YMPLAYER_H
