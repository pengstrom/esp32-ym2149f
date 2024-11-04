#if !defined(_YM2149_H)
#define _YM2149_H

#include <driver/gpio.h>

class Ym2149
{
public:
  static const size_t DATA_BITS = 8;
  static const uint64_t MAX_CLOCK = 2 * 1000 * 1000;
  static const uint64_t MAX_CLOCK_SEL = MAX_CLOCK * 2; // If sel pin low and clock halved

  static constexpr float MAX_FREQ = MAX_CLOCK / (16.0 * 1);
  static constexpr float MIN_FREQ = MAX_CLOCK / (16.0 * 0xFFF);

  static constexpr float MAX_NOISE_FREQ = MAX_FREQ;
  static constexpr float MIN_NOISE_FREQ = MAX_CLOCK / (16.0 * 0b11111);

  struct config_t
  {
    gpio_num_t data_pins[DATA_BITS];

    // upper address pins
    // Not used
    // gpio_num_t a8_pin;
    // gpio_num_t a9_pin;

    gpio_num_t reset_pin;

    // Bus control
    gpio_num_t bdir_pin;
    gpio_num_t bc1_pin;
    // Not used
    // Connected to Vcc
    // gpio_num_t bc2_pin;

    uint32_t master_clock;

    // Clock divider, low => 1/2 clock
    bool sel = false;
  };

  // Correspoding [bc1,bcdir] pin values
  enum BusControl : uint8_t
  {
    Inactive,
    Read,
    Write,
    Address,
  };

  enum Register : uint8_t
  {
    FreqAFine,
    FreqARough,
    FreqBFine,
    FreqBRough,
    FreqCFine,
    FreqCRough,
    FreqNoise,
    Mixer,
    LevelA,
    LevelB,
    LevelC,
    FreqEnvFine,
    FreqEnvRough,
    EnvShape,
    DataA,
    DataB
  };

  struct mixer_t
  {
    bool chan[3];
    bool noise[3];
  };

  enum LevelMode : uint8_t
  {
    LevelFixed = 0,
    LevelEnvelope = 1
  };

  struct envelope_t
  {
    bool hold = false;
    bool attack = false;
    bool alternate = false;
    bool repeat = false;
  };

  enum Channel : uint8_t
  {
    ChanA,
    ChanB,
    ChanC
  };

  Ym2149(config_t cfg);
  ~Ym2149();

  void reset();

  void writeChanFreqFull(Channel chan, float fq);
  void writeChanFreqFine(Channel chan, float fq);
  void writeChanFreqRough(Channel chan, float fq);

  void writeChanAFreqFull(float fq);
  void writeChanAFreqFine(float fq);
  void writeChanAFreqRough(float fq);

  void writeChanBFreqFull(float fq);
  void writeChanBFreqFine(float fq);
  void writeChanBFreqRough(float fq);

  void writeChanCFreqFull(float fq);
  void writeChanCFreqFine(float fq);
  void writeChanCFreqRough(float fq);

  void writeNoiseFreq(float fq);

  void writeMixerSetting(mixer_t mix);

  void writeLevel(Channel chan, LevelMode mode, uint8_t level);
  void writeLevelA(LevelMode mode, uint8_t level);
  void writeLevelB(LevelMode mode, uint8_t level);
  void writeLevelC(LevelMode mode, uint8_t level);

  void writeEnvelopeFreqFull(float fq);
  void writeEnvelopeFreqFine(float fq);
  void writeEnvelopeFreqRough(float fq);

  /// @brief The Envelope Generator further counts down the envelope frequency by 16, producing a 16-state per cycle envelope pattern as defined by its 4-bit counter output, E3 E2 El E0. The particular shape and cycle pattern of any desired envelope is accomplished by controlling the count pattern (count up/count down) of the 4-bit counter and by defining a single-cycle or repeat-cycle pattern
  /// @param cfg.repeat when set to logic “1”, the cycle pattern will be as defined by the Hold bit; when set to logic “0”, the envelope generator will reset to 0000 after one cycle and hold at that count.
  /// @param cfg.attack when set to logic “1": the envelope counter will count up (attack) from E3 E2 E1 E0=0000 to E3 E2 E1 E0=1111; when set to logic “0”, the envelope counter will count down (decay) from 1111 to 0000.
  /// @param cfg.alternate when set to logic “1”, the envelope counter reverses count direction (up-down) after each cycle.
  /// @param cfg.hold when set to logic “1”, limits the envelope to one cycle, holding the last count of the envelope counter (E3--E0=0000 or 1111, depending on whether the envelope counter was in a count-down or count-up mode, respectively).
  void writeEnvelopeShape(envelope_t cfg);

  void writeRegister(Register reg, uint8_t data);
  uint8_t readRegister(Register reg);

  void writeAddress(Register reg);
  void writeData(uint8_t data);

  uint8_t readData();

  void setBusControl(BusControl ctrl);
  void setBusInactive();
  void setBusAddress();
  void setBusRead();
  void setBusWrite();

protected:
  config_t _config;
  bool _sel = false;

  BusControl _bus = Inactive;
  uint32_t _bc1_bit;
  uint32_t _bdir_bit;

  uint32_t _reset_bit;

  uint32_t _data_pin_bits;

  // Tone or noise freq TP
  uint16_t getToneFreq(float fq);
  // Envelope freq tp
  uint16_t getEnvelopeFreq(float fq);

  // Clear pins by mask, to ground and set to low
  void setDataZeroBits(uint8_t data);

  void setupOutputPin(gpio_num_t pin);
  void setupBusControlPins(gpio_num_t bc1_pin, gpio_num_t bdir_pin);

  // Setup open drain bidirectional pins
  void setupDataPins();
  // Set data pins floating
  void resetDataPins();
};

#endif // _YM2149_H
