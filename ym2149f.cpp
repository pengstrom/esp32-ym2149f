#include "ym2149f.h"
#include <math.h>
#include <rom/ets_sys.h>
#include <cassert>
#include <../../include/config.h>
#include <soc/soc.h>
#include <soc/gpio_reg.h>
#include <string.h>

char binary_string[12] = "0b0000_0000";

char *binstr(uint8_t x)
{
  for (size_t i = 0; i < 8; i++)
  {
    size_t idx = 10 - i;
    if (i >= 4)
    {
      idx--;
    }
    binary_string[idx] = (x & 1) ? '1' : '0';
    x >>= 1;
  }
  // binary_string[11] = '\0';
  return binary_string;
}

Ym2149::Ym2149(config_t cfg) : _config(cfg)
{
  // printf("Pins:\n");
  // printf("  reset_pin: %u\n", cfg.reset_pin);
  // printf("  bc1_pin:   %u\n", cfg.bc1_pin);
  // printf("  bdir_pin:  %u\n", cfg.bdir_pin);
  // for (size_t i = 0; i < DATA_BITS; i++)
  // {
  //   printf("  d1_pin:    %u\n", cfg.data_pins[i]);
  // }

  assert(GPIO_IS_VALID_OUTPUT_GPIO(Config::MASTER_CLOCK_PIN));

  // setupOutputPin(cfg.a8_pin);
  // setupOutputPin(cfg.a9_pin);

  setupBusControlPins(cfg.bc1_pin, cfg.bdir_pin);
  // setupOutputPin(cfg.bc2_pin);

  setupOutputPin(cfg.reset_pin);
  _reset_bit = 1 << cfg.reset_pin;

  // setupOutputPin(cfg.sel_pin);

  setupDataPins();

  // uint64_t gpio_mask = 0;
  // gpio_mask |= (1 << cfg.reset_pin);
  // gpio_mask |= (1 << cfg.bc1_pin);
  // gpio_mask |= (1 << cfg.bdir_pin);
  // gpio_mask |= (1 << Config::MASTER_CLOCK_PIN);
  // for (size_t i = 0; i < DATA_BITS; i++)
  // {
  //   gpio_mask |= (1 << cfg.data_pins[i]);
  // }
  // gpio_dump_io_configuration(stdout, gpio_mask);
}

Ym2149::~Ym2149()
{
}

void Ym2149::setBusControl(BusControl ctrl)
{
  switch (ctrl)
  {
  case Inactive:
    setBusInactive();
    break;
  case Address:
    setBusAddress();
    break;
  case Read:
    setBusRead();
    break;
  case Write:
    setBusWrite();
    break;
  }
}

void Ym2149::setBusInactive()
{
  REG_WRITE(GPIO_OUT_W1TC_REG, _bc1_bit | _bdir_bit);
  _bus = Inactive;
}

void Ym2149::setBusAddress()
{
  REG_WRITE(GPIO_OUT_W1TS_REG, _bc1_bit | _bdir_bit);
  _bus = Address;
}

void Ym2149::setBusRead()
{
  // Assumes inactive
  REG_WRITE(GPIO_OUT_W1TS_REG, _bc1_bit);
  _bus = Read;
}

void Ym2149::setBusWrite()
{
  // Assumes inactive
  REG_WRITE(GPIO_OUT_W1TS_REG, _bdir_bit);
  _bus = Write;
}

void Ym2149::reset()
{
  REG_WRITE(GPIO_OUT_W1TC_REG, _reset_bit);
  ets_delay_us(1); // Reset pulse
  REG_WRITE(GPIO_OUT_W1TS_REG, _reset_bit);
  ets_delay_us(1); // Reset delay
}

void Ym2149::writeChanFreqFull(Channel chan, float fq)
{
  switch (chan)
  {
  case ChanA:
    writeChanAFreqFull(fq);
    return;
  case ChanB:
    writeChanBFreqFull(fq);
    return;
  case ChanC:
    writeChanCFreqFull(fq);
    return;
  }
}

void Ym2149::writeChanFreqFine(Channel chan, float fq)
{
  switch (chan)
  {
  case ChanA:
    writeChanAFreqFine(fq);
    return;
  case ChanB:
    writeChanBFreqFine(fq);
    return;
  case ChanC:
    writeChanCFreqFine(fq);
    return;
  }
}

void Ym2149::writeChanFreqRough(Channel chan, float fq)
{
  switch (chan)
  {
  case ChanA:
    writeChanAFreqRough(fq);
    return;
  case ChanB:
    writeChanBFreqRough(fq);
    return;
  case ChanC:
    writeChanCFreqRough(fq);
    return;
  }
}

void Ym2149::writeAddress(Register reg)
{
  resetDataPins();

  uint8_t reg_addr = reg & 0b1111;
  // printf("Writing register address as %s\n", binstr(reg_addr));

  setBusAddress();
  setDataZeroBits(reg_addr);
  ets_delay_us(1); // Allow adress to set
  setBusInactive();
  ets_delay_us(1); // Address hold

  uint8_t data = readData();
  // printf("Read register address as %s\n", binstr(data));

  resetDataPins();
}

void Ym2149::writeData(uint8_t data)
{
  resetDataPins();

  // printf("Writing data: %s\n", binstr(data));
  // setupDataPinsWrite();

  setDataZeroBits(data);

  ets_delay_us(1); // Data setup
  setBusWrite();
  ets_delay_us(1); // Data signal
  setBusInactive();
  ets_delay_us(1); // Data hold

  // printf("Read back data immediately as %s\n", binstr(readData()));

  resetDataPins();
}

uint8_t Ym2149::readData()
{
  resetDataPins();

  setBusRead();
  ets_delay_us(1); // Data access

  uint32_t high_mask = REG_READ(GPIO_IN_REG);
  // char w0[12];
  // char w1[12];
  // char w2[12];
  // char w3[12];
  // strcpy(w0, binstr(high_mask & 0xFF));
  // strcpy(w1, binstr((high_mask >> 8) & 0xFF));
  // strcpy(w2, binstr((high_mask >> 8 * 2) & 0xFF));
  // strcpy(w3, binstr((high_mask >> 8 * 3) & 0xFF));
  // printf("GPIO_REG: %s %s %s %s\n", w3, w2, w1, w0);
  uint8_t out = 0;
  for (size_t i = 0; i < DATA_BITS; i++)
  {
    gpio_num_t pin = _config.data_pins[i];
    out |= ((high_mask >> pin) & 1) << i;
  }

  // for (size_t i = 0; i < DATA_BITS; i++)
  // {
  //   gpio_num_t pin = _config.data_pins[DATA_BITS - i - 1];
  //   uint32_t level = gpio_get_level(pin);

  //   out <<= 1;
  //   out |= level;
  // }

  setBusInactive();
  resetDataPins();
  ets_delay_us(1); // High-impedance delay

  return out;
}

void Ym2149::writeRegister(Register reg, uint8_t data)
{
  // char data_str[12];
  // char reg_str[12];
  // strcpy(data_str, binstr(data));
  // strcpy(reg_str, binstr(reg));
  // printf("Writing data %s to register %s\n", data_str, reg_str);
  writeAddress(reg);
  writeData(data);

  // printf("Read data properly from register: %s\n\n", binstr(readRegister(reg)));
}

uint8_t Ym2149::readRegister(Register reg)
{
  writeAddress(reg);
  return readData();
}

void Ym2149::writeChanAFreqFull(float fq)
{
  writeChanAFreqFine(fq);
  writeChanAFreqRough(fq);
}

void Ym2149::writeChanAFreqFine(float fq)
{
  uint16_t freq = getToneFreq(fq);
  freq &= 0xFF;

  writeRegister(FreqAFine, freq);
}

void Ym2149::writeChanAFreqRough(float fq)
{
  uint16_t freq = getToneFreq(fq);
  freq >>= DATA_BITS;
  freq &= 0xF;

  writeRegister(FreqARough, freq);
}

void Ym2149::writeChanBFreqFull(float fq)
{
  writeChanBFreqFine(fq);
  writeChanBFreqRough(fq);
}

void Ym2149::writeChanBFreqFine(float fq)
{
  uint16_t freq = getToneFreq(fq);
  freq &= 0xFF;

  writeRegister(FreqBFine, freq);
}

void Ym2149::writeChanBFreqRough(float fq)
{
  uint16_t freq = getToneFreq(fq);
  freq >>= DATA_BITS;
  freq &= 0xF;

  writeRegister(FreqBRough, freq);
}

void Ym2149::writeChanCFreqFull(float fq)
{
  writeChanCFreqFine(fq);
  writeChanCFreqRough(fq);
}

void Ym2149::writeChanCFreqFine(float fq)
{
  uint16_t freq = getToneFreq(fq);
  freq &= 0xFF;

  writeRegister(FreqCFine, freq);
}

void Ym2149::writeChanCFreqRough(float fq)
{
  uint16_t freq = getToneFreq(fq);
  freq >>= DATA_BITS;
  freq &= 0xF;

  writeRegister(FreqCRough, freq);
}

void Ym2149::writeNoiseFreq(float fq)
{
  auto freq = getToneFreq(fq);
  freq &= 0b11111;
  writeRegister(FreqNoise, freq);
}

void Ym2149::writeMixerSetting(mixer_t mix)
{
  uint8_t channels = (mix.chan[2] << 2) | (mix.chan[1] << 1) | mix.chan[0];
  uint8_t noise = (mix.noise[2] << 2) | (mix.noise[1] << 1) | mix.noise[0];
  uint8_t data = (noise << 3) | channels;
  data = ~data; // 0 means ON
  writeRegister(Mixer, data);
}

void Ym2149::writeLevel(Channel chan, LevelMode mode, uint8_t level)
{
  switch (chan)
  {
  case ChanA:
    writeLevelA(mode, level);
    return;
  case ChanB:
    writeLevelB(mode, level);
    return;
  case ChanC:
    writeLevelC(mode, level);
    return;
  }
}

void Ym2149::writeLevelA(LevelMode mode, uint8_t level)
{
  uint8_t data = (mode << 4) | (level & 0xF);
  writeRegister(LevelA, data);
}

void Ym2149::writeLevelB(LevelMode mode, uint8_t level)
{
  uint8_t data = (mode << 4) | (level & 0xF);
  writeRegister(LevelB, data);
}

void Ym2149::writeLevelC(LevelMode mode, uint8_t level)
{
  uint8_t data = (mode << 4) | (level & 0xF);
  writeRegister(LevelC, data);
}

void Ym2149::writeEnvelopeFreqFull(float fq)
{
  writeEnvelopeFreqFine(fq);
  writeEnvelopeFreqRough(fq);
}

void Ym2149::writeEnvelopeFreqFine(float fq)
{
  uint16_t freq = getEnvelopeFreq(fq);

  uint8_t freqLsb = freq & 0xFF;
  writeRegister(FreqEnvFine, freqLsb);
}

void Ym2149::writeEnvelopeFreqRough(float fq)
{
  uint16_t freq = getEnvelopeFreq(fq);

  uint8_t freqMsb = freq >> 8;
  writeRegister(FreqEnvRough, freqMsb);
}

void Ym2149::writeEnvelopeShape(envelope_t cfg = {})
{
  uint8_t data = (cfg.repeat << 3) | (cfg.attack << 2) | (cfg.alternate << 1) | cfg.hold;
  writeRegister(EnvShape, data);
}

void Ym2149::setupOutputPin(gpio_num_t pin)
{
  assert(GPIO_IS_VALID_OUTPUT_GPIO(pin));

  gpio_reset_pin(pin);

  gpio_set_direction(pin, GPIO_MODE_OUTPUT);
  gpio_set_drive_capability(pin, GPIO_DRIVE_CAP_3);
  gpio_pullup_dis(pin);
}

void Ym2149::setupBusControlPins(gpio_num_t bc1_pin, gpio_num_t bdir_pin)
{
  setupOutputPin(bc1_pin);
  setupOutputPin(bdir_pin);
  _bc1_bit = 1 << bc1_pin;
  _bdir_bit = 1 << bdir_pin;
  setBusInactive();
}

void Ym2149::setupDataPins()
{
  _data_pin_bits = 0;
  for (size_t i = 0; i < DATA_BITS; i++)
  {
    gpio_num_t pin = _config.data_pins[i];
    assert(GPIO_IS_VALID_OUTPUT_GPIO(pin));

    gpio_reset_pin(pin);

    gpio_set_direction(pin, GPIO_MODE_INPUT_OUTPUT_OD);
    gpio_set_drive_capability(pin, GPIO_DRIVE_CAP_3);
    gpio_pullup_en(pin);
    // gpio_set_level(pin, 1);
    _data_pin_bits |= (1 << pin);
  }

  resetDataPins();
}

void Ym2149::resetDataPins()
{
  REG_WRITE(GPIO_OUT_W1TS_REG, _data_pin_bits); // High-impedance, no source
}

void Ym2149::setDataZeroBits(uint8_t data)
{
  uint32_t pin_mask = 0;
  for (size_t i = 0; i < DATA_BITS; i++)
  {
    gpio_num_t pin = _config.data_pins[i];
    uint32_t current_data_bit = data & 1;             // current data bit value
    uint32_t data_bit_is_zero = current_data_bit ^ 1; // current data bit zero-ness
    pin_mask |= data_bit_is_zero << pin;              // Set 1 on each pin bit position for data bits that are 0
    data >>= 1;
  }

  REG_WRITE(GPIO_OUT_W1TC_REG, pin_mask);
}

uint16_t Ym2149::getToneFreq(float fq)
{
  float sel = _config.sel ? 2.0f : 1.0f;
  // Half master clock if sel active (low)
  return std::round(_config.master_clock / (16.0f * fq * sel));
}

uint16_t Ym2149::getEnvelopeFreq(float fq)
{
  float sel = _config.sel ? 2.0f : 1.0f;
  // Half master clock if sel active (low)
  return std::round(_config.master_clock / (256.0f * fq * sel));
}
