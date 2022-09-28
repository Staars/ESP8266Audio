/*
  AudioGeneratorI2SMicro
  Audio output generator for an I2S Microphone
  
  Copyright (C) 2022  Earle F. Philhower, III

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifdef ESP32
#include "AudioGeneratorI2SMicro.h"

#include <driver/i2s.h>

AudioGeneratorI2SMicro::AudioGeneratorI2SMicro(int port, uint32_t freq)
{
  running = false;
  output = NULL;
  buffSize = 1024;
  buff = NULL;
  buffPtr = 0;
  buffLen = 0;

  bitsPerSample = 16; //only 16 bit for now
  sampleRate = freq;

  portNo = port;
}

bool AudioGeneratorI2SMicro::SetPinout()
{
    i2s_pin_config_t pins = {
        .mck_io_num = mclkPin,
        .bck_io_num = bclkPin,
        .ws_io_num = wclkPin,
        .data_out_num = doutPin,
        .data_in_num = dinPin
      };

    i2s_set_pin((i2s_port_t)portNo, &pins);
    return true;
}

bool AudioGeneratorI2SMicro::SetPinout(int bclk, int wclk, int din)
{
  bclkPin = bclk;
  wclkPin = wclk;
  doutPin = I2S_PIN_NO_CHANGE;
  mclkPin = I2S_PIN_NO_CHANGE;
  dinPin = din;

  if (running)
    return SetPinout();

  return true;
}

AudioGeneratorI2SMicro::~AudioGeneratorI2SMicro()
{
  i2s_driver_uninstall((i2s_port_t)portNo);
  free(buff);
  buff = NULL;
}

bool AudioGeneratorI2SMicro::stop()
{
  if (!running) return true;
  running = false;
  free(buff);
  buff = NULL;
  output->stop();
  i2s_stop((i2s_port_t)portNo);
  return running;
}

bool AudioGeneratorI2SMicro::isRunning()
{
  return running;
}


// Handle buffered reading, reload each time we run out of data
bool AudioGeneratorI2SMicro::GetBufferedData(int bytes, void *dest)
{
  if (!running) return false; // Nothing to do here!
  uint8_t *p = reinterpret_cast<uint8_t*>(dest);
  while (bytes--) {
    // Potentially load next batch of data...
    if (buffPtr >= buffLen) {
      buffPtr = 0;
      buffLen = 0;
      i2s_read((i2s_port_t)portNo, (char *)buff, buffSize, (size_t*)&buffLen, (100 / portTICK_RATE_MS));
    }
    if (buffPtr >= buffLen)
      return false; // No data left!
    *(p++) = buff[buffPtr++];
  }
  return true;
}

bool AudioGeneratorI2SMicro::loop()
{
  if (!running) goto done; // Nothing to do here!

  // First, try and push in the stored sample.  If we can't, then punt and try later
  if (!output->ConsumeSample(lastSample)) goto done; // Can't send, but no error detected

  // Try and stuff the buffer one sample at a time
  do
  {
    if (bitsPerSample == 8) {
      uint8_t l, r;
      if (!GetBufferedData(1, &l)) stop();
      lastSample[AudioOutput::LEFTCHANNEL] = l;

    } else if (bitsPerSample == 16) {
      if (!GetBufferedData(2, &lastSample[AudioOutput::LEFTCHANNEL])) stop();
    }
  } while (running && output->ConsumeSample(lastSample));

done:
  // no file loop
  output->loop();

  return running;
}


bool AudioGeneratorI2SMicro::begin(AudioFileSource *source, AudioOutput *output)
{
  // There is no source file, so pass NULL or simply ignore it

  if (!output) {
    Serial.printf_P(PSTR("AudioGeneratorI2SMicro::begin: invalid output\n"));
    return false;
  }

  if (!output->SetRate( sampleRate )) {
    Serial.printf_P(PSTR("AudioGeneratorI2SMicro::begin: failed to SetRate in output\n"));
    return false;
  }
  if (!output->SetBitsPerSample( bitsPerSample )) {
    Serial.printf_P(PSTR("AudioGeneratorI2SMicro::begin: failed to SetBitsPerSample in output\n"));
    return false;
  }
  if (!output->SetChannels( 1 )) {
    Serial.printf_P(PSTR("AudioGeneratorI2SMicro::begin: failed to SetChannels in output\n"));
    return false;
  }
  if (!output->begin()) {
    Serial.printf_P(PSTR("AudioGeneratorI2SMicro::begin: output's begin did not return true\n"));
    return false;
  }

  esp_err_t err = ESP_OK;

  i2s_config_t i2s_config = {
      .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_TX),
      .sample_rate = sampleRate,
      .bits_per_sample = (i2s_bits_per_sample_t)bitsPerSample,
      .channel_format = I2S_CHANNEL_FMT_ONLY_RIGHT,
      .communication_format = I2S_COMM_FORMAT_STAND_I2S,
      .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
      .dma_buf_count = 2,
      .dma_buf_len = (int)buffSize,
      .use_apll = 0,
      .tx_desc_auto_clear     = true,
      .fixed_mclk             = 12000000,
      .mclk_multiple          = I2S_MCLK_MULTIPLE_DEFAULT,
      .bits_per_chan          = I2S_BITS_PER_CHAN_16BIT
  };

  err += i2s_driver_install((i2s_port_t)portNo, &i2s_config, 0, NULL);

  SetPinout();
  
  err += i2s_set_clk((i2s_port_t)portNo, sampleRate, (i2s_bits_per_sample_t)bitsPerSample, I2S_CHANNEL_MONO);

  running = true;

  return true;
}

#endif //ESP32
