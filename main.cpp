/*
 * ICS-43434 Frequenzmesser - ESP32-Variante
 * Portiert aus: RaspberryPi/ics43434-frequenzmesser-zero2
 * (esp32/ics43434_frequenzmesser.ino)
 *
 * Liest das I2S-MEMS-Mikrofon (Adafruit ICS43434) ein, berechnet per FFT
 * die dominante Frequenz im Bereich MIN_FREQUENCY_HZ..MAX_FREQUENCY_HZ und
 * gibt sie zusammen mit RMS und Peak ueber die serielle Konsole aus.
 *
 * Das hier installierte PlatformIO-Espressif32-Paket bringt Arduino-ESP32
 * Core 2.0.17 mit (nicht Core 3.x) - die neue ESP_I2S.h/I2SClass-API mit
 * setPins()/I2S_MODE_STD existiert darin noch nicht. Deshalb wird hier der
 * Legacy-Treiber driver/i2s.h verwendet, mit der Konfiguration, die an
 * dieser Hardware bereits verifiziert wurde (Stereo-Empfang, Mikrofondaten
 * im linken Kanal / Sample-Index 0).
 *
 * Verkabelung (an dieser Hardware verifiziert):
 *   3V   -> 3V3
 *   GND  -> GND
 *   BCLK -> GPIO26
 *   WS   -> GPIO25
 *   DOUT -> GPIO33
 *   SEL  -> GND   (Mikrofonsignal liegt dann im linken Kanal, CHANNEL = 0)
 */

#include <Arduino.h>
#include <driver/i2s.h>
#include <math.h>

// ---------- I2S-Pins ----------
constexpr int I2S_BCLK_PIN = 26;
constexpr int I2S_WS_PIN = 25;
constexpr int I2S_DATA_PIN = 33;
constexpr i2s_port_t I2S_PORT = I2S_NUM_0;

// ---------- FFT / Frequenzmess-Parameter ----------
constexpr uint32_t SAMPLE_RATE = 48000;
constexpr size_t FFT_SIZE = 4096;
constexpr float MIN_FREQUENCY_HZ = 20.0f;
constexpr float MAX_FREQUENCY_HZ = 8000.0f;
constexpr float RMS_THRESHOLD = 250.0f;

// An Hardware verifiziert: SEL=GND -> Mikrofondaten im linken Kanal
// (gerader Sample-Index) eines Stereo-I2S-Streams.
constexpr size_t CHANNEL = 0;
constexpr size_t CHANNELS = 2;

constexpr int I2S_DMA_BUF_COUNT = 8;
constexpr int I2S_DMA_BUF_LEN = 256; // Woerter (L+R gemischt) pro DMA-Buffer

int32_t rawAudio[FFT_SIZE * CHANNELS];
float realPart[FFT_SIZE];
float imaginaryPart[FFT_SIZE];
float windowValues[FFT_SIZE];

void setupI2SMic() {
  i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = SAMPLE_RATE,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,
    .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
    .communication_format = (i2s_comm_format_t)(I2S_COMM_FORMAT_STAND_I2S),
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = I2S_DMA_BUF_COUNT,
    .dma_buf_len = I2S_DMA_BUF_LEN,
    .use_apll = false,
    .tx_desc_auto_clear = false,
    .fixed_mclk = 0
  };

  i2s_pin_config_t pin_config = {
    .bck_io_num = I2S_BCLK_PIN,
    .ws_io_num = I2S_WS_PIN,
    .data_out_num = I2S_PIN_NO_CHANGE,
    .data_in_num = I2S_DATA_PIN
  };

  esp_err_t err = i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL);
  if (err != ESP_OK) {
    Serial.printf("Fehler bei i2s_driver_install: %d\n", err);
    while (true) { delay(1000); }
  }

  err = i2s_set_pin(I2S_PORT, &pin_config);
  if (err != ESP_OK) {
    Serial.printf("Fehler bei i2s_set_pin: %d\n", err);
    while (true) { delay(1000); }
  }

  i2s_zero_dma_buffer(I2S_PORT);
}

void readAudioBlock() {
  const size_t expectedBytes = sizeof(rawAudio);
  size_t receivedBytes = 0;

  while (receivedBytes < expectedBytes) {
    size_t bytesRead = 0;
    esp_err_t err = i2s_read(I2S_PORT,
                              reinterpret_cast<uint8_t*>(rawAudio) + receivedBytes,
                              expectedBytes - receivedBytes,
                              &bytesRead, portMAX_DELAY);
    if (err != ESP_OK || bytesRead == 0) {
      Serial.println("Unvollstaendiger FFT-Block.");
      return;
    }
    receivedBytes += bytesRead;
  }
}

float calculateRms() {
  double sum = 0.0;

  for (size_t index = 0; index < FFT_SIZE; ++index) {
    const float sample = static_cast<float>(rawAudio[index * CHANNELS + CHANNEL] >> 8);
    sum += static_cast<double>(sample) * sample;
  }

  return sqrtf(static_cast<float>(sum / FFT_SIZE));
}

void prepareSamples() {
  double mean = 0.0;

  for (size_t index = 0; index < FFT_SIZE; ++index) {
    mean += static_cast<double>(rawAudio[index * CHANNELS + CHANNEL] >> 8);
  }
  mean /= FFT_SIZE;

  for (size_t index = 0; index < FFT_SIZE; ++index) {
    const float sample = static_cast<float>(rawAudio[index * CHANNELS + CHANNEL] >> 8) - mean;
    realPart[index] = sample * windowValues[index];
    imaginaryPart[index] = 0.0f;
  }
}

void calculateFft() {
  // Bit-reversal permutation.
  for (size_t index = 1, reversed = 0; index < FFT_SIZE; ++index) {
    size_t bit = FFT_SIZE >> 1;
    for (; reversed & bit; bit >>= 1) {
      reversed ^= bit;
    }
    reversed ^= bit;

    if (index < reversed) {
      float temp = realPart[index];
      realPart[index] = realPart[reversed];
      realPart[reversed] = temp;

      temp = imaginaryPart[index];
      imaginaryPart[index] = imaginaryPart[reversed];
      imaginaryPart[reversed] = temp;
    }
  }

  for (size_t length = 2; length <= FFT_SIZE; length <<= 1) {
    const float angle = -2.0f * PI / static_cast<float>(length);
    const float sine = sinf(angle);
    const float cosine = cosf(angle);

    for (size_t start = 0; start < FFT_SIZE; start += length) {
      float currentCosine = 1.0f;
      float currentSine = 0.0f;
      const size_t halfLength = length >> 1;

      for (size_t offset = 0; offset < halfLength; ++offset) {
        const size_t even = start + offset;
        const size_t odd = even + halfLength;
        const float multipliedReal = currentCosine * realPart[odd] - currentSine * imaginaryPart[odd];
        const float multipliedImaginary = currentCosine * imaginaryPart[odd] + currentSine * realPart[odd];

        realPart[odd] = realPart[even] - multipliedReal;
        imaginaryPart[odd] = imaginaryPart[even] - multipliedImaginary;
        realPart[even] += multipliedReal;
        imaginaryPart[even] += multipliedImaginary;

        const float nextCosine = currentCosine * cosine - currentSine * sine;
        currentSine = currentCosine * sine + currentSine * cosine;
        currentCosine = nextCosine;
      }
    }
  }
}

// Timecode als HH:MM:SS.mmm seit dem Start des ESP32 (kein RTC/WLAN noetig).
void printTimecode() {
  const uint32_t ms = millis();
  const uint32_t hours = ms / 3600000UL;
  const uint32_t minutes = (ms / 60000UL) % 60;
  const uint32_t seconds = (ms / 1000UL) % 60;
  const uint32_t millisPart = ms % 1000UL;
  Serial.printf("[%02lu:%02lu:%02lu.%03lu] ",
                (unsigned long)hours, (unsigned long)minutes,
                (unsigned long)seconds, (unsigned long)millisPart);
}

void printDominantFrequency(float rms) {
  size_t firstBin = static_cast<size_t>(ceilf(MIN_FREQUENCY_HZ * FFT_SIZE / SAMPLE_RATE));
  size_t lastBin = static_cast<size_t>(floorf(MAX_FREQUENCY_HZ * FFT_SIZE / SAMPLE_RATE));
  size_t peakBin = firstBin;
  float peakMagnitude = 0.0f;

  for (size_t bin = firstBin; bin <= lastBin; ++bin) {
    const float magnitude = sqrtf(
        realPart[bin] * realPart[bin] + imaginaryPart[bin] * imaginaryPart[bin]);
    if (magnitude > peakMagnitude) {
      peakMagnitude = magnitude;
      peakBin = bin;
    }
  }

  const float frequency = static_cast<float>(peakBin) * SAMPLE_RATE / FFT_SIZE;

  printTimecode();
  if (rms < RMS_THRESHOLD) {
    Serial.printf("Frequenz:      --- Hz | RMS: %10.1f | Peak: %12.0f\n", rms, peakMagnitude);
  } else {
    Serial.printf("Frequenz: %8.1f Hz | RMS: %10.1f | Peak: %12.0f\n", frequency, rms, peakMagnitude);
  }
}

void setup() {
  Serial.begin(115200);
  delay(500);

  for (size_t index = 0; index < FFT_SIZE; ++index) {
    windowValues[index] = 0.5f * (1.0f - cosf(2.0f * PI * index / (FFT_SIZE - 1)));
  }

  setupI2SMic();

  Serial.println();
  Serial.println("======================================");
  Serial.println(" ICS-43434 LIVE FFT");
  Serial.println(" ESP32");
  Serial.println("======================================");
  Serial.printf("Sample Rate : %lu Hz\n", SAMPLE_RATE);
  Serial.printf("FFT         : %u\n", FFT_SIZE);
  Serial.printf("Aufloesung  : %.5f Hz\n", static_cast<float>(SAMPLE_RATE) / FFT_SIZE);
  Serial.printf("I2S-Pins    : BCLK %d, WS %d, DATA %d\n", I2S_BCLK_PIN, I2S_WS_PIN, I2S_DATA_PIN);
  Serial.println("Kanal       : links");
  Serial.println("Baudrate    : 115200");
  Serial.println();
}

void loop() {
  readAudioBlock();

  const float rms = calculateRms();
  prepareSamples();
  calculateFft();
  printDominantFrequency(rms);
}
