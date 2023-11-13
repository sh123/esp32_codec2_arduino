#include <DebugLog.h>
#include <RadioLib.h>
#include <Wire.h>
#include <driver/i2s.h>
#include <codec2.h>
#include <CircularBuffer.h>
#include <arduino-timer.h>

//#define ENABLE_LIGHT_SLEEP      true

// serial1
#define SERIAL_BAUD_RATE        115200  // USB serial baud rate

// ptt button
#define PTTBTN_PIN              39          // PTT button pin number
#define PTTBTN_GPIO_PIN         GPIO_NUM_39 // PTT button light wake up GPIO pin
volatile bool btn_pressed_ = false;         // true when button is held

// lora module

// lora module pinouts (SX1268)
#define LORA_RADIO_PIN_SS       SS  // NSS pin
#define LORA_RADIO_PIN_RST      27  // NRST pin
#define LORA_RADIO_PIN_A        12  // DIO1 pin
#define LORA_RADIO_PIN_B        14  // BUSY pin
#define LORA_RADIO_PIN_RXEN     32  // RX enable pin
#define LORA_RADIO_PIN_TXEN     33  // TX enable pin

// lora modulation parameters
#define LORA_RADIO_FREQ         433.775 // frequency (MHz)
#define LORA_RADIO_BW           125.0   // bandwidth (kHz)
#define LORA_RADIO_SF           9       // spreading factor
#define LORA_RADIO_CR           7       // coding rate
#define LORA_RADIO_PWR          2       // power in dbm (real is +10db if module has amplifier)
#define LORA_RADIO_CRC          1       // CRC bytes count
#define LORA_RADIO_EXPL         true    // comment out to use implicit mode (for spreading factor 6)
#define LORA_RADIO_SYNC         0x34    // sync word

// lora support config
#define LORA_RADIO_BUF_LEN      256   // packets buffer size
#define LORA_RADIO_QUEUE_LEN    512   // queues length

#define LORA_RADIO_TASK_RX_BIT  0x01  // lora task rx bit command
#define LORA_RADIO_TASK_TX_BIT  0x02  // lora task tx bit command

// lora task packet and packet index/size queues
CircularBuffer<uint8_t, LORA_RADIO_QUEUE_LEN> lora_radio_rx_queue_;
CircularBuffer<uint8_t, LORA_RADIO_QUEUE_LEN> lora_radio_rx_queue_index_;
CircularBuffer<uint8_t, LORA_RADIO_QUEUE_LEN> lora_radio_tx_queue_;
CircularBuffer<uint8_t, LORA_RADIO_QUEUE_LEN> lora_radio_tx_queue_index_;

// packet buffers
byte lora_radio_rx_buf_[LORA_RADIO_BUF_LEN];  // tx packet buffer
byte lora_radio_tx_buf_[LORA_RADIO_BUF_LEN];  // rx packet buffer

TaskHandle_t lora_task_;                // lora rx/tx task
volatile bool lora_enable_isr_ = true;  // true to enable rx isr, disabled on tx
SX1268 lora_radio_ = new Module(LORA_RADIO_PIN_SS, LORA_RADIO_PIN_A, LORA_RADIO_PIN_RST, LORA_RADIO_PIN_B);

// audio speaker pinouts (MAX98357A)
#define AUDIO_SPEAKER_BCLK      26      // i2s clock (SLK)
#define AUDIO_SPEAKER_LRC       13      // i2s word select (WS)
#define AUDIO_SPEAKER_DIN       25      // i2s data (SD)

// audio mic pinouts (INMP441)
#define AUDIO_MIC_SD            2       // i2s data
#define AUDIO_MIC_WS            15      // i2s word select
#define AUDIO_MIC_SCK           4       // i2s clock

// audio support
#define AUDIO_CODEC2_MODE       CODEC2_MODE_700C // codec2 mode
#define AUDIO_SAMPLE_RATE       8000    // audio sample rate
#define AUDIO_MAX_PACKET_SIZE   48      // maximum packet size, multiple audio frames are inside
#define AUDIO_TASK_PLAY_BIT     0x01    // task bit flag to start playback
#define AUDIO_TASK_RECORD_BIT   0x02    // task bit flag to start recording

// audio task
TaskHandle_t audio_task_;       // audio playback/record task

// codec2 
struct CODEC2* c2_;             // codec2 instance
int c2_samples_per_frame_;      // how many raw samples in one frame
int c2_bytes_per_frame_;        // how many bytes in encoded frame
int16_t *c2_samples_;           // buffer for raw samples
uint8_t *c2_bits_;              // buffer for encoded frame

// light sleep
#define LIGHT_SLEEP_DELAY_MS    5000  // how long to wait before engering light sleep
#define LIGHT_SLEEP_BITMASK     (uint64_t)(1 << LORA_RADIO_PIN_B) // bit mask for ext1 high pin wake up

Timer<1> light_sleep_timer_;             // light sleep timer
Timer<1>::Task light_sleep_timer_task_;  // light sleep timer task

void setup() {
  // setup logging
  LOG_SET_LEVEL(DebugLogLevel::LVL_INFO);
  //LOG_SET_LEVEL(DebugLogLevel::LVL_DEBUG);
  LOG_SET_OPTION(false, false, true);  // disable file, line, enable func

  // initialize serial
  Serial.begin(SERIAL_BAUD_RATE);
  while (!Serial);
  LOG_INFO("Board setup started");

  // setup lora radio
  int lora_radio_state = lora_radio_.begin(LORA_RADIO_FREQ, LORA_RADIO_BW, LORA_RADIO_SF, LORA_RADIO_CR, LORA_RADIO_SYNC, LORA_RADIO_PWR);
  if (lora_radio_state == RADIOLIB_ERR_NONE) {
    LOG_INFO("Lora radio initialized");
    lora_radio_.setCRC(LORA_RADIO_CRC);
    lora_radio_.setRfSwitchPins(LORA_RADIO_PIN_RXEN, LORA_RADIO_PIN_TXEN);
    lora_radio_.clearDio1Action();
#ifdef LORA_RADIO_EXPL 
    LOG_INFO("Using explicit header");
    lora_radio_.explicitHeader();
#else
    LOG_INFO("Using implicit header");
    lora_radio_.implicitHeader();
#endif
    lora_radio_.setDio1Action(onLoraDataAvailableIsr);
  } else {
    LOG_ERROR("Lora radio start failed:", lora_radio_state);
  }

  // setup ptt button
  pinMode(PTTBTN_PIN, INPUT);

  // setup speaker
  i2s_config_t i2s_speaker_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX),
    .sample_rate = AUDIO_SAMPLE_RATE,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
    .communication_format = (i2s_comm_format_t)(I2S_COMM_FORMAT_I2S | I2S_COMM_FORMAT_I2S_MSB),
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 8,
    .dma_buf_len = 1024,
    .use_apll=0,
    .tx_desc_auto_clear= true, 
    .fixed_mclk=-1    
  };
  i2s_pin_config_t i2s_speaker_pin_config = {
    .bck_io_num = AUDIO_SPEAKER_BCLK,
    .ws_io_num = AUDIO_SPEAKER_LRC,
    .data_out_num = AUDIO_SPEAKER_DIN,
    .data_in_num = I2S_PIN_NO_CHANGE
  };
  if (i2s_driver_install(I2S_NUM_0, &i2s_speaker_config, 0, NULL) != ESP_OK) {
    LOG_ERROR("Failed to install i2s speaker driver");
  }
  if (i2s_set_pin(I2S_NUM_0, &i2s_speaker_pin_config) != ESP_OK) {
    LOG_ERROR("Failed to set i2s speaker pins");
  }

  // setup microphone
  i2s_config_t i2s_mic_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = AUDIO_SAMPLE_RATE,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
    .channel_format = I2S_CHANNEL_FMT_ONLY_RIGHT,
    .communication_format = (i2s_comm_format_t)(I2S_COMM_FORMAT_I2S | I2S_COMM_FORMAT_I2S_MSB),
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 8,
    .dma_buf_len = 1024,
    .use_apll=0,
    .tx_desc_auto_clear= true,
    .fixed_mclk=-1
  };
  i2s_pin_config_t i2s_mic_pin_config = {
    .bck_io_num = AUDIO_MIC_SCK,
    .ws_io_num = AUDIO_MIC_WS,
    .data_out_num = I2S_PIN_NO_CHANGE,
    .data_in_num = AUDIO_MIC_SD
  };
  if (i2s_driver_install(I2S_NUM_1, &i2s_mic_config, 0, NULL) != ESP_OK) {
    LOG_ERROR("Failed to install i2s mic driver");
  }
  if (i2s_set_pin(I2S_NUM_1, &i2s_mic_pin_config) != ESP_OK) {
    LOG_ERROR("Failed to set i2s mic pins");
  }

  // start audio playback/recordign task
  xTaskCreate(&audio_task, "audio_task", 32000, NULL, 5, &audio_task_);

  // start lora receive/transmit task task
  xTaskCreate(&lora_task, "lora_task", 8000, NULL, 5, &lora_task_);
  int state = lora_radio_.startReceive();
  if (state != RADIOLIB_ERR_NONE) {
    LOG_ERROR("Receive start error:", state);
  }
  LOG_INFO("Board setup completed");

#ifdef ENABLE_LIGHT_SLEEP
  LOG_INFO("Light sleep is enabled");
#endif
  light_sleep_reset();
}

// ISR is called when new data is available from radio
ICACHE_RAM_ATTR void onLoraDataAvailableIsr() {
  if (!lora_enable_isr_) return;
  BaseType_t xHigherPriorityTaskWoken;
  uint32_t lora_rx_bit = LORA_RADIO_TASK_RX_BIT;
  // notify radio receive task on new data arrival
  xTaskNotifyFromISR(lora_task_, lora_rx_bit, eSetBits, &xHigherPriorityTaskWoken);
}

// start counting timer for light sleep
void light_sleep_reset() {
#ifdef ENABLE_LIGHT_SLEEP
  LOG_DEBUG("Reset light sleep");
  if (light_sleep_timer_task_ != NULL) light_sleep_timer_.cancel(light_sleep_timer_task_);
  light_sleep_timer_task_ = light_sleep_timer_.in(LIGHT_SLEEP_DELAY_MS, light_sleep);
#endif
}

// called by timer to enter light sleep
bool light_sleep(void *param) {
#ifdef ENABLE_LIGHT_SLEEP
  LOG_INFO("Entering light sleep");
  // wake up on ptt button or lora radio incoming data
  esp_sleep_enable_ext0_wakeup(GPIO_NUM_39, 0);
  esp_sleep_enable_ext1_wakeup(LIGHT_SLEEP_BITMASK, ESP_EXT1_WAKEUP_ANY_HIGH);
  delay(100);
  esp_light_sleep_start();
  LOG_INFO("Exiting light sleep");
#endif
  return false;
}

// lora trasmit receive task
void lora_task(void *param) {
  LOG_INFO("Lora task started");

  // wait for ISR notification, read data and send for audio processing
  while (true) {
    uint32_t lora_status_bits = 0;
    xTaskNotifyWaitIndexed(0, 0x00, ULONG_MAX, &lora_status_bits, portMAX_DELAY);

    LOG_DEBUG("Lora task bits", lora_status_bits);

    // lora rx
    if (lora_status_bits & LORA_RADIO_TASK_RX_BIT) {
      int packet_size = lora_radio_.getPacketLength();
      if (packet_size > 0) {
        int state = lora_radio_.readData(lora_radio_rx_buf_, packet_size);
        if (state == RADIOLIB_ERR_NONE) {
          // process packet
          LOG_DEBUG("Received packet, size", packet_size);
          if (packet_size % c2_bytes_per_frame_ == 0) {
            for (int i = 0; i < packet_size; i++) {
                lora_radio_rx_queue_.push(lora_radio_rx_buf_[i]);
            }
            lora_radio_rx_queue_index_.push(packet_size);
            uint32_t audio_play_bit = AUDIO_TASK_PLAY_BIT;
            xTaskNotify(audio_task_, audio_play_bit, eSetBits);
          } else {
            LOG_ERROR("Audio packet of wrong size, expected mod", c2_bytes_per_frame_);
          }
        } else {
          LOG_ERROR("Read data error: ", state);
        }
        // probably not needed, still in receive
        state = lora_radio_.startReceive();
        if (state != RADIOLIB_ERR_NONE) {
          LOG_ERROR("Start receive error: ", state);
        }
        light_sleep_reset();
      } // packet size > 0
    } // lora rx
    // lora tx data
    else if (lora_status_bits & LORA_RADIO_TASK_TX_BIT) {
      lora_enable_isr_ = false;
      // take packet by packet
      while (lora_radio_tx_queue_index_.size() > 0) {
        // take packet size and read it
        int tx_bytes_cnt = lora_radio_tx_queue_index_.shift();
        for (int i = 0; i < tx_bytes_cnt; i++) {
          lora_radio_tx_buf_[i] = lora_radio_tx_queue_.shift();
        }
        // transmit packet
        int lora_radio_state = lora_radio_.transmit(lora_radio_tx_buf_, tx_bytes_cnt);
        if (lora_radio_state != RADIOLIB_ERR_NONE) {
          LOG_ERROR("Lora radio transmit failed:", lora_radio_state);
        }
        LOG_DEBUG("Transmitted packet", tx_bytes_cnt);
        vTaskDelay(1);
      } // packet transmit loop
      
      // switch to receive after all transmitted
      int lora_radio_state = lora_radio_.startReceive();
      if (lora_radio_state != RADIOLIB_ERR_NONE) {
        LOG_ERROR("Start receive error: ", lora_radio_state);
      }
      lora_enable_isr_ = true;
      light_sleep_reset();
    } // lora tx
  }  // task loop
}

// audio record/playback encode/decode task
void audio_task(void *param) {
  LOG_INFO("Audio task started");

  // construct codec2
  c2_ = codec2_create(AUDIO_CODEC2_MODE);
  if (c2_ == NULL) {
    LOG_ERROR("Failed to create codec2");
    return;
  } else {
    c2_samples_per_frame_ = codec2_samples_per_frame(c2_);
    c2_bytes_per_frame_ = codec2_bytes_per_frame(c2_);
    c2_samples_ = (int16_t*)malloc(sizeof(int16_t) * c2_samples_per_frame_);
    c2_bits_ = (uint8_t*)malloc(sizeof(uint8_t) * c2_bytes_per_frame_);
    LOG_INFO("C2 initialized", c2_samples_per_frame_, c2_bytes_per_frame_);
  }

  // wait for data notification, decode frames and playback
  size_t bytes_read, bytes_written;
  while(true) {
    uint32_t audio_bits = 0;
    xTaskNotifyWaitIndexed(0, 0x00, ULONG_MAX, &audio_bits, portMAX_DELAY);

    LOG_DEBUG("Audio task bits", audio_bits);

    // audio rx-decode-playback
    if (audio_bits & AUDIO_TASK_PLAY_BIT) {
      LOG_DEBUG("Playing audio");
      // while rx frames are available and button is not pressed
      while (!btn_pressed_ && lora_radio_rx_queue_index_.size() > 0) {
        int packet_size = lora_radio_rx_queue_index_.shift();
        LOG_DEBUG("Playing packet", packet_size);
        // split by frame, decode and play
        for (int i = 0; i < packet_size; i++) {
          c2_bits_[i % c2_bytes_per_frame_] = lora_radio_rx_queue_.shift();
          if (i % c2_bytes_per_frame_ == c2_bytes_per_frame_ - 1) {
            codec2_decode(c2_, c2_samples_, c2_bits_);
            i2s_write(I2S_NUM_0, c2_samples_, sizeof(uint16_t) * c2_samples_per_frame_, &bytes_written, portMAX_DELAY);
            vTaskDelay(1);
          }
        }
      } // while rx data available
    } // audio decode playback
    // audio record-encode-tx
    else if (audio_bits & AUDIO_TASK_RECORD_BIT) {
      LOG_DEBUG("Recording audio");
      int packet_size = 0;
      // record while button is pressed
      while (btn_pressed_) {
        // send packet if enough audio encoded frames are accumulated
        if (packet_size + c2_bytes_per_frame_ > AUDIO_MAX_PACKET_SIZE) {
          LOG_DEBUG("Recorded packet", packet_size);
          lora_radio_tx_queue_index_.push(packet_size);
          uint32_t lora_tx_bits = LORA_RADIO_TASK_TX_BIT;
          xTaskNotify(lora_task_, lora_tx_bits, eSetBits);
          packet_size = 0;
        }
        // read and encode one sample
        size_t bytes_read;
        i2s_read(I2S_NUM_1, c2_samples_, sizeof(uint16_t) * c2_samples_per_frame_, &bytes_read, portMAX_DELAY);
        codec2_encode(c2_, c2_bits_, c2_samples_);
        for (int i = 0; i < c2_bytes_per_frame_; i++) {
          lora_radio_tx_queue_.push(c2_bits_[i]);
        }
        packet_size += c2_bytes_per_frame_;
        vTaskDelay(1);
      } // btn_pressed_
      // send remaining tail audio encoded samples
      if (packet_size > 0) {
          LOG_DEBUG("Recorded packet", packet_size);
          lora_radio_tx_queue_index_.push(packet_size);
          uint32_t lora_tx_bits = LORA_RADIO_TASK_TX_BIT;
          xTaskNotify(lora_task_, lora_tx_bits, eSetBits);        
          packet_size = 0;
      }
      vTaskDelay(1);
    } // task bit
  }
}

void loop() {
  // handle PTT button 
  if (digitalRead(PTTBTN_PIN) == LOW && !btn_pressed_) {
    LOG_DEBUG("PTT pushed, start TX");
    btn_pressed_ = true;
    // notify to start recording
    uint32_t audio_bits = AUDIO_TASK_RECORD_BIT;
    xTaskNotify(audio_task_, audio_bits, eSetBits);
  } else if (digitalRead(PTTBTN_PIN) == HIGH && btn_pressed_) {
    LOG_DEBUG("PTT released");
    btn_pressed_ = false;
  }
  light_sleep_timer_.tick();
  delay(50);
}

