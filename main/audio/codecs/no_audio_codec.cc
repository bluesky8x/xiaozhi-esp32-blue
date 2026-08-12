#include "no_audio_codec.h"

#include <esp_log.h>
#include <esp_timer.h>
#include <cmath>
#include <cstdint>
#include <cstring>

#ifndef AUDIO_MIC_DEBUG_LOG
#define AUDIO_MIC_DEBUG_LOG 0
#endif

#ifndef AUDIO_MIC_SHIFT_BITS
#define AUDIO_MIC_SHIFT_BITS 12
#endif

#ifndef AUDIO_MIC_SOFT_LIMIT
#define AUDIO_MIC_SOFT_LIMIT 0
#endif

#ifndef AUDIO_MIC_AGC_ENABLE
#define AUDIO_MIC_AGC_ENABLE 0
#endif

#ifndef AUDIO_MIC_AGC_TARGET_PEAK
#define AUDIO_MIC_AGC_TARGET_PEAK 4500
#endif

#ifndef AUDIO_MIC_AGC_MAX_GAIN
#define AUDIO_MIC_AGC_MAX_GAIN 8.0f
#endif

#ifndef AUDIO_MIC_AGC_MIN_GAIN
#define AUDIO_MIC_AGC_MIN_GAIN 0.35f
#endif

#ifndef AUDIO_MIC_AGC_ATTACK
#define AUDIO_MIC_AGC_ATTACK 0.70f
#endif

#ifndef AUDIO_MIC_AGC_RELEASE
#define AUDIO_MIC_AGC_RELEASE 0.05f
#endif

#ifndef AUDIO_MIC_COMPRESS_KNEE
#define AUDIO_MIC_COMPRESS_KNEE 10000
#endif

#ifndef AUDIO_MIC_COMPRESS_RATIO
#define AUDIO_MIC_COMPRESS_RATIO 4
#endif

#define TAG "NoAudioCodec"

namespace {

#if AUDIO_MIC_AGC_ENABLE
float s_agc_gain = 1.0f;
#endif

int32_t SoftCompressSample(int32_t sample) {
#if AUDIO_MIC_SOFT_LIMIT > 0 && AUDIO_MIC_COMPRESS_KNEE > 0
    const int32_t abs_sample = sample >= 0 ? sample : -sample;
    if (abs_sample <= AUDIO_MIC_COMPRESS_KNEE) {
        return sample;
    }
    int32_t compressed = AUDIO_MIC_COMPRESS_KNEE + (abs_sample - AUDIO_MIC_COMPRESS_KNEE) / AUDIO_MIC_COMPRESS_RATIO;
    if (compressed > AUDIO_MIC_SOFT_LIMIT) {
        compressed = AUDIO_MIC_SOFT_LIMIT;
    }
    return sample >= 0 ? compressed : -compressed;
#else
    return sample;
#endif
}

void SoftLimitPcm(int16_t* dest, int samples) {
#if AUDIO_MIC_SOFT_LIMIT > 0 && !AUDIO_MIC_AGC_ENABLE
    for (int i = 0; i < samples; i++) {
        if (dest[i] > AUDIO_MIC_SOFT_LIMIT) {
            dest[i] = AUDIO_MIC_SOFT_LIMIT;
        } else if (dest[i] < -AUDIO_MIC_SOFT_LIMIT) {
            dest[i] = static_cast<int16_t>(-AUDIO_MIC_SOFT_LIMIT);
        }
    }
#endif
}

void ApplyInputGain(int16_t* dest, int samples, float gain) {
    if (samples <= 0 || gain <= 0.0f || gain == 1.0f) {
        return;
    }
    for (int i = 0; i < samples; i++) {
        const float amplified = static_cast<float>(dest[i]) * gain;
        dest[i] = (amplified > INT16_MAX)   ? INT16_MAX
                  : (amplified < -INT16_MAX) ? static_cast<int16_t>(-INT16_MAX)
                                             : static_cast<int16_t>(amplified);
    }
}

void ProcessMicSamples(const int32_t* raw_i2s, int16_t* dest, int samples, float base_gain,
                       int32_t* raw_peak_out, int32_t* pre_peak_out, int32_t* post_peak_out,
                       float* effective_gain_out) {
    int32_t raw_peak = 0;
    int32_t pre_peak = 0;

    for (int i = 0; i < samples; i++) {
        const int32_t raw_abs = raw_i2s[i] >= 0 ? raw_i2s[i] : -raw_i2s[i];
        if (raw_abs > raw_peak) {
            raw_peak = raw_abs;
        }
        int32_t value = raw_i2s[i] >> AUDIO_MIC_SHIFT_BITS;
        if (value > INT16_MAX) {
            value = INT16_MAX;
        } else if (value < -INT16_MAX) {
            value = static_cast<int16_t>(-INT16_MAX);
        }
        const int32_t pre_abs = value >= 0 ? value : -value;
        if (pre_abs > pre_peak) {
            pre_peak = pre_abs;
        }
        dest[i] = static_cast<int16_t>(value);
    }

    float effective_gain = base_gain;
#if AUDIO_MIC_AGC_ENABLE
    {
        const float ref_peak = static_cast<float>(pre_peak > 32 ? pre_peak : 32);
        float block_gain = static_cast<float>(AUDIO_MIC_AGC_TARGET_PEAK) / ref_peak;
        if (block_gain > AUDIO_MIC_AGC_MAX_GAIN) {
            block_gain = AUDIO_MIC_AGC_MAX_GAIN;
        }
        if (block_gain < AUDIO_MIC_AGC_MIN_GAIN) {
            block_gain = AUDIO_MIC_AGC_MIN_GAIN;
        }
        if (block_gain < s_agc_gain) {
            s_agc_gain = s_agc_gain * (1.0f - AUDIO_MIC_AGC_ATTACK) + block_gain * AUDIO_MIC_AGC_ATTACK;
        } else {
            s_agc_gain = s_agc_gain * (1.0f - AUDIO_MIC_AGC_RELEASE) + block_gain * AUDIO_MIC_AGC_RELEASE;
        }
        effective_gain = base_gain * s_agc_gain;
    }

    for (int i = 0; i < samples; i++) {
        float amplified = static_cast<float>(dest[i]) * effective_gain;
        int32_t sample = static_cast<int32_t>(amplified);
        if (sample > INT16_MAX) {
            sample = INT16_MAX;
        } else if (sample < -INT16_MAX) {
            sample = static_cast<int16_t>(-INT16_MAX);
        }
        sample = SoftCompressSample(sample);
#if AUDIO_MIC_SOFT_LIMIT > 0
        if (sample > AUDIO_MIC_SOFT_LIMIT) {
            sample = AUDIO_MIC_SOFT_LIMIT;
        } else if (sample < -AUDIO_MIC_SOFT_LIMIT) {
            sample = -AUDIO_MIC_SOFT_LIMIT;
        }
#endif
        dest[i] = static_cast<int16_t>(sample);
    }
#else
    ApplyInputGain(dest, samples, base_gain);
    SoftLimitPcm(dest, samples);
#endif

    if (effective_gain_out != nullptr) {
        *effective_gain_out = effective_gain;
    }
    if (raw_peak_out != nullptr) {
        *raw_peak_out = raw_peak;
    }
    if (pre_peak_out != nullptr) {
        *pre_peak_out = pre_peak;
    }
    if (post_peak_out != nullptr) {
        int32_t post_peak = 0;
        for (int i = 0; i < samples; i++) {
            const int32_t post_abs = dest[i] >= 0 ? dest[i] : -dest[i];
            if (post_abs > post_peak) {
                post_peak = post_abs;
            }
        }
        *post_peak_out = post_peak;
    }
}

#if AUDIO_MIC_DEBUG_LOG
void LogMicLevels(int samples, int32_t raw_peak, int32_t pre_gain_peak, int32_t post_gain_peak, float gain,
                  bool read_failed) {
    static int64_t last_log_us = 0;
    const int64_t now_us = esp_timer_get_time();
    if (now_us - last_log_us < 1000000) {
        return;
    }
    last_log_us = now_us;

    if (read_failed || samples <= 0) {
        ESP_LOGW(TAG,
                 "Mic: I2S read failed or 0 samples (duplex needs TX clock; check INMP441 WS/BCLK/SD, L/R=GND)");
        return;
    }

    const char* hint = "OK";
    if (post_gain_peak >= 32000) {
        hint = "HOT — spike/noise (AGC limiting)";
    } else if (post_gain_peak < 350) {
        hint = "quiet idle";
    } else if (post_gain_peak < 3000) {
        hint = "weak";
    }

#if AUDIO_MIC_AGC_ENABLE
    ESP_LOGI(TAG, "Mic: raw=%ld pre=%ld post=%ld base=%.1f agc=%.2f (%s)", static_cast<long>(raw_peak),
             static_cast<long>(pre_gain_peak), static_cast<long>(post_gain_peak), gain, s_agc_gain, hint);
#else
    ESP_LOGI(TAG, "Mic: raw=%ld pre=%ld post=%ld gain=%.1f (%s)", static_cast<long>(raw_peak),
             static_cast<long>(pre_gain_peak), static_cast<long>(post_gain_peak), gain, hint);
#endif
}
#endif

}  // namespace

NoAudioCodec::~NoAudioCodec() {
    if (rx_handle_ != nullptr) {
        ESP_ERROR_CHECK(i2s_channel_disable(rx_handle_));
    }
    if (tx_handle_ != nullptr) {
        ESP_ERROR_CHECK(i2s_channel_disable(tx_handle_));
    }
}

NoAudioCodecDuplex::NoAudioCodecDuplex(int input_sample_rate, int output_sample_rate, gpio_num_t bclk, gpio_num_t ws, gpio_num_t dout, gpio_num_t din) {
    duplex_ = true;
    input_sample_rate_ = input_sample_rate;
    output_sample_rate_ = output_sample_rate;

    i2s_chan_config_t chan_cfg = {
        .id = XIAOZHI_I2S_PORT(0),
        .role = I2S_ROLE_MASTER,
        .dma_desc_num = AUDIO_CODEC_DMA_DESC_NUM,
        .dma_frame_num = AUDIO_CODEC_DMA_FRAME_NUM,
        .auto_clear_after_cb = true,
        .auto_clear_before_cb = false,
        .intr_priority = 0,
    };
    ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg, &tx_handle_, &rx_handle_));

    i2s_std_config_t std_cfg = {
        .clk_cfg = {
            .sample_rate_hz = (uint32_t)output_sample_rate_,
            .clk_src = I2S_CLK_SRC_DEFAULT,
            .mclk_multiple = I2S_MCLK_MULTIPLE_256,
			#ifdef   I2S_HW_VERSION_2
				.ext_clk_freq_hz = 0,
			#endif

        },
        .slot_cfg = {
            .data_bit_width = I2S_DATA_BIT_WIDTH_32BIT,
            .slot_bit_width = I2S_SLOT_BIT_WIDTH_AUTO,
            .slot_mode = I2S_SLOT_MODE_MONO,
            .slot_mask = I2S_STD_SLOT_LEFT,
            .ws_width = I2S_DATA_BIT_WIDTH_32BIT,
            .ws_pol = false,
            .bit_shift = true,
            #ifdef   I2S_HW_VERSION_2
                .left_align = true,
                .big_endian = false,
                .bit_order_lsb = false
            #endif

        },
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,
            .bclk = bclk,
            .ws = ws,
            .dout = dout,
            .din = din,
            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv = false
            }
        }
    };
    ESP_ERROR_CHECK(i2s_channel_init_std_mode(tx_handle_, &std_cfg));
    ESP_ERROR_CHECK(i2s_channel_init_std_mode(rx_handle_, &std_cfg));
    ESP_LOGI(TAG, "Duplex channels created");
}


NoAudioCodecSimplex::NoAudioCodecSimplex(int input_sample_rate, int output_sample_rate, gpio_num_t spk_bclk, gpio_num_t spk_ws, gpio_num_t spk_dout, gpio_num_t mic_sck, gpio_num_t mic_ws, gpio_num_t mic_din) {
    duplex_ = false;
    input_sample_rate_ = input_sample_rate;
    output_sample_rate_ = output_sample_rate;

    // Create a new channel for speaker
    i2s_chan_config_t chan_cfg = {
        .id = XIAOZHI_I2S_PORT(0),
        .role = I2S_ROLE_MASTER,
        .dma_desc_num = AUDIO_CODEC_DMA_DESC_NUM,
        .dma_frame_num = AUDIO_CODEC_DMA_FRAME_NUM,
        .auto_clear_after_cb = true,
        .auto_clear_before_cb = false,
        .intr_priority = 0,
    };
    ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg, &tx_handle_, nullptr));

    i2s_std_config_t std_cfg = {
        .clk_cfg = {
            .sample_rate_hz = (uint32_t)output_sample_rate_,
            .clk_src = I2S_CLK_SRC_DEFAULT,
            .mclk_multiple = I2S_MCLK_MULTIPLE_256,
			#ifdef   I2S_HW_VERSION_2
				.ext_clk_freq_hz = 0,
			#endif

        },
        .slot_cfg = {
            .data_bit_width = I2S_DATA_BIT_WIDTH_32BIT,
            .slot_bit_width = I2S_SLOT_BIT_WIDTH_AUTO,
            .slot_mode = I2S_SLOT_MODE_MONO,
            .slot_mask = I2S_STD_SLOT_LEFT,
            .ws_width = I2S_DATA_BIT_WIDTH_32BIT,
            .ws_pol = false,
            .bit_shift = true,
            #ifdef   I2S_HW_VERSION_2
                .left_align = true,
                .big_endian = false,
                .bit_order_lsb = false
            #endif

        },
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,
            .bclk = spk_bclk,
            .ws = spk_ws,
            .dout = spk_dout,
            .din = I2S_GPIO_UNUSED,
            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv = false
            }
        }
    };
    ESP_ERROR_CHECK(i2s_channel_init_std_mode(tx_handle_, &std_cfg));

    // Create a new channel for MIC
    chan_cfg.id = XIAOZHI_I2S_PORT(1);
    ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg, nullptr, &rx_handle_));
    std_cfg.clk_cfg.sample_rate_hz = (uint32_t)input_sample_rate_;
    std_cfg.gpio_cfg.bclk = mic_sck;
    std_cfg.gpio_cfg.ws = mic_ws;
    std_cfg.gpio_cfg.dout = I2S_GPIO_UNUSED;
    std_cfg.gpio_cfg.din = mic_din;
    ESP_ERROR_CHECK(i2s_channel_init_std_mode(rx_handle_, &std_cfg));
    ESP_LOGI(TAG, "Simplex channels created");
}

NoAudioCodecSimplex::NoAudioCodecSimplex(int input_sample_rate, int output_sample_rate, gpio_num_t spk_bclk, gpio_num_t spk_ws, gpio_num_t spk_dout, i2s_std_slot_mask_t spk_slot_mask, gpio_num_t mic_sck, gpio_num_t mic_ws, gpio_num_t mic_din, i2s_std_slot_mask_t mic_slot_mask){
    duplex_ = false;
    input_sample_rate_ = input_sample_rate;
    output_sample_rate_ = output_sample_rate;

    // Create a new channel for speaker
    i2s_chan_config_t chan_cfg = {
        .id = XIAOZHI_I2S_PORT(0),
        .role = I2S_ROLE_MASTER,
        .dma_desc_num = AUDIO_CODEC_DMA_DESC_NUM,
        .dma_frame_num = AUDIO_CODEC_DMA_FRAME_NUM,
        .auto_clear_after_cb = true,
        .auto_clear_before_cb = false,
        .intr_priority = 0,
    };
    ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg, &tx_handle_, nullptr));

    i2s_std_config_t std_cfg = {
        .clk_cfg = {
            .sample_rate_hz = (uint32_t)output_sample_rate_,
            .clk_src = I2S_CLK_SRC_DEFAULT,
            .mclk_multiple = I2S_MCLK_MULTIPLE_256,
			#ifdef   I2S_HW_VERSION_2
				.ext_clk_freq_hz = 0,
			#endif

        },
        .slot_cfg = {
            .data_bit_width = I2S_DATA_BIT_WIDTH_32BIT,
            .slot_bit_width = I2S_SLOT_BIT_WIDTH_AUTO,
            .slot_mode = I2S_SLOT_MODE_MONO,
            .slot_mask = spk_slot_mask,
            .ws_width = I2S_DATA_BIT_WIDTH_32BIT,
            .ws_pol = false,
            .bit_shift = true,
            #ifdef   I2S_HW_VERSION_2
                .left_align = true,
                .big_endian = false,
                .bit_order_lsb = false
            #endif

        },
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,
            .bclk = spk_bclk,
            .ws = spk_ws,
            .dout = spk_dout,
            .din = I2S_GPIO_UNUSED,
            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv = false
            }
        }
    };
    ESP_ERROR_CHECK(i2s_channel_init_std_mode(tx_handle_, &std_cfg));

    // Create a new channel for MIC
    chan_cfg.id = XIAOZHI_I2S_PORT(1);
    ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg, nullptr, &rx_handle_));
    std_cfg.clk_cfg.sample_rate_hz = (uint32_t)input_sample_rate_;
    std_cfg.slot_cfg.slot_mask = mic_slot_mask;
    std_cfg.gpio_cfg.bclk = mic_sck;
    std_cfg.gpio_cfg.ws = mic_ws;
    std_cfg.gpio_cfg.dout = I2S_GPIO_UNUSED;
    std_cfg.gpio_cfg.din = mic_din;
    ESP_ERROR_CHECK(i2s_channel_init_std_mode(rx_handle_, &std_cfg));
    ESP_LOGI(TAG, "Simplex channels created");
}

int NoAudioCodec::Write(const int16_t* data, int samples) {
    std::lock_guard<std::mutex> lock(data_if_mutex_);
    std::vector<int32_t> buffer(samples);

    // output_volume_: 0-100
    // volume_factor_: 0-65536
    int32_t volume_factor = pow(double(output_volume_) / 100.0, 2) * 65536;
    for (int i = 0; i < samples; i++) {
        int64_t temp = int64_t(data[i]) * volume_factor; // 使用 int64_t 进行乘法运算
        if (temp > INT32_MAX) {
            buffer[i] = INT32_MAX;
        } else if (temp < INT32_MIN) {
            buffer[i] = INT32_MIN;
        } else {
            buffer[i] = static_cast<int32_t>(temp);
        }
    }

    size_t bytes_written;
    ESP_ERROR_CHECK(i2s_channel_write(tx_handle_, buffer.data(), samples * sizeof(int32_t), &bytes_written, portMAX_DELAY));
    return bytes_written / sizeof(int32_t);
}

int NoAudioCodec::Read(int16_t* dest, int samples) {
    size_t bytes_read;
    constexpr uint32_t kReadTimeoutMs = 200;

    std::vector<int32_t> bit32_buffer(samples);
    bool read_ok = false;
    {
        std::lock_guard<std::mutex> lock(data_if_mutex_);
        read_ok =
            i2s_channel_read(rx_handle_, bit32_buffer.data(), samples * sizeof(int32_t),
                             &bytes_read, kReadTimeoutMs) == ESP_OK;
    }

#if AUDIO_MIC_DEBUG_LOG
    if (!read_ok) {
        LogMicLevels(0, 0, 0, 0, input_gain_, true);
        return 0;
    }
#else
    if (!read_ok) {
        return 0;
    }
#endif

    samples = bytes_read / sizeof(int32_t);
#if AUDIO_MIC_DEBUG_LOG
    int32_t raw_peak = 0;
    int32_t pre_gain_peak = 0;
    int32_t post_gain_peak = 0;
    float effective_gain = input_gain_;
    ProcessMicSamples(bit32_buffer.data(), dest, samples, input_gain_, &raw_peak, &pre_gain_peak,
                      &post_gain_peak, &effective_gain);
    LogMicLevels(samples, raw_peak, pre_gain_peak, post_gain_peak, input_gain_, false);
#else
    ProcessMicSamples(bit32_buffer.data(), dest, samples, input_gain_, nullptr, nullptr, nullptr, nullptr);
#endif
    return samples;
}

void NoAudioCodec::EnableInput(bool enable) {
    std::lock_guard<std::mutex> lock(data_if_mutex_);
    if (enable == input_enabled_) {
        return;
    }
    if (enable) {
        // INMP441 + MAX98357 on shared WS/BCLK: ESP32 master must enable TX for clock.
        if (duplex_ && !output_enabled_) {
            ESP_ERROR_CHECK(i2s_channel_enable(tx_handle_));
            output_enabled_ = true;
            ESP_LOGI(TAG, "Duplex: enabled TX for shared I2S clock (mic RX)");
        }
        ESP_ERROR_CHECK(i2s_channel_enable(rx_handle_));
    } else {
        ESP_ERROR_CHECK(i2s_channel_disable(rx_handle_));
    }
    AudioCodec::EnableInput(enable);
}

void NoAudioCodec::EnableOutput(bool enable) {
    std::lock_guard<std::mutex> lock(data_if_mutex_);
    if (enable == output_enabled_) {
        return;
    }
    if (enable) {
        ESP_ERROR_CHECK(i2s_channel_enable(tx_handle_));
    } else {
        ESP_ERROR_CHECK(i2s_channel_disable(tx_handle_));
    }
    AudioCodec::EnableOutput(enable);
}

// Delegating constructor: calls the main constructor with default slot mask
NoAudioCodecSimplexPdm::NoAudioCodecSimplexPdm(int input_sample_rate, int output_sample_rate, gpio_num_t spk_bclk, gpio_num_t spk_ws, gpio_num_t spk_dout, gpio_num_t mic_sck, gpio_num_t mic_din) 
    : NoAudioCodecSimplexPdm(input_sample_rate, output_sample_rate, spk_bclk, spk_ws, spk_dout, I2S_STD_SLOT_LEFT, mic_sck, mic_din) {
    // All initialization is handled by the delegated constructor
}

NoAudioCodecSimplexPdm::NoAudioCodecSimplexPdm(int input_sample_rate, int output_sample_rate, gpio_num_t spk_bclk, gpio_num_t spk_ws, gpio_num_t spk_dout, i2s_std_slot_mask_t spk_slot_mask, gpio_num_t mic_sck, gpio_num_t mic_din) {
    duplex_ = false;
    input_sample_rate_ = input_sample_rate;
    output_sample_rate_ = output_sample_rate;

    // Create a new channel for speaker
    i2s_chan_config_t tx_chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(XIAOZHI_I2S_PORT(1), I2S_ROLE_MASTER);
    tx_chan_cfg.dma_desc_num = AUDIO_CODEC_DMA_DESC_NUM;
    tx_chan_cfg.dma_frame_num = AUDIO_CODEC_DMA_FRAME_NUM;
    tx_chan_cfg.auto_clear_after_cb = true;
    tx_chan_cfg.auto_clear_before_cb = false;
    tx_chan_cfg.intr_priority = 0;
    ESP_ERROR_CHECK(i2s_new_channel(&tx_chan_cfg, &tx_handle_, NULL));


    i2s_std_config_t tx_std_cfg = {
        .clk_cfg = {
            .sample_rate_hz = (uint32_t)output_sample_rate_,
            .clk_src = I2S_CLK_SRC_DEFAULT,
            .mclk_multiple = I2S_MCLK_MULTIPLE_256,
			#ifdef   I2S_HW_VERSION_2
				.ext_clk_freq_hz = 0,
			#endif

        },
        .slot_cfg = {
            .data_bit_width = I2S_DATA_BIT_WIDTH_32BIT,
            .slot_bit_width = I2S_SLOT_BIT_WIDTH_AUTO,
            .slot_mode = I2S_SLOT_MODE_MONO,
            .slot_mask = spk_slot_mask,
            .ws_width = I2S_DATA_BIT_WIDTH_32BIT,
            .ws_pol = false,
            .bit_shift = true,
            #ifdef   I2S_HW_VERSION_2
                .left_align = true,
                .big_endian = false,
                .bit_order_lsb = false
            #endif

        },
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,
            .bclk = spk_bclk,
            .ws = spk_ws,
            .dout = spk_dout,
            .din = I2S_GPIO_UNUSED,
            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv   = false,
            },
        },
    };
    ESP_ERROR_CHECK(i2s_channel_init_std_mode(tx_handle_, &tx_std_cfg));
#if SOC_I2S_SUPPORTS_PDM_RX
    // Create a new channel for MIC in PDM mode
    i2s_chan_config_t rx_chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(XIAOZHI_I2S_PORT(0), I2S_ROLE_MASTER);
    ESP_ERROR_CHECK(i2s_new_channel(&rx_chan_cfg, NULL, &rx_handle_));
    i2s_pdm_rx_config_t pdm_rx_cfg = {
        .clk_cfg = I2S_PDM_RX_CLK_DEFAULT_CONFIG((uint32_t)input_sample_rate_),
        /* The data bit-width of PDM mode is fixed to 16 */
        .slot_cfg = I2S_PDM_RX_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_MONO),
        .gpio_cfg = {
            .clk = mic_sck,
            .din = mic_din,

            .invert_flags = {
                .clk_inv = false,
            },
        },
    };
    ESP_ERROR_CHECK(i2s_channel_init_pdm_rx_mode(rx_handle_, &pdm_rx_cfg));
#else
    ESP_LOGE(TAG, "PDM is not supported");
#endif
    ESP_LOGI(TAG, "Simplex channels created");
}

int NoAudioCodecSimplexPdm::Read(int16_t* dest, int samples) {
    size_t bytes_read;

    // PDM 解调后的数据位宽为 16 位，直接读取到目标缓冲区
    if (i2s_channel_read(rx_handle_, dest, samples * sizeof(int16_t), &bytes_read, portMAX_DELAY) != ESP_OK) {
        ESP_LOGE(TAG, "Read Failed!");
        return 0;
    }

    samples = bytes_read / sizeof(int16_t);
    ApplyInputGain(dest, samples, input_gain_);
    SoftLimitPcm(dest, samples);
    return samples;
}
