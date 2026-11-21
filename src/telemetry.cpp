#include "telemetry.h"
#include <Arduino.h>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>

char   TelemetryManager::rx_buf_[TelemetryManager::RX_BUF_SIZE]{};
size_t TelemetryManager::rx_len_ = 0;

// Стандартний поліном CRC-16-CCITT (0x1021), сумісний із Python binascii.crc_hqx(data, 0xFFFF)
uint16_t TelemetryManager::computeCRC16(const uint8_t* data, size_t length) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < length; ++i) {
        crc ^= (static_cast<uint16_t>(data[i]) << 8);
        for (uint8_t bit = 0; bit < 8; ++bit) {
            if (crc & 0x8000U) {
                crc = static_cast<uint16_t>((crc << 1) ^ 0x1021U);
            } else {
                crc = static_cast<uint16_t>(crc << 1);
            }
        }
    }
    return crc;
}

void TelemetryManager::sendFrameCSV(const TelemetryFrame& frame) {
    char payload[160];
    int len = snprintf(payload, sizeof(payload),
                       "$RTM,%lu,%lu,%u,%u,%.3f,%.3f,%.3f,%.2f,%.2f,%.4f,%.2f,%.3f",
                       static_cast<unsigned long>(frame.frame_counter),
                       static_cast<unsigned long>(frame.timestamp_ms),
                       static_cast<unsigned int>(frame.mode),
                       static_cast<unsigned int>(frame.is_ready),
                       frame.v_ant_mv,
                       frame.v_et_mv,
                       frame.dv_cor_mv,
                       frame.t_a_celsius,
                       frame.t_et_celsius,
                       frame.kappa_curr,
                       frame.tau_opt_sec,
                       frame.sigma_t_opt);

    // Захист від виходу за межі буфера при обрізанні snprintf
    if (len > 0 && static_cast<size_t>(len) < sizeof(payload)) {
        uint16_t crc = computeCRC16(reinterpret_cast<const uint8_t*>(payload), static_cast<size_t>(len));
        Serial.print(payload);
        Serial.print('*');
        if (crc < 0x1000U) Serial.print('0');
        if (crc < 0x0100U) Serial.print('0');
        if (crc < 0x0010U) Serial.print('0');
        Serial.println(crc, HEX);
    }
}

void TelemetryManager::sendInterpolatedSampleCSV(uint32_t timestamp_ms,
                                                 float dv_interp_lin,
                                                 float t_a_interp_c,
                                                 float k_interp) {
    char payload[96];
    int len = snprintf(payload, sizeof(payload),
                       "$RTM_COR,%lu,%.3f,%.2f,%.4f",
                       static_cast<unsigned long>(timestamp_ms),
                       dv_interp_lin,
                       t_a_interp_c,
                       k_interp);

    if (len > 0 && static_cast<size_t>(len) < sizeof(payload)) {
        uint16_t crc = computeCRC16(reinterpret_cast<const uint8_t*>(payload), static_cast<size_t>(len));
        Serial.print(payload);
        Serial.print('*');
        if (crc < 0x1000U) Serial.print('0');
        if (crc < 0x0100U) Serial.print('0');
        if (crc < 0x0010U) Serial.print('0');
        Serial.println(crc, HEX);
    }
}

void TelemetryManager::processIncomingCommands(SystemMode& mode, DSPPipeline& dsp, AllanTracker& allan) {
    // Неблокуюче побайтове зчитування без динамічної пам'яті (нульовий таймаут)
    while (Serial.available() > 0) {
        char c = static_cast<char>(Serial.read());
        if (c == '\r') {
            continue;
        }

        if (c == '\n') {
            rx_buf_[rx_len_] = '\0';
            if (rx_len_ > 0) {
                handleCommandLine(rx_buf_, mode, dsp, allan);
            }
            rx_len_ = 0;
        } else if (rx_len_ < RX_BUF_SIZE - 1) {
            rx_buf_[rx_len_++] = c;
        } else {
            // Захист від переповнення буфера при отриманні сміття на лінії
            rx_len_ = 0;
        }
    }
}

void TelemetryManager::handleCommandLine(const char* cmd, SystemMode& mode, DSPPipeline& dsp, AllanTracker& allan) {
    if (std::strcmp(cmd, "CMD:START") == 0) {
        mode = SystemMode::Measuring;
    } else if (std::strcmp(cmd, "CMD:STOP") == 0) {
        mode = SystemMode::Idle;
    } else if (std::strcmp(cmd, "CMD:CAL_ASYM") == 0) {
        dsp.beginAsymmetryCalibration();
        mode = SystemMode::CalibratingAsym;
    } else if (std::strncmp(cmd, "CMD:KCAL=", 9) == 0) {
        char* endptr = nullptr;
        float val    = strtof(cmd + 9, &endptr);
        if (endptr != (cmd + 9) && std::isfinite(val) && val > 0.01f && val < 1.0e6f) {
            dsp.setKCal(val);
        }
    } else if (std::strncmp(cmd, "CMD:DTDOP=", 10) == 0) {
        char* endptr = nullptr;
        float val    = strtof(cmd + 10, &endptr);
        if (endptr != (cmd + 10) && std::isfinite(val) && val > 0.01f && val <= 5.0f) {
            allan.setAllowedError(val);
        }
    }
}