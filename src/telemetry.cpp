#include "telemetry.h"
#include <Arduino.h>

// Стандартний поліном CRC-16-CCITT (0x1021), сумісний з Python binascii.crc_hqx(data, 0xFFFF)
uint16_t TelemetryManager::computeCRC16(const uint8_t* data, size_t length) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < length; ++i) {
        crc ^= (static_cast<uint16_t>(data[i]) << 8);
        for (uint8_t bit = 0; bit < 8; ++bit) {
            if (crc & 0x8000) {
                crc = (crc << 1) ^ 0x1021;
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}

void TelemetryManager::sendFrameCSV(const TelemetryFrame& frame) {
    char payload[160];
    // Формуємо рядок кадру без CRC
    int len = snprintf(payload, sizeof(payload),
                       "$RTM,%lu,%lu,%u,%u,%.3f,%.3f,%.3f,%.2f,%.2f,%.4f,%.2f,%.3f",
                       frame.frame_counter,
                       frame.timestamp_ms,
                       frame.mode,
                       frame.is_ready,
                       frame.v_ant_mv,
                       frame.v_et_mv,
                       frame.dv_cor_mv,
                       frame.t_a_celsius,
                       frame.t_et_celsius,
                       frame.kappa_curr,
                       frame.tau_opt_sec,
                       frame.sigma_t_opt);

    if (len > 0) {
        uint16_t crc = computeCRC16(reinterpret_cast<const uint8_t*>(payload), static_cast<size_t>(len));
        Serial.print(payload);
        Serial.print('*');
        if (crc < 0x1000) Serial.print('0');
        if (crc < 0x0100) Serial.print('0');
        if (crc < 0x0010) Serial.print('0');
        Serial.println(crc, HEX);
    }
}

void TelemetryManager::processIncomingCommands(SystemMode& mode, DSPPipeline& dsp, AllanTracker& allan) {
    // Команди від ПК (Python):
    // "CMD:START"      -> перехід у штатний режим вимірювання
    // "CMD:STOP"       -> зупинка (Idle)
    // "CMD:CAL_ASYM"   -> запуск калібрування асиметрії комутатора k
    // "CMD:KCAL=16.5"  -> оновлення крутизни двоточкового калібрування
    if (!Serial.available()) return;

    String cmd = Serial.readStringUntil('\n');
    cmd.trim();

    if (cmd == "CMD:START") {
        mode = SystemMode::Measuring;
    } else if (cmd == "CMD:STOP") {
        mode = SystemMode::Idle;
    } else if (cmd == "CMD:CAL_ASYM") {
        mode = SystemMode::CalibratingAsym;
    } else if (cmd.startsWith("CMD:KCAL=")) {
        float val = cmd.substring(9).toFloat();
        if (val > 0.1f) dsp.setKCal(val);
    } else if (cmd.startsWith("CMD:DTDOP=")) {
        float val = cmd.substring(10).toFloat();
        if (val > 0.01f) allan.setAllowedError(val);
    }
}