#pragma once
#include <cstddef>
#include <cstdint>
#include "types.h"
#include "dsp_pipeline.h"
#include "allan_tracker.h"

class TelemetryManager {
public:
    static uint16_t computeCRC16(const uint8_t* data, size_t length);

    // Оперативний кадр телеметрії ($RTM,...*CRC16)
    static void sendFrameCSV(const TelemetryFrame& frame);

    // Ретроспективно скоригований відлік після лінійної інтерполяції k_interp(t) ($RTM_COR,...*CRC16)
    static void sendInterpolatedSampleCSV(uint32_t timestamp_ms,
                                          float dv_interp_lin,
                                          float t_a_interp_c,
                                          float k_interp);

    // Неблокуюче зчитування команд керування від ПК по UART
    static void processIncomingCommands(SystemMode& mode, DSPPipeline& dsp, AllanTracker& allan);

private:
    static void handleCommandLine(const char* cmd, SystemMode& mode, DSPPipeline& dsp, AllanTracker& allan);

    static constexpr size_t RX_BUF_SIZE = 64;
    static char             rx_buf_[RX_BUF_SIZE];
    static size_t           rx_len_;
};