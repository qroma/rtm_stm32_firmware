#pragma once
#include "types.h"
#include "dsp_pipeline.h"
#include "allan_tracker.h"

class TelemetryManager {
public:
    static uint16_t computeCRC16(const uint8_t* data, size_t length);
    static void sendFrameCSV(const TelemetryFrame& frame);
    static void processIncomingCommands(SystemMode& mode, DSPPipeline& dsp, AllanTracker& allan);
};