//-----------------------------------------------------------------------------//
// ESP32 C++ DBC/CAN parser - Spare time mod and FreeRTOS port for fun         //
// Laurent Lardinois https://be.linkedin.com/in/laurentlardinois               //
//                                                                             //
// https://github.com/type-one/can-utils-esp32                                 //
//                                                                             //
// A C++ DBC file parser, and a CAN telemetry tool, adapted for                //
// ESP32 micro-controller forked from MIREO version at                         //
// https://github.com/mireo/can-utils                                          //
//-----------------------------------------------------------------------------//

/**
 * @file example.cpp
 * @brief Example DBC file for parsing CAN data, aggregating signals, and publishing aggregated frames.
 *
 * This file contains the definition of CAN messages and signals for a vehicle's CAN bus system.
 * It includes the following components:
 * - CAN messages with their respective signals
 * - Signal properties such as bit length, byte order, scaling, and units
 * - Event definitions for transmission frequencies
 * - Attribute definitions for signal aggregation types and transmission group frequencies
 *
 * The DBC file defines the following CAN messages:
 * - GPSLatLong: Contains GPS accuracy, longitude, and latitude signals.
 * - GPSAltitude: Contains GPS altitude signal.
 * - SOC: Contains state of charge (SOC) average signal.
 * - GPSSpeed: Contains GPS speed signal.
 * - BatteryCurrent: Contains raw battery current, smoothed battery current, and battery voltage signals.
 * - PowerState: Contains power state signal.
 *
 * The following events are defined for transmission frequencies:
 * - V2CTxTime: Transmission time for V2C.
 * - GPSGroupTxFreq: Transmission frequency for GPS group.
 * - EnergyGroupTxFreq: Transmission frequency for energy group.
 *
 * The following attributes are defined for signal aggregation types:
 * - AggType: Defines the aggregation type for each signal (e.g., LAST, AVG).
 *
 * The following attributes are defined for transmission group frequencies:
 * - TxGroupFreq: Defines the transmission group frequency for each CAN message.
 */

// Example modified from https://github.com/mireo/can-utils
/*==========================================================================================
    Copyright (c) 2001-2023 Mireo, EU

    Extremely efficient DBC file format parser, built on top of Boost Spirit.

    Strictly follows the grammar rules from the last updated DBC specification available at
    http://mcu.so/Microcontroller/Automotive/dbc-file-format-documentation_compress.pdf
===========================================================================================*/

#include <array>
#include <bit>
#include <chrono>
#include <cinttypes>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <random>


#include "tools/platform_detection.hpp"

#if defined(FREERTOS_PLATFORM)
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#else
#include <exception>
#endif

#if defined(ESP_PLATFORM)
#include <esp_system.h>
#include <sdkconfig.h>
#endif

#include "dbc/dbc_parser.hpp"
#include "v2c/v2c_transcoder.hpp"

#include "tools/logger.hpp"

inline void print_stats()
{
#if defined(ESP_PLATFORM)
    std::printf("Current free heap size: %" PRIu32 " bytes\n", esp_get_free_heap_size());
    std::printf("Minimum free heap size: %" PRIu32 " bytes\n", esp_get_minimum_free_heap_size());
#endif
#if defined(FREERTOS_PLATFORM)
    UBaseType_t ux_high_water_mark = uxTaskGetStackHighWaterMark(nullptr);
    std::printf("Minimum free stack size: %d bytes\n", ux_high_water_mark);
#endif
}

//--------------------------------------------------------------------------------------------------------------------------------

/**
 * @brief Generates a CAN frame with random data and CAN ID.
 *
 * This function creates a CAN frame with a random 64-bit signed integer as data
 * and a random CAN ID between 1 and 7 (inclusive). The random values are generated
 * using a uniform distribution.
 *
 * @return can_frame A CAN frame with random data and CAN ID.
 */
can_frame generate_frame()
{
    static std::default_random_engine generator;
    static std::uniform_int_distribution<int64_t> frame_data_dist(LLONG_MIN, LLONG_MAX); // random 64-bit signed integer
    static std::uniform_int_distribution<int64_t> can_id_dist(1, 7); // 7 is the highest can_id in example.dbc

    can_frame frame;
    *(std::int64_t*)frame.data = frame_data_dist(generator);
    frame.can_id = can_id_dist(generator);

    return frame;
}

/**
 * @brief Prints the details of CAN frames from a frame packet.
 *
 * This function prints the timestamp, CAN ID, and decoded signals of each frame in the provided frame packet.
 *
 * @param fp The frame packet containing CAN frames to be printed.
 * @param transcoder The transcoder used to decode the CAN frame messages.
 * @param frame_counter The number of frames in the frame packet.
 */
void print_frames(const can::frame_packet& fp, can::v2c_transcoder& transcoder, std::int32_t frame_counter)
{
    using namespace std::chrono;

    std::printf("New frame_packet (from %" PRId32 " frames\n", frame_counter);

    for (const auto& [ts, frame] : fp)
    {
        auto frame_data = std::bit_cast<std::int64_t>(frame.data);
        auto t = duration_cast<milliseconds>(ts.time_since_epoch()).count() / 1000.0;

        std::printf(" can_frame at t: %f ms (can id %" PRIu32 ")\n", t, frame.can_id);

        auto msg = transcoder.find_message(frame.can_id);
        for (const auto& sig : msg->signals(frame_data))
        {
            const auto raw = static_cast<std::int64_t>(sig.decode(frame_data));
            std::printf(
                "  %s: %" PRId64 " (raw) = %f %s\n", sig.name().c_str(), raw, sig.convert(raw), sig.unit().c_str());
        }
    }
}

// inlined example.dbc file
static const std::string example_dbc = R"(VERSION "0.1"

NS_ :

BS_:

BU_: Receiver ChassisBus VehicleBus V2C

BO_ 2 GPSLatLong: 8 ChassisBus
 SG_ GPSAccuracy : 57|7@1+ (0.2,0) [0|25] "m" Receiver
 SG_ GPSLongitude : 28|28@1- (1e-06,0) [-134.22|134.22] "Deg" Receiver
 SG_ GPSLatitude : 0|28@1- (1e-06,0) [-134.22|134.22] "Deg" Receiver

BO_ 3 GPSAltitude: 2 VehicleBus
 SG_ GPSAltitude : 0|14@1- (1,0) [-1000|7000] "M" Receiver

BO_ 4 SOC: 8 VehicleBus
 SG_ SOCavg : 30|10@1+ (0.1,0) [0|100] "%" Receiver

BO_ 5 GPSSpeed: 8 VehicleBus
 SG_ GPSSpeed : 12|12@1+ (0.2,-10) [-50|300] "kph" Receiver

BO_ 6 BatteryCurrent: 8 VehicleBus
 SG_ RawBattCurrent : 32|16@1- (0.05,-800) [-1000|1000] "A" Receiver
 SG_ SmoothBattCurrent : 16|16@1- (-0.5,0) [-1000|1000] "A" Receiver
 SG_ BattVoltage : 0|16@1+ (0.1,0) [0|1000] "V" Receiver

BO_ 7 PowerState: 2 VehicleBus
 SG_ PowerState : 14|2@1+ (1,0) [0|3] "" Receiver

EV_ V2CTxTime: 0 [0|60000] "ms" 2000 1 DUMMY_NODE_VECTOR1 V2C;
EV_ GPSGroupTxFreq: 0 [0|60000] "ms" 600 11 DUMMY_NODE_VECTOR1 V2C;
EV_ EnergyGroupTxFreq: 0 [0|60000] "ms" 500 12 DUMMY_NODE_VECTOR1 V2C;

BA_DEF_ BO_ "TxGroupFreq" STRING ;
BA_DEF_ SG_ "AggType" STRING ;

BA_ "AggType" SG_ 2 GPSAccuracy "LAST";
BA_ "AggType" SG_ 2 GPSLongitude "LAST";
BA_ "AggType" SG_ 2 GPSLatitude "LAST";
BA_ "AggType" SG_ 3 GPSAltitude "LAST";
BA_ "AggType" SG_ 5 GPSSpeed "LAST";
BA_ "AggType" SG_ 7 PowerState "LAST";
BA_ "AggType" SG_ 4 SOCavg "LAST";
BA_ "AggType" SG_ 6 RawBattCurrent "AVG";
BA_ "AggType" SG_ 6 SmoothBattCurrent "AVG";
BA_ "AggType" SG_ 6 BattVoltage "AVG";

BA_ "TxGroupFreq" BO_ 2 "GPSGroupTxFreq";
BA_ "TxGroupFreq" BO_ 3 "GPSGroupTxFreq";
BA_ "TxGroupFreq" BO_ 5 "GPSGroupTxFreq";
BA_ "TxGroupFreq" BO_ 7 "GPSGroupTxFreq";

BA_ "TxGroupFreq" BO_ 4 "EnergyGroupTxFreq";
BA_ "TxGroupFreq" BO_ 6 "EnergyGroupTxFreq";
)";


/**
 * @brief Test function for DBC CAN.
 *
 * This function demonstrates the parsing of a DBC file and the transcoding of CAN frames.
 * It prints the parsing time and processes incoming CAN frames in an infinite loop.
 * The processed frames are printed to stdout.
 *
 * @note This function runs indefinitely, simulating the reception of CAN frames.
 */
void test_dbc_can()
{
    LOG_INFO("-- DBC CAN --");
    print_stats();

    can::v2c_transcoder transcoder;

    auto start = std::chrono::system_clock::now();

    bool parsed = can::parse_dbc(example_dbc, std::ref(transcoder));
    if (!parsed)
    {
        return;
    }

    auto end = std::chrono::system_clock::now();

    std::printf(
        "Parsed DBC in %" PRIu64 " ms\n", std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count());

    std::int32_t frame_counter = 0;

    while (true)
    {
        // Simulate receiving a frame from CAN bus
        can_frame frame = generate_frame();
        frame_counter++;

        auto fp = transcoder.transcode(std::chrono::system_clock::now(), frame);

        if (fp.empty())
        {
            continue;
        }

        // Send the aggregated frame_packet to a remote server, store it locally, or process it.
        // This example just prints it to stdout.

        print_frames(fp, transcoder, frame_counter);
        frame_counter = 0;
    }
}

//--------------------------------------------------------------------------------------------------------------------------------

void runner()
{
    test_dbc_can();

    std::printf("This is The END\n");
}

//--------------------------------------------------------------------------------------------------------------------------------
#if defined(FREERTOS_PLATFORM)
// https://docs.espressif.com/projects/esp-idf/en/v4.4/esp32/api-reference/system/freertos.html

constexpr const std::size_t stack_size = 32768;

// Structure that will hold the TCB of the task being created.
StaticTask_t x_task_buffer = {}; // NOLINT required for xTaskCreateStatic

// Buffer that the task being created will use as its stack.  Note this is
// an array of StackType_t variables.  The size of StackType_t is dependent on
// the RTOS port.
std::array<StackType_t, stack_size> x_stack = {}; // NOLINT required for xTaskCreateStatic

// Function that implements the task being created.
void v_task_code(void* pv_parameters)
{
    // The parameter value is expected to be 1 as 1 is passed in the
    // pvParameters value in the call to xTaskCreateStatic().
    configASSERT((std::uint32_t)pv_parameters == 1UL);

    runner();

    vTaskDelete(nullptr); // delete itself
}

// Function that creates a task.
void launch_runner() noexcept
{
    TaskHandle_t x_handle = nullptr;

    // Create the task without using any dynamic memory allocation.
    x_handle = xTaskCreateStatic(v_task_code, // Function that implements the task.
        "RUNNER",                             // Text name for the task.
        stack_size,                           // Stack size in bytes, not words.
        reinterpret_cast<void*>(1),           // Parameter passed into the task.
        tskIDLE_PRIORITY,                     // Priority at which the task is created.
        x_stack.data(),                       // Array to use as the task's stack.
        &x_task_buffer);                      // Variable to hold the task's data structure.

    (void)x_handle;

    vTaskSuspend(nullptr); // suspend main task
}
#endif
//--------------------------------------------------------------------------------------------------------------------------------

#if !defined(FREERTOS_PLATFORM)
void runner_except_catch()
{
    try
    {
        runner();
    }
    catch (std::exception& exc)
    {
        LOG_ERROR("Exception catched - %s", exc.what());
    }
}
#endif

#if defined(FREERTOS_PLATFORM)
extern "C" void app_main() noexcept
#else
int main() noexcept
#endif
{

#if defined(FREERTOS_PLATFORM)
    launch_runner();
#else
    runner_except_catch();
    return 0;
#endif
}
