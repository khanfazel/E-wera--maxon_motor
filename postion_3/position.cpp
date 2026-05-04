#include <Definitions.h>
#include <iostream>
#include <thread>
#include <chrono>
#include <cstdint>
#include <cmath>
#include <iomanip>
#include <fstream>   // for CSV log file
#include <cstdlib>   // for std::llabs

// ---------- Globals ----------
static void* g_handle = nullptr;
static unsigned int g_err = 0;

// ---------- Object Dictionary ----------
constexpr uint8_t  SUB0            = 0x00;
constexpr uint16_t IDX_STATUSWORD  = 0x6041;
constexpr uint16_t IDX_OPMODE_DISP = 0x6061;
constexpr uint16_t IDX_POS_ACT     = 0x6064;
constexpr uint16_t IDX_TARGET_POS  = 0x607A;
constexpr uint16_t IDX_ERROR_CODE  = 0x603F;

// These EPOS4 objects are used only for logging.
// 0x30D1: Current actual values, sub 1 = averaged, sub 2 = actual [mA].
// 0x30D2: Torque actual values, sub 1 = averaged [per-thousand of motor rated torque].
// 0x30D3: Velocity actual values, sub 1 = averaged [EPOS velocity units].
// 0x6076: Motor rated torque [mNm], used to convert torque raw value to Nm.
constexpr uint16_t IDX_MOTOR_RATED_TORQUE = 0x6076;
constexpr uint16_t IDX_CURRENT_ACTUALS     = 0x30D1;
constexpr uint16_t IDX_TORQUE_ACTUALS      = 0x30D2;
constexpr uint16_t IDX_VELOCITY_ACTUALS    = 0x30D3;
constexpr uint8_t  SUB_AVERAGED            = 0x01;
constexpr uint8_t  SUB_ACTUAL              = 0x02;

// ---------- Helpers ----------
static inline bool ok(bool success, const char* step)
{
    if (!success)
    {
        std::cerr << "[EPOS ERROR] " << step
                  << " (err=0x" << std::hex << g_err << std::dec << ")\n";
    }
    return success;
}

static inline bool sdo_read_any(uint16_t nodeId,
                                uint16_t idx,
                                uint8_t sub,
                                void* data,
                                uint32_t size,
                                const char* tag)
{
    uint32_t bytesRead = 0;
    return ok(VCS_GetObject(g_handle, nodeId, idx, sub,
                            data, size, &bytesRead, &g_err), tag);
}

static inline bool sdo_read_optional(uint16_t nodeId,
                                     uint16_t idx,
                                     uint8_t sub,
                                     void* data,
                                     uint32_t size)
{
    uint32_t bytesRead = 0;
    unsigned int localErr = 0;
    return VCS_GetObject(g_handle, nodeId, idx, sub,
                         data, size, &bytesRead, &localErr);
}

// ---------- Conversion ----------
static inline int32_t joint_deg_to_motor_counts(double jointDeg,
                                                double gearRatio,
                                                int countsPerMotorRev)
{
    return static_cast<int32_t>(
        std::llround((jointDeg / 360.0) * gearRatio * countsPerMotorRev)
    );
}

static inline double motor_counts_to_joint_deg(int32_t motorCounts,
                                               double gearRatio,
                                               int countsPerMotorRev)
{
    return (static_cast<double>(motorCounts) * 360.0) /
           (gearRatio * static_cast<double>(countsPerMotorRev));
}

static inline double motor_rpm_to_joint_deg_s(int32_t motorVelocityRpm,
                                               double gearRatio)
{
    // If your EPOS velocity unit is rpm, this converts motor rpm to joint deg/s.
    // joint_deg/s = motor_rpm * 360 deg/rev / 60 s/min / gearRatio
    return (static_cast<double>(motorVelocityRpm) * 6.0) / gearRatio;
}

static inline double torque_actual_raw_to_motor_Nm(int16_t torqueRaw,
                                                    uint32_t motorRatedTorque_mNm)
{
    // EPOS torque actual raw value is in 1/1000 of motor rated torque.
    // motorRatedTorque_mNm / 1000 gives Nm.
    return (static_cast<double>(torqueRaw) / 1000.0) *
           (static_cast<double>(motorRatedTorque_mNm) / 1000.0);
}

static inline void print_epos_error_code(uint16_t nodeId)
{
    uint16_t eposErrCode = 0;
    if (sdo_read_any(nodeId, IDX_ERROR_CODE, SUB0, &eposErrCode, sizeof(eposErrCode),
                     "Read 0x603F Error Code"))
    {
        std::cerr << "EPOS device error code = 0x"
                  << std::hex << eposErrCode << std::dec << "\n";
    }
}

int main()
{
    // ================= USER SETTINGS =================
    char deviceName[]        = "EPOS4";
    char protocolStackName[] = "MAXON SERIAL V2";
    char interfaceName[]     = "USB";
    char portName[]          = "USB0";
    const uint16_t nodeId    = 5;

    // Mechanics
    const int countsPerMotorRev = 4096; // 1024 * 4
    const double gearRatio      = 80.0;

    // Command
    double targetJointDeg = 0.0;
char moveMode = 'r';   // r = relative, a = absolute

std::cout << "Enter target joint angle in deg: ";
std::cin >> targetJointDeg;

if (!std::cin)
{
    std::cerr << "Invalid angle input.\n";
    return 1;
}

std::cout << "Move mode? (r = relative, a = absolute): ";
std::cin >> moveMode;

if (!std::cin || (moveMode != 'r' && moveMode != 'a'))
{
    std::cerr << "Invalid move mode.\n";
    return 1;
}

    const bool absoluteMove = (moveMode == 'a');  // start small
   

    // IMPORTANT:
    // These are profile values for the EPOS position profile command.
    // Keep them modest first.
    const uint32_t profileVelocity     = 150;   // conservative
    const uint32_t profileAcceleration = 300;   // conservative
    const uint32_t profileDeceleration = 300;   // conservative

    const int loopPeriodMs = 50;
    const int timeoutMs    = 15000;
    const int32_t reachTolCounts = 100;
    // =================================================

    std::cout << std::fixed << std::setprecision(3);

    // ---------- Create CSV log file ----------
    // This file will be saved in the same folder from where you run the program.
    std::ofstream logFile("step_response_log.csv");

    if (!logFile.is_open())
    {
        std::cerr << "Could not create step_response_log.csv\n";
        return 1;
    }

    // Write column names once at the top of the CSV file.
    logFile << "time_s,"
            << "target_joint_deg,"
            << "actual_joint_deg,"
            << "target_counts,"
            << "actual_counts,"
            << "error_counts,"
            << "error_joint_deg,"
            << "velocity_actual_avg_units,"
            << "joint_velocity_est_deg_s,"
            << "current_actual_mA,"
            << "current_actual_avg_mA,"
            << "torque_actual_avg_raw_per_1000,"
            << "motor_rated_torque_mNm,"
            << "motor_torque_actual_avg_Nm,"
            << "joint_torque_est_avg_Nm_no_efficiency,"
            << "statusword_hex"
            << "\n";

    const int32_t moveCounts =
        joint_deg_to_motor_counts(targetJointDeg, gearRatio, countsPerMotorRev);

    std::cout << "Requested joint move: " << targetJointDeg
              << " deg -> " << moveCounts << " motor counts\n";

    // ---------- Open ----------
    g_handle = VCS_OpenDevice(deviceName, protocolStackName, interfaceName, portName, &g_err);
    if (!ok(g_handle != nullptr, "OpenDevice"))
        return 1;

    // ---------- Clear fault ----------
    if (!ok(VCS_ClearFault(g_handle, nodeId, &g_err), "ClearFault"))
    {
        VCS_CloseDevice(g_handle, &g_err);
        return 1;
    }

    // ---------- Activate PPM ----------
    if (!ok(VCS_ActivateProfilePositionMode(g_handle, nodeId, &g_err),
            "ActivateProfilePositionMode"))
    {
        VCS_CloseDevice(g_handle, &g_err);
        return 1;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(20));

    // ---------- Read mode ----------
    {
        int8_t opModeDisp = -1;
        if (sdo_read_any(nodeId, IDX_OPMODE_DISP, SUB0, &opModeDisp, sizeof(opModeDisp),
                         "Read 0x6061 Modes of operation display"))
        {
            std::cout << "Mode display = " << static_cast<int>(opModeDisp) << "\n";
        }
    }

    // ---------- Read motor rated torque for torque conversion ----------
    // 0x6076 is motor rated torque in mNm. If this read fails, the code still logs
    // the raw torque value from 0x30D2:01, but Nm conversion will stay 0.
    uint32_t motorRatedTorque_mNm = 0;
    if (sdo_read_any(nodeId, IDX_MOTOR_RATED_TORQUE, SUB0,
                     &motorRatedTorque_mNm, sizeof(motorRatedTorque_mNm),
                     "Read 0x6076 Motor Rated Torque"))
    {
        std::cout << "Motor rated torque: " << motorRatedTorque_mNm << " mNm\n";
    }
    else
    {
        std::cerr << "Warning: Could not read motor rated torque. Torque raw value will still be logged.\n";
    }

    // ---------- Set profile ----------
    if (!ok(VCS_SetPositionProfile(g_handle, nodeId,
                                   profileVelocity,
                                   profileAcceleration,
                                   profileDeceleration,
                                   &g_err),
            "SetPositionProfile"))
    {
        print_epos_error_code(nodeId);
        VCS_CloseDevice(g_handle, &g_err);
        return 1;
    }

    // ---------- Enable ----------
    if (!ok(VCS_SetEnableState(g_handle, nodeId, &g_err), "SetEnableState"))
    {
        VCS_CloseDevice(g_handle, &g_err);
        return 1;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(20));

    // ---------- Read current position ----------
    int32_t posBefore = 0;
    if (!sdo_read_any(nodeId, IDX_POS_ACT, SUB0, &posBefore, sizeof(posBefore),
                      "Read 0x6064 Position Actual Value"))
    {
        VCS_SetDisableState(g_handle, nodeId, &g_err);
        VCS_CloseDevice(g_handle, &g_err);
        return 1;
    }

    std::cout << "Before move: " << posBefore
              << " counts = "
              << motor_counts_to_joint_deg(posBefore, gearRatio, countsPerMotorRev)
              << " joint deg\n";

    const int32_t targetCounts = absoluteMove ? moveCounts : (posBefore + moveCounts);

    std::cout << "Commanded target: " << targetCounts
              << " counts = "
              << motor_counts_to_joint_deg(targetCounts, gearRatio, countsPerMotorRev)
              << " joint deg\n";

    // ---------- Move ----------
    if (!ok(VCS_MoveToPosition(g_handle, nodeId,
                               static_cast<long>(targetCounts),
                               absoluteMove ? 1 : 0,
                               1,
                               &g_err),
            "MoveToPosition"))
    {
        print_epos_error_code(nodeId);
        VCS_SetDisableState(g_handle, nodeId, &g_err);
        VCS_CloseDevice(g_handle, &g_err);
        return 1;
    }

    std::cout << "Move started...\n";

    auto t0 = std::chrono::steady_clock::now();

    while (true)
    {
        int32_t posActual = 0;
        int32_t tgtActual = 0;
        uint16_t statusword = 0;

        // Extra values for logging
        int32_t velocityActualAvg = 0;     // 0x30D3:01, velocity actual value averaged
        int32_t currentActualAvg_mA = 0;   // 0x30D1:01, current actual value averaged [mA]
        int32_t currentActual_mA = 0;      // 0x30D1:02, current actual value [mA]
        int16_t torqueActualAvgRaw = 0;    // 0x30D2:01, torque actual avg [1/1000 rated torque]

        if (!sdo_read_any(nodeId, IDX_POS_ACT, SUB0, &posActual, sizeof(posActual),
                          "Read 0x6064 Position Actual Value"))
        {
            break;
        }

        sdo_read_any(nodeId, IDX_TARGET_POS, SUB0, &tgtActual, sizeof(tgtActual),
                     "Read 0x607A Target Position");
        sdo_read_any(nodeId, IDX_STATUSWORD, SUB0, &statusword, sizeof(statusword),
                     "Read 0x6041 Statusword");

        // ---------- Read dynamic response values for logging ----------
        // These reads are non-fatal: if one value fails, the program keeps moving
        // and logs zero for that value.
        sdo_read_optional(nodeId, IDX_VELOCITY_ACTUALS, SUB_AVERAGED,
                          &velocityActualAvg, sizeof(velocityActualAvg));

        sdo_read_optional(nodeId, IDX_CURRENT_ACTUALS, SUB_AVERAGED,
                          &currentActualAvg_mA, sizeof(currentActualAvg_mA));

        sdo_read_optional(nodeId, IDX_CURRENT_ACTUALS, SUB_ACTUAL,
                          &currentActual_mA, sizeof(currentActual_mA));

        sdo_read_optional(nodeId, IDX_TORQUE_ACTUALS, SUB_AVERAGED,
                          &torqueActualAvgRaw, sizeof(torqueActualAvgRaw));

        const int32_t errCounts = tgtActual - posActual;
        const double jointDeg =
            motor_counts_to_joint_deg(posActual, gearRatio, countsPerMotorRev);
        const double targetJointActualDeg =
            motor_counts_to_joint_deg(tgtActual, gearRatio, countsPerMotorRev);
        const double errorJointDeg = targetJointActualDeg - jointDeg;

        const double jointVelocityDegS =
            motor_rpm_to_joint_deg_s(velocityActualAvg, gearRatio);

        const double motorTorqueActualAvgNm =
            torque_actual_raw_to_motor_Nm(torqueActualAvgRaw, motorRatedTorque_mNm);

        // This is only an estimate. It ignores gearbox efficiency and losses.
        const double jointTorqueActualAvgNm_noEfficiency =
            motorTorqueActualAvgNm * gearRatio;

        const auto now = std::chrono::steady_clock::now();
        const double elapsedSec =
            std::chrono::duration<double>(now - t0).count();

        // ---------- Write one row to CSV log file ----------
        logFile << elapsedSec << ","
                << targetJointActualDeg << ","
                << jointDeg << ","
                << tgtActual << ","
                << posActual << ","
                << errCounts << ","
                << errorJointDeg << ","
                << velocityActualAvg << ","
                << jointVelocityDegS << ","
                << currentActual_mA << ","
                << currentActualAvg_mA << ","
                << torqueActualAvgRaw << ","
                << motorRatedTorque_mNm << ","
                << motorTorqueActualAvgNm << ","
                << jointTorqueActualAvgNm_noEfficiency << ","
                << "0x" << std::hex << statusword << std::dec
                << "\n";

        // Force data to be written to disk during the test.
        // This is useful if the program stops early or faults.
        logFile.flush();

        std::cout << "Actual: " << posActual
                  << " counts = " << jointDeg << " joint deg"
                  << " | Target: " << tgtActual
                  << " | Err: " << errCounts
                  << " | Vel: " << velocityActualAvg << " units"
                  << " | I: " << currentActual_mA << " mA"
                  << " | TorqueRaw: " << torqueActualAvgRaw
                  << " | Statusword: 0x" << std::hex << statusword << std::dec
                  << "\n";

        const bool fault = ((statusword & 0x0008) != 0);
        const bool reachedBit = ((statusword & 0x0400) != 0);
        const bool reachedTol = (std::llabs(static_cast<long long>(errCounts)) <= reachTolCounts);

        if (fault)
        {
            std::cerr << "Fault detected during move.\n";
            print_epos_error_code(nodeId);
            break;
        }

        if (reachedBit || reachedTol)
        {
            std::cout << "Target reached.\n";
            break;
        }

        const auto elapsedMs =
            std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now() - t0).count();

        if (elapsedMs > timeoutMs)
        {
            std::cerr << "Timeout waiting for target.\n";
            break;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(loopPeriodMs));
    }

    // ---------- Final ----------
    {
        int32_t posFinal = 0;
        if (sdo_read_any(nodeId, IDX_POS_ACT, SUB0, &posFinal, sizeof(posFinal),
                         "Read final 0x6064 Position Actual Value"))
        {
            std::cout << "Final: " << posFinal
                      << " counts = "
                      << motor_counts_to_joint_deg(posFinal, gearRatio, countsPerMotorRev)
                      << " joint deg\n";
        }
    }

    // ---------- Close CSV log file ----------
    logFile.close();
    std::cout << "CSV log saved as: step_response_log.csv\n";

    VCS_SetDisableState(g_handle, nodeId, &g_err);
    VCS_CloseDevice(g_handle, &g_err);

    std::cout << "Done.\n";
    return 0;
}
