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
constexpr uint16_t IDX_POS_DEMAND  = 0x6062;  // Position demand value
constexpr uint16_t IDX_POS_ACT     = 0x6064;  // Position actual value
constexpr uint16_t IDX_FOLLOW_WIN  = 0x6065;  // Following error window
constexpr uint16_t IDX_TARGET_POS  = 0x607A;  // Final target position
constexpr uint16_t IDX_FOLLOW_ERR  = 0x60F4;  // Following error actual value
constexpr uint16_t IDX_ERROR_CODE  = 0x603F;

// These EPOS4 objects are used only for logging.
// 0x30D1: Current actual values, sub 1 = averaged, sub 2 = actual [mA].
// 0x30D2: Torque actual values, sub 1 = averaged [per-thousand of motor rated torque].
// 0x30D3: Velocity actual values, sub 1 = averaged [EPOS velocity units].
// 0x6076: Motor rated torque [mNm], used to convert torque raw value to Nm.
constexpr uint16_t IDX_MOTOR_RATED_TORQUE = 0x6076;
constexpr uint16_t IDX_CURRENT_ACTUALS    = 0x30D1;
constexpr uint16_t IDX_TORQUE_ACTUALS     = 0x30D2;
constexpr uint16_t IDX_VELOCITY_ACTUALS   = 0x30D3;
constexpr uint8_t  SUB_AVERAGED           = 0x01;
constexpr uint8_t  SUB_ACTUAL             = 0x02;

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
    const uint16_t nodeId    = 3;

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

    const bool absoluteMove = (moveMode == 'a');

    // Position profile values
    const uint32_t profileVelocity     = 500;   // rpm if default velocity units
    const uint32_t profileAcceleration = 500;   // rpm/s if default acceleration units
    const uint32_t profileDeceleration = 500;   // rpm/s if default acceleration units

    const int loopPeriodMs = 50;
    const int timeoutMs    = 15000;
    const int32_t reachTolCounts = 100;
    // =================================================

    std::cout << std::fixed << std::setprecision(3);

    // ---------- Create CSV log file ----------
    std::ofstream logFile("step_response_log.csv");

    if (!logFile.is_open())
    {
        std::cerr << "Could not create step_response_log.csv\n";
        return 1;
    }

    // CSV header
    logFile << "time_s,"
            << "target_position_0x607A_counts,"
            << "target_joint_deg,"
            << "position_demand_0x6062_counts,"
            << "position_demand_joint_deg,"
            << "position_actual_0x6064_counts,"
            << "actual_joint_deg,"
            << "target_minus_actual_counts,"
            << "target_minus_actual_deg,"
            << "following_error_actual_0x60F4_counts,"
            << "following_error_actual_deg,"
            << "manual_following_error_6062_minus_6064_counts,"
            << "manual_following_error_deg,"
            << "following_error_window_0x6065_counts,"
            << "following_error_window_deg,"
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

    // ---------- Read motor rated torque ----------
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

    // ---------- Read following error window ----------
    uint32_t followingErrorWindowCounts = 0;
    if (sdo_read_any(nodeId, IDX_FOLLOW_WIN, SUB0,
                     &followingErrorWindowCounts, sizeof(followingErrorWindowCounts),
                     "Read 0x6065 Following Error Window"))
    {
        std::cout << "Following error window 0x6065 = "
                  << followingErrorWindowCounts
                  << " counts = "
                  << motor_counts_to_joint_deg(static_cast<int32_t>(followingErrorWindowCounts),
                                               gearRatio, countsPerMotorRev)
                  << " joint deg\n";
    }
    else
    {
        std::cerr << "Warning: Could not read following error window.\n";
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

    std::cout << "Profile velocity: " << profileVelocity << "\n";
    std::cout << "Profile acceleration: " << profileAcceleration << "\n";
    std::cout << "Profile deceleration: " << profileDeceleration << "\n";

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
        int32_t posActual = 0;          // 0x6064
        int32_t posDemand = 0;          // 0x6062
        int32_t tgtActual = 0;          // 0x607A
        int32_t followingErrActual = 0; // 0x60F4
        uint16_t statusword = 0;

        // Extra values for logging
        int32_t velocityActualAvg = 0;     // 0x30D3:01
        int32_t currentActualAvg_mA = 0;   // 0x30D1:01
        int32_t currentActual_mA = 0;      // 0x30D1:02
        int16_t torqueActualAvgRaw = 0;    // 0x30D2:01

        if (!sdo_read_any(nodeId, IDX_POS_ACT, SUB0, &posActual, sizeof(posActual),
                          "Read 0x6064 Position Actual Value"))
        {
            break;
        }

        // Read final target position
        sdo_read_optional(nodeId, IDX_TARGET_POS, SUB0,
                          &tgtActual, sizeof(tgtActual));

        // Read real EPOS trajectory demand position
        sdo_read_optional(nodeId, IDX_POS_DEMAND, SUB0,
                          &posDemand, sizeof(posDemand));

        // Read real EPOS following error actual value
        sdo_read_optional(nodeId, IDX_FOLLOW_ERR, SUB0,
                          &followingErrActual, sizeof(followingErrActual));

        // Read following error window each loop too, in case you change it in EPOS Studio
        sdo_read_optional(nodeId, IDX_FOLLOW_WIN, SUB0,
                          &followingErrorWindowCounts, sizeof(followingErrorWindowCounts));

        sdo_read_any(nodeId, IDX_STATUSWORD, SUB0,
                     &statusword, sizeof(statusword),
                     "Read 0x6041 Statusword");

        // ---------- Read dynamic response values ----------
        sdo_read_optional(nodeId, IDX_VELOCITY_ACTUALS, SUB_AVERAGED,
                          &velocityActualAvg, sizeof(velocityActualAvg));

        sdo_read_optional(nodeId, IDX_CURRENT_ACTUALS, SUB_AVERAGED,
                          &currentActualAvg_mA, sizeof(currentActualAvg_mA));

        sdo_read_optional(nodeId, IDX_CURRENT_ACTUALS, SUB_ACTUAL,
                          &currentActual_mA, sizeof(currentActual_mA));

        sdo_read_optional(nodeId, IDX_TORQUE_ACTUALS, SUB_AVERAGED,
                          &torqueActualAvgRaw, sizeof(torqueActualAvgRaw));

        // ---------- Calculations ----------
        const int32_t targetMinusActualCounts = tgtActual - posActual;

        // Manual following error should match 0x60F4:
        // following error = position demand 0x6062 - position actual 0x6064
        const int32_t manualFollowingErrCounts = posDemand - posActual;

        const double jointDeg =
            motor_counts_to_joint_deg(posActual, gearRatio, countsPerMotorRev);

        const double targetJointActualDeg =
            motor_counts_to_joint_deg(tgtActual, gearRatio, countsPerMotorRev);

        const double demandJointDeg =
            motor_counts_to_joint_deg(posDemand, gearRatio, countsPerMotorRev);

        const double targetMinusActualDeg =
            targetJointActualDeg - jointDeg;

        const double manualFollowingErrDeg =
            demandJointDeg - jointDeg;

        const double followingErrActualDeg =
            motor_counts_to_joint_deg(followingErrActual, gearRatio, countsPerMotorRev);

        const double followingErrorWindowDeg =
            motor_counts_to_joint_deg(static_cast<int32_t>(followingErrorWindowCounts),
                                       gearRatio, countsPerMotorRev);

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

        // ---------- Write one row to CSV ----------
        logFile << elapsedSec << ","
                << tgtActual << ","
                << targetJointActualDeg << ","
                << posDemand << ","
                << demandJointDeg << ","
                << posActual << ","
                << jointDeg << ","
                << targetMinusActualCounts << ","
                << targetMinusActualDeg << ","
                << followingErrActual << ","
                << followingErrActualDeg << ","
                << manualFollowingErrCounts << ","
                << manualFollowingErrDeg << ","
                << followingErrorWindowCounts << ","
                << followingErrorWindowDeg << ","
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

        logFile.flush();

        // ---------- Terminal print ----------
        std::cout << "Actual: " << posActual
                  << " counts = " << jointDeg << " deg"
                  << " | Target(607A): " << tgtActual
                  << " | Demand(6062): " << posDemand
                  << " = " << demandJointDeg << " deg"
                  << " | Target-Actual: " << targetMinusActualCounts
                  << " counts"
                  << " | FollowErr(60F4): " << followingErrActual
                  << " counts = " << followingErrActualDeg << " deg"
                  << " | ManualFE: " << manualFollowingErrCounts
                  << " counts = " << manualFollowingErrDeg << " deg"
                  << " | FE Window(6065): " << followingErrorWindowCounts
                  << " counts = " << followingErrorWindowDeg << " deg"
                  << " | Vel: " << velocityActualAvg << " units"
                  << " | I: " << currentActual_mA << " mA"
                  << " | TorqueRaw: " << torqueActualAvgRaw
                  << " | Statusword: 0x" << std::hex << statusword << std::dec
                  << "\n";

        const bool fault = ((statusword & 0x0008) != 0);
        const bool reachedBit = ((statusword & 0x0400) != 0);

        // For reaching final target, use target 0x607A - actual 0x6064.
        const bool reachedTol =
            (std::llabs(static_cast<long long>(targetMinusActualCounts)) <= reachTolCounts);

        if (fault)
        {
            std::cerr << "Fault detected during move.\n";
            print_epos_error_code(nodeId);

            std::cerr << "At fault:\n";
            std::cerr << "  Position demand 0x6062 = " << posDemand << " counts\n";
            std::cerr << "  Position actual 0x6064 = " << posActual << " counts\n";
            std::cerr << "  Following error 0x60F4 = " << followingErrActual << " counts\n";
            std::cerr << "  Manual following error = "
                      << manualFollowingErrCounts << " counts\n";
            std::cerr << "  Following error window 0x6065 = "
                      << followingErrorWindowCounts << " counts\n";

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
