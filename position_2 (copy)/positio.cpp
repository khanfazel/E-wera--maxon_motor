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

// ---------- Conversion ----------
static inline int32_t joint_deg_to_motor_counts(double jointDeg,
                                                double gearRatio,
                                                int countsPerMotorRev)
{
    return static_cast<int32_t>(
        std::llround(-(jointDeg / 360.0) * gearRatio * countsPerMotorRev)
    );
}

static inline double motor_counts_to_joint_deg(int32_t motorCounts,
                                               double gearRatio,
                                               int countsPerMotorRev)
{
    return (static_cast<double>(-motorCounts) * 360.0) /
           (gearRatio * static_cast<double>(countsPerMotorRev));
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
std::ofstream logFile("step_response_log.csv");

if (!logFile.is_open())
{
    std::cerr << "Could not create step_response_log.csv\n";
    return 1;
}

// Write column names
logFile << "time_s,"
        << "target_joint_deg,"
        << "actual_joint_deg,"
        << "target_counts,"
        << "actual_counts,"
        << "error_counts,"
        << "error_joint_deg,"
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

        if (!sdo_read_any(nodeId, IDX_POS_ACT, SUB0, &posActual, sizeof(posActual),
                          "Read 0x6064 Position Actual Value"))
        {
            break;
        }

        sdo_read_any(nodeId, IDX_TARGET_POS, SUB0, &tgtActual, sizeof(tgtActual),
                     "Read 0x607A Target Position");
        sdo_read_any(nodeId, IDX_STATUSWORD, SUB0, &statusword, sizeof(statusword),
                     "Read 0x6041 Statusword");

        const int32_t errCounts = tgtActual - posActual;
        const double jointDeg =
            motor_counts_to_joint_deg(posActual, gearRatio, countsPerMotorRev);
const double targetJointActualDeg =
    motor_counts_to_joint_deg(tgtActual, gearRatio, countsPerMotorRev);

const double errorJointDeg = targetJointActualDeg - jointDeg;

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
        << "0x" << std::hex << statusword << std::dec
        << "\n";

logFile.flush();
        std::cout << "Actual: " << posActual
                  << " counts = " << jointDeg << " joint deg"
                  << " | Target: " << tgtActual
                  << " | Err: " << errCounts
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

    VCS_SetDisableState(g_handle, nodeId, &g_err);
    VCS_CloseDevice(g_handle, &g_err);
    logFile.close();
std::cout << "CSV log saved as: step_response_log.csv\n";

    std::cout << "Done.\n";
    return 0;
}
