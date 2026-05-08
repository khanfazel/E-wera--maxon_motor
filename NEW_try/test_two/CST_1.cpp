// cst_run_nm.cpp  — EPOS4 CST over USB, command torque in N·m
// Build:
// g++ cst_run_nm.cpp -I/opt/maxon/EposCmd/Include -L/opt/maxon/EposCmd/Lib -lEposCmd -Wl,-rpath,/opt/maxon/EposCmd/Lib -o cst_run_nm
// Run:
// ./cst_run_nm

#include <Definitions.h>
#include <iostream>
#include <thread>
#include <chrono>
#include <cstdint>
#include <cmath>
#include <iomanip>
#include <fstream>   // ADDED: for CSV logging

// ---------- Globals ----------
static void* handle = nullptr;
static unsigned int err = 0;

static inline bool ok(bool success, const char* step) {
    if(!success) {
        std::cerr << "[EPOS ERROR] " << step << " (err=0x"
                  << std::hex << err << std::dec << ")\n";
    }
    return success;
}

// ---------- SDO helpers ----------
static inline bool sdo_write_u16(unsigned short nodeId, unsigned short idx, unsigned char sub,
                                 const void* p, unsigned int n, const char* tag)
{
    unsigned int nW = 0;
    return ok(VCS_SetObject(handle, nodeId, idx, sub, const_cast<void*>(p), n, &nW, &err) && nW == n, tag);
}

static inline bool sdo_write_any(unsigned short nodeId, unsigned short idx, unsigned char sub,
                                 const void* p, unsigned int n, const char* tag)
{
    unsigned int nW = 0;
    return ok(VCS_SetObject(handle, nodeId, idx, sub, const_cast<void*>(p), n, &nW, &err), tag);
}

static inline bool sdo_read_any(unsigned short nodeId, unsigned short idx, unsigned char sub,
                                void* p, unsigned int n, const char* tag)
{
    unsigned int nR = 0;
    return ok(VCS_GetObject(handle, nodeId, idx, sub, p, n, &nR, &err), tag);
}

// ---------- Object indices ----------
constexpr unsigned char  SUB0              = 0x00;
constexpr unsigned short IDX_OPMODE        = 0x6060;
constexpr unsigned short IDX_OPMODE_DISP   = 0x6061;
constexpr unsigned short IDX_CONTROLWORD   = 0x6040;
constexpr unsigned short IDX_STATUSWORD    = 0x6041;
constexpr unsigned short IDX_TARGET_TORQUE = 0x6071;
constexpr unsigned short IDX_TORQUE_ACT    = 0x6077;
constexpr unsigned short IDX_MAX_TORQUE    = 0x6072;
constexpr uint16_t       IDX_POS_ACT       = 0x6064;
constexpr unsigned short IDX_VEL_ACT       = 0x606C;

// ---------- Convert & command torque in N·m ----------
static inline int16_t nm_to_permille(double tau_Nm, double ratedTorque_Nm, int16_t clamp_abs_pm = 1000)
{
    double pm_f = (tau_Nm / ratedTorque_Nm) * 1000.0;
    long   pm_l = std::lround(pm_f);

    if(pm_l >  clamp_abs_pm) pm_l =  clamp_abs_pm;
    if(pm_l < -clamp_abs_pm) pm_l = -clamp_abs_pm;

    return static_cast<int16_t>(pm_l);
}

static inline bool set_torque_Nm(unsigned short nodeId, double tau_Nm, double ratedTorque_Nm)
{
    int16_t permille = nm_to_permille(tau_Nm, ratedTorque_Nm);

    return sdo_write_any(nodeId, IDX_TARGET_TORQUE, SUB0, &permille, sizeof(permille),
                         "SDO: Set 0x6071 Target Torque");
}

int main() {
    // ======== USER SETTINGS ========
    char deviceName[]        = "EPOS4";
    char protocolStackName[] = "MAXON SERIAL V2";
    char interfaceName[]     = "USB";
    char portName[]          = "USB0";
    const unsigned short nodeId = 3;

    // Motor/drive scaling
    const double ratedTorque_Nm = 1.068;
    const double Kt_Nm_per_A    = 0.0712;

    // Desired command
    const double target_tau_Nm  = 0.30;
    const uint16_t maxTorque_permille = 1000;

    // Loop settings
    const int runSeconds    = 12;
    const int loopPeriod_ms = 2;
    const int enc_counts_rev = 1024;

    // CSV log file name
    const char logFileName[] = "cst_torque_count_log.csv";
    // ===============================

    std::cout << std::fixed << std::setprecision(5);

    // ---------- Open CSV log file ----------
    std::ofstream logFile(logFileName);

    if(!logFile.is_open()) {
        std::cerr << "[ERROR] Could not open CSV log file.\n";
        return 1;
    }

    // CSV header
    logFile << "time_s,"
            << "encoder_count,"
            << "position_deg_motor,"
            << "velocity_rpm,"
            << "target_torque_permille,"
            << "target_torque_Nm,"
            << "actual_torque_permille,"
            << "actual_torque_Nm\n";

    // ---------- Open EPOS device ----------
    handle = VCS_OpenDevice(deviceName, protocolStackName, interfaceName, portName, &err);
    if(!ok(handle != nullptr, "OpenDevice")) return 1;

    // ---------- Clear faults ----------
    if(!ok(VCS_ClearFault(handle, nodeId, &err), "ClearFault")) return 1;

    // ---------- Read drive rated torque ----------
    uint16_t ratedTorque_mNm = 0;

    if(sdo_read_any(nodeId, 0x6076, 0x00, &ratedTorque_mNm,
                    sizeof(ratedTorque_mNm), "Read 0x6076 RatedTorque"))
    {
        std::cout << "\n[INFO] Drive's stored Rated Torque = "
                  << ratedTorque_mNm << " mN·m ("
                  << (ratedTorque_mNm / 1000.0) << " N·m)\n";
    }
    else {
        std::cerr << "[WARN] Could not read 0x6076 Rated Torque.\n";
    }

    std::cout << "----------------------------------------------\n";

    // ---------- Set CST mode ----------
    {
        int8_t op = 10; // CST

        if(!ok(VCS_SetOperationMode(handle, nodeId, op, &err), "SetOperationMode=CST")) return 1;

        std::this_thread::sleep_for(std::chrono::milliseconds(5));

        int8_t op_disp = -1;
        sdo_read_any(nodeId, IDX_OPMODE_DISP, SUB0, &op_disp, sizeof(op_disp), "Read ModeDisplay");

        if(op_disp != 10) {
            std::cerr << "[WARN] Mode display != CST (10), got " << (int)op_disp << "\n";
        }
    }

    // ---------- Set Max Torque ----------
    {
        uint16_t maxTq_rb = 0;

        if(sdo_read_any(nodeId, 0x6072, 0x00, &maxTq_rb, sizeof(maxTq_rb), "Probe 0x6072")) {
            uint16_t maxTq = 600;
            sdo_write_u16(nodeId, 0x6072, 0x00, &maxTq, sizeof(maxTq), "Set 0x6072 Max Torque");
        }
        else {
            std::cerr << "[INFO] 0x6072 not supported on this device/firmware; using current limits only.\n";
        }
    }

    // ---------- Enable drive ----------
    if(!ok(VCS_SetEnableState(handle, nodeId, &err), "SetEnableState")) return 1;

    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    // ---------- Announce setpoint ----------
    int16_t target_permille = nm_to_permille(target_tau_Nm, ratedTorque_Nm);
    double  est_I_A = target_tau_Nm / Kt_Nm_per_A;

    std::cout << "CST setpoint: " << target_tau_Nm << " N·m  ("
              << target_permille << " ‰ of rated " << ratedTorque_Nm << " N·m), "
              << "est. current ~ " << est_I_A << " A\n";

    std::cout << "Logging to: " << logFileName << "\n";

    // ---------- Main loop ----------
    auto t0     = std::chrono::steady_clock::now();
    auto period = std::chrono::milliseconds(loopPeriod_ms);
    int  tick   = 0;

    std::cout << "Applying torque for " << runSeconds << " s...\n";

    while(std::chrono::steady_clock::now() - t0 < std::chrono::seconds(runSeconds)) {

        // Current time
        auto now = std::chrono::steady_clock::now();
        double time_s = std::chrono::duration<double>(now - t0).count();

        // Command torque
        if(!set_torque_Nm(nodeId, target_tau_Nm, ratedTorque_Nm)) break;

        // Read encoder count, velocity, and actual torque
        int32_t pos = 0;
        int32_t vel = 0;
        int16_t tq_pm = 0;

        sdo_read_any(nodeId, IDX_POS_ACT,    SUB0, &pos,   sizeof(pos),   "Read 0x6064 Position");
        sdo_read_any(nodeId, IDX_VEL_ACT,    SUB0, &vel,   sizeof(vel),   "Read 0x606C Velocity");
        sdo_read_any(nodeId, IDX_TORQUE_ACT, SUB0, &tq_pm, sizeof(tq_pm), "Read 0x6077 Torque");

        // Convert for logging
        double pos_deg = (pos * 360.0) / enc_counts_rev;
        double actual_tq_Nm = (tq_pm / 1000.0) * ratedTorque_Nm;

        // ---------- Write one CSV row ----------
        logFile << time_s << ","
                << pos << ","
                << pos_deg << ","
                << vel << ","
                << target_permille << ","
                << target_tau_Nm << ","
                << tq_pm << ","
                << actual_tq_Nm << "\n";

        // Print roughly every 100 ms
        if(++tick % (100 / loopPeriod_ms) == 0) {
            std::cout << "count=" << pos
                      << "  pos=" << pos_deg << " deg est"
                      << "  vel=" << vel
                      << "  actual_torque=" << tq_pm << " ‰ (" << actual_tq_Nm << " N·m)\n";
        }

        std::this_thread::sleep_for(period);
    }

    // ---------- Ramp down smoothly to 0 torque ----------
    {
        int16_t cmd_pm = target_permille;
        const int16_t step = (cmd_pm > 0) ? -10 : 10;

        while(cmd_pm != 0) {
            cmd_pm = (std::abs(cmd_pm) < std::abs(step)) ? 0 : (cmd_pm + step);

            sdo_write_any(nodeId, IDX_TARGET_TORQUE, SUB0, &cmd_pm, sizeof(cmd_pm), "SDO: Ramp 0x6071");

            std::this_thread::sleep_for(period);
        }
    }

    // ---------- Close everything ----------
    logFile.close();

    VCS_SetDisableState(handle, nodeId, &err);
    VCS_CloseDevice(handle, &err);

    std::cout << "Done.\n";
    std::cout << "CSV saved as: " << logFileName << "\n";

    return 0;
}
