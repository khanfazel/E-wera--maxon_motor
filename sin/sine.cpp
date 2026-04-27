// sine_pd_cst.cpp
// EPOS4 CST mode with outer PD position control
//
// Idea:
//   - EPOS remains in CST (torque mode)
//   - We generate a sinusoidal desired POSITION trajectory
//   - Outer PD computes torque command
//
// Build:
// g++ sine_pd_cst.cpp \
//   -I/home/e-wear/EPOS_Linux_Library/include \
//   -L/home/e-wear/EPOS_Linux_Library/lib/arm/v8 \
//   -lEposCmd -Wl,-rpath,/home/e-wear/EPOS_Linux_Library/lib/arm/v8 \
//   -std=c++17 -O2 -Wall -o sine_pd_cst
//
// Run:
// sudo env LD_LIBRARY_PATH=/home/e-wear/EPOS_Linux_Library/lib/arm/v8 ./sine_pd_cst

#include <Definitions.h>
#include <iostream>
#include <thread>
#include <chrono>
#include <cstdint>
#include <cmath>
#include <iomanip>
#include <algorithm>

// ---------- Globals ----------
static void* handle = nullptr;
static unsigned int err = 0;

static inline bool ok(int success, const char* step)
{
    if (!success) {
        std::cerr << "[EPOS ERROR] " << step
                  << " (err=0x" << std::hex << err << std::dec << ")\n";
    }
    return success != 0;
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
constexpr unsigned short IDX_OPMODE_DISP   = 0x6061;
constexpr unsigned short IDX_TARGET_TORQUE = 0x6071;
constexpr unsigned short IDX_TORQUE_ACT    = 0x6077;
constexpr unsigned short IDX_MAX_TORQUE    = 0x6072;
constexpr unsigned short IDX_POS_ACT       = 0x6064;

// ---------- Helpers ----------
static inline double deg2rad(double deg) { return deg * M_PI / 180.0; }
static inline double rad2deg(double rad) { return rad * 180.0 / M_PI; }

static inline int16_t nm_to_permille(double tau_Nm, double ratedTorque_Nm, int16_t clamp_abs_pm = 1000)
{
    double pm_f = (tau_Nm / ratedTorque_Nm) * 1000.0;
    long pm_l = std::lround(pm_f);

    if (pm_l > clamp_abs_pm)  pm_l = clamp_abs_pm;
    if (pm_l < -clamp_abs_pm) pm_l = -clamp_abs_pm;

    return static_cast<int16_t>(pm_l);
}

static inline bool set_torque_Nm(unsigned short nodeId, double tau_Nm, double ratedTorque_Nm, int16_t clamp_pm)
{
    int16_t permille = nm_to_permille(tau_Nm, ratedTorque_Nm, clamp_pm);
    return sdo_write_any(nodeId, IDX_TARGET_TORQUE, SUB0, &permille, sizeof(permille),
                         "SDO: Set 0x6071 Target Torque");
}

int main()
{
    // ================= USER SETTINGS =================
    char deviceName[]        = "EPOS4";
    char protocolStackName[] = "MAXON SERIAL V2";
    char interfaceName[]     = "USB";
    char portName[]          = "USB0";
    const unsigned short nodeId = 3;

    // Motor scaling
    const double ratedTorque_Nm = 1.068;   // fallback if 0x6076 read is unavailable

    // Encoder scaling
    // Put your real encoder counts/rev here
const int counts_per_rev = 4096;   // try this first if your encoder is quadrature-counted
const double Kp_Nm_per_rad   = 0.0;
const double Kd_Nm_per_rad_s = 0.0;

    // Desired sinusoidal POSITION trajectory
    // theta_d(t) = offset + A*sin(2*pi*f*t)
    const double trajAmplitude_deg = 40.0;   // peak angle
    const double trajFrequency_Hz  = 0.5;   // Hz
    const double trajOffset_deg    = 0.0;    // deg


    // Optional torque bias
    const double torqueBias_Nm = 0.0;

    // Velocity estimate low-pass filter
    const double velFilterAlpha = 0.2;   // 0..1, larger = less filtering

    // Safety
    const uint16_t maxTorque_permille = 400; // writes 0x6072 = 40% rated
    const int16_t  clampCommand_pm    = 400; // also software clamp = 40% rated

    // Timing
    const double runSeconds  = 20.0;
    const int loopPeriod_ms  = 2;            // 500 Hz

    // =================================================
    std::cout << std::fixed << std::setprecision(5);

    // Open device
    handle = VCS_OpenDevice(deviceName, protocolStackName, interfaceName, portName, &err);
    if (!ok(handle != nullptr, "OpenDevice")) return 1;

    // Clear faults
    if (!ok(VCS_ClearFault(handle, nodeId, &err), "ClearFault")) {
        VCS_CloseDevice(handle, &err);
        return 1;
    }

    // Read rated torque from drive if available
    uint16_t ratedTorque_mNm = 0;
    double ratedTorqueUsed_Nm = ratedTorque_Nm;
    if (sdo_read_any(nodeId, 0x6076, 0x00, &ratedTorque_mNm, sizeof(ratedTorque_mNm), "Read 0x6076 RatedTorque")) {
        ratedTorqueUsed_Nm = ratedTorque_mNm / 1000.0;
        std::cout << "[INFO] Drive stored rated torque = "
                  << ratedTorque_mNm << " mN.m = "
                  << ratedTorqueUsed_Nm << " N.m\n";
    } else {
        std::cerr << "[WARN] Could not read 0x6076 Rated Torque. Using code value = "
                  << ratedTorqueUsed_Nm << " N.m\n";
    }

    // Set CST mode = 10
    {
        int8_t op = 10;
        if (!ok(VCS_SetOperationMode(handle, nodeId, op, &err), "SetOperationMode=CST")) {
            VCS_CloseDevice(handle, &err);
            return 1;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(5));

        int8_t op_disp = -1;
        sdo_read_any(nodeId, IDX_OPMODE_DISP, SUB0, &op_disp, sizeof(op_disp), "Read ModeDisplay");
        std::cout << "[INFO] Mode display = " << static_cast<int>(op_disp) << "\n";
    }

    // Set max torque 0x6072 if available
    {
        uint16_t probe = 0;
        if (sdo_read_any(nodeId, IDX_MAX_TORQUE, SUB0, &probe, sizeof(probe), "Probe 0x6072")) {
            uint16_t maxTq = maxTorque_permille;
            sdo_write_u16(nodeId, IDX_MAX_TORQUE, SUB0, &maxTq, sizeof(maxTq), "Set 0x6072 Max Torque");
            std::cout << "[INFO] Max torque set to " << maxTq << " permille\n";
        } else {
            std::cerr << "[INFO] 0x6072 not supported; continuing.\n";
        }
    }

    // Enable
    if (!ok(VCS_SetEnableState(handle, nodeId, &err), "SetEnableState")) {
        VCS_CloseDevice(handle, &err);
        return 1;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    std::cout << "[INFO] Running CST with outer PD position control\n";
    std::cout << "       theta_d(t) = " << trajOffset_deg
              << " + " << trajAmplitude_deg
              << " * sin(2*pi*" << trajFrequency_Hz << "*t) [deg]\n";
    std::cout << "       Kp = " << Kp_Nm_per_rad   << " Nm/rad\n";
    std::cout << "       Kd = " << Kd_Nm_per_rad_s << " Nm/(rad/s)\n";
    std::cout << "       loop = " << loopPeriod_ms << " ms\n";

    // Initial position
    int32_t pos0_counts = 0;
    if (!sdo_read_any(nodeId, IDX_POS_ACT, SUB0, &pos0_counts, sizeof(pos0_counts), "Read initial position")) {
        VCS_SetDisableState(handle, nodeId, &err);
        VCS_CloseDevice(handle, &err);
        return 1;
    }

    const auto t0 = std::chrono::steady_clock::now();
    auto next = t0;
    const auto period = std::chrono::milliseconds(loopPeriod_ms);
    const double dt = loopPeriod_ms / 1000.0;

    int tick = 0;
    const int printDivider = std::max(1, 100 / loopPeriod_ms); // print ~ every 100 ms

    int32_t pos_prev_counts = pos0_counts;
    double vel_meas_rad_s = 0.0;

    while (std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count() < runSeconds) {
        const auto now = std::chrono::steady_clock::now();
        const double t = std::chrono::duration<double>(now - t0).count();

        // Read actual position
        int32_t pos_counts = 0;
        if (!sdo_read_any(nodeId, IDX_POS_ACT, SUB0, &pos_counts, sizeof(pos_counts), "Read 0x6064 Position")) {
            break;
        }

        // Relative position from start
        const int32_t pos_rel_counts = pos_counts - pos0_counts;
        const double theta_meas_rad =
            (static_cast<double>(pos_rel_counts) / static_cast<double>(counts_per_rev)) * 2.0 * M_PI;

        // Velocity estimate from position difference
        const int32_t dcounts = pos_counts - pos_prev_counts;
        const double vel_raw_rad_s =
            (static_cast<double>(dcounts) / static_cast<double>(counts_per_rev)) * 2.0 * M_PI / dt;

        vel_meas_rad_s = velFilterAlpha * vel_raw_rad_s + (1.0 - velFilterAlpha) * vel_meas_rad_s;
        pos_prev_counts = pos_counts;

        // Desired trajectory
        const double A_rad      = deg2rad(trajAmplitude_deg);
        const double offset_rad = deg2rad(trajOffset_deg);
        const double omega      = 2.0 * M_PI * trajFrequency_Hz;

        const double theta_des_rad = offset_rad + A_rad * std::sin(omega * t);
        const double vel_des_rad_s = A_rad * omega * std::cos(omega * t);

        // PD control
        const double e_pos = theta_des_rad - theta_meas_rad;
        const double e_vel = vel_des_rad_s - vel_meas_rad_s;

        double tau_cmd = Kp_Nm_per_rad * e_pos
                       + Kd_Nm_per_rad_s * e_vel
                       + torqueBias_Nm;

        // Clamp in physical units
        const double tau_limit = ratedTorqueUsed_Nm * (static_cast<double>(clampCommand_pm) / 1000.0);
        tau_cmd = std::clamp(tau_cmd, -tau_limit, +tau_limit);

        if (!set_torque_Nm(nodeId, tau_cmd, ratedTorqueUsed_Nm, clampCommand_pm)) {
            break;
        }

        if (++tick % printDivider == 0) {
            int16_t tq_pm = 0;
            sdo_read_any(nodeId, IDX_TORQUE_ACT, SUB0, &tq_pm, sizeof(tq_pm), "Read 0x6077 Torque");

            const double tq_act_Nm = (tq_pm / 1000.0) * ratedTorqueUsed_Nm;

            std::cout << "t=" << t
                      << " s | th_d="   << rad2deg(theta_des_rad)
                      << " deg | th="   << rad2deg(theta_meas_rad)
                      << " deg | w_d="  << rad2deg(vel_des_rad_s)
                      << " deg/s | w="  << rad2deg(vel_meas_rad_s)
                      << " deg/s | e="  << rad2deg(e_pos)
                      << " deg | tau="  << tau_cmd
                      << " Nm | tau_act=" << tq_act_Nm
                      << " Nm\n";
        }

        next += period;
        std::this_thread::sleep_until(next);
    }

    // Stop with zero torque
    {
        double tau_stop = 0.0;
        set_torque_Nm(nodeId, tau_stop, ratedTorqueUsed_Nm, clampCommand_pm);
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    // Disable and close
    VCS_SetDisableState(handle, nodeId, &err);
    VCS_CloseDevice(handle, &err);

    std::cout << "[INFO] Done.\n";
    return 0;
}
