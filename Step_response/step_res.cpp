#include <Definitions.h>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <thread>


static void* handle = nullptr;
static unsigned int err = 0;

static inline bool ok(bool success, const char* step)
{
    if (!success) {
        std::cerr << "[EPOS ERROR] " << step << "  err=0x"
                  << std::hex << err << std::dec << "\n";
    }
    return success;
}

// ============================================================
// SDO helpers
// ============================================================

static inline bool sdo_write_any(
    unsigned short nodeId,
    unsigned short idx,
    unsigned char sub,
    const void* p,
    unsigned int n,
    const char* tag)
{
    unsigned int nW = 0;
    return ok(VCS_SetObject(handle, nodeId, idx, sub,
                            const_cast<void*>(p), n, &nW, &err), tag);
}

static inline bool sdo_read_any(
    unsigned short nodeId,
    unsigned short idx,
    unsigned char sub,
    void* p,
    unsigned int n,
    const char* tag)
{
    unsigned int nR = 0;
    return ok(VCS_GetObject(handle, nodeId, idx, sub, p, n, &nR, &err), tag);
}

// ============================================================
// EPOS object dictionary indices
// ============================================================

constexpr unsigned char  SUB0              = 0x00;
constexpr unsigned short IDX_OPMODE_DISP   = 0x6061;
constexpr unsigned short IDX_TARGET_TORQUE = 0x6071;
constexpr unsigned short IDX_TORQUE_ACT    = 0x6077;
constexpr unsigned short IDX_RATED_TORQUE  = 0x6076;
constexpr unsigned short IDX_MAX_TORQUE    = 0x6072;
constexpr unsigned short IDX_POS_ACT       = 0x6064;
constexpr unsigned short IDX_VEL_ACT       = 0x606C;
constexpr unsigned short IDX_CURRENT_ACTS  = 0x30D1; // sub 1 averaged, sub 2 actual

// ============================================================
// Utility functions
// ============================================================

static inline double clamp_double(double x, double lo, double hi)
{
    return std::max(lo, std::min(x, hi));
}

static inline int16_t nm_to_permille(
    double tau_Nm,
    double ratedTorque_Nm,
    int16_t clamp_abs_pm)
{
    if (ratedTorque_Nm <= 0.0) return 0;

    double pm_f = (tau_Nm / ratedTorque_Nm) * 1000.0;
    long pm_l = std::lround(pm_f);

    if (pm_l >  clamp_abs_pm) pm_l =  clamp_abs_pm;
    if (pm_l < -clamp_abs_pm) pm_l = -clamp_abs_pm;

    return static_cast<int16_t>(pm_l);
}

static inline bool set_torque_permille(unsigned short nodeId, int16_t torque_pm)
{
    return sdo_write_any(nodeId,
                         IDX_TARGET_TORQUE,
                         SUB0,
                         &torque_pm,
                         sizeof(torque_pm),
                         "Write target torque 0x6071");
}

static inline bool set_torque_Nm(
    unsigned short nodeId,
    double tau_motor_Nm,
    double ratedTorque_Nm,
    int16_t clamp_abs_pm)
{
    int16_t pm = nm_to_permille(tau_motor_Nm, ratedTorque_Nm, clamp_abs_pm);
    return set_torque_permille(nodeId, pm);
}

// ============================================================
// Main
// ============================================================

int main()
{
    // ========================================================
    // USER SETTINGS
    // ========================================================

    char deviceName[]        = "EPOS4";
    char protocolStackName[] = "MAXON SERIAL V2";
    char interfaceName[]     = "USB";
    char portName[]          = "USB0";

    const unsigned short nodeId = 3;

    // Encoder and gearbox
    const double encoder_cpt = 1024.0;     // MILE encoder pulses per motor rev
    const double quadrature  = 4.0;        // EPOS counts 4x
    const double gearRatio   = 80.0;       // harmonic drive ratio

    // IMPORTANT SIGN:
    // Your previous convention:
    // q_link = -count * 360 / (1024*4*80)
    const double encoderToLinkSign = 1.0;

    // Torque conversion sign:
    // From your feedforward-only test, motor torque = -joint torque / N
    const double jointToMotorTorqueSign = -1.0;

    // Motor constants
    double ratedTorque_Nm = 1.068;         // fallback if 0x6076 read fails
    const double Kt_Nm_per_A = 0.0712;     // used only for estimated current log

    // PD gains at LINK side
    // tau_joint = Kp * error + Kd * error_dot
    // Units:
    // Kp: Nm/deg
    // Kd: Nm/(deg/s)
    //
    // Start low. Increase slowly.
    const double Kp_Nm_per_deg     = 10.0;
    const double Kd_Nm_per_deg_s   = 0.37;

    // Limits
    const double maxJointTorque_Nm = 40.0;      // link-side torque limit
    const double maxMotorTorque_Nm = 1.25;     // motor-side torque limit
    const int16_t maxTorque_pm     = 600;      // 600 permille = 60% rated torque

    // Motion stop condition
    const double positionTolerance_deg = 0.2;
    const double velocityTolerance_deg_s = 2.0;
    const double holdTime_s = 1.0;

    // Loop settings
    const int runSeconds = 15;
    const int loopPeriod_ms = 2;

    const char logFileName[] = "pd_joint_position_log.csv";

    // ========================================================
    // User input
    // ========================================================

    double q_ref_deg = 0.0;

    std::cout << std::fixed << std::setprecision(5);
    std::cout << "Enter reference LINK joint angle in deg: ";
    std::cin >> q_ref_deg;

    std::cout << "\nReference joint angle = " << q_ref_deg << " deg\n";
    std::cout << "Kp = " << Kp_Nm_per_deg << " Nm/deg\n";
    std::cout << "Kd = " << Kd_Nm_per_deg_s << " Nm/(deg/s)\n";
    std::cout << "encoderToLinkSign = " << encoderToLinkSign << "\n";
    std::cout << "jointToMotorTorqueSign = " << jointToMotorTorqueSign << "\n\n";

    // ========================================================
    // Open CSV log
    // ========================================================

    std::ofstream logFile(logFileName);

    if (!logFile.is_open()) {
        std::cerr << "[ERROR] Could not open CSV log file.\n";
        return 1;
    }

    logFile << "time_s,"
            << "encoder_count,"
            << "q_ref_deg,"
            << "q_link_deg,"
            << "error_deg,"
            << "qdot_link_deg_s,"
            << "error_dot_deg_s,"
            << "tau_pd_joint_raw_Nm,"
            << "tau_pd_joint_clamped_Nm,"
            << "tau_motor_cmd_Nm,"
            << "target_torque_permille,"
            << "actual_torque_permille,"
            << "actual_torque_Nm,"
            << "velocity_motor_rpm_606C,"
            << "current_actual_mA,"
            << "current_avg_mA,"
            << "estimated_current_cmd_A,"
            << "mode_display\n";

    // ========================================================
    // Open EPOS device
    // ========================================================

    handle = VCS_OpenDevice(deviceName,
                            protocolStackName,
                            interfaceName,
                            portName,
                            &err);

    if (!ok(handle != nullptr, "OpenDevice")) {
        return 1;
    }

    // Clear faults
    if (!ok(VCS_ClearFault(handle, nodeId, &err), "ClearFault")) {
        VCS_CloseDevice(handle, &err);
        return 1;
    }

    // Read rated torque from drive
    uint32_t ratedTorque_mNm = 0;
    if (sdo_read_any(nodeId,
                     IDX_RATED_TORQUE,
                     SUB0,
                     &ratedTorque_mNm,
                     sizeof(ratedTorque_mNm),
                     "Read motor rated torque 0x6076"))
    {
        if (ratedTorque_mNm > 0) {
            ratedTorque_Nm = ratedTorque_mNm / 1000000.00;
        }

        std::cout << "[INFO] Drive rated torque = "
                  << ratedTorque_mNm << " mNm = "
                  << ratedTorque_Nm << " Nm\n";
    }
    else {
        std::cout << "[WARN] Could not read 0x6076. Using fallback ratedTorque_Nm = "
                  << ratedTorque_Nm << "\n";
    }

    // ========================================================
    // Set CST mode
    // ========================================================

    {
        int8_t opMode = 10; // CST = 0x0A

        if (!ok(VCS_SetOperationMode(handle, nodeId, opMode, &err),
                "SetOperationMode CST")) {
            VCS_CloseDevice(handle, &err);
            return 1;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(10));

        int8_t opDisplay = 0;
        sdo_read_any(nodeId,
                     IDX_OPMODE_DISP,
                     SUB0,
                     &opDisplay,
                     sizeof(opDisplay),
                     "Read mode display 0x6061");

        if (opDisplay != 10) {
            std::cerr << "[WARN] Mode display is not CST. Got "
                      << static_cast<int>(opDisplay) << "\n";
        }
    }

    // Set max torque limit if supported
    {
        uint16_t maxTq = static_cast<uint16_t>(maxTorque_pm);
        sdo_write_any(nodeId,
                      IDX_MAX_TORQUE,
                      SUB0,
                      &maxTq,
                      sizeof(maxTq),
                      "Set max torque 0x6072");
    }

    // Enable drive
    if (!ok(VCS_SetEnableState(handle, nodeId, &err), "SetEnableState")) {
        VCS_CloseDevice(handle, &err);
        return 1;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(20));

    // ========================================================
    // Initialize feedback variables
    // ========================================================

    int32_t count0 = 0;
    sdo_read_any(nodeId,
                 IDX_POS_ACT,
                 SUB0,
                 &count0,
                 sizeof(count0),
                 "Initial read position 0x6064");

    const double q0_link_deg =
        encoderToLinkSign * static_cast<double>(count0) * 360.0 /
        (encoder_cpt * quadrature * gearRatio);

    double prev_q_link_deg = q0_link_deg;
    double prev_time_s = 0.0;

    double holdTimer_s = 0.0;

    std::cout << "[INFO] Initial encoder count = " << count0 << "\n";
    std::cout << "[INFO] Initial link angle = " << q0_link_deg << " deg\n";
    std::cout << "[INFO] Running PD position control in CST mode...\n";
    std::cout << "[INFO] CSV log: " << logFileName << "\n\n";

    // ========================================================
    // Main control loop
    // ========================================================

    auto t0 = std::chrono::steady_clock::now();
    auto period = std::chrono::milliseconds(loopPeriod_ms);

    int tick = 0;

    while (std::chrono::steady_clock::now() - t0 <
           std::chrono::seconds(runSeconds))
    {
        auto now = std::chrono::steady_clock::now();
        double time_s = std::chrono::duration<double>(now - t0).count();

        double dt = time_s - prev_time_s;
        if (dt <= 0.0) dt = loopPeriod_ms / 1000.0;

        // ----------------------------------------------------
        // Read EPOS feedback
        // ----------------------------------------------------

        int32_t encoder_count = 0;
        int32_t velocity_motor_rpm = 0;
        int16_t actual_torque_pm = 0;
        int32_t current_avg_mA = 0;
        int32_t current_actual_mA = 0;
        int8_t mode_display = 0;

        sdo_read_any(nodeId,
                     IDX_POS_ACT,
                     SUB0,
                     &encoder_count,
                     sizeof(encoder_count),
                     "Read position actual 0x6064");

        sdo_read_any(nodeId,
                     IDX_VEL_ACT,
                     SUB0,
                     &velocity_motor_rpm,
                     sizeof(velocity_motor_rpm),
                     "Read velocity actual 0x606C");

        sdo_read_any(nodeId,
                     IDX_TORQUE_ACT,
                     SUB0,
                     &actual_torque_pm,
                     sizeof(actual_torque_pm),
                     "Read torque actual 0x6077");

        sdo_read_any(nodeId,
                     IDX_CURRENT_ACTS,
                     0x01,
                     &current_avg_mA,
                     sizeof(current_avg_mA),
                     "Read current actual averaged 0x30D1-01");

        sdo_read_any(nodeId,
                     IDX_CURRENT_ACTS,
                     0x02,
                     &current_actual_mA,
                     sizeof(current_actual_mA),
                     "Read current actual 0x30D1-02");

        sdo_read_any(nodeId,
                     IDX_OPMODE_DISP,
                     SUB0,
                     &mode_display,
                     sizeof(mode_display),
                     "Read mode display 0x6061");

        // ----------------------------------------------------
        // Convert encoder count to LINK angle
        // ----------------------------------------------------

        double q_link_deg =
            encoderToLinkSign * static_cast<double>(encoder_count) * 360.0 /
            (encoder_cpt * quadrature * gearRatio);

        double qdot_link_deg_s =
            (q_link_deg - prev_q_link_deg) / dt;

        // ----------------------------------------------------
        // PD control at LINK side
        // ----------------------------------------------------

        double error_deg = q_ref_deg - q_link_deg;

        // Reference velocity is zero for point-to-point hold
        double error_dot_deg_s = 0.0 - qdot_link_deg_s;

// PD torque BEFORE clamping
double tau_pd_joint_raw_Nm =
    Kp_Nm_per_deg   * error_deg +
    Kd_Nm_per_deg_s * error_dot_deg_s;

// PD torque AFTER clamping
double tau_pd_joint_Nm =
    clamp_double(tau_pd_joint_raw_Nm,
                 -maxJointTorque_Nm,
                  maxJointTorque_Nm);

// Convert joint torque to motor torque through gearbox
double tau_motor_cmd_Nm =
    jointToMotorTorqueSign * tau_pd_joint_Nm / gearRatio;

        tau_motor_cmd_Nm =
            clamp_double(tau_motor_cmd_Nm,
                         -maxMotorTorque_Nm,
                          maxMotorTorque_Nm);

        int16_t target_torque_pm =
            nm_to_permille(tau_motor_cmd_Nm,
                           ratedTorque_Nm,
                           maxTorque_pm);

        // ----------------------------------------------------
        // Send torque command
        // ----------------------------------------------------

        if (!set_torque_permille(nodeId, target_torque_pm)) {
            std::cerr << "[ERROR] Failed to command torque. Breaking loop.\n";
            break;
        }

        double actual_torque_Nm =
            (static_cast<double>(actual_torque_pm) / 1000.0) * ratedTorque_Nm;

        double estimated_current_cmd_A =
            (Kt_Nm_per_A > 0.0) ? tau_motor_cmd_Nm / Kt_Nm_per_A : 0.0;

        // ----------------------------------------------------
        // Log CSV
        // ----------------------------------------------------
logFile << time_s << ","
        << encoder_count << ","
        << q_ref_deg << ","
        << q_link_deg << ","
        << error_deg << ","
        << qdot_link_deg_s << ","
        << error_dot_deg_s << ","
        << tau_pd_joint_raw_Nm << ","
        << tau_pd_joint_Nm << ","
        << tau_motor_cmd_Nm << ","
        << target_torque_pm << ","
        << actual_torque_pm << ","
        << actual_torque_Nm << ","
        << velocity_motor_rpm << ","
        << current_actual_mA << ","
        << current_avg_mA << ","
        << estimated_current_cmd_A << ","
        << static_cast<int>(mode_display) << "\n";
        // ----------------------------------------------------
        // Print every ~100 ms
        // ----------------------------------------------------

        int printEvery = std::max(1, 100 / loopPeriod_ms);

        if (++tick % printEvery == 0) {
            std::cout << "t=" << time_s
                      << "  q_ref=" << q_ref_deg
                      << "  q=" << q_link_deg
                      << "  err=" << error_deg
                      << "  qdot=" << qdot_link_deg_s
                      << "  tauJ_raw=" << tau_pd_joint_raw_Nm
                      << "  tauJ=" << tau_pd_joint_Nm
                      << "  tauM=" << tau_motor_cmd_Nm
                      << "  cmd=" << target_torque_pm << " pm"
                      << "  I=" << current_actual_mA << " mA"
                      << "\n";
        }

        // ----------------------------------------------------
        // Optional stop condition after reaching target
        // ----------------------------------------------------

        if (std::abs(error_deg) < positionTolerance_deg &&
            std::abs(qdot_link_deg_s) < velocityTolerance_deg_s)
        {
            holdTimer_s += dt;
            if (holdTimer_s >= holdTime_s) {
                std::cout << "[INFO] Target reached and held for "
                          << holdTime_s << " s.\n";
                break;
            }
        }
        else {
            holdTimer_s = 0.0;
        }

        prev_q_link_deg = q_link_deg;
        prev_time_s = time_s;

        std::this_thread::sleep_for(period);
    }

    // ========================================================
    // Ramp torque down to zero
    // ========================================================

    std::cout << "[INFO] Ramping torque to zero...\n";

    for (int16_t pm = 0; ; ) {
        // Read current commanded value approximately from last loop is not stored here,
        // so just command zero directly in small repeated writes.
        pm = 0;
        set_torque_permille(nodeId, pm);
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
        break;
    }

    // Extra zero commands for safety
    for (int i = 0; i < 10; ++i) {
        set_torque_permille(nodeId, 0);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    // ========================================================
    // Close
    // ========================================================

    logFile.close();

    VCS_SetDisableState(handle, nodeId, &err);
    VCS_CloseDevice(handle, &err);

    std::cout << "[DONE] CSV saved as: " << logFileName << "\n";

    return 0;
}
