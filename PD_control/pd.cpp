#include <Definitions.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

// ---------- Globals ----------
static void* handle = nullptr;
static unsigned int err = 0;

// ---------- Basic helpers ----------
static inline bool ok(bool success, const char* step)
{
    if (!success) {
        std::cerr << "[EPOS ERROR] " << step
                  << " (err=0x" << std::hex << err << std::dec << ")\n";
    }
    return success;
}

// ---------- SDO helpers ----------
static inline bool sdo_write_u16(unsigned short nodeId,
                                 unsigned short idx,
                                 unsigned char sub,
                                 const void* p,
                                 unsigned int n,
                                 const char* tag)
{
    unsigned int nW = 0;
    return ok(VCS_SetObject(handle, nodeId, idx, sub,
                            const_cast<void*>(p), n, &nW, &err) && nW == n,
              tag);
}

static inline bool sdo_write_any(unsigned short nodeId,
                                 unsigned short idx,
                                 unsigned char sub,
                                 const void* p,
                                 unsigned int n,
                                 const char* tag)
{
    unsigned int nW = 0;
    return ok(VCS_SetObject(handle, nodeId, idx, sub,
                            const_cast<void*>(p), n, &nW, &err),
              tag);
}

static inline bool sdo_read_any(unsigned short nodeId,
                                unsigned short idx,
                                unsigned char sub,
                                void* p,
                                unsigned int n,
                                const char* tag)
{
    unsigned int nR = 0;
    return ok(VCS_GetObject(handle, nodeId, idx, sub, p, n, &nR, &err),
              tag);
}

// ---------- Object indices ----------
constexpr unsigned char  SUB0              = 0x00;
constexpr unsigned short IDX_OPMODE        = 0x6060;
constexpr unsigned short IDX_OPMODE_DISP   = 0x6061;
constexpr unsigned short IDX_TARGET_TORQUE = 0x6071;
constexpr unsigned short IDX_TORQUE_ACT    = 0x6077;
constexpr unsigned short IDX_MAX_TORQUE    = 0x6072;
constexpr unsigned short IDX_POS_ACT       = 0x6064;
constexpr unsigned short IDX_VEL_ACT       = 0x606C;

// ---------- Math helpers ----------
static constexpr double PI = 3.14159265358979323846;

static inline double deg2rad(double deg)
{
    return deg * PI / 180.0;
}

static inline double rad2deg(double rad)
{
    return rad * 180.0 / PI;
}

static inline double signum(double x)
{
    if (x > 0.0) return 1.0;
    if (x < 0.0) return -1.0;
    return 0.0;
}

static inline int16_t nm_to_permille(double tau_Nm,
                                     double ratedTorque_Nm,
                                     int16_t clamp_abs_pm = 1000)
{
    const double pm_f = (tau_Nm / ratedTorque_Nm) * 1000.0;
    long pm_l = std::lround(pm_f);

    if (pm_l >  clamp_abs_pm) pm_l =  clamp_abs_pm;
    if (pm_l < -clamp_abs_pm) pm_l = -clamp_abs_pm;

    return static_cast<int16_t>(pm_l);
}

static inline bool set_torque_Nm(unsigned short nodeId,
                                 double tau_Nm,
                                 double ratedTorque_Nm,
                                 int16_t clamp_abs_pm = 1000)
{
    int16_t permille = nm_to_permille(tau_Nm, ratedTorque_Nm, clamp_abs_pm);

    return sdo_write_any(nodeId,
                         IDX_TARGET_TORQUE,
                         SUB0,
                         &permille,
                         sizeof(permille),
                         "SDO: Set 0x6071 Target Torque");
}

// ---------- Trajectory data ----------
struct Sample
{
    double q_deg = 0.0;       // desired joint angle from CSV [deg]
    double qdd_deg_s2 = 0.0;  // desired joint acceleration from CSV [deg/s^2]
};

struct LogRow
{
    double t_s = 0.0;

    // Desired/reference joint values
    double q_ref_deg = 0.0;
    double qd_ref_deg_s = 0.0;
    double qdd_ref_deg_s2 = 0.0;

    // Actual link/joint values from encoder
    double q_link_deg = 0.0;
    double qd_link_deg_s = 0.0;

    // Errors
    double pos_error_deg = 0.0;
    double vel_error_deg_s = 0.0;

    // Torques
    double tau_joint_ff_Nm = 0.0;
    double tau_pd_joint_Nm = 0.0;
    double tau_joint_total_Nm = 0.0;
    double tau_dead_motor_Nm = 0.0;
    double tau_motor_cmd_Nm = 0.0;
    double tau_motor_actual_Nm = 0.0;

    // Raw EPOS values
    int32_t motor_pos_counts = 0;
    int32_t motor_vel_native = 0;
};

// ---------- CSV parsing ----------
static inline std::string trim(const std::string& s)
{
    const auto b = s.find_first_not_of(" \t\r\n");
    if (b == std::string::npos) return "";

    const auto e = s.find_last_not_of(" \t\r\n");
    return s.substr(b, e - b + 1);
}

static bool parse_two_numeric_columns(const std::string& line,
                                      double& a,
                                      double& b)
{
    std::stringstream ss(line);
    std::string item;
    std::vector<std::string> cols;

    while (std::getline(ss, item, ',')) {
        cols.push_back(trim(item));
    }

    if (cols.size() < 2) return false;

    try {
        a = std::stod(cols[0]);
        b = std::stod(cols[1]);
        return true;
    } catch (...) {
        return false;
    }
}

static bool load_csv_two_columns(const std::string& csvPath,
                                 std::vector<Sample>& samples)
{
    std::ifstream fin(csvPath);

    if (!fin.is_open()) {
        std::cerr << "[ERROR] Could not open CSV: " << csvPath << "\n";
        return false;
    }

    samples.clear();

    std::string line;

    while (std::getline(fin, line)) {
        line = trim(line);

        if (line.empty()) continue;

        double q_deg = 0.0;
        double qdd_deg_s2 = 0.0;

        if (!parse_two_numeric_columns(line, q_deg, qdd_deg_s2)) {
            // Skip header or malformed rows
            continue;
        }

        Sample s;
        s.q_deg = q_deg;
        s.qdd_deg_s2 = qdd_deg_s2;

        samples.push_back(s);
    }

    if (samples.empty()) {
        std::cerr << "[ERROR] No valid numeric rows found in CSV.\n";
        return false;
    }

    return true;
}

// ---------- Desired velocity estimation from CSV ----------
static std::vector<double> estimate_qd_deg_s(const std::vector<Sample>& samples,
                                             double dt)
{
    const size_t n = samples.size();
    std::vector<double> qd(n, 0.0);

    if (n < 2 || dt <= 0.0) return qd;

    // Forward difference for first point
    qd[0] = (samples[1].q_deg - samples[0].q_deg) / dt;

    // Central difference for middle points
    for (size_t i = 1; i + 1 < n; ++i) {
        qd[i] = (samples[i + 1].q_deg - samples[i - 1].q_deg) / (2.0 * dt);
    }

    // Backward difference for last point
    qd[n - 1] = (samples[n - 1].q_deg - samples[n - 2].q_deg) / dt;

    return qd;
}

// ---------- Feedforward parameters ----------
struct FFParams
{
    double I1_kg_m2 = 0.0;      // link inertia about joint axis [kg*m^2]
    double m1_kg = 0.0;         // link mass [kg]
    double lc1_m = 0.0;         // COM distance from joint axis [m]
    double g = 9.81;            // gravity [m/s^2]

    double gearRatio = 1.0;     // motor angle / joint angle
    double efficiency = 1.0;    // gearbox efficiency

    double deadTorque_motor_Nm = 0.1602; // motor-side dead torque [Nm]
    double velThreshold_deg_s = 0.5;     // desired velocity threshold [deg/s]

    double maxMotorTorqueCmd_Nm = 0.50;  // software torque clamp [Nm]
};

// ---------- PD parameters ----------
struct PDParams
{
    // Joint-side proportional gain [Nm/rad]
    double Kp = 0.0;

    // Joint-side derivative gain [Nm/(rad/s)]
    double Kd = 0.0;
};

// ---------- Dynamics ----------
static double compute_joint_feedforward_torque_Nm(double q_deg,
                                                  double qdd_deg_s2,
                                                  const FFParams& p)
{
    // Convert desired angle and acceleration to radians
    const double q_rad = deg2rad(q_deg);
    const double qdd_rad_s2 = deg2rad(qdd_deg_s2);

    // Inertia torque: I*qdd
    const double inertia_term = p.I1_kg_m2 * qdd_rad_s2;

    // Gravity torque: m*g*lc*sin(q)
    const double gravity_term = p.m1_kg * p.g * p.lc1_m * std::sin(q_rad);

    // Total feedforward torque on joint side
    return inertia_term + gravity_term;
}

static double compute_pd_torque_joint_Nm(double q_ref_deg,
                                         double q_link_deg,
                                         double qd_ref_deg_s,
                                         double qd_link_deg_s,
                                         const PDParams& pd)
{
    // Position error [deg]
    const double error_deg = q_ref_deg - q_link_deg;

    // Velocity error [deg/s]
    const double error_dot_deg_s = qd_ref_deg_s - qd_link_deg_s;

    // Convert errors to SI units
    const double error_rad = deg2rad(error_deg);
    const double error_dot_rad_s = deg2rad(error_dot_deg_s);

    // Joint-side PD torque [Nm]
    return pd.Kp * error_rad + pd.Kd * error_dot_rad_s;
}

static double joint_to_motor_torque_Nm(double tau_joint_Nm,
                                       const FFParams& p)
{
    const double N = (p.gearRatio <= 0.0) ? 1.0 : p.gearRatio;
    const double eta = (p.efficiency <= 0.0) ? 1.0 : p.efficiency;

    // Motor torque = joint torque / gear ratio / efficiency
    return tau_joint_Nm / (N * eta);
}

static double signed_dead_torque_Nm(double qd_ref_deg_s,
                                    const FFParams& p)
{
    // If desired sine velocity is close to zero, do not add dead torque
    if (std::abs(qd_ref_deg_s) < p.velThreshold_deg_s) {
        return 0.0;
    }

    // Dead torque sign follows desired joint velocity direction
    return p.deadTorque_motor_Nm * signum(qd_ref_deg_s);
}

// ---------- Logging ----------
static bool save_log_csv(const std::string& outPath,
                         const std::vector<LogRow>& logs)
{
    std::ofstream fout(outPath);

    if (!fout.is_open()) {
        std::cerr << "[ERROR] Could not open log file for writing: "
                  << outPath << "\n";
        return false;
    }

    fout << "t_s,"
         << "q_ref_deg,"
         << "qd_ref_deg_s,"
         << "qdd_ref_deg_s2,"
         << "q_link_deg,"
         << "qd_link_deg_s,"
         << "pos_error_deg,"
         << "vel_error_deg_s,"
         << "tau_joint_ff_Nm,"
         << "tau_pd_joint_Nm,"
         << "tau_joint_total_Nm,"
         << "tau_dead_motor_Nm,"
         << "tau_motor_cmd_Nm,"
         << "tau_motor_actual_Nm,"
         << "motor_pos_counts,"
         << "motor_vel_native\n";

    fout << std::fixed << std::setprecision(6);

    for (const auto& r : logs) {
        fout << r.t_s << ","
             << r.q_ref_deg << ","
             << r.qd_ref_deg_s << ","
             << r.qdd_ref_deg_s2 << ","
             << r.q_link_deg << ","
             << r.qd_link_deg_s << ","
             << r.pos_error_deg << ","
             << r.vel_error_deg_s << ","
             << r.tau_joint_ff_Nm << ","
             << r.tau_pd_joint_Nm << ","
             << r.tau_joint_total_Nm << ","
             << r.tau_dead_motor_Nm << ","
             << r.tau_motor_cmd_Nm << ","
             << r.tau_motor_actual_Nm << ","
             << r.motor_pos_counts << ","
             << r.motor_vel_native << "\n";
    }

    return true;
}

// ---------- Main ----------
int main()
{
    // =========================================================
    // USER SETTINGS
    // =========================================================

    // EPOS connection
    char deviceName[]        = "EPOS4";
    char protocolStackName[] = "MAXON SERIAL V2";
    char interfaceName[]     = "USB";
    char portName[]          = "USB0";
    const unsigned short nodeId = 5;

    // Files
    const std::string inputCsvPath = "trajectory.csv";
    const std::string outputLogPath = "pd_feedforward_log.csv";

    // CSV sample time
    const double dt_s = 0.002; // 2 ms between CSV rows

    // Drive scaling
    const double ratedTorque_Nm = 1.068;  // must match EPOS 0x6076 / motor setting
    const double Kt_Nm_per_A    = 0.0712; // optional display only

    // Encoder resolution
    const double encoder_resolution = 1024.0 * 4.0;

    // Safety
    const uint16_t maxTorque_permille = 1000; // 1000 = 100% rated torque
    const int printEveryN = 50;

    // ---------- Dynamic model parameters ----------
    FFParams ff;

    ff.I1_kg_m2 = 0.019815364780523; // link inertia about joint axis [kg*m^2]
    ff.m1_kg = 0.650;                // link mass [kg]
    ff.lc1_m = 0.130;                // COM distance [m]
    ff.g = 9.81;

    ff.gearRatio = 80.0;
    ff.efficiency = 1.0;

    ff.deadTorque_motor_Nm = 0.1602;
    ff.velThreshold_deg_s = 0.5;

    // Motor-side software clamp.
    // Start low for safety.
    ff.maxMotorTorqueCmd_Nm = 0.50;

    // ---------- PD control parameters ----------
    PDParams pd;

    // These gains are on the JOINT SIDE.
    // Start small first.
    pd.Kp = 30.0;    // Nm/rad
    pd.Kd = 5;   // Nm/(rad/s)

    // =========================================================

    std::cout << std::fixed << std::setprecision(6);

    // ---------- Load trajectory ----------
    std::vector<Sample> samples;

    if (!load_csv_two_columns(inputCsvPath, samples)) {
        return 1;
    }

    // Desired joint velocity from desired joint angle
    const std::vector<double> qd_ref_deg_s_vec =
        estimate_qd_deg_s(samples, dt_s);

    std::cout << "[INFO] Loaded " << samples.size()
              << " samples from " << inputCsvPath << "\n";

    std::cout << "[INFO] dt = " << dt_s
              << " s, total time = " << samples.size() * dt_s
              << " s\n";

    std::cout << "[INFO] PD gains: Kp = " << pd.Kp
              << " Nm/rad, Kd = " << pd.Kd
              << " Nm/(rad/s)\n";

    // ---------- Open device ----------
    handle = VCS_OpenDevice(deviceName,
                            protocolStackName,
                            interfaceName,
                            portName,
                            &err);

    if (!ok(handle != nullptr, "OpenDevice")) {
        return 1;
    }

    // ---------- Clear faults ----------
    if (!ok(VCS_ClearFault(handle, nodeId, &err), "ClearFault")) {
        VCS_CloseDevice(handle, &err);
        return 1;
    }

    // ---------- Read stored rated torque ----------
    uint16_t ratedTorque_mNm = 0;

    if (sdo_read_any(nodeId,
                     0x6076,
                     0x00,
                     &ratedTorque_mNm,
                     sizeof(ratedTorque_mNm),
                     "Read 0x6076 RatedTorque")) {
        std::cout << "[INFO] Drive stored rated torque = "
                  << ratedTorque_mNm << " mN*m = "
                  << ratedTorque_mNm / 1000.0 << " N*m\n";
    } else {
        std::cerr << "[WARN] Could not read 0x6076.\n";
    }

    // ---------- Set CST mode while disabled ----------
    {
        int8_t op = 10; // CST mode

        if (!ok(VCS_SetOperationMode(handle, nodeId, op, &err),
                "SetOperationMode=CST")) {
            VCS_CloseDevice(handle, &err);
            return 1;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(5));

        int8_t opDisp = 0;

        sdo_read_any(nodeId,
                     IDX_OPMODE_DISP,
                     SUB0,
                     &opDisp,
                     sizeof(opDisp),
                     "Read 0x6061 OpmodeDisp");

        std::cout << "[INFO] Mode display = "
                  << static_cast<int>(opDisp)
                  << " ; expect 10 for CST\n";
    }

    // ---------- Set max torque while disabled ----------
    {
        uint16_t probe = 0;

        if (sdo_read_any(nodeId,
                         IDX_MAX_TORQUE,
                         0x00,
                         &probe,
                         sizeof(probe),
                         "Probe 0x6072")) {
            uint16_t maxTq = maxTorque_permille;

            sdo_write_u16(nodeId,
                          IDX_MAX_TORQUE,
                          0x00,
                          &maxTq,
                          sizeof(maxTq),
                          "Set 0x6072 MaxTorque");

            std::cout << "[INFO] Set max torque limit = "
                      << maxTq << " permille\n";
        } else {
            std::cerr << "[INFO] 0x6072 not supported; continuing.\n";
        }
    }

    // ---------- Enable drive ----------
    if (!ok(VCS_SetEnableState(handle, nodeId, &err), "SetEnableState")) {
        VCS_CloseDevice(handle, &err);
        return 1;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    // ---------- Main playback loop ----------
    std::vector<LogRow> logs;
    logs.reserve(samples.size());

    const auto t0 = std::chrono::steady_clock::now();
    const auto period = std::chrono::duration<double>(dt_s);

    bool first_velocity_sample = true;
    double prev_q_link_deg = 0.0;

    std::cout << "[INFO] Starting PD + feedforward torque playback...\n";

    for (size_t k = 0; k < samples.size(); ++k) {

        // =====================================================
        // 1. Desired trajectory from CSV
        // =====================================================

        // Desired joint angle [deg]
        const double q_ref_deg = samples[k].q_deg;

        // Desired joint acceleration [deg/s^2]
        const double qdd_ref_deg_s2 = samples[k].qdd_deg_s2;

        // Desired joint velocity calculated from CSV angle [deg/s]
        const double qd_ref_deg_s = qd_ref_deg_s_vec[k];

        // =====================================================
        // 2. Read actual encoder from EPOS
        // =====================================================

        int32_t pos_counts = 0;
        int32_t vel_native = 0;
        int16_t tq_pm = 0;

        sdo_read_any(nodeId,
                     IDX_POS_ACT,
                     SUB0,
                     &pos_counts,
                     sizeof(pos_counts),
                     "Read 0x6064 Position");

        sdo_read_any(nodeId,
                     IDX_VEL_ACT,
                     SUB0,
                     &vel_native,
                     sizeof(vel_native),
                     "Read 0x606C Velocity");

        sdo_read_any(nodeId,
                     IDX_TORQUE_ACT,
                     SUB0,
                     &tq_pm,
                     sizeof(tq_pm),
                     "Read 0x6077 Torque");

        // =====================================================
        // 3. Convert motor encoder counts to actual joint angle
        // =====================================================

        // Motor angle = counts * 360 / encoder_resolution
        // Joint angle = motor angle / gear ratio
        const double q_link_deg =
            (static_cast<double>(pos_counts) * 360.0) /
            (encoder_resolution * ff.gearRatio);

        // =====================================================
        // 4. Actual joint velocity by numerical differentiation
        // =====================================================

        double qd_link_deg_s = 0.0;

        if (first_velocity_sample) {
            qd_link_deg_s = 0.0;
            first_velocity_sample = false;
        } else {
            qd_link_deg_s = (q_link_deg - prev_q_link_deg) / dt_s;
        }

        prev_q_link_deg = q_link_deg;

        // =====================================================
        // 5. Dynamic model feedforward torque on joint side
        // =====================================================

        const double tau_joint_ff_Nm =
            compute_joint_feedforward_torque_Nm(q_ref_deg,
                                                qdd_ref_deg_s2,
                                                ff);

        // =====================================================
        // 6. PD feedback torque on joint side
        // =====================================================

        const double tau_pd_joint_Nm =
            compute_pd_torque_joint_Nm(q_ref_deg,
                                       q_link_deg,
                                       qd_ref_deg_s,
                                       qd_link_deg_s,
                                       pd);

        // =====================================================
        // 7. Total joint torque
        // =====================================================

        const double tau_joint_total_Nm =
            tau_joint_ff_Nm + tau_pd_joint_Nm;

        // =====================================================
        // 8. Convert total joint torque to motor torque
        // =====================================================

        double tau_motor_cmd_Nm =
            joint_to_motor_torque_Nm(tau_joint_total_Nm, ff);

        // =====================================================
        // 9. Add signed motor-side dead torque
        //    Sign uses desired sine-wave velocity
        // =====================================================

        const double tau_dead_motor_Nm =
            signed_dead_torque_Nm(qd_ref_deg_s, ff);

        tau_motor_cmd_Nm += tau_dead_motor_Nm;

        // =====================================================
        // 10. Clamp motor torque command
        // =====================================================

        tau_motor_cmd_Nm =
            std::clamp(tau_motor_cmd_Nm,
                       -ff.maxMotorTorqueCmd_Nm,
                       +ff.maxMotorTorqueCmd_Nm);

        // =====================================================
        // 11. Send torque to EPOS
        // =====================================================

        if (!set_torque_Nm(nodeId,
                           tau_motor_cmd_Nm,
                           ratedTorque_Nm,
                           static_cast<int16_t>(maxTorque_permille))) {
            std::cerr << "[ERROR] Failed to send torque at sample "
                      << k << "\n";
            break;
        }

        // =====================================================
        // 12. Logging
        // =====================================================

        const double tau_motor_actual_Nm =
            (static_cast<double>(tq_pm) / 1000.0) * ratedTorque_Nm;

        const double pos_error_deg = q_ref_deg - q_link_deg;
        const double vel_error_deg_s = qd_ref_deg_s - qd_link_deg_s;

        LogRow row;

        row.t_s = k * dt_s;

        row.q_ref_deg = q_ref_deg;
        row.qd_ref_deg_s = qd_ref_deg_s;
        row.qdd_ref_deg_s2 = qdd_ref_deg_s2;

        row.q_link_deg = q_link_deg;
        row.qd_link_deg_s = qd_link_deg_s;

        row.pos_error_deg = pos_error_deg;
        row.vel_error_deg_s = vel_error_deg_s;

        row.tau_joint_ff_Nm = tau_joint_ff_Nm;
        row.tau_pd_joint_Nm = tau_pd_joint_Nm;
        row.tau_joint_total_Nm = tau_joint_total_Nm;
        row.tau_dead_motor_Nm = tau_dead_motor_Nm;
        row.tau_motor_cmd_Nm = tau_motor_cmd_Nm;
        row.tau_motor_actual_Nm = tau_motor_actual_Nm;

        row.motor_pos_counts = pos_counts;
        row.motor_vel_native = vel_native;

        logs.push_back(row);

        // =====================================================
        // 13. Print status
        // =====================================================

        if (static_cast<int>(k) % printEveryN == 0) {
            const double est_I_A = tau_motor_cmd_Nm / Kt_Nm_per_A;

            std::cout << "k=" << k
                      << " t=" << row.t_s << " s"
                      << " q_ref=" << q_ref_deg << " deg"
                      << " q_link=" << q_link_deg << " deg"
                      << " err=" << pos_error_deg << " deg"
                      << " qd_ref=" << qd_ref_deg_s << " deg/s"
                      << " qd_link=" << qd_link_deg_s << " deg/s"
                      << " tau_ff=" << tau_joint_ff_Nm << " Nm"
                      << " tau_PD=" << tau_pd_joint_Nm << " Nm"
                      << " tau_joint_total=" << tau_joint_total_Nm << " Nm"
                      << " tau_dead_motor=" << tau_dead_motor_Nm << " Nm"
                      << " tau_motor_cmd=" << tau_motor_cmd_Nm << " Nm"
                      << " tau_motor_act=" << tau_motor_actual_Nm << " Nm"
                      << " I_est~" << est_I_A << " A"
                      << "\n";
        }

        // Wait until next sample time
        const auto next_time =
            t0 + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                     (k + 1) * period);

        std::this_thread::sleep_until(next_time);
    }

    // ---------- Ramp down to zero ----------
    std::cout << "[INFO] Ramping torque back to zero...\n";

    {
        int16_t tq_pm = 0;

        sdo_read_any(nodeId,
                     IDX_TORQUE_ACT,
                     SUB0,
                     &tq_pm,
                     sizeof(tq_pm),
                     "Read actual torque for ramp");

        int16_t cmd_pm = tq_pm;

        const int16_t step = (cmd_pm > 0) ? -10 : 10;

        while (cmd_pm != 0) {
            if (std::abs(cmd_pm) < std::abs(step)) {
                cmd_pm = 0;
            } else {
                cmd_pm = static_cast<int16_t>(cmd_pm + step);
            }

            sdo_write_any(nodeId,
                          IDX_TARGET_TORQUE,
                          SUB0,
                          &cmd_pm,
                          sizeof(cmd_pm),
                          "Ramp 0x6071");

            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
    }

    // ---------- Disable and close ----------
    VCS_SetDisableState(handle, nodeId, &err);
    VCS_CloseDevice(handle, &err);

    // ---------- Save log ----------
    if (!save_log_csv(outputLogPath, logs)) {
        return 1;
    }

    std::cout << "[INFO] Log saved to: " << outputLogPath << "\n";
    std::cout << "[INFO] Done.\n";

    return 0;
}
