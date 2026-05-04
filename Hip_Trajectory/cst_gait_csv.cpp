// cst_gait_csv.cpp
// EPOS4 CST trajectory follower for one hip joint
//
// Reads a CSV exported from Excel with columns:
// gait_percent, torque_norm
//
// Assumed input torque unit:
// torque_norm = normalized joint torque in [Nm/kg]
//
// Conversion:
// joint torque [Nm] = torque_norm * body_mass_kg
// motor torque [Nm] = joint torque / (gear_ratio * gear_efficiency)
// sign flip applied if gear direction is inverted


#include <Definitions.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

// ---------------- Globals ----------------
static void* handle = nullptr;
static unsigned int err = 0;

static inline bool ok(bool success, const char* step)
{
    if (!success) {
        std::cerr << "[EPOS ERROR] " << step
                  << " (err=0x" << std::hex << err << std::dec << ")\n";
    }
    return success;
}

// ---------------- SDO helpers ----------------
static inline bool sdo_write_u16(
    unsigned short nodeId,
    unsigned short idx,
    unsigned char sub,
    const void* p,
    unsigned int n,
    const char* tag)
{
    unsigned int nW = 0;
    return ok(
        VCS_SetObject(handle, nodeId, idx, sub, const_cast<void*>(p), n, &nW, &err) && nW == n,
        tag
    );
}

static inline bool sdo_write_any(
    unsigned short nodeId,
    unsigned short idx,
    unsigned char sub,
    const void* p,
    unsigned int n,
    const char* tag)
{
    unsigned int nW = 0;
    return ok(
        VCS_SetObject(handle, nodeId, idx, sub, const_cast<void*>(p), n, &nW, &err),
        tag
    );
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

// ---------------- Object indices ----------------
constexpr unsigned char SUB0 = 0x00;
constexpr unsigned short IDX_OPMODE_DISP   = 0x6061;
constexpr unsigned short IDX_TARGET_TORQUE = 0x6071;
constexpr unsigned short IDX_TORQUE_ACT    = 0x6077;
constexpr unsigned short IDX_MAX_TORQUE    = 0x6072;
constexpr unsigned short IDX_POS_ACT       = 0x6064;
constexpr unsigned short IDX_VEL_ACT       = 0x606C;
constexpr unsigned short IDX_RATED_TORQUE  = 0x6076;

// ---------------- Data structures ----------------
struct Sample {
    double gait_percent = 0.0; // 0..100
    double value = 0.0;        // normalized torque or other configured mode
};

enum class InputTorqueMode {
    NormalizedJointTorque_NmPerKg, // Nm/kg
    JointTorque_Nm,                // Nm
    MotorTorque_Nm                 // Nm
};

struct Config {
    // ----- EPOS connection -----
    std::string deviceName = "EPOS4";
    std::string protocolStackName = "MAXON SERIAL V2";
    std::string interfaceName = "USB";
    std::string portName = "USB0";
    unsigned short nodeId = 5;   // CHANGE if needed

    // ----- file -----
    std::string csvPath = "hip_torque.csv";

    // ----- trajectory meaning -----
    InputTorqueMode inputMode = InputTorqueMode::NormalizedJointTorque_NmPerKg;
    double bodyMassKg = 1.0;

    // ----- drivetrain -----
    double gearRatio = 80.0;
    double gearEfficiency = 1.0; // set e.g. 0.75 if you know it
    bool gearInverted = true;    // from your EPOS Studio setup

    // ----- timing -----
    double gaitCycleDurationSec = 1.344;  // one full 0..100% gait cycle
    int loopPeriodMs = 2;
    int numberOfCycles = 5;

    // ----- safety -----
    int16_t torqueClampPermille = 600;   // softer first test than 1000
    uint16_t maxTorquePermille = 600;    // write to 0x6072 if supported
    double startupRampSec = 0;         // smoothly fade in over first cycle portion
    bool repeatTrajectory = true;

    // ----- display -----
    int printEveryMs = 100;
};

// ---------------- Utility ---------------- 265
static inline std::string trim(const std::string& s)
{
    const char* ws = " \t\r\n";
    const auto start = s.find_first_not_of(ws);
    if (start == std::string::npos) return "";
    const auto end = s.find_last_not_of(ws);
    return s.substr(start, end - start + 1);
}

static inline bool parse_double(const std::string& s, double& out)
{
    try {
        size_t idx = 0;
        out = std::stod(trim(s), &idx);
        return idx > 0;
    } catch (...) {
        return false;
    }
}

static std::vector<std::string> split_csv_line(const std::string& line)
{
    std::vector<std::string> fields;
    std::stringstream ss(line);
    std::string item;
    while (std::getline(ss, item, ',')) {
        fields.push_back(trim(item));
    }
    return fields;
}

static std::vector<Sample> load_trajectory_csv(const std::string& path)
{
    std::ifstream fin(path);
    if (!fin.is_open()) {
        throw std::runtime_error("Could not open CSV file: " + path);
    }

    std::vector<Sample> traj;
    std::string line;
    bool header_checked = false;

    while (std::getline(fin, line)) {
        line = trim(line);
        if (line.empty()) continue;

        auto fields = split_csv_line(line);
        if (fields.size() < 2) continue;

        double x = 0.0, y = 0.0;
        bool okx = parse_double(fields[0], x);
        bool oky = parse_double(fields[1], y);

        if (!header_checked) {
            header_checked = true;
            if (!okx || !oky) {
                // skip header row
                continue;
            }
        }

        if (!okx || !oky) continue;
        traj.push_back({x, y});
    }

    if (traj.size() < 2) {
        throw std::runtime_error("Trajectory needs at least 2 numeric rows.");
    }

    std::sort(traj.begin(), traj.end(),
              [](const Sample& a, const Sample& b) { return a.gait_percent < b.gait_percent; });

    return traj;
}

static double interpolate_traj(const std::vector<Sample>& traj, double gaitPercent)
{
    if (traj.empty()) return 0.0;

    // wrap 0..100
    while (gaitPercent < 0.0) gaitPercent += 100.0;
    while (gaitPercent > 100.0) gaitPercent -= 100.0;

    if (gaitPercent <= traj.front().gait_percent) return traj.front().value;
    if (gaitPercent >= traj.back().gait_percent)  return traj.back().value;

    for (size_t i = 0; i + 1 < traj.size(); ++i) {
        const auto& a = traj[i];
        const auto& b = traj[i + 1];
        if (gaitPercent >= a.gait_percent && gaitPercent <= b.gait_percent) {
            const double dx = b.gait_percent - a.gait_percent;
            if (dx <= 1e-12) return a.value;
            const double alpha = (gaitPercent - a.gait_percent) / dx;
            return a.value + alpha * (b.value - a.value);
        }
    }

    return traj.back().value;
}

static inline int16_t nm_to_permille(double tauNm, double ratedTorqueNm, int16_t clampAbsPm)
{
    double pm = (tauNm / ratedTorqueNm) * 1000.0;
    long pmRounded = std::lround(pm);
    if (pmRounded > clampAbsPm) pmRounded = clampAbsPm;
    if (pmRounded < -clampAbsPm) pmRounded = -clampAbsPm;
    return static_cast<int16_t>(pmRounded);
}

static inline bool set_torque_nm(
    unsigned short nodeId,
    double motorTorqueNm,
    double ratedTorqueNm,
    int16_t clampAbsPm)
{
    const int16_t permille = nm_to_permille(motorTorqueNm, ratedTorqueNm, clampAbsPm);
    return sdo_write_any(
        nodeId, IDX_TARGET_TORQUE, SUB0,
        &permille, sizeof(permille),
        "SDO: Set 0x6071 Target Torque"
   );
}

static double input_to_motor_torque_nm(const Config& cfg, double inputValue)
{
    double motorTorqueNm = 0.0;

    switch (cfg.inputMode) {
        case InputTorqueMode::NormalizedJointTorque_NmPerKg: {
            const double jointTorqueNm = inputValue * cfg.bodyMassKg;
            motorTorqueNm = jointTorqueNm / (cfg.gearRatio * cfg.gearEfficiency);
            break;
        }
        case InputTorqueMode::JointTorque_Nm: {
            motorTorqueNm = inputValue / (cfg.gearRatio * cfg.gearEfficiency);
            break;
        }
        case InputTorqueMode::MotorTorque_Nm: {
            motorTorqueNm = inputValue;
            break;
        }
    }

    if (cfg.gearInverted) {
        motorTorqueNm = -motorTorqueNm;
    }

    return motorTorqueNm;
}

static double fade_in_scale(double elapsedSec, double rampSec)
{
    if (rampSec <= 1e-9) return 1.0;
    if (elapsedSec <= 0.0) return 0.0;
    if (elapsedSec >= rampSec) return 1.0;
    return elapsedSec / rampSec;
}

int main()
{
    Config cfg;

    // --------- Print config ----------
    std::cout << std::fixed << std::setprecision(5);
    std::cout << "[INFO] CSV path          : " << cfg.csvPath << "\n";
    std::cout << "[INFO] Body mass         : " << cfg.bodyMassKg << " kg\n";
    std::cout << "[INFO] Gear ratio        : " << cfg.gearRatio << ":1\n";
    std::cout << "[INFO] Gear efficiency   : " << cfg.gearEfficiency << "\n";
    std::cout << "[INFO] Gear inverted     : " << (cfg.gearInverted ? "true" : "false") << "\n";
    std::cout << "[INFO] Gait cycle        : " << cfg.gaitCycleDurationSec << " s\n";
    std::cout << "[INFO] Loop period       : " << cfg.loopPeriodMs << " ms\n";
    std::cout << "[INFO] Number of cycles  : " << cfg.numberOfCycles << "\n";

    // --------- Load trajectory ----------
    std::vector<Sample> traj;
    try {
        traj = load_trajectory_csv(cfg.csvPath);
    } catch (const std::exception& e) {
        std::cerr << "[ERROR] " << e.what() << "\n";
        return 1;
    }

    std::cout << "[INFO] Loaded " << traj.size() << " trajectory samples.\n";

    // --------- Open device ----------
    char deviceName[64];
    char protocolStackName[64];
    char interfaceName[64];
    char portName[64];

    std::snprintf(deviceName, sizeof(deviceName), "%s", cfg.deviceName.c_str());
    std::snprintf(protocolStackName, sizeof(protocolStackName), "%s", cfg.protocolStackName.c_str());
    std::snprintf(interfaceName, sizeof(interfaceName), "%s", cfg.interfaceName.c_str());
    std::snprintf(portName, sizeof(portName), "%s", cfg.portName.c_str());

    handle = VCS_OpenDevice(deviceName, protocolStackName, interfaceName, portName, &err);
    if (!ok(handle != nullptr, "OpenDevice")) return 1;

    if (!ok(VCS_ClearFault(handle, cfg.nodeId, &err), "ClearFault")) {
        VCS_CloseDevice(handle, &err);
        return 1;
    }

    // --------- Read rated torque ----------
    uint16_t ratedTorque_mNm = 0;
    if (!sdo_read_any(cfg.nodeId, IDX_RATED_TORQUE, SUB0,
                      &ratedTorque_mNm, sizeof(ratedTorque_mNm),
                      "Read 0x6076 Rated Torque"))
    {
        VCS_CloseDevice(handle, &err);
        return 1;
    }

    //const double ratedTorqueNm = ratedTorque_mNm / 1000.0;
    //std::cout << "[INFO] Rated torque      : " << ratedTorqueNm << " Nm\n";
    
    const double ratedTorqueNm = 0.729;
	std::cout << "[INFO] Using fixed rated torque : " << ratedTorqueNm << " Nm\n";

    // --------- Set CST mode while disabled ----------
    {
        int8_t op = 10; // CST
        if (!ok(VCS_SetOperationMode(handle, cfg.nodeId, op, &err), "SetOperationMode=CST")) {
            VCS_CloseDevice(handle, &err);
            return 1;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(5));

        int8_t opDisp = -1;
        sdo_read_any(cfg.nodeId, IDX_OPMODE_DISP, SUB0, &opDisp, sizeof(opDisp), "Read ModeDisplay");
        if (opDisp != 10) {
            std::cerr << "[WARN] Mode display != CST, got " << static_cast<int>(opDisp) << "\n";
        }
    }

    // --------- Set max torque if available ----------
    {
        uint16_t probe = 0;
        if (sdo_read_any(cfg.nodeId, IDX_MAX_TORQUE, SUB0, &probe, sizeof(probe), "Probe 0x6072")) {
            uint16_t maxTq = cfg.maxTorquePermille;
            sdo_write_u16(cfg.nodeId, IDX_MAX_TORQUE, SUB0, &maxTq, sizeof(maxTq), "Set 0x6072 Max Torque");
        } else {
            std::cerr << "[INFO] 0x6072 not supported; continuing.\n";
        }
    }

    // --------- Enable ----------
    if (!ok(VCS_SetEnableState(handle, cfg.nodeId, &err), "SetEnableState")) {
        VCS_CloseDevice(handle, &err);
        return 1;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    // --------- Main loop ----------
    const auto period = std::chrono::milliseconds(cfg.loopPeriodMs);
    const int printDivider = std::max(1, cfg.printEveryMs / cfg.loopPeriodMs);

    const double totalDurationSec = cfg.numberOfCycles * cfg.gaitCycleDurationSec;
    auto t0 = std::chrono::steady_clock::now();
    int tick = 0;

    std::cout << "[INFO] Starting CST gait trajectory...\n";

    while (true) {
        const auto now = std::chrono::steady_clock::now();
        const double elapsedSec =
            std::chrono::duration_cast<std::chrono::duration<double>>(now - t0).count();

        if (elapsedSec >= totalDurationSec) {
            break;
        }

        const double cycleTimeSec = std::fmod(elapsedSec, cfg.gaitCycleDurationSec);
        const double gaitPercent = (cycleTimeSec / cfg.gaitCycleDurationSec) * 100.0;

        const double rawInput = interpolate_traj(traj, gaitPercent);
        double motorTorqueNm = input_to_motor_torque_nm(cfg, rawInput);

        // smooth fade-in
        motorTorqueNm *= fade_in_scale(elapsedSec, cfg.startupRampSec);

        if (!set_torque_nm(cfg.nodeId, motorTorqueNm, ratedTorqueNm, cfg.torqueClampPermille)) {
            break;
        }

        if (++tick % printDivider == 0) {
            int32_t pos = 0;
            int32_t vel = 0;
            int16_t tq_pm = 0;

            sdo_read_any(cfg.nodeId, IDX_POS_ACT, SUB0, &pos, sizeof(pos), "Read 0x6064 Position");
            sdo_read_any(cfg.nodeId, IDX_VEL_ACT, SUB0, &vel, sizeof(vel), "Read 0x606C Velocity");
            sdo_read_any(cfg.nodeId, IDX_TORQUE_ACT, SUB0, &tq_pm, sizeof(tq_pm), "Read 0x6077 Torque");

            const double actualMotorTorqueNm = (tq_pm / 1000.0) * ratedTorqueNm;

            std::cout
                << "t=" << elapsedSec << " s"
                << "  gait=" << gaitPercent << " %"
                << "  raw=" << rawInput
                << "  cmd_motor=" << motorTorqueNm << " Nm"
                << "  act_motor=" << actualMotorTorqueNm << " Nm"
                << "  tq=" << tq_pm << " pm"
                << "  vel=" << vel
                << "  pos=" << pos
                << "\n";
        }

        std::this_thread::sleep_for(period);
    }

    // --------- Ramp to zero ----------
    {
        int16_t tq_pm = 0;
        sdo_read_any(cfg.nodeId, IDX_TORQUE_ACT, SUB0, &tq_pm, sizeof(tq_pm), "Read 0x6077 before ramp-down");

        int16_t cmd = tq_pm;
        const int16_t step = (cmd > 0) ? -10 : 10;

        while (cmd != 0) {
            cmd = (std::abs(cmd) < std::abs(step)) ? 0 : static_cast<int16_t>(cmd + step);
            sdo_write_any(cfg.nodeId, IDX_TARGET_TORQUE, SUB0, &cmd, sizeof(cmd), "Ramp-down 0x6071");
            std::this_thread::sleep_for(period);
        }
    }

    VCS_SetDisableState(handle, cfg.nodeId, &err);
    VCS_CloseDevice(handle, &err);

    std::cout << "[INFO] Done.\n";
    return 0;
}
