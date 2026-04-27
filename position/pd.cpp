#include <Definitions.h>

#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <string>
#include <thread>

namespace {
constexpr uint16_t IDX_CONTROLWORD            = 0x6040;
constexpr uint8_t  SUB0                       = 0x00;
constexpr uint16_t IDX_MODES_OF_OPERATION     = 0x6060;
constexpr uint16_t IDX_MODES_OF_OP_DISPLAY    = 0x6061;
constexpr uint16_t IDX_POSITION_ACTUAL_VALUE  = 0x6064;
constexpr uint16_t IDX_TARGET_TORQUE          = 0x6071;
constexpr uint16_t IDX_MOTOR_RATED_TORQUE     = 0x6076;
constexpr uint16_t IDX_TORQUE_ACTUAL_VALUE    = 0x6077;
constexpr uint16_t IDX_MAX_MOTOR_SPEED        = 0x6080;
constexpr uint16_t IDX_PROFILE_DECEL          = 0x6084;
constexpr uint16_t IDX_QUICKSTOP_DECEL        = 0x6085;

constexpr int8_t CST_MODE = 10; // 0x0A
}

struct Config {
    std::string deviceName    = "EPOS4";
    std::string protocolStack = "MAXON SERIAL V2";
    std::string interfaceName = "USB";
    std::string portName      = "USB0";
    uint16_t    nodeId        = 4;

    // Mechanics
    int         encoderCPT      = 1024;   // 512 / 1024 / 2048 / 4096
    double      gearRatio       = 80.0;   // motor rev / joint rev
    bool        quadratureX4    = true;
    int32_t     jointZeroOffset = 0;      // counts

    // Host PD loop
    double      loopDtSec       = 0.005;  // 5 ms
    double      Kp_permille_per_count = 0.2;
    double      Kd_permille_per_count_per_sec = 0.1;

    // Saturation
    int16_t     maxTorquePermille = 100;
    bool        invertTorqueSign  = false;

    // Sine trajectory
    double      amplitudeDeg   = 20.0;
    double      offsetDeg      = 0.0;
    double      frequencyHz    = 0.10;
    double      durationSec    = 10.0;

    // CST stop behavior parameters
    uint32_t    maxMotorSpeedRpm = 200;
    uint32_t    profileDecelRpmPerSec = 100;
    uint32_t    quickStopDecelRpmPerSec = 100;

    // Log file
    std::string logFileName = "pd_log.csv";
};

static bool ok(int result, const char* what) {
    if (result == 0) {
        std::cerr << "[ERROR] " << what << " failed\n";
        return false;
    }
    return true;
}

static void printErrorInfo(unsigned int errCode) {
    char errText[256] = {};
    VCS_GetErrorInfo(errCode, errText, static_cast<unsigned short>(sizeof(errText)));
    std::cerr << "[EPOS ERROR] " << errText << "\n";
}

static bool readObject(
    void* handle,
    uint16_t nodeId,
    uint16_t index,
    uint8_t subIndex,
    void* data,
    unsigned int bytes,
    const char* what)
{
    unsigned int err = 0;
    unsigned int bytesRead = 0;

    if (!ok(VCS_GetObject(handle, nodeId, index, subIndex, data, bytes, &bytesRead, &err), what)) {
        printErrorInfo(err);
        return false;
    }

    if (bytesRead != bytes) {
        std::cerr << "[ERROR] " << what << ": unexpected byte count "
                  << bytesRead << " (expected " << bytes << ")\n";
        return false;
    }

    return true;
}

static bool writeObject(
    void* handle,
    uint16_t nodeId,
    uint16_t index,
    uint8_t subIndex,
    const void* data,
    unsigned int bytes,
    const char* what)
{
    unsigned int err = 0;
    unsigned int bytesWritten = 0;

    if (!ok(VCS_SetObject(handle, nodeId, index, subIndex, const_cast<void*>(data), bytes, &bytesWritten, &err), what)) {
        printErrorInfo(err);
        return false;
    }

    if (bytesWritten != bytes) {
        std::cerr << "[ERROR] " << what << ": unexpected byte count "
                  << bytesWritten << " (expected " << bytes << ")\n";
        return false;
    }

    return true;
}

static int64_t jointDegToCounts(double jointDeg, const Config& cfg) {
    const double jointRev = jointDeg / 360.0;
    const double motorRev = jointRev * cfg.gearRatio;
    const double countsPerMotorRev = cfg.encoderCPT * (cfg.quadratureX4 ? 4.0 : 1.0);
    return static_cast<int64_t>(std::llround(motorRev * countsPerMotorRev));
}

static double countsToJointDeg(int64_t counts, const Config& cfg) {
    const double countsPerMotorRev = cfg.encoderCPT * (cfg.quadratureX4 ? 4.0 : 1.0);
    const double motorRev = static_cast<double>(counts) / countsPerMotorRev;
    const double jointRev = motorRev / cfg.gearRatio;
    return jointRev * 360.0;
}

static double jointDegPerSecToCountsPerSec(double degPerSec, const Config& cfg) {
    const double countsPerMotorRev = cfg.encoderCPT * (cfg.quadratureX4 ? 4.0 : 1.0);
    return (degPerSec / 360.0) * cfg.gearRatio * countsPerMotorRev;
}

static int16_t clampI16(long v) {
    if (v > std::numeric_limits<int16_t>::max()) return std::numeric_limits<int16_t>::max();
    if (v < std::numeric_limits<int16_t>::min()) return std::numeric_limits<int16_t>::min();
    return static_cast<int16_t>(v);
}

int main() {
    Config cfg;
    unsigned int err = 0;

    std::ofstream logFile(cfg.logFileName);
    if (!logFile.is_open()) {
        std::cerr << "[ERROR] Could not open " << cfg.logFileName << " for writing\n";
        return 1;
    }

    logFile << "t_s,qd_deg,q_deg,e_deg,tq_cmd_permille,tq_act_permille,pos_counts,qd_counts,qd_dot_counts_s,qdot_counts_s\n";
    logFile << std::fixed << std::setprecision(6);

    void* handle = VCS_OpenDevice(
        const_cast<char*>(cfg.deviceName.c_str()),
        const_cast<char*>(cfg.protocolStack.c_str()),
        const_cast<char*>(cfg.interfaceName.c_str()),
        const_cast<char*>(cfg.portName.c_str()),
        &err);

    if (handle == nullptr) {
        std::cerr << "[ERROR] VCS_OpenDevice failed\n";
        printErrorInfo(err);
        return 1;
    }

    VCS_ClearFault(handle, cfg.nodeId, &err);

    if (!ok(VCS_SetEnableState(handle, cfg.nodeId, &err), "VCS_SetEnableState")) {
        printErrorInfo(err);
        VCS_CloseDevice(handle, &err);
        return 1;
    }

    if (!writeObject(handle, cfg.nodeId, IDX_MAX_MOTOR_SPEED, SUB0,
                     &cfg.maxMotorSpeedRpm, sizeof(cfg.maxMotorSpeedRpm), "Write 0x6080 Max motor speed")) {
        VCS_CloseDevice(handle, &err);
        return 1;
    }

    if (!writeObject(handle, cfg.nodeId, IDX_PROFILE_DECEL, SUB0,
                     &cfg.profileDecelRpmPerSec, sizeof(cfg.profileDecelRpmPerSec), "Write 0x6084 Profile decel")) {
        VCS_CloseDevice(handle, &err);
        return 1;
    }

    if (!writeObject(handle, cfg.nodeId, IDX_QUICKSTOP_DECEL, SUB0,
                     &cfg.quickStopDecelRpmPerSec, sizeof(cfg.quickStopDecelRpmPerSec), "Write 0x6085 Quick-stop decel")) {
        VCS_CloseDevice(handle, &err);
        return 1;
    }

    int8_t mode = CST_MODE;
    if (!writeObject(handle, cfg.nodeId, IDX_MODES_OF_OPERATION, SUB0,
                     &mode, sizeof(mode), "Write 0x6060 CST mode")) {
        VCS_CloseDevice(handle, &err);
        return 1;
    }

    int8_t modeDisp = 0;
    if (!readObject(handle, cfg.nodeId, IDX_MODES_OF_OP_DISPLAY, SUB0,
                    &modeDisp, sizeof(modeDisp), "Read 0x6061 Mode display")) {
        VCS_CloseDevice(handle, &err);
        return 1;
    }

    std::cout << "Active mode display = " << static_cast<int>(modeDisp) << "\n";

    {
        uint16_t cw = 0x0006;
        if (!writeObject(handle, cfg.nodeId, IDX_CONTROLWORD, SUB0, &cw, sizeof(cw), "Write 0x6040 Shutdown")) {
            VCS_CloseDevice(handle, &err);
            return 1;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(10));

        cw = 0x000F;
        if (!writeObject(handle, cfg.nodeId, IDX_CONTROLWORD, SUB0, &cw, sizeof(cw), "Write 0x6040 Enable op")) {
            VCS_CloseDevice(handle, &err);
            return 1;
        }
    }

    {
        int16_t tq = 0;
        if (!writeObject(handle, cfg.nodeId, IDX_TARGET_TORQUE, SUB0, &tq, sizeof(tq), "Write 0x6071 Zero torque")) {
            VCS_CloseDevice(handle, &err);
            return 1;
        }
    }

    int16_t ratedTorque_mNm = 0;
    if (!readObject(handle, cfg.nodeId, IDX_MOTOR_RATED_TORQUE, SUB0,
                    &ratedTorque_mNm, sizeof(ratedTorque_mNm), "Read 0x6076 Motor rated torque")) {
        VCS_CloseDevice(handle, &err);
        return 1;
    }

    std::cout << "Motor rated torque = " << ratedTorque_mNm << " mNm\n";

    int32_t posCounts = 0;
    if (!readObject(handle, cfg.nodeId, IDX_POSITION_ACTUAL_VALUE, SUB0,
                    &posCounts, sizeof(posCounts), "Read 0x6064 Position actual")) {
        VCS_CloseDevice(handle, &err);
        return 1;
    }

    int32_t lastPosCounts = posCounts;

    std::cout << "Starting host PD over CST\n";
    std::cout << "Amplitude: " << cfg.amplitudeDeg << " deg\n";
    std::cout << "Offset   : " << cfg.offsetDeg << " deg\n";
    std::cout << "Freq     : " << cfg.frequencyHz << " Hz\n";
    std::cout << "Duration : " << cfg.durationSec << " s\n";
    std::cout << "dt       : " << cfg.loopDtSec << " s\n";
    std::cout << "Kp       : " << cfg.Kp_permille_per_count << " permille/count\n";
    std::cout << "Kd       : " << cfg.Kd_permille_per_count_per_sec << " permille/(count/s)\n";
    std::cout << "Torque sat: +/-" << cfg.maxTorquePermille << " permille\n";
    std::cout << "Logging to: " << cfg.logFileName << "\n\n";

    const auto tStart = std::chrono::steady_clock::now();
    auto nextTick = tStart;

    while (true) {
        const auto now = std::chrono::steady_clock::now();
        const double t = std::chrono::duration<double>(now - tStart).count();
        if (t > cfg.durationSec) {
            break;
        }

        const double w = 2.0 * M_PI * cfg.frequencyHz;
        const double qd_deg = cfg.offsetDeg + cfg.amplitudeDeg * std::sin(w * t);
        const double qd_dot_deg_s = cfg.amplitudeDeg * w * std::cos(w * t);

        const int64_t qd_counts64 = static_cast<int64_t>(cfg.jointZeroOffset) + jointDegToCounts(qd_deg, cfg);
        const double qd_dot_counts_s = jointDegPerSecToCountsPerSec(qd_dot_deg_s, cfg);

        if (!readObject(handle, cfg.nodeId, IDX_POSITION_ACTUAL_VALUE, SUB0,
                        &posCounts, sizeof(posCounts), "Read 0x6064 Position actual")) {
            break;
        }

        const double q_counts = static_cast<double>(posCounts);
        const double qdot_counts_s = static_cast<double>(posCounts - lastPosCounts) / cfg.loopDtSec;
        lastPosCounts = posCounts;

        const double e_counts = static_cast<double>(qd_counts64) - q_counts;
        const double edot_counts_s = qd_dot_counts_s - qdot_counts_s;

        double u_permille =
            cfg.Kp_permille_per_count * e_counts +
            cfg.Kd_permille_per_count_per_sec * edot_counts_s;

        if (cfg.invertTorqueSign) {
            u_permille = -u_permille;
        }

        if (u_permille > cfg.maxTorquePermille)  u_permille = cfg.maxTorquePermille;
        if (u_permille < -cfg.maxTorquePermille) u_permille = -cfg.maxTorquePermille;

        const int16_t targetTorquePermille = clampI16(std::lround(u_permille));

        if (!writeObject(handle, cfg.nodeId, IDX_TARGET_TORQUE, SUB0,
                         &targetTorquePermille, sizeof(targetTorquePermille), "Write 0x6071 Target torque")) {
            break;
        }

        int16_t tqActPermille = 0;
        if (!readObject(handle, cfg.nodeId, IDX_TORQUE_ACTUAL_VALUE, SUB0,
                        &tqActPermille, sizeof(tqActPermille), "Read 0x6077 Torque actual")) {
            tqActPermille = 0;
        }

        const double q_deg = countsToJointDeg(static_cast<int64_t>(posCounts) - cfg.jointZeroOffset, cfg);
        const double e_deg = qd_deg - q_deg;

        logFile
            << t << ","
            << qd_deg << ","
            << q_deg << ","
            << e_deg << ","
            << targetTorquePermille << ","
            << tqActPermille << ","
            << posCounts << ","
            << qd_counts64 << ","
            << qd_dot_counts_s << ","
            << qdot_counts_s
            << "\n";

        std::cout
            << "t=" << t
            << " | qd=" << qd_deg << " deg"
            << " | q=" << q_deg << " deg"
            << " | e=" << e_deg << " deg"
            << " | tq_cmd=" << targetTorquePermille << " permille"
            << " | tq_act=" << tqActPermille << " permille"
            << "      \r" << std::flush;

        nextTick += std::chrono::duration_cast<std::chrono::steady_clock::duration>(
            std::chrono::duration<double>(cfg.loopDtSec));
        std::this_thread::sleep_until(nextTick);
    }

    std::cout << "\nStopping...\n";

    {
        int16_t tq = 0;
        writeObject(handle, cfg.nodeId, IDX_TARGET_TORQUE, SUB0, &tq, sizeof(tq), "Write 0x6071 Zero torque");
    }

    logFile.close();
    VCS_CloseDevice(handle, &err);
    return 0;
}