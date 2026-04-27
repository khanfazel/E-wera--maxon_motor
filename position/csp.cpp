#include <Definitions.h>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <thread>
#include <chrono>
#include <string>
#include <limits>

struct Config
{
    std::string deviceName     = "EPOS4";
    std::string protocolStack  = "MAXON SERIAL V2";
    std::string interfaceName  = "USB";
    std::string portName       = "USB0";   // Change if needed
    uint16_t    nodeId         = 4;

    // Encoder / mechanics
    int         encoderCPT     = 1024;     // 512 / 1024 / 2048 / 4096
    double      gearRatio      = 80.0;     // motor rev / joint rev
    bool        useQuadratureX4 = true;

    // Motion profile
    uint32_t    profileVelocity = 500;   // counts/s
    uint32_t    profileAccel    = 1500;   // counts/s^2
    uint32_t    profileDecel    = 1500;   // counts/s^2

    // Trajectory
    double      amplitudeDeg    = 20.0;    // +/- amplitude
    double      offsetDeg       = 0.0;     // center angle
    double      frequencyHz     = 0.20;    // sine frequency
    double      durationSec     = 20.0;    // total run time
    double      sampleTimeSec   = 0.02;    // 20 ms = 50 Hz update

    // Joint zero offset
    int32_t     jointZeroOffsetCounts = 0;
};

static bool ok(int result, const char* what)
{
    if(result == 0)
    {
        std::cerr << "[ERROR] " << what << " failed\n";
        return false;
    }
    return true;
}
static void printErrorInfo(unsigned int errCode)
{
    char errText[256] = {};
    VCS_GetErrorInfo(errCode, errText, static_cast<unsigned short>(sizeof(errText)));
    std::cerr << "[EPOS ERROR] " << errText << "\n";
}

static int64_t degToCounts(double jointDeg, const Config& cfg)
{
    const double jointRev = jointDeg / 360.0;
    const double motorRev = jointRev * cfg.gearRatio;
    const double countsPerMotorRev = cfg.encoderCPT * (cfg.useQuadratureX4 ? 4.0 : 1.0);
    const double counts = motorRev * countsPerMotorRev;
    return static_cast<int64_t>(std::llround(counts));
}

static double countsToDeg(int64_t counts, const Config& cfg)
{
    const double countsPerMotorRev = cfg.encoderCPT * (cfg.useQuadratureX4 ? 4.0 : 1.0);
    const double motorRev = static_cast<double>(counts) / countsPerMotorRev;
    const double jointRev = motorRev / cfg.gearRatio;
    return jointRev * 360.0;
}

int main()
{
    Config cfg;
    unsigned int err = 0;

    // Open device
    void* handle = VCS_OpenDevice(
        const_cast<char*>(cfg.deviceName.c_str()),
        const_cast<char*>(cfg.protocolStack.c_str()),
        const_cast<char*>(cfg.interfaceName.c_str()),
        const_cast<char*>(cfg.portName.c_str()),
        &err);

    if(handle == nullptr)
    {
        std::cerr << "[ERROR] VCS_OpenDevice failed\n";
        printErrorInfo(err);
        return 1;
    }

    // Clear fault
    VCS_ClearFault(handle, cfg.nodeId, &err);

    // Enable
    if(!ok(VCS_SetEnableState(handle, cfg.nodeId, &err), "VCS_SetEnableState"))
    {
        printErrorInfo(err);
        VCS_CloseDevice(handle, &err);
        return 1;
    }

    // Activate Profile Position Mode
    if(!ok(VCS_ActivateProfilePositionMode(handle, cfg.nodeId, &err), "VCS_ActivateProfilePositionMode"))
    {
        printErrorInfo(err);
        VCS_CloseDevice(handle, &err);
        return 1;
    }

    // Set profile
    if(!ok(VCS_SetPositionProfile(
            handle,
            cfg.nodeId,
            cfg.profileVelocity,
            cfg.profileAccel,
            cfg.profileDecel,
            &err),
        "VCS_SetPositionProfile"))
    {
        printErrorInfo(err);
        VCS_CloseDevice(handle, &err);
        return 1;
    }

    // Read current position
    int currentPosCounts = 0;
    if(!ok(VCS_GetPositionIs(handle, cfg.nodeId, &currentPosCounts, &err), "VCS_GetPositionIs"))
    {
        printErrorInfo(err);
        VCS_CloseDevice(handle, &err);
        return 1;
    }

    std::cout << "Current position = " << currentPosCounts
              << " counts, "
              << countsToDeg(currentPosCounts - cfg.jointZeroOffsetCounts, cfg)
              << " deg\n";

    std::cout << "\nStarting sinusoidal trajectory...\n";
    std::cout << "Amplitude  : " << cfg.amplitudeDeg << " deg\n";
    std::cout << "Offset     : " << cfg.offsetDeg << " deg\n";
    std::cout << "Frequency  : " << cfg.frequencyHz << " Hz\n";
    std::cout << "Duration   : " << cfg.durationSec << " s\n";
    std::cout << "Sample time: " << cfg.sampleTimeSec << " s\n\n";

    const auto t0 = std::chrono::steady_clock::now();
    double t = 0.0;

    while(t <= cfg.durationSec)
    {
        // Sinusoidal joint angle in degrees
        double jointDeg =
            cfg.offsetDeg +
            cfg.amplitudeDeg * std::sin(2.0 * M_PI * cfg.frequencyHz * t);

        // Convert to absolute target counts
        int64_t targetCounts64 =
            static_cast<int64_t>(cfg.jointZeroOffsetCounts) +
            degToCounts(jointDeg, cfg);

        if(targetCounts64 > std::numeric_limits<long>::max() ||
           targetCounts64 < std::numeric_limits<long>::min())
        {
            std::cerr << "[ERROR] Target overflow\n";
            break;
        }

        long targetCounts = static_cast<long>(targetCounts64);

        // Send absolute target
        // absolute = 1, immediately = 1
        if(!ok(VCS_MoveToPosition(handle, cfg.nodeId, targetCounts, 1, 1, &err), "VCS_MoveToPosition"))
        {
            printErrorInfo(err);
            break;
        }

        // Read actual position for monitoring
        int posNow = 0;
        if(ok(VCS_GetPositionIs(handle, cfg.nodeId, &posNow, &err), "VCS_GetPositionIs"))
        {
            double actualJointDeg = countsToDeg(posNow - cfg.jointZeroOffsetCounts, cfg);

            std::cout << "t=" << t
                      << " s | target=" << jointDeg
                      << " deg | actual=" << actualJointDeg
                      << " deg | counts=" << posNow << "\r" << std::flush;
        }

        std::this_thread::sleep_for(
            std::chrono::duration<double>(cfg.sampleTimeSec));

        t = std::chrono::duration<double>(
                std::chrono::steady_clock::now() - t0).count();
    }

    std::cout << "\n\nTrajectory finished.\n";

    // Optional: hold last position, or disable here
    // VCS_SetDisableState(handle, cfg.nodeId, &err);

    VCS_CloseDevice(handle, &err);
    return 0;
}
