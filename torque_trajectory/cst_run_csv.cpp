// cst_run_csv_with_log.cpp
// EPOS4 CST over USB
// Reads motor torque values from CSV in N.m
// Sends them to Target Torque 0x6071
// Logs link angle, torque command, and actual torque to CSV

#include <Definitions.h>

#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <thread>
#include <chrono>
#include <cstdint>
#include <cmath>
#include <iomanip>
#include <string>
#include <algorithm>

// ---------- Globals ----------
static void* handle = nullptr;
static unsigned int err = 0;

// ---------- Object indices ----------
constexpr unsigned char  SUB0              = 0x00;
constexpr unsigned short IDX_OPMODE_DISP   = 0x6061;
constexpr unsigned short IDX_TARGET_TORQUE = 0x6071;
constexpr unsigned short IDX_TORQUE_ACT    = 0x6077;
constexpr unsigned short IDX_MAX_TORQUE    = 0x6072;
constexpr unsigned short IDX_POS_ACT       = 0x6064;
constexpr unsigned short IDX_VEL_ACT       = 0x606C;
constexpr unsigned short IDX_RATED_TORQUE  = 0x6076;

// ---------- Error helper ----------
static inline bool ok(bool success, const char* step)
{
    if(!success) {
        std::cerr << "[EPOS ERROR] " << step
                  << "  err=0x" << std::hex << err << std::dec << "\n";
    }
    return success;
}

// ---------- SDO write helper ----------
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
        VCS_SetObject(
            handle,
            nodeId,
            idx,
            sub,
            const_cast<void*>(p),
            n,
            &nW,
            &err
        ),
        tag
    );
}

// ---------- SDO read helper ----------
static inline bool sdo_read_any(
    unsigned short nodeId,
    unsigned short idx,
    unsigned char sub,
    void* p,
    unsigned int n,
    const char* tag)
{
    unsigned int nR = 0;

    return ok(
        VCS_GetObject(
            handle,
            nodeId,
            idx,
            sub,
            p,
            n,
            &nR,
            &err
        ),
        tag
    );
}

// ---------- Convert N.m to EPOS target torque permille ----------
static inline int16_t nm_to_permille(
    double tau_Nm,
    double ratedTorque_Nm,
    int16_t clamp_abs_pm)
{
    double pm_f = (tau_Nm / ratedTorque_Nm) * 1000.0;
    long pm_l = std::lround(pm_f);

    if(pm_l >  clamp_abs_pm) pm_l =  clamp_abs_pm;
    if(pm_l < -clamp_abs_pm) pm_l = -clamp_abs_pm;

    return static_cast<int16_t>(pm_l);
}

// ---------- Convert EPOS torque permille to N.m ----------
static inline double permille_to_nm(
    int16_t torque_pm,
    double ratedTorque_Nm)
{
    return (static_cast<double>(torque_pm) / 1000.0) * ratedTorque_Nm;
}

// ---------- Command torque in permille directly ----------
static inline bool set_torque_permille(
    unsigned short nodeId,
    int16_t permille)
{
    return sdo_write_any(
        nodeId,
        IDX_TARGET_TORQUE,
        SUB0,
        &permille,
        sizeof(permille),
        "SDO: Set 0x6071 Target Torque"
    );
}

// ---------- Read CSV torque values ----------
std::vector<double> readTorqueCsv(
    const std::string& filename,
    int torqueColumn)
{
    std::vector<double> torqueValues;
    std::ifstream file(filename);

    if(!file.is_open()) {
        std::cerr << "[ERROR] Could not open CSV file: "
                  << filename << "\n";
        return torqueValues;
    }

    std::string line;

    while(std::getline(file, line)) {

        if(line.empty()) {
            continue;
        }

        std::stringstream ss(line);
        std::string cell;
        std::vector<std::string> cells;

        while(std::getline(ss, cell, ',')) {
            cells.push_back(cell);
        }

        if(torqueColumn >= static_cast<int>(cells.size())) {
            continue;
        }

        try {
            double value = std::stod(cells[torqueColumn]);
            torqueValues.push_back(value);
        }
        catch(...) {
            // Skip header or non-numeric row
            continue;
        }
    }

    return torqueValues;
}

int main()
{
    // ================= USER SETTINGS =================

    // EPOS connection
    char deviceName[]        = "EPOS4";
    char protocolStackName[] = "MAXON SERIAL V2";
    char interfaceName[]     = "USB";
    char portName[]          = "USB0";

    const unsigned short nodeId = 3;

    // Input CSV file
    const std::string csvFile = "motor_torque.csv";

    // Output log CSV file
    const std::string logFile = "cst_torque_link_log.csv";

    // Torque column in input CSV
    // If torque is in first column, use 0.
    // If torque is in second column, use 1.
    const int TORQUE_COLUMN = 0;

    // Loop timing
    // 1000 points with 10 ms = 10 seconds total motion
    const int loopPeriod_ms = 10;

    // Safety torque command limit in permille
    // 300 means ±30 percent of motor rated torque
    const int16_t maxCommand_permille = 300;

    // EPOS max torque object 0x6072
    // 300 means ±30 percent of rated motor torque
    const uint16_t maxTorqueObject_permille = 300;

    // Encoder and gearbox settings
    const double encoder_cpt = 1024.0;   // encoder counts per turn
    const double quadrature  = 4.0;      // quadrature factor
    const double gearRatio   = 80.0;     // gearbox ratio

    // =================================================

    std::cout << std::fixed << std::setprecision(6);

    // ---------- Read torque CSV first ----------
    std::vector<double> torqueNm = readTorqueCsv(
        csvFile,
        TORQUE_COLUMN
    );

    if(torqueNm.empty()) {
        std::cerr << "[ERROR] No valid torque values found in CSV.\n";
        return 1;
    }

    std::cout << "[INFO] Loaded "
              << torqueNm.size()
              << " torque points from "
              << csvFile
              << "\n";

    double minTau = *std::min_element(
        torqueNm.begin(),
        torqueNm.end()
    );

    double maxTau = *std::max_element(
        torqueNm.begin(),
        torqueNm.end()
    );

    std::cout << "[INFO] CSV torque range = "
              << minTau
              << " to "
              << maxTau
              << " N.m\n";

    // ---------- Open EPOS ----------
    handle = VCS_OpenDevice(
        deviceName,
        protocolStackName,
        interfaceName,
        portName,
        &err
    );

    if(!ok(handle != nullptr, "OpenDevice")) {
        return 1;
    }

    // ---------- Clear fault ----------
    if(!ok(VCS_ClearFault(handle, nodeId, &err), "ClearFault")) {
        VCS_CloseDevice(handle, &err);
        return 1;
    }

    // ---------- Read motor rated torque from EPOS ----------
    uint32_t ratedTorque_uNm = 0;

    if(!sdo_read_any(
        nodeId,
        IDX_RATED_TORQUE,
        SUB0,
        &ratedTorque_uNm,
        sizeof(ratedTorque_uNm),
        "Read 0x6076 Motor Rated Torque"))
    {
        std::cerr << "[ERROR] Could not read motor rated torque 0x6076.\n";
        VCS_CloseDevice(handle, &err);
        return 1;
    }

    double ratedTorque_Nm =
        static_cast<double>(ratedTorque_uNm) / 1000000.0;

    std::cout << "[INFO] EPOS rated torque 0x6076 = "
              << ratedTorque_uNm
              << " uNm = "
              << ratedTorque_Nm
              << " N.m\n";

    if(ratedTorque_Nm <= 0.0) {
        std::cerr << "[ERROR] Rated torque is zero or invalid.\n";
        VCS_CloseDevice(handle, &err);
        return 1;
    }

    // ---------- Set CST mode ----------
    int8_t opMode = 10; // CST mode

    if(!ok(
        VCS_SetOperationMode(
            handle,
            nodeId,
            opMode,
            &err
        ),
        "SetOperationMode CST"))
    {
        VCS_CloseDevice(handle, &err);
        return 1;
    }

    std::this_thread::sleep_for(
        std::chrono::milliseconds(10)
    );

    // ---------- Check operation mode display ----------
    int8_t opDisplay = 0;

    sdo_read_any(
        nodeId,
        IDX_OPMODE_DISP,
        SUB0,
        &opDisplay,
        sizeof(opDisplay),
        "Read 0x6061 Mode Display"
    );

    if(opDisplay != 10) {
        std::cerr << "[WARN] Mode display is not CST. Got "
                  << static_cast<int>(opDisplay)
                  << "\n";
    }

    // ---------- Set max torque limit ----------
    uint16_t maxTq = maxTorqueObject_permille;

    sdo_write_any(
        nodeId,
        IDX_MAX_TORQUE,
        SUB0,
        &maxTq,
        sizeof(maxTq),
        "Set 0x6072 Max Torque"
    );

    // ---------- Enable drive ----------
    if(!ok(VCS_SetEnableState(handle, nodeId, &err), "SetEnableState")) {
        VCS_CloseDevice(handle, &err);
        return 1;
    }

    std::this_thread::sleep_for(
        std::chrono::milliseconds(20)
    );

    // ---------- Open output log file ----------
    std::ofstream log(logFile);

    if(!log.is_open()) {
        std::cerr << "[ERROR] Could not open log file: "
                  << logFile
                  << "\n";

        set_torque_permille(nodeId, 0);
        VCS_SetDisableState(handle, nodeId, &err);
        VCS_CloseDevice(handle, &err);

        return 1;
    }

    // CSV header
    log << "index,"
        << "time_sec,"
        << "motor_count,"
        << "link_angle_rad,"
        << "link_angle_deg,"
        << "torque_cmd_Nm,"
        << "torque_cmd_permille,"
        << "torque_actual_Nm,"
        << "torque_actual_permille,"
        << "velocity_rpm"
        << "\n";

    std::cout << "[INFO] Starting CSV torque playback...\n";

    auto period = std::chrono::milliseconds(loopPeriod_ms);

    // ---------- Main loop ----------
    for(size_t i = 0; i < torqueNm.size(); ++i) {

        // Time based on loop index
        double time_sec =
            static_cast<double>(i) *
            static_cast<double>(loopPeriod_ms) /
            1000.0;

        // Torque command from CSV in N.m
        double tau_cmd_Nm = torqueNm[i];

        // Convert N.m torque to EPOS permille
        int16_t cmd_pm = nm_to_permille(
            tau_cmd_Nm,
            ratedTorque_Nm,
            maxCommand_permille
        );

        // Send torque command
        if(!set_torque_permille(nodeId, cmd_pm)) {
            std::cerr << "[ERROR] Failed while commanding torque at point "
                      << i
                      << "\n";
            break;
        }

        // ---------- Read actual motor data ----------
        int32_t pos_count = 0;
        int32_t vel_rpm = 0;
        int16_t tqActual_pm = 0;

        sdo_read_any(
            nodeId,
            IDX_POS_ACT,
            SUB0,
            &pos_count,
            sizeof(pos_count),
            "Read Position"
        );

        sdo_read_any(
            nodeId,
            IDX_VEL_ACT,
            SUB0,
            &vel_rpm,
            sizeof(vel_rpm),
            "Read Velocity"
        );

        sdo_read_any(
            nodeId,
            IDX_TORQUE_ACT,
            SUB0,
            &tqActual_pm,
            sizeof(tqActual_pm),
            "Read Torque Actual"
        );

        // ---------- Convert count to link angle ----------
        // Your equation:
        // Link = (-count * pi * 2) / (1024 * 4 * 80)
        double link_angle_rad =
            (-static_cast<double>(pos_count) * 2.0 * M_PI) /
            (encoder_cpt * quadrature * gearRatio);

        double link_angle_deg =
            link_angle_rad * 180.0 / M_PI;

        // ---------- Convert actual torque from permille to N.m ----------
        double tqActual_Nm = permille_to_nm(
            tqActual_pm,
            ratedTorque_Nm
        );

        // ---------- Write one row to log CSV ----------
        log << i << ","
            << time_sec << ","
            << pos_count << ","
            << link_angle_rad << ","
            << link_angle_deg << ","
            << tau_cmd_Nm << ","
            << cmd_pm << ","
            << tqActual_Nm << ","
            << tqActual_pm << ","
            << vel_rpm
            << "\n";

        // ---------- Print every 100 ms ----------
        int printEvery = std::max(
            1,
            100 / loopPeriod_ms
        );

        if(i % printEvery == 0) {
            std::cout << "i=" << i
                      << "  t=" << time_sec << " s"
                      << "  count=" << pos_count
                      << "  link=" << link_angle_deg << " deg"
                      << "  cmd=" << tau_cmd_Nm << " N.m"
                      << "  cmd_pm=" << cmd_pm
                      << "  actual=" << tqActual_Nm << " N.m"
                      << "  actual_pm=" << tqActual_pm
                      << "  vel=" << vel_rpm << " rpm"
                      << "\n";
        }

        std::this_thread::sleep_for(period);
    }

    // ---------- Close log file ----------
    log.close();

    std::cout << "[INFO] Log saved to "
              << logFile
              << "\n";

    // ---------- Ramp torque down to zero ----------
    std::cout << "[INFO] Ramping torque to zero...\n";

    for(int k = 0; k < 20; ++k) {
        set_torque_permille(nodeId, 0);
        std::this_thread::sleep_for(
            std::chrono::milliseconds(10)
        );
    }

    // ---------- Disable drive ----------
    VCS_SetDisableState(handle, nodeId, &err);

    // ---------- Close EPOS ----------
    VCS_CloseDevice(handle, &err);

    std::cout << "[INFO] Done.\n";

    return 0;
}
