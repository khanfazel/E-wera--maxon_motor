#include <Definitions.h>
#include <iostream>
#include <vector>
#include <fstream>
#include <sstream>
#include <cmath>
#include <thread>
#include <chrono>

// ================= PARAMETERS =================
const double dt = 0.005;

// Gains (start conservative)
double Kp = 20.0;
double Kd = 2.0;

// friction compensation
double tau_min = 0.15;

// EPOS scaling
double ratedTorque = 1.068;

// encoder conversion
double counts_per_rev = 1024 * 4;
double gear_ratio = 80.0;

// ================= DATA =================
std::vector<double> qd_data;
std::vector<double> qd_dot_data;
std::vector<double> tau_data;

// ================= UTIL =================
double countsToRad(int32_t counts)
{
    return (counts / counts_per_rev) * 2 * M_PI / gear_ratio;
}

// ================= LOAD CSV =================
// CSV format:
// time_s, qd_rad, qd_dot, qd_ddot, tau_Nm

void loadCSV(const std::string& filename)
{
    std::ifstream file(filename);
    std::string line;

    std::getline(file, line); // skip header

    while (std::getline(file, line))
    {
        std::stringstream ss(line);
        std::string cell;

        std::getline(ss, cell, ','); // time

        std::getline(ss, cell, ',');
        qd_data.push_back(std::stod(cell));

        std::getline(ss, cell, ',');
        qd_dot_data.push_back(std::stod(cell));

        std::getline(ss, cell, ','); // qddot (skip)

        std::getline(ss, cell, ',');
        tau_data.push_back(std::stod(cell));
    }
}

// ================= MAIN =================
int main()
{
    loadCSV("trajectory_full.csv");

    unsigned int err = 0, bytes = 0;
    uint16_t nodeId = 3;

    void* handle = VCS_OpenDevice(
        (char*)"EPOS4",
        (char*)"MAXON SERIAL V2",
        (char*)"USB",
        (char*)"USB0",
        &err);

    VCS_SetEnableState(handle, nodeId, &err);

    int8_t mode = 10; // CST
    VCS_SetObject(handle, nodeId, 0x6060, 0,
                  &mode, sizeof(mode), &bytes, &err);

    int32_t posCounts = 0, lastCounts = 0;
    double q = 0, qdot = 0;

    std::cout << "Starting FF + PD (FULL)...\n";

    for (size_t i = 0; i < tau_data.size(); i++)
    {
        auto start = std::chrono::steady_clock::now();

        // ===== READ ENCODER =====
        VCS_GetObject(handle, nodeId, 0x6064, 0,
                      &posCounts, sizeof(posCounts),
                      &bytes, &err);

        q = countsToRad(posCounts);

        // ===== VELOCITY =====
        double counts_diff = posCounts - lastCounts;
        double qdot_raw = countsToRad(counts_diff) / dt;

        // low-pass filter
        qdot = 0.9 * qdot + 0.1 * qdot_raw;

        lastCounts = posCounts;

        // ===== DESIRED =====
        double qd = qd_data[i];
        double qd_dot = qd_dot_data[i];
        double tau_ff = tau_data[i];

        // ===== ERRORS =====
        double e = qd - q;
        double edot = qd_dot - qdot;

        // ===== CONTROL =====
        double tau = tau_ff + Kp * e + Kd * edot;

        // ===== FRICTION COMP =====
        if (fabs(tau) < tau_min)
        {
            tau += tau_min * (tau > 0 ? 1 : -1);
        }

        // ===== CONVERT TO PERMILLE =====
        double torque_permille = (tau / ratedTorque) * 1000.0;

        // clamp (safe)
        if (torque_permille > 400) torque_permille = 400;
        if (torque_permille < -400) torque_permille = -400;

        int16_t tq = (int16_t)torque_permille;

        VCS_SetObject(handle, nodeId, 0x6071, 0,
                      &tq, sizeof(tq), &bytes, &err);

        // ===== DEBUG =====
        std::cout << "q=" << q
                  << " qd=" << qd
                  << " e=" << e
                  << " tau=" << tau
                  << " perm=" << tq << "\r";

        std::this_thread::sleep_until(start + std::chrono::milliseconds(5));
    }

    std::cout << "\nDone.\n";
    VCS_CloseDevice(handle, &err);
    return 0;
}
