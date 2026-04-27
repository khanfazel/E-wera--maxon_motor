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

// EPOS scaling
double ratedTorque = 1.068;

// friction break (optional but recommended)
double tau_min = 0.15;

// ================= DATA =================
std::vector<double> tau_data;

// ================= LOAD CSV =================
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
        std::getline(ss, cell, ','); // qd

        std::getline(ss, cell, ','); // tau
        tau_data.push_back(std::stod(cell));
    }
}

// ================= MAIN =================
int main()
{
    loadCSV("torque_single_subject.csv");

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

    std::cout << "Starting FEEDFORWARD ONLY...\n";

    for (int i = 0; i < tau_data.size(); i++)
    {
        auto start = std::chrono::steady_clock::now();

        double tau = tau_data[i];

        // ===== OPTIONAL friction assist =====
        if (fabs(tau) < tau_min)
        {
            tau += tau_min * (tau > 0 ? 1 : -1);
        }

        // ===== CONVERT TO PERMILLE =====
        double torque_permille = (tau / ratedTorque) * 1000.0;

        // clamp (safe)
        if (torque_permille > 300) torque_permille = 300;
        if (torque_permille < -300) torque_permille = -300;

        int16_t tq = (int16_t)torque_permille;

        VCS_SetObject(handle, nodeId, 0x6071, 0,
                      &tq, sizeof(tq), &bytes, &err);

        // ===== DEBUG =====
        std::cout << "tau=" << tau
                  << " perm=" << tq << "\r";

        std::this_thread::sleep_until(start + std::chrono::milliseconds(5));
    }

    std::cout << "\nDone.\n";
    VCS_CloseDevice(handle, &err);
    return 0;
}

