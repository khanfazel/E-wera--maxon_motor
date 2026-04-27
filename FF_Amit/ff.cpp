#include <Definitions.h>

#include <cmath>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>
#include <vector>
#include <algorithm>
#include <chrono>

// ================= GLOBAL =================
void* handle = nullptr;
unsigned int err = 0;

constexpr double PI = 3.141592653589793;

// ================= HELPERS =================
double deg2rad(double d) { return d * PI / 180.0; }

bool ok(bool success, const char* msg)
{
    if (!success) {
        std::cerr << "[ERROR] " << msg << " err=0x"
                  << std::hex << err << std::dec << "\n";
    }
    return success;
}

// ================= TORQUE =================
int16_t Nm_to_permille(double tau, double rated)
{
    double pm = (tau / rated) * 1000.0;
    pm = std::clamp(pm, -1000.0, 1000.0);
    return static_cast<int16_t>(std::lround(pm));
}

bool send_torque(unsigned short nodeId, double tau, double rated)
{
    int16_t pm = Nm_to_permille(tau, rated);
    unsigned int written = 0;

    return VCS_SetObject(handle, nodeId, 0x6071, 0x00,
                         &pm, sizeof(pm), &written, &err);
}

// ================= TRAJECTORY =================
struct Sample
{
    double q_deg;
    double qd_deg_s;
    double qdd_deg_s2;
};

bool load_csv(const std::string& path, std::vector<Sample>& data)
{
    std::ifstream f(path);
    if (!f.is_open()) return false;

    std::string line;
    while (std::getline(f, line)) {
        std::stringstream ss(line);
        std::string a, b, c;

        if (!std::getline(ss, a, ',')) continue;
        if (!std::getline(ss, b, ',')) continue;
        if (!std::getline(ss, c, ',')) continue;

        try {
            data.push_back({
                std::stod(a),
                std::stod(b),
                std::stod(c)
            });
        } catch (...) {}
    }
    return !data.empty();
}

// ================= MODEL =================
struct Params
{
    // joint
    double I = 0.0198;
    double m = 0.65;
    double lc = 0.13;
    double g = 9.81;

    // transmission
    double N = 80.0;
    double eta = 0.75;

    // motor
    double Jm = 0.000288;

    // friction
    double tau_f = 0.1602;
    double sigma = 2.0;

    // safety
    double tau_max = 0.5;
};

//  CORRECT MOTOR MODEL
double motor_tau(const Sample& s, const Params& p)
{
    double q = deg2rad(s.q_deg);
    double qdd = deg2rad(s.qdd_deg_s2);

    // joint dynamics
    double tau_joint =
        p.I * qdd +
        p.m * p.g * p.lc * std::sin(q);

    // load at motor
    double tau_load =
        tau_joint / (p.N * p.eta);

    // rotor inertia
    double tau_rotor =
        p.Jm * p.N * qdd;

    // smooth friction
    double tau_fric =
        p.tau_f * std::tanh(s.qd_deg_s / p.sigma);

    return tau_load + tau_rotor - tau_fric;
}

// ================= MAIN =================
int main()
{
    unsigned short nodeId = 3;

    std::string file = "trajectory_motor_filtered.csv";

    double dt = 0.005;
    double ratedTorque = 1.068;

    Params p;

    // ===== LOAD =====
    std::vector<Sample> traj;
    if (!load_csv(file, traj)) {
        std::cerr << "CSV load failed\n";
        return 1;
    }

    std::cout << "Loaded " << traj.size() << " samples\n";

    // ===== EPOS INIT =====
    char deviceName[] = "EPOS4";
    char protocolStackName[] = "MAXON SERIAL V2";
    char interfaceName[] = "USB";
    char portName[] = "USB0";

    handle = VCS_OpenDevice(deviceName,
                        protocolStackName,
                        interfaceName,
                        portName,
                        &err);
    if (!ok(handle != nullptr, "OpenDevice")) return 1;

    VCS_ClearFault(handle, nodeId, &err);
    VCS_SetOperationMode(handle, nodeId, 10, &err);
    VCS_SetEnableState(handle, nodeId, &err);

    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    // ===== LOOP =====
    for (size_t i = 0; i < traj.size(); ++i)
    {
        double tau = motor_tau(traj[i], p);

        tau = std::clamp(tau, -p.tau_max, p.tau_max);

        send_torque(nodeId, tau, ratedTorque);

        if (i % 100 == 0) {
            std::cout << "i=" << i
                      << " tau=" << tau << " Nm\n";
        }

        std::this_thread::sleep_for(
            std::chrono::milliseconds((int)(dt * 1000)));
    }

    // ===== SAFE STOP =====
    for (int i = 0; i < 100; ++i) {
        send_torque(nodeId, 0.0, ratedTorque);
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }

    VCS_SetDisableState(handle, nodeId, &err);
    VCS_CloseDevice(handle, &err);

    std::cout << "Done.\n";
    return 0;
}
