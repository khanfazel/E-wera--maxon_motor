#include <Definitions.h>
#include <iostream>
#include <vector>
#include <fstream>
#include <sstream>
#include <cmath>
#include <thread>
#include <chrono>

// ================= CONTROL MODE =================
#define MODE_PD_ONLY          0
#define MODE_PD_GRAVITY       1
#define MODE_FULL_CTC         2

int CONTROL_MODE = MODE_PD_ONLY;

// ================= PARAMETERS =================
const double dt = 0.005;
const int interp_steps = 1000;

// ===== PHYSICAL PARAMS (TUNE THESE) =====
double m  = 2.37;
double lc = 0.13;
double J_eq = 7e-6;

// ===== GAINS =====
double Kp = 15.0;
double Kd = 0.5;

// ===== EPOS =====
uint16_t nodeId = 4;

// =================================================

// ================= CSV LOADER =================
std::vector<double> loadCSVRow(const std::string& filename)
{
    std::ifstream file(filename);
    std::string line;

    if (!std::getline(file, line))
        throw std::runtime_error("CSV empty!");

    std::stringstream ss(line);
    std::string cell;

    std::vector<double> data;

    // Skip subject ID
    std::getline(ss, cell, ',');

    while (std::getline(ss, cell, ','))
        data.push_back(std::stod(cell));

    return data;
}

// ================= INTERPOLATION =================
std::vector<double> interpolate(const std::vector<double>& data)
{
    std::vector<double> out(interp_steps);

    for (int i = 0; i < interp_steps; i++)
    {
        double x = (double)i / (interp_steps - 1) * (data.size() - 1);
        int x0 = floor(x);
        int x1 = std::min((int)data.size() - 1, x0 + 1);

        double t = x - x0;
        out[i] = (1 - t) * data[x0] + t * data[x1];
    }
    return out;
}

// ================= SAVITZKY-GOLAY =================
void savgol(const std::vector<double>& input,
            std::vector<double>& smooth,
            std::vector<double>& deriv1,
            std::vector<double>& deriv2,
            double dt)
{
    int n = input.size();

    smooth.resize(n);
    deriv1.resize(n);
    deriv2.resize(n);

    for (int i = 0; i < n; i++)
    {
        if (i < 2 || i > n - 3)
        {
            smooth[i] = input[i];
            deriv1[i] = 0;
            deriv2[i] = 0;
            continue;
        }

        double y_m2 = input[i-2];
        double y_m1 = input[i-1];
        double y_0  = input[i];
        double y_p1 = input[i+1];
        double y_p2 = input[i+2];

        // Smoothed signal
        smooth[i] = (-3*y_m2 + 12*y_m1 + 17*y_0 + 12*y_p1 - 3*y_p2) / 35.0;

        // First derivative
        deriv1[i] = (-y_p2 + 8*y_p1 - 8*y_m1 + y_m2) / (12*dt);

        // Second derivative
        deriv2[i] = (-y_p2 + 16*y_p1 - 30*y_0 + 16*y_m1 - y_m2) / (12*dt*dt);
    }
}

// ================= LOW PASS =================
double lowpass(double prev, double raw, double alpha = 0.95)
{
    return alpha*prev + (1-alpha)*raw;
}

// ================= COUNTS → RAD =================
double countsToRad(int32_t counts)
{
    double counts_per_rev = 1024 * 4;
    double gear = 80.0;
    return (counts / counts_per_rev) * 2*M_PI / gear;
}

// =================================================

int main()
{
    // ===== LOAD CSV =====
    std::string file = "Angles_comfortable.csv";

    auto qd_deg = loadCSVRow(file);

    std::cout << "Loaded points: " << qd_deg.size() << std::endl;

    // Convert to radians
    for (auto& v : qd_deg)
        v = v * M_PI / 180.0;

    // Interpolate
    auto qd_interp = interpolate(qd_deg);

    // Apply Savitzky–Golay
    std::vector<double> qd, qd_dot, qd_ddot;
    savgol(qd_interp, qd, qd_dot, qd_ddot, dt);

    // ===== EPOS INIT =====
    unsigned int err = 0;
    unsigned int bytes = 0;

    void* handle = VCS_OpenDevice((char*)"EPOS4",
                                 (char*)"MAXON SERIAL V2",
                                 (char*)"USB",
                                 (char*)"USB0",
                                 &err);

    VCS_SetEnableState(handle, nodeId, &err);

    int8_t mode = 10; // CST
    VCS_SetObject(handle, nodeId, 0x6060, 0, &mode, sizeof(mode), &bytes, &err);

    int16_t ratedTorque_mNm;
    VCS_GetObject(handle, nodeId, 0x6076, 0,
                  &ratedTorque_mNm, sizeof(ratedTorque_mNm),
                  &bytes, &err);

    double ratedTorque = ratedTorque_mNm / 1000.0;

    // ===== STATE =====
    int32_t posCounts = 0, lastPos = 0;
    double q = 0, qdot = 0, qdot_prev = 0;

    std::cout << "Starting control...\n";

    // ===== CONTROL LOOP =====
    for (int i = 0; i < interp_steps; i++)
    {
        auto loop_start = std::chrono::steady_clock::now();

        // ---- READ POSITION ----
        VCS_GetObject(handle, nodeId, 0x6064, 0,
                      &posCounts, sizeof(posCounts),
                      &bytes, &err);

        q = countsToRad(posCounts);

        // ---- VELOCITY ----
        double qdot_raw = (posCounts - lastPos) / dt;
        qdot = lowpass(qdot_prev, qdot_raw);

        lastPos = posCounts;
        qdot_prev = qdot;

        // ---- DESIRED ----
        double qd_i = qd[i];
        double qd_dot_i = qd_dot[i];
        double qd_ddot_i = qd_ddot[i];

        // ---- ERRORS ----
        double e = qd_i - q;
        double edot = qd_dot_i - qdot;

        double tau = 0;

        // -------- MODE 1 --------
        if (CONTROL_MODE == MODE_PD_ONLY)
            tau = Kp * e + Kd * edot;

        // -------- MODE 2 --------
        if (CONTROL_MODE == MODE_PD_GRAVITY)
        {
            double tau_g = m * 9.81 * lc * sin(q);
            tau = tau_g + Kp * e + Kd * edot;
        }

        // -------- MODE 3 --------
        if (CONTROL_MODE == MODE_FULL_CTC)
        {
            double tau_ff = J_eq * qd_ddot_i + m * 9.81 * lc * sin(q);
            double tau_pd = Kp * e + Kd * edot;
            tau = tau_ff + tau_pd;
        }

        // ---- TORQUE → PERMILLE ----
        double torque_permille = (tau / ratedTorque) * 1000.0;

        if (torque_permille > 1000) torque_permille = 1000;
        if (torque_permille < -1000) torque_permille = -1000;

        int16_t tq = (int16_t)torque_permille;

        VCS_SetObject(handle, nodeId, 0x6071, 0,
                      &tq, sizeof(tq), &bytes, &err);

        std::cout << "q=" << q << " qd=" << qd_i << " tau=" << tq << "\r";

        std::this_thread::sleep_until(loop_start + std::chrono::milliseconds(5));
    }

    std::cout << "\nDone.\n";
    VCS_CloseDevice(handle, &err);
    return 0;
}