#include <iostream>
#include <vector>
#include <fstream>
#include <sstream>
#include <cmath>
#include <iomanip>
#include <stdexcept>
#include <algorithm>

// ================= PARAMETERS =================
const double dt = 0.005;          // sample time [s]
const int interp_steps = 1000;    // output points

// ===== PHYSICAL PARAMETERS =====
double m    = 2.37;     // link mass [kg]
double lc   = 0.13;     // COM distance from joint [m]
double J_eq = 0.0198;   // equivalent inertia [kg*m^2]

// ===== FILES =====
const std::string input_filename  = "Angles_comfortable.csv";
const std::string output_filename = "calculated_torque.csv";

// =================================================

// ================= CSV LOADER =================
// Reads first row, skips first column (subject ID), reads remaining values as angles in deg
std::vector<double> loadCSVRow(const std::string& filename)
{
    std::ifstream file(filename);
    if (!file.is_open())
        throw std::runtime_error("Could not open input file: " + filename);

    std::string line;
    if (!std::getline(file, line))
        throw std::runtime_error("CSV file is empty: " + filename);

    std::stringstream ss(line);
    std::string cell;
    std::vector<double> data;

    // Skip first column (subject ID or label)
    std::getline(ss, cell, ',');

    while (std::getline(ss, cell, ','))
    {
        if (!cell.empty())
            data.push_back(std::stod(cell));
    }

    if (data.empty())
        throw std::runtime_error("No numeric trajectory data found in CSV.");

    return data;
}

// ================= LINEAR INTERPOLATION =================
std::vector<double> interpolate(const std::vector<double>& data)
{
    std::vector<double> out(interp_steps);

    for (int i = 0; i < interp_steps; i++)
    {
        double x = static_cast<double>(i) / (interp_steps - 1) * (data.size() - 1);
        int x0 = static_cast<int>(std::floor(x));
        int x1 = std::min(static_cast<int>(data.size()) - 1, x0 + 1);

        double t = x - x0;
        out[i] = (1.0 - t) * data[x0] + t * data[x1];
    }

    return out;
}

// ================= SAVITZKY-GOLAY =================
// 5-point smoothing + derivatives
void savgol(const std::vector<double>& input,
            std::vector<double>& smooth,
            std::vector<double>& deriv1,
            std::vector<double>& deriv2,
            double dt)
{
    int n = static_cast<int>(input.size());

    smooth.resize(n);
    deriv1.resize(n);
    deriv2.resize(n);

    for (int i = 0; i < n; i++)
    {
        if (i < 2 || i > n - 3)
        {
            smooth[i] = input[i];
            deriv1[i] = 0.0;
            deriv2[i] = 0.0;
            continue;
        }

        double y_m2 = input[i - 2];
        double y_m1 = input[i - 1];
        double y_0  = input[i];
        double y_p1 = input[i + 1];
        double y_p2 = input[i + 2];

        // Smoothed signal
        smooth[i] = (-3.0 * y_m2 + 12.0 * y_m1 + 17.0 * y_0 + 12.0 * y_p1 - 3.0 * y_p2) / 35.0;

        // First derivative
        deriv1[i] = (-y_p2 + 8.0 * y_p1 - 8.0 * y_m1 + y_m2) / (12.0 * dt);

        // Second derivative
        deriv2[i] = (-y_p2 + 16.0 * y_p1 - 30.0 * y_0 + 16.0 * y_m1 - y_m2) / (12.0 * dt * dt);
    }
}

int main()
{
    try
    {
        // ===== LOAD ANGLE TRAJECTORY =====
        auto qd_deg_raw = loadCSVRow(input_filename);
        std::cout << "Loaded points: " << qd_deg_raw.size() << std::endl;

        // Convert degrees -> radians
        for (auto& v : qd_deg_raw)
            v = v * M_PI / 180.0;

        // Interpolate
        auto qd_interp = interpolate(qd_deg_raw);

        // Smooth and differentiate
        std::vector<double> qd, qd_dot, qd_ddot;
        savgol(qd_interp, qd, qd_dot, qd_ddot, dt);

        // ===== OPEN OUTPUT CSV =====
        std::ofstream out(output_filename);
        if (!out.is_open())
            throw std::runtime_error("Could not open output file: " + output_filename);

        out << "index,time_s,qd_rad,qd_deg,qd_dot_rad_s,qd_ddot_rad_s2,tau_inertia_Nm,tau_gravity_Nm,tau_total_Nm\n";
        out << std::fixed << std::setprecision(6);

        // ===== COMPUTE TORQUE FROM DYNAMIC MODEL =====
        for (int i = 0; i < interp_steps; i++)
        {
            double t = i * dt;

            double q     = qd[i];
            double qdot  = qd_dot[i];
            double qddot = qd_ddot[i];

            double tau_inertia = J_eq * qddot;
            double tau_gravity = m * 9.81 * lc * std::sin(q);
            double tau_total   = tau_inertia + tau_gravity;

            out << i << ","
                << t << ","
                << q << ","
                << (q * 180.0 / M_PI) << ","
                << qdot << ","
                << qddot << ","
                << tau_inertia << ","
                << tau_gravity << ","
                << tau_total
                << "\n";
        }

        out.close();

        std::cout << "Torque CSV written to: " << output_filename << std::endl;
        std::cout << "Done.\n";
    }
    catch (const std::exception& e)
    {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}
