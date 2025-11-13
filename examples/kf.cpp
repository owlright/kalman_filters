#include <iostream>
#include <kalman_filters.hpp>
#include <random>
#include <fstream>

int main(int argc, char const *argv[]) {
    double duration = 5.0; // seconds
    double dt = 0.01;
    double acc = 1.0;
    Eigen::Vector2d x0, x;
    x0 << 0, 0; // initial state [x, v]
    Eigen::VectorXd u0(1);
    u0 << acc; // initial control input [a]
    Eigen::MatrixXd A(2, 2);
    A << 1, dt,
         0, 1; // state transition matrix x_{k} = x_{k-1} + v*dt, v_{k} = v_{k-1}
    Eigen::MatrixXd B(2, 1);
    B << 0.5 * dt * dt, dt;
    Eigen::MatrixXd Q(2, 2);
    Q << 0.1, 0.0,
         0.0, 0.1; // process noise covariance
    Eigen::MatrixXd C(2, 2);
    C << 1.0, 0.0,
         0.0, 1.0; // observation matrix
    Eigen::MatrixXd R(2, 2);
    R << 0.8, 0,
         0, 0.8; // measurement noise covariance
    Eigen::MatrixXd P0(2, 2);
    P0 << 10.0, 0.0,
          0.0, 10.0; // initial estimation error covariance
    KalmanFilter m(A, B, C, P0, R, Q);
    m.init(x0);
    size_t rounds = static_cast<size_t>(duration / dt);
    std::vector<double> true_states;
    std::vector<double> estimates;
    std::vector<double> measurements;
    x = x0;
    std::mt19937 gen(123);
    std::normal_distribution<double> p_noise(0.0, std::sqrt(R(0, 0)));
    std::normal_distribution<double> v_noise(0.0, std::sqrt(R(1, 1)));

    for (size_t i = 0; i < rounds; ++i) {
        m.predict(u0);
        auto x_true = A * x + B * u0;
        true_states.push_back(x_true(0));
        Eigen::Vector2d N;
        N << p_noise(gen), v_noise(gen);
        auto y = C * x_true + N;
        measurements.push_back(y(0));
        m.update(y);
        estimates.push_back(m.get_state()(0));
        std::cout << "Time: " << (i + 1) * dt
                  << " True Position: " << true_states.back()
                  << " Measured Position: " << measurements.back()
                  << " Estimated Position: " << estimates.back()
                  << std::endl;
        x = x_true;
    }
    std::ofstream file("kf.txt");
    for (size_t i = 0; i < rounds; ++i) {
        file << (i + 1) * dt << " "
             << true_states[i] << " "
             << measurements[i] << " "
             << estimates[i] << std::endl;
    }
    std::cout << "Final Covariance: \n" << m.get_cov() << std::endl;
    file.close();
}