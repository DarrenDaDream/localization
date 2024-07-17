#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

#include <Eigen/Dense>

class KalmanFilter {
public:
    KalmanFilter();

    // Initialize the Kalman filter with initial parameters
    void init(double initial_estimate, double initial_error_estimate, double process_noise);

    // Update the Kalman filter with a new measurement and return the filtered estimate
    double update(double measurement);

private:
    // State variables
    double x_estimate;        // Estimated state
    double error_estimate;    // Error estimate
    double process_noise;     // Process noise

    // Kalman filter parameters
    double process_variance;  // Process variance (Q)
    double measurement_variance; // Measurement variance (R)
};

#endif  // KALMAN_FILTER_H