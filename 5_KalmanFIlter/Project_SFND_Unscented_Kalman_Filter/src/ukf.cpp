#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;

inline const float UKF::CalculateNISValue(const VectorXd z_prediction, const VectorXd z_measurement,
                                          const MatrixXd covariance) {
    VectorXd difference{ z_measurement - z_prediction };
    return difference.transpose() * covariance.inverse() * difference;
}

inline const bool UKF::isUsingLaser(const MeasurementPackage &meas_package) {
    return meas_package.sensor_type_ == MeasurementPackage::LASER && use_laser_;
}

inline const bool UKF::isUsingRadar(const MeasurementPackage &meas_package) {
    return meas_package.sensor_type_ == MeasurementPackage::RADAR && use_radar_;
}

inline void UKF::decrementNormalize(VectorXd &vec, const int index) {
    while (vec(index) > M_PI) { vec(index) -= twice_m_pi; }
}

inline void UKF::incrementNormalize(VectorXd &vec, const int index, const bool negativePi = false) {
    if (negativePi) {
        while (vec(index) < -M_PI) { vec(index) += twice_m_pi; }
    } else {
        while (vec(index) < M_PI) { vec(index) += twice_m_pi; }
    }
}

/**
 * Initializes Unscented Kalman filter
 */
UKF::UKF() {
    n_x_ = 5;
    n_aug_ = 7;

    use_laser_ = true;         // if this is false, laser measurements will be ignored (except during init)
    use_radar_ = true;         // if this is false, radar measurements will be ignored (except during init)
    x_ = VectorXd(n_x_);       // initial state vector
    P_ = MatrixXd(n_x_, n_x_); // initial covariance matrix
    std_a_ = 30;               // Process noise standard deviation longitudinal acceleration in m/s^2
    std_yawdd_ = 30;           // Process noise standard deviation yaw acceleration in rad/s^2

    // DO NOT MODIFY measurement noise values below. These are provided by the sensor manufacturer.
    // --------------------------------------------------------------------------------------------
    std_laspx_ = 0.15;  // Laser measurement noise standard deviation position1 in m
    std_laspy_ = 0.15;  // Laser measurement noise standard deviation position2 in m
    std_radr_ = 0.3;    // Radar measurement noise standard deviation radius in m
    std_radphi_ = 0.03; // Radar measurement noise standard deviation angle in rad
    std_radrd_ = 0.3;   // Radar measurement noise standard deviation radius change in m/s
    // --------------------------------------------------------------------------------------------
    // END DO NOT MODIFY section for measurement noise values

    /*
     * TODO: Complete the initialization. See ukf.h for other member properties.
     * Hint: one or more values initialized above might be wildly off...
     */

    aug_dim = 2 * n_aug_ + 1;

    std_a_ = 3.0;     // Process noise standard deviation longitudinal acceleration in m/s^2
    std_yawdd_ = 1.0; // Process noise standard deviation yaw acceleration in rad/s^2
    Xsig_pred_ = MatrixXd(n_x_, aug_dim); // predicted sigma points matrix
    weights_ = VectorXd(aug_dim);         // create vector for weights
    lambda_ = 3 - n_aug_;                 // Sigma point spreading parameter

    is_initialized_ = false;
}

UKF::~UKF() {
}

void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
    /**
     * TODO: Complete this function! Make sure you switch between lidar and radar
     * measurements.
     */

    constexpr int RADIAL_DIST{ 0 };
    constexpr int BEARING_VEL{ 1 };
    constexpr int RADIAL_VEL{ 2 };

    if (!is_initialized_) {
        const double radial_diff{ meas_package.raw_measurements_[RADIAL_DIST] };
        const double bearing_angle{ meas_package.raw_measurements_[BEARING_VEL] };

        if (isUsingRadar(meas_package)) {
            const double radial_velocity{ meas_package.raw_measurements_[RADIAL_VEL] };

            const double sin_radial_diff{ sin(radial_diff) };
            const double sin_bearing_angle{ sin(bearing_angle) };
            const double cos_bearing_angle{ cos(bearing_angle) };

            const double velocity{ sqrt(
                radial_velocity * sin_bearing_angle * radial_velocity * sin_bearing_angle +
                radial_velocity * cos_bearing_angle * radial_velocity * cos_bearing_angle) };

            P_ << 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1;

            x_ << (radial_diff * cos_bearing_angle), (radial_diff * sin_bearing_angle), velocity, 0, 0;

        } else if (isUsingLaser(meas_package)) {
            x_ << radial_diff, bearing_angle, 0, 0, 0;

            const double std_laspx_squared{ std_laspx_ * std_laspx_ };

            P_ << std_laspx_squared, 0, 0, 0, 0, 0, std_laspx_squared, 0, 0, 0, 0, 0, 5, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1;
        }

        prior_time_stamp = meas_package.timestamp_;
        is_initialized_ = true;
        return;
    }

    const double divisor{ lambda_ + n_aug_ };

    weights_.fill(0.0);
    weights_(0) = (lambda_ / divisor);

    for (int index{ 1 }; index < aug_dim; index++) { weights_(index) = 0.5 / divisor; }

    const double time_delta{ (meas_package.timestamp_ - prior_time_stamp) / 1000000.0 };

    prior_time_stamp = meas_package.timestamp_;
    Prediction(time_delta);

    if (isUsingLaser(meas_package)) {
        UpdateLidar(meas_package);

    } else if (isUsingRadar(meas_package)) {
        UpdateRadar(meas_package);
    }
}

void UKF::Prediction(double delta_t) {
    /**
     * Estimate the object's location.
     * Modify the state vector, x_. Predict sigma points, the state,
     * and the state covariance matrix.
     */
    VectorXd x_augmented_matrix{ VectorXd(n_aug_) };
    MatrixXd P_augmented_matrix{ MatrixXd(n_aug_, n_aug_) };
    MatrixXd x_sigma_augmented{ MatrixXd(n_aug_, aug_dim) };

    x_augmented_matrix.head(n_x_) = x_;
    x_augmented_matrix(n_x_) = 0;
    x_augmented_matrix(n_x_ + 1) = 0;

    P_augmented_matrix.fill(0);
    P_augmented_matrix.topLeftCorner(n_x_, n_x_) = P_;
    P_augmented_matrix(n_x_, n_x_) = std_a_ * std_a_;
    P_augmented_matrix(n_x_ + 1, n_x_ + 1) = std_yawdd_ * std_yawdd_;

    MatrixXd sqrt_matrix{ P_augmented_matrix.llt().matrixL() };

    x_sigma_augmented.fill(0);
    x_sigma_augmented.col(0) = x_augmented_matrix;

    const double shift{ sqrt(lambda_ + n_aug_) };

    for (int index{ 0 }; index < n_aug_; index++) {
        x_sigma_augmented.col(index + 1) = x_augmented_matrix + shift * sqrt_matrix.col(index);
        x_sigma_augmented.col(index + 1 + n_aug_) = x_augmented_matrix - shift * sqrt_matrix.col(index);
    }

    for (int index{ 0 }; index < aug_dim; index++) {
        const double x_point{ x_sigma_augmented(0, index) };
        const double y_point{ x_sigma_augmented(1, index) };
        const double velocity{ x_sigma_augmented(2, index) };
        const double yaw{ x_sigma_augmented(3, index) };
        const double yaw_diff{ x_sigma_augmented(4, index) };
        const double nu_angle{ x_sigma_augmented(5, index) };
        const double nu_yaw_diff{ x_sigma_augmented(6, index) };

        double predicted_x, predicted_y;

        if (fabs(yaw_diff) > 0.001) {
            predicted_x = x_point + velocity / yaw_diff * (sin(yaw + yaw_diff * delta_t) - sin(yaw));
            predicted_y = y_point + velocity / yaw_diff * (cos(yaw) - cos(yaw + yaw_diff * delta_t));
        } else {
            predicted_x = x_point + velocity * delta_t * cos(yaw);
            predicted_y = y_point + velocity * delta_t * sin(yaw);
        }

        double predicted_velocity{ velocity };
        double yaw_predicted{ yaw + yaw_diff * delta_t };
        double yaw_diff_predicted{ yaw_diff };
        const double predicate{ 0.5 * nu_angle * delta_t * delta_t };

        predicted_x += predicate * cos(yaw);
        predicted_y += predicate * sin(yaw);
        predicted_velocity += nu_angle * delta_t;

        yaw_predicted += 0.5 * nu_yaw_diff * delta_t * delta_t;
        yaw_diff_predicted += nu_yaw_diff * delta_t;

        Xsig_pred_(0, index) = predicted_x;
        Xsig_pred_(1, index) = predicted_y;
        Xsig_pred_(2, index) = predicted_velocity;
        Xsig_pred_(3, index) = yaw_predicted;
        Xsig_pred_(4, index) = yaw_diff_predicted;
    }

    x_.fill(0.0);

    for (int index{ 0 }; index < aug_dim; index++) { x_ += weights_(index) * Xsig_pred_.col(index); }

    P_.fill(0.0);

    for (int index{ 0 }; index < aug_dim; index++) {
        VectorXd x_angle_difference{ Xsig_pred_.col(index) - x_ };

        decrementNormalize(x_angle_difference, 3);
        incrementNormalize(x_angle_difference, 3, true);

        P_ += weights_(index) * x_angle_difference * x_angle_difference.transpose();
    }
}

void UKF::UpdateLidar(MeasurementPackage meas_package) {
    /**
     * Lidar data is used to update the belief about the object's position.
     * Modifications: the state vector, x_, and covariance, P_.
     * NIS, if desired.
     */
    MatrixXd state{ MatrixXd(2, n_x_) };
    state << 1, 0, 0, 0, 0, 0, 1, 0, 0, 0;
    MatrixXd lidar_covariance{ MatrixXd(2, 2) };
    lidar_covariance << std_laspx_ * std_laspx_, 0, 0, std_laspy_ * std_laspy_;
    VectorXd z_vector{ meas_package.raw_measurements_ };
    VectorXd z_prediction{ state * x_ };
    VectorXd y_vector{ z_vector - z_prediction };
    MatrixXd state_transpose{ state.transpose() };
    MatrixXd measurement_covariance{ state * P_ * state_transpose + lidar_covariance };
    MatrixXd measurement_covariance_inverse{ measurement_covariance.inverse() };
    MatrixXd gain{ P_ * state_transpose * measurement_covariance_inverse };

    x_ += gain * y_vector;

    MatrixXd identity_matrix{ MatrixXd::Identity(x_.size(), x_.size()) };
    P_ = (identity_matrix - gain * state) * P_;

    std::cout << "Lidar NIS: " << CalculateNISValue(z_prediction, z_vector, measurement_covariance)
              << std::endl;
}

void UKF::UpdateRadar(MeasurementPackage meas_package) {
    /**
     * Radar data is used to update the belief about the object's position.
     * Modified: the state vector, x_, and covariance, P_.
     * NIS, if desired.
     */
    constexpr int measurement_dim{ 3 };

    MatrixXd z_sigma{ MatrixXd(measurement_dim, aug_dim) };
    VectorXd z_prediction{ VectorXd(measurement_dim) };
    MatrixXd measurement_covariance{ MatrixXd(measurement_dim, measurement_dim) };

    for (int index{ 0 }; index < aug_dim; index++) {
        const double x_point{ Xsig_pred_(0, index) };
        const double y_point{ Xsig_pred_(1, index) };
        const double velocity{ Xsig_pred_(2, index) };
        const double yaw{ Xsig_pred_(3, index) };

        const double velocity_x{ velocity * cos(yaw) };
        const double velocity_y{ velocity * sin(yaw) };

        const double dist{ sqrt(x_point * x_point + y_point * y_point) };

        z_sigma(0, index) = dist;
        z_sigma(1, index) = atan2(y_point, x_point);
        z_sigma(2, index) = (x_point * velocity_x + y_point * velocity_y) / dist;
    }

    z_prediction.fill(0);

    for (int index{ 0 }; index < aug_dim; index++) { z_prediction += weights_(index) * z_sigma.col(index); }

    measurement_covariance.fill(0);

    for (int index{ 0 }; index < aug_dim; index++) {
        VectorXd z_angle_difference{ z_sigma.col(index) - z_prediction };

        decrementNormalize(z_angle_difference, 1);
        incrementNormalize(z_angle_difference, 1);

        measurement_covariance += weights_(index) * z_angle_difference * z_angle_difference.transpose();
    }

    MatrixXd noise{ MatrixXd(measurement_dim, measurement_dim) };

    const double squared_std_radr{ std_radr_ * std_radr_ };

    noise << squared_std_radr, 0, 0, 0, squared_std_radr, 0, 0, 0, squared_std_radr;

    measurement_covariance = measurement_covariance + noise;
    MatrixXd cross_correlation{ MatrixXd(n_x_, measurement_dim) };
    cross_correlation.fill(0);

    for (int index{ 0 }; index < aug_dim; index++) {
        VectorXd z_angle_difference{ z_sigma.col(index) - z_prediction };

        decrementNormalize(z_angle_difference, 1);
        incrementNormalize(z_angle_difference, 1);

        VectorXd x_angle_difference{ Xsig_pred_.col(index) - x_ };

        decrementNormalize(x_angle_difference, 3);
        incrementNormalize(x_angle_difference, 3, true);

        cross_correlation += weights_(index) * x_angle_difference * z_angle_difference.transpose();
    }

    MatrixXd gain{ cross_correlation * measurement_covariance.inverse() };

    VectorXd z_vector{ meas_package.raw_measurements_ };
    VectorXd z_angle_difference{ z_vector - z_prediction };

    incrementNormalize(z_angle_difference, 1);
    decrementNormalize(z_angle_difference, 1);

    x_ = x_ + gain * z_angle_difference;
    P_ = P_ - gain * measurement_covariance * gain.transpose();

    std::cout << "Radar NIS: " << CalculateNISValue(z_prediction, z_vector, measurement_covariance) << std::endl;
}
