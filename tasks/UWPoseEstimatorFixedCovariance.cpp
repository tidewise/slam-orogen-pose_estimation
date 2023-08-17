/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "UWPoseEstimatorFixedCovariance.hpp"

using namespace pose_estimation;

UWPoseEstimatorFixedCovariance::UWPoseEstimatorFixedCovariance(std::string const& name)
    : UWPoseEstimatorFixedCovarianceBase(name)
{
}

UWPoseEstimatorFixedCovariance::~UWPoseEstimatorFixedCovariance()
{
}

/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See UWPoseEstimatorFixedCovariance.hpp for more detailed
// documentation about them.

bool UWPoseEstimatorFixedCovariance::configureHook()
{
    if (!UWPoseEstimatorFixedCovarianceBase::configureHook())
        return false;

    m_cov_depth = _cov_depth.get();
    m_cov_orientation = _cov_orientation.get();
    return true;
}
bool UWPoseEstimatorFixedCovariance::startHook()
{
    if (!UWPoseEstimatorFixedCovarianceBase::startHook())
        return false;
    return true;
}

void UWPoseEstimatorFixedCovariance::updateHook()
{
    UWPoseEstimatorFixedCovarianceBase::updateHook();
}

void UWPoseEstimatorFixedCovariance::depth_samplesTransformerCallback(
    const base::Time& ts,
    const ::base::samples::RigidBodyState& depth_samples_sample)
{
    // receive sensor to body transformation
    Eigen::Affine3d sensorInBody;
    if (!getSensorInBodyPose(_pressure_sensor2body, ts, sensorInBody))
        return;

    PoseUKF::State current_state;
    if (!base::isNaN(depth_samples_sample.position.z()) && !base::isNaN(m_cov_depth) &&
        pose_estimator->getCurrentState(current_state)) {
        predictionStep(ts);
        try {
            // apply sensorInBody transformation to measurement
            Eigen::Matrix<double, 1, 1> depth;
            depth << depth_samples_sample.position.z() -
                         (current_state.orientation * sensorInBody.translation()).z();

            PoseUKF::ZMeasurement measurement;
            measurement.mu = depth;
            measurement.cov(0, 0) = m_cov_depth;
            pose_estimator->integrateMeasurement(measurement);
        }
        catch (const std::runtime_error& e) {
            RTT::log(RTT::Error)
                << "Failed to add depth measurement: " << e.what() << RTT::endlog();
        }
    }
    else
        RTT::log(RTT::Error) << "Depth measurement contains NaN's, it will be skipped!"
                             << RTT::endlog();
}

void UWPoseEstimatorFixedCovariance::orientation_samplesTransformerCallback(
    const base::Time& ts,
    const ::base::samples::RigidBodyState& orientation_samples_sample)
{
    // receive sensor to body transformation
    Eigen::Affine3d sensorInBody;
    if (!getSensorInBodyPose(_imu2body, ts, sensorInBody))
        return;

    if (orientation_samples_sample.hasValidOrientation() &&
        base::samples::RigidBodyState::isValidCovariance(m_cov_orientation)) {
        predictionStep(ts);
        try {
            // apply sensorInBody transformation to measurement
            Eigen::Quaterniond transformed_orientation(
                (orientation_samples_sample.orientation * sensorInBody.inverse())
                    .linear());

            PoseUKF::OrientationMeasurement measurement;
            measurement.mu = MTK::SO3<double>::log(transformed_orientation);
            measurement.cov = sensorInBody.rotation() * m_cov_orientation *
                              sensorInBody.rotation().transpose();
            pose_estimator->integrateMeasurement(measurement);
        }
        catch (const std::runtime_error& e) {
            RTT::log(RTT::Error)
                << "Failed to add orientation measurement: " << e.what() << RTT::endlog();
        }
    }
    else
        RTT::log(RTT::Error)
            << "Orientation measurement contains NaN's, it will be skipped!"
            << RTT::endlog();
}

void UWPoseEstimatorFixedCovariance::errorHook()
{
    UWPoseEstimatorFixedCovarianceBase::errorHook();
}
void UWPoseEstimatorFixedCovariance::stopHook()
{
    UWPoseEstimatorFixedCovarianceBase::stopHook();
}
void UWPoseEstimatorFixedCovariance::cleanupHook()
{
    UWPoseEstimatorFixedCovarianceBase::cleanupHook();
}
