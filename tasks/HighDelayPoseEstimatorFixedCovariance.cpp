/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "HighDelayPoseEstimatorFixedCovariance.hpp"

using namespace pose_estimation;

HighDelayPoseEstimatorFixedCovariance::HighDelayPoseEstimatorFixedCovariance(
    std::string const& name)
    : HighDelayPoseEstimatorFixedCovarianceBase(name)
{
}

HighDelayPoseEstimatorFixedCovariance::~HighDelayPoseEstimatorFixedCovariance()
{
}

/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See HighDelayPoseEstimatorFixedCovariance.hpp for more
// detailed documentation about them.

bool HighDelayPoseEstimatorFixedCovariance::configureHook()
{
    if (!HighDelayPoseEstimatorFixedCovarianceBase::configureHook())
        return false;

    m_cov_xy_samples = _cov_xy_samples.get();
    return true;
}

bool HighDelayPoseEstimatorFixedCovariance::startHook()
{
    if (!HighDelayPoseEstimatorFixedCovarianceBase::startHook())
        return false;
    return true;
}

void HighDelayPoseEstimatorFixedCovariance::updateHook()
{
    HighDelayPoseEstimatorFixedCovarianceBase::updateHook();
}

void HighDelayPoseEstimatorFixedCovariance::xy_position_samplesTransformerCallback(
    const base::Time& ts,
    const ::base::samples::RigidBodyState& xy_position_samples_sample)
{
    if (!pose_estimator->isInitialized()) {
        RTT::log(RTT::Error)
            << "Waiting for pose samples, filter has not jet been initialized"
            << RTT::endlog();
        return;
    }

    // receive sensor to body transformation
    Eigen::Affine3d sensor_map2target_map;
    if (!getSensorInBodyPose(_xy_map2target_map, ts, sensor_map2target_map))
        return;

    if (xy_position_samples_sample.position.block(0, 0, 2, 1).allFinite() &&
        m_cov_xy_samples.block(0, 0, 2, 2).allFinite()) {
        predictionStep(ts);
        try {
            // this transforms a position measurement expressed in a different map frame
            // to a position expressed in the target map frame
            Eigen::Vector3d transformed_position_sample =
                sensor_map2target_map *
                Eigen::Vector3d(xy_position_samples_sample.position.x(),
                    xy_position_samples_sample.position.y(),
                    0.0);

            PoseUKF::XYMeasurement measurement;
            measurement.mu = transformed_position_sample.block(0, 0, 2, 1);
            measurement.cov = m_cov_xy_samples.block(0, 0, 2, 2);
            pose_estimator->integrateMeasurement(measurement);
        }
        catch (const std::runtime_error& e) {
            RTT::log(RTT::Error)
                << "Failed to add delayed position measurement: " << e.what()
                << RTT::endlog();
        }
    }
    else
        RTT::log(RTT::Error)
            << "Delayed position measurement contains NaN's, it will be skipped!"
            << RTT::endlog();
}

void HighDelayPoseEstimatorFixedCovariance::errorHook()
{
    HighDelayPoseEstimatorFixedCovarianceBase::errorHook();
}
void HighDelayPoseEstimatorFixedCovariance::stopHook()
{
    HighDelayPoseEstimatorFixedCovarianceBase::stopHook();
}
void HighDelayPoseEstimatorFixedCovariance::cleanupHook()
{
    HighDelayPoseEstimatorFixedCovarianceBase::cleanupHook();
}
