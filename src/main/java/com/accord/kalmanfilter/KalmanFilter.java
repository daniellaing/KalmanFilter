package com.accord.kalmanfilter;

import org.ejml.simple.SimpleMatrix;

/**
 * Interface representing a Kalman filter.
 *
 * @author daniel.laing
 */
public interface KalmanFilter {

  /** Step the Kalman filter, predicting the next state. */
  public void predict();

  /**
   * {@code R} defaults to the zero matrix.
   *
   * @see KalmanFilter#update(SimpleMatrix, SimpleMatrix)
   */
  public void update(SimpleMatrix z);

  /**
   * Update the Kalman filter with a measurement.
   *
   * @param z One column {@link SimpleMatrix} containing the measurement.
   * @param R The measurement noise covariance matrix
   */
  @SuppressWarnings({"checkstyle:ParameterName"})
  public void update(SimpleMatrix z, SimpleMatrix R);

  /**
   * Get the current state of the Kalman filter.
   *
   * @return A one column {@link SimpleMatrix} containing the current state variables.
   */
  public SimpleMatrix getState();

  /**
   * Get the covariance matrix of the current state.
   *
   * @return The covariance matrix of the current state variables.
   */
  public SimpleMatrix getStateCovariance();

  /**
   * Get the innovations for the previous step.
   *
   * @return A one column {@link SimpleMatrix} containing the innovations.
   */
  public SimpleMatrix getInnovations();

  /**
   * Get the covariance matrix of the innovations for the previous step.
   *
   * @return The covariance matrix of the innovations.
   */
  public SimpleMatrix getInnovationCovariance();
}
