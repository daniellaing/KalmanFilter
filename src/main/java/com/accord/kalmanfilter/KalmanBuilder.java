package com.accord.kalmanfilter;

import com.accord.kalmanfilter.exceptions.InvalidShapeException;
import org.ejml.simple.SimpleMatrix;

/** Build a {@link KalmanFilter}. */
@SuppressWarnings({"checkstyle:MemberName"})
public class KalmanBuilder {
  /** Number of state variables. */
  private int d;

  /** State transition matrix. */
  private SimpleMatrix F;

  /** Observation matrix. */
  private SimpleMatrix H;

  /** Process noise covariance matrix. */
  private SimpleMatrix Q;

  /** Measurement noise covariance matrix. */
  private SimpleMatrix R;

  /** Initial state covariance matrix. */
  private SimpleMatrix P;

  /** State variables. */
  private SimpleMatrix x;

  /**
   * Create a Kalman filter with {@code d} state variables.
   *
   * @param d Number of state variables.
   */
  public KalmanBuilder(int d) {
    if (d < 1) {
      throw new IllegalArgumentException("A Kalman filter must have at least one state variable.");
    }
    this.d = d;
  }

  /**
   * Set the state transition matrix.
   *
   * @param stateTransition A {@link SimpleMatrix} defining the state transition.
   */
  public KalmanBuilder stateTransition(SimpleMatrix stateTransition) {
    F = stateTransition;
    return this;
  }

  /** Alias for {@link KalmanBuilder#stateTransition(SimpleMatrix)}. */
  @SuppressWarnings({"checkstyle:ParameterName", "checkstyle:MethodName"})
  public KalmanBuilder F(SimpleMatrix F) {
    this.F = F;
    return this;
  }

  /**
   * Set the Kalman filter to have a stead state.
   *
   * <p>That is, the state transition matrix is an identity matrix.
   */
  public KalmanBuilder steadyState() {
    F = SimpleMatrix.identity(this.d);
    return this;
  }

  /**
   * Set the observation matrix.
   *
   * @param observation A {@link SimpleMatrix} defining how the measurements relate to the state
   *     variables.
   */
  public KalmanBuilder observation(SimpleMatrix observation) {
    H = observation;
    return this;
  }

  /** Alias for {@link KalmanBuilder#observation(SimpleMatrix)}. */
  @SuppressWarnings({"checkstyle:ParameterName", "checkstyle:MethodName"})
  public KalmanBuilder H(SimpleMatrix H) {
    this.H = H;
    return this;
  }

  /**
   * Set the process noise covariance matrix.
   *
   * @param processNoise A {@link SimpleMatrix} defining the process noise covariance.
   */
  public KalmanBuilder processNoise(SimpleMatrix processNoise) {
    Q = processNoise;
    return this;
  }

  /** Alias for {@link KalmanBuilder#processNoise(SimpleMatrix)}. */
  @SuppressWarnings({"checkstyle:ParameterName", "checkstyle:MethodName"})
  public KalmanBuilder Q(SimpleMatrix Q) {
    this.Q = Q;
    return this;
  }

  /**
   * Set the default measurement noise covariance matrix.
   *
   * <p>The measurement noise covariance can optionally be set at every update. This provides a
   * default in the case when it is not given.
   *
   * @param measurementNoise A {@link SimpleMatrix} defining the default measurement noise
   *     covariance.
   */
  public KalmanBuilder measurementNoise(SimpleMatrix measurementNoise) {
    R = measurementNoise;
    return this;
  }

  /** Alias for {@link KalmanBuilder#measurementNoise(SimpleMatrix)}. */
  @SuppressWarnings({"checkstyle:ParameterName", "checkstyle:MethodName"})
  public KalmanBuilder R(SimpleMatrix R) {
    this.R = R;
    return this;
  }

  /**
   * Set the initial state covariance matrix.
   *
   * @param initialCovariance A {@link SimpleMatrix} defining the initial state covariance.
   */
  public KalmanBuilder initialCovariance(SimpleMatrix initialCovariance) {
    P = initialCovariance;
    return this;
  }

  /** Alias for {@link KalmanBuilder#initialCovariance(SimpleMatrix)}. */
  @SuppressWarnings({"checkstyle:ParameterName", "checkstyle:MethodName"})
  public KalmanBuilder P(SimpleMatrix P) {
    this.P = P;
    return this;
  }

  /**
   * Set the initial state.
   *
   * @param initialState A one-column {@link SimpleMatrix} defining the initial state.
   */
  public KalmanBuilder initialState(SimpleMatrix initialState) throws InvalidShapeException {
    if (initialState.getNumCols() == 1) {

      x = initialState;
    } else if (initialState.getNumRows() == 1) {
      x = initialState.transpose();
    } else {
      throw new InvalidShapeException("The initial state must be a vector.");
    }
    return this;
  }

  /** Alias for {@link KalmanBuilder#initialState(SimpleMatrix)}. */
  public KalmanBuilder x0(SimpleMatrix x0) {
    initialState(x0);
    return this;
  }

  /**
   * Build a {@link ClassicalKalmanFilter}.
   *
   * @return A {@link ClassicalKalmanFilter}.
   */
  public ClassicalKalmanFilter buildClassical() {
    return new ClassicalKalmanFilter(F, H, Q, R, P, x);
  }
}
