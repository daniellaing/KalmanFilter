package com.accord.kalmanfilter;

import static org.ejml.dense.row.CommonOps_DDRM.addEquals;
import static org.ejml.dense.row.CommonOps_DDRM.mult;
import static org.ejml.dense.row.CommonOps_DDRM.multTransA;
import static org.ejml.dense.row.CommonOps_DDRM.multTransB;
import static org.ejml.dense.row.CommonOps_DDRM.subtract;
import static org.ejml.dense.row.CommonOps_DDRM.subtractEquals;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.factory.LinearSolverFactory_DDRM;
import org.ejml.interfaces.linsol.LinearSolverDense;
import org.ejml.simple.SimpleMatrix;

/**
 * Represents a Kalman filter.
 *
 * <p>Implemented by the classical Kalman filter equations.
 *
 * @author daniel.laing
 */
@SuppressWarnings({
  "checkstyle:MemberName",
  "checkstyle:MultipleVariableDeclarations",
  "checkstyle:ParameterName"
})
public class ClassicalKalmanFilter implements KalmanFilter {
  private DMatrixRMaj F, H, Q, R, P, x;

  // Helper variables
  private DMatrixRMaj y, S, Sinv, K;
  private DMatrixRMaj a, b, c, d;

  private LinearSolverDense<DMatrixRMaj> solver;

  /**
   * Constructor for a Kalman filter.
   *
   * @param F State transition matrix
   * @param H Observation matrix
   * @param Q Process noise covariance matrix
   * @param R Measurement noise covariance matrix
   * @param P Initial state covariance matrix
   * @param x Initial state variables
   */
  public ClassicalKalmanFilter(
      SimpleMatrix F,
      SimpleMatrix H,
      SimpleMatrix Q,
      SimpleMatrix R,
      SimpleMatrix P,
      SimpleMatrix x) {
    this.F = F.getMatrix();
    this.H = H.getMatrix();
    this.Q = Q.getMatrix();
    this.R = R.getMatrix();
    this.P = P.getMatrix();
    this.x = x.getMatrix();

    int dimx = this.F.numCols;
    int dimz = this.H.numRows;

    a = new DMatrixRMaj(dimx, 1);
    b = new DMatrixRMaj(dimx, dimx);
    c = new DMatrixRMaj(dimz, dimx);
    d = new DMatrixRMaj(dimz, dimz);

    y = new DMatrixRMaj(dimz, 1);
    S = new DMatrixRMaj(dimz, dimz);
    Sinv = new DMatrixRMaj(dimz, dimz);
    K = new DMatrixRMaj(dimx, dimz);

    solver = LinearSolverFactory_DDRM.symmPosDef(dimz);
  }

  public DMatrixRMaj getQ() {
    return Q;
  }

  public void setQ(DMatrixRMaj q) {
    Q = q;
  }

  public DMatrixRMaj getR() {
    return R;
  }

  public void setR(DMatrixRMaj r) {
    R = r;
  }

  public SimpleMatrix getStateCovariance() {
    return SimpleMatrix.wrap(P);
  }

  public SimpleMatrix getInnovations() {
    return SimpleMatrix.wrap(y);
  }

  public SimpleMatrix getInnovationCovariance() {
    return SimpleMatrix.wrap(S);
  }

  public DMatrixRMaj getK() {
    return K;
  }

  /** Predict the next state variables. */
  public void predict() {
    // Calculate state prediction
    // x = F*x
    mult(F, x, a); // a = F*x
    x.setTo(a);

    // Calculate prediction covariance
    // P = F*P*F' + Q
    mult(F, P, b); // b = F*P
    multTransB(b, F, P); // P = b*F'
    addEquals(P, Q); // P = P + Q
  }

  /**
   * Predict the next state variables.
   *
   * @param u Inputs
   * @param B Input matrix
   */
  public void predict(SimpleMatrix u, SimpleMatrix B) {
    // Calculate state prediction
    // x = F*x
    mult(F, x, a); // a = F*x
    x.setTo(a);
    // Calculate inputs
    // a = B*u
    mult(B.getMatrix(), u.getMatrix(), a);
    // Add inputs to x
    addEquals(x, a);

    // Calculate prediction covariance
    // P = F*P*F' + Q
    mult(F, P, b); // b = F*P
    multTransB(b, F, P); // P = b*F'
    addEquals(P, Q); // P = P + Q
  }

  /**
   * Update state variable prediction.
   *
   * @param z Measurement
   */
  public void update(SimpleMatrix z) {
    updateInternal(z.getMatrix(), this.R);
  }

  /**
   * Update state variable prediction.
   *
   * @param z Measurement
   * @param R Measurement covariance
   */
  public void update(SimpleMatrix z, SimpleMatrix R) {
    updateInternal(z.getMatrix(), R.getMatrix());
  }

  /**
   * Get the current state variables.
   *
   * @return A SimpleMatrix containing the state variables.
   */
  public SimpleMatrix getState() {
    return SimpleMatrix.wrap(this.x);
  }

  /**
   * Update state variable predictions.
   *
   * @param z Measrument
   * @param R Measurement covariance
   */
  private void updateInternal(DMatrixRMaj z, DMatrixRMaj R) {
    // Calculate innovations based on predicted state
    // y = z - H*x
    mult(H, x, y); // y = H*x
    subtract(z, y, y); // y = z - y

    // Calculate innovation covariance
    // S = H*P*H' +R
    mult(H, P, c); // c = H*P
    multTransB(c, H, S); // S = c*H'
    addEquals(S, R); // S = S + R

    // Calculate Kalman gain
    // K = P*H'*S^-1
    if (!solver.setA(S)) {
      throw new RuntimeException("Could not invert S");
    }
    solver.invert(Sinv); // Sinv = S^-1
    multTransA(H, Sinv, d); // d = H'S^-1
    mult(P, d, K); // K = P*d

    // Calculate updated state
    // x = x + K*y
    mult(K, y, a); // a = K*y

    addEquals(x, a); // x = x + a

    // Calculate updated state covariance
    // P = P - K*H*P
    mult(K, H, c); // c = K*H
    mult(c, P, b); // b = c*P
    subtractEquals(P, b); // P = P - b
  }
}
