package com.accord.kalmanfilter;

import java.util.Random;
import org.ejml.simple.SimpleMatrix;
import org.junit.Test;

/** Unit test for simple App. */
@SuppressWarnings({"checkstyle:LocalVariableName"})
public class TrolleyTest {
  /**
   * Test Kalman filter with trolley example. Taken from the Kalman filter page on Wikipedia.
   *
   * <p>Consider a truck on a straight frictionless track. It starts at rest, position 0, and is
   * buffeted by random forces. Every interval, the truck's position is measured. We model the
   * truck's position and velocity with the Kalman filter.
   */
  @Test
  public void trolleyTest() {
    Random rng = new Random();
    int num_steps = 30;
    // Time interval between measurements.
    double dt = 1;
    double a_variance = 1; // Variance in the forces applied to the trolley.
    double m_variance = 0.25; // Variance in the measurement noise

    var F = new SimpleMatrix(new double[][] {new double[] {1F, dt}, new double[] {0F, 1F}});
    var G = new SimpleMatrix(new double[] {0.5F * dt * dt, dt});
    var Q =
        new SimpleMatrix(
                new double[][] {
                  new double[] {0.25F * (double) Math.pow(dt, 4), 0.5F * (double) Math.pow(dt, 3)},
                  new double[] {0.5F * (double) Math.pow(dt, 3), dt * dt}
                })
            .scale(a_variance);
    var H = new SimpleMatrix(new double[] {1F, 0F}).transpose();
    var R = new SimpleMatrix(new double[] {m_variance});
    var x0 = new SimpleMatrix(new double[] {0, 0});
    var P = SimpleMatrix.diag(0, 0);

    var k = new KalmanFilter(F, H, Q, R, P, x0);

    var x_true = x0;
    for (int i = 0; i < num_steps; i++) {
      // Calculate true position
      var a = rng.nextGaussian(0, Math.sqrt(a_variance));
      x_true = F.mult(x_true).plus(G.mult(SimpleMatrix.diag(a)));

      // Simulate measurement
      var z = H.mult(x_true).plus(rng.nextGaussian(0, Math.sqrt(m_variance)));

      // Step the Kalman filter
      k.predict();
      k.update(z);

      // Print out the error
      System.out.println(k.getState().get(0) - x_true.get(0));
    }
  }
}
