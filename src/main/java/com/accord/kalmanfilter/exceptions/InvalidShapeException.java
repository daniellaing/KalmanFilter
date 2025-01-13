package com.accord.kalmanfilter.exceptions;

/**
 * Invalid shape for matrix.
 *
 * <p>Thrown when a matrix has a shape invalid for an operation.
 */
public class InvalidShapeException extends IllegalArgumentException {
  /** Create a new {@link InvalidShapeException}. */
  public InvalidShapeException() {}

  /**
   * Create a new {@link InvalidShapeException}.
   *
   * @see InvalidShapeException#InvalidShapeException(String, Throwable)
   */
  public InvalidShapeException(String msg) {
    super(msg);
  }

  /**
   * Create a new {@link InvalidShapeException}.
   *
   * @see InvalidShapeException#InvalidShapeException(String, Throwable)
   */
  public InvalidShapeException(Throwable cause) {
    super(cause);
  }

  /**
   * Create a new {@link InvalidShapeException}.
   *
   * @param msg Message.
   * @param cause Cause of the exception.
   */
  public InvalidShapeException(String msg, Throwable cause) {
    super(msg, cause);
  }
}
