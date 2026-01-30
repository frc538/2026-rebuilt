package frc.robot.subsystems.Intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {

  @AutoLog
  public static class IntakeIOInputs {
    public double positionRad = 0.0;
  }

  /** Updates all sensor inputs */
  public default void updateInputs(IntakeIOInputs inputs) {}

  /** Runs the intake roller */
  public default void runIntake(double speed) {}

  /** Sets vortex velocity in rad/s */
  public default void setVortexVelocity(double radiansPerSecond) {}

  /** Extends/retracts intake to a position in radians */
  public default void setIntakePosition(double radians) {}
}
