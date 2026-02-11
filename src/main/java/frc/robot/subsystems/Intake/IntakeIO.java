package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {

  @AutoLog
  public static class IntakeIOInputs {
    public double rotatoOutput = 0.0;
    public double rotatoBusVoltage = 0.0;
    public double rotatoCurrent = 0.0;

    public double MovEncoderValue = 1;
    public double armMotorOutput = 0.0;
    public double armMotorBusVoltage = 0.0;
    public double armMotorCurrent = 0.0;

    public double positionRad = 0.0;

    /** Gives rpm of rotato */
    public double rotatoRpm = 0.0;

    /** Gives rpm of movement motor */
    public double MovementMotorRPM = 0.0;

    /** Gives position in rotations of the up/down motor */
    public double MovementMotorRotation = 0.0;
  }

  /** Updates all sensor inputs */
  public default void updateInputs(IntakeIOInputs inputs) {}

  /** Runs the intake roller */
  public default void runRotato(double speed) {}

  /** Extends/retracts intake to a position in radians */
  public default void setIntakePosition(double radians) {}
}
