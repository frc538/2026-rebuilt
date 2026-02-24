package frc.robot.subsystems.Intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {

  @AutoLog
  public static class IntakeIOInputs {
    public double RightrotatoOutput = 0.0;
    public double RightrotatoBusVoltage = 0.0;
    public double RightrotatoCurrent = 0.0;

    public double LeftrotatoOutput = 0.0;
    public double LeftrotatoBusVoltage = 0.0;
    public double LeftrotatoCurrent = 0.0;

    public double MovEncoderValue = 1;
    public double armMotorOutput = 0.0;
    public double armMotorBusVoltage = 0.0;
    public double armMotorCurrent = 0.0;

    public double positionRad = 0.0;

    /** Gives rpm of rotato */
    public double RightrotatoRpm = 0.0;

    public double LeftrotatoRpm = 0.0;

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

  /*Complicated Trapezoid Stuff. */
  public default void ffCommand(double angle) {}
}
