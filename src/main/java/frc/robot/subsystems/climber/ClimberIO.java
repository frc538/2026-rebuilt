package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {
  @AutoLog
  public static class ClimberIOInputs {
    public double motorPosition = 0.0;
    public double motorVoltage = 0;
    public double motorCurrent = 0;
  }

  public default void setOutput(double speed) {}

  public default void updateInputs(ClimberIOInputs inputs) {}
}
