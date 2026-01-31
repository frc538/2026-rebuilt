package frc.robot.subsystems.flywheel;

import org.littletonrobotics.junction.AutoLog;

public interface FlywheelIO {
  @AutoLog
  public static class FlywheelIOInputs {
    public double rpm;
  }

  public default void updateInputs(FlywheelIOInputs inputs) {}

  public default void setVoltage(double voltage) {}

  public default void simLaunch() {}
}
