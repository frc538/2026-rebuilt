package frc.robot.subsystems.launcher;

import org.littletonrobotics.junction.AutoLog;

public interface LauncherIO {
  @AutoLog
  public static class LauncherIOInputs {
    public double rpm;
    public double projectileSpeed;
    public double projectileRotationalSpeed;
  }

  public default void updateInputs(LauncherIOInputs inputs) {}

  public default void setVoltage(double voltage) {}

  public default void simLaunch() {}
}
