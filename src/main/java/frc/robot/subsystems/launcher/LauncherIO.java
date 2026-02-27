package frc.robot.subsystems.launcher;

import org.littletonrobotics.junction.AutoLog;

public interface LauncherIO {
  @AutoLog
  public static class LauncherIOInputs {
    // Launcher Flywheel
    public double rpm;
    public double projectileSpeed;
    public double projectileRotationalSpeed;

    // Turret
    public double turretAngle;
    public double turretSpeed;
  }

  public default void updateInputs(LauncherIOInputs inputs) {}

  // Need to update this IO interface to add an RPM or rad/s function that sets the flywheel speed
  public default void setRadPerS(double rps) {}

  // Set launcher voltage
  public default void setVoltage(double voltage) {}

  // Command to point the launcher at an angle in degrees
  public default void pointAt(double angle) {}

  // Sets the minimum and maximum accepted angles in degrees
  public default void setLockout(double minAngle, double maxAngle) {}

  // Simulate feeding a projectile into the launcher
  public default void simLaunch() {}

  public default void testTurn(double voltage) {}

  public default void testFlyWheelTurn(double voltage) {}

  public default void resetEncoder() {}
}
