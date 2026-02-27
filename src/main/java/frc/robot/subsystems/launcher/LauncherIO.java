package frc.robot.subsystems.launcher;

import org.littletonrobotics.junction.AutoLog;

public interface LauncherIO {
  @AutoLog
  public static class LauncherIOInputs {
    // Launcher Flywheel
    public double rpm = 0.0;
    public double projectileSpeed = 0.0;
    public double projectileRotationalSpeed = 0.0;

    // Turret
    public double turretAngle = 0.0;
    public double turretSpeed = 0.0;

    // launcher motor
    public double launcherMotorVoltage = 0.0;
    public double launcherStatorCurrent = 0.0;
    public double launcherTorqueCurrent = 0.0;
    public double launcherAcceleration = 0.0;
    public double launcherClosedLoopError = 0.0;
    public double launcherVelocity = 0.0;
    public double launcherSupplyCurrent = 0.0;
    public double launcherSupplyVoltage = 0.0;

    // turn motor
    public double turnMotorAppliedOutput = 0.0;
    public double turnMotorBusVoltage = 0.0;
    public double turnMotorOutputCurrent = 0.0;

    // turn encoder
    public double turnEncoderVelocity = 0.0;
    public double turnEncoderPosition = 0.0;
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

  public default void invertTurn() {}

  public default void deinvertTurn() {}

  public default void testFlyWheelTurn(double voltage) {}

  public default void resetEncoder() {}
}
