package frc.robot.subsystems.launcher;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import org.littletonrobotics.junction.AutoLog;

public interface LauncherIO {
  @AutoLog
  public static class LauncherIOInputs {
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
    public double turnPotentiometer = 0.0;
  }

  public default void updateInputs(LauncherIOInputs inputs) {}

  public default void TurretCalibrate(double radians) {}

  public default void TurretDisable() {}

  // Need to update this IO interface to add an RPM or rad/s function that sets the flywheel speed
  public default void setRadPerS(double rps) {}

  // Set flywheel voltage
  public default void setVoltage(double voltage) {}

  // Command to point the launcher at an angle in radians
  public default void pointAt(double rad, double radPerSec) {}

  // Command to set the turret motor voltage
  public default void turretVoltage(double voltage) {}

  // Sets the minimum and maximum accepted angles in degrees
  public default void setLockout(double minAngle, double maxAngle) {}

  // Simulate feeding a projectile into the launcher
  public default void simLaunch() {}

  public default void testTurn(double voltage) {}

  public default void invertTurn() {}

  public default void deinvertTurn() {}

  public default void testFlyWheelTurn(double voltage) {}

  public default void resetEncoder() {}

  public default void updateRobotInfo(Pose2d robotPose, ChassisSpeeds robotVelocity) {}
}
