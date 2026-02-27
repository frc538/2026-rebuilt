package frc.robot.subsystems.launcher;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
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

    public double turretBusVoltage;
    public double turretCurrent;
    public double turretOutput;
  }

  public default void updateInputs(LauncherIOInputs inputs) {}

  // Need to update this IO interface to add an RPM or rad/s function that sets the flywheel speed
  public default void setRadPerS(double rps) {}

  // Set flywheel voltage
  public default void setVoltage(double voltage) {}

  // Command to point the launcher at an angle in degrees
  public default void pointAt(double angle) {}

  // Command to set the turret motor voltage
  public default void turretVoltage(double voltage) {}

  // Sets the minimum and maximum accepted angles in degrees
  public default void setLockout(double minAngle, double maxAngle) {}

  // Simulate feeding a projectile into the launcher
  public default void simLaunch() {}

  public default void updateRobotInfo(Pose2d robotPose, ChassisSpeeds robotVelocity) {}
  ;
}
