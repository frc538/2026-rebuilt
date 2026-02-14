// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static class ClimberConstants {
    public static final int CurrentLimit = 50;
    public static final double ClimberPositionConversionFactor =
        90.0 / 480.0; // 480 revolutions gives us 90 degrees on the climber.
    public static final int ClimberVelocityConversionFactor = /* put real value here--> */ 1;
    public static final int ClimberWheelFreeSpeedMetersPerSecond = /* put real value here--> */ 1;
    public static final int ClimberMotorCANId = 13;
  }

  public final class Hopper {
    public static final double SpindexGearing = 1.0;
    public static final double FeedGearing = 1.0;
    public static final double SDConversionFactor = 1.0;
    public static final double FDConversionFactor = 1.0;
    public static final double SpindexSpeed = 1.0;
    public static final double FeedSpeed = 1.0;
    public static final int FeedCanId = 3;
    public static final int SpindexCanId = 4;
  }

  public final class launcherConstants {
    public static final int launchMotorID = 5;
    public static final Pose2d hubBlue = new Pose2d(4.625, 4.030, new Rotation2d());
    public static final Pose2d hubRed = new Pose2d(11.915, 4.030, new Rotation2d());
    public static final Pose2d leftRed = new Pose2d(14.000, 1.7, new Rotation2d());
    public static final Pose2d rightRed = new Pose2d(14.000, 6.250, new Rotation2d());
    public static final Pose2d leftBlue = new Pose2d(2.500, 6.250, new Rotation2d());
    public static final Pose2d rightBlue = new Pose2d(2.5000, 1.7, new Rotation2d());
    public static final double hubHeight = 1.8288; // meters
    public static final double launcherHeight = 0.508; // meters estimate
    public static final double launcherAngle = 1.0472; // radians

    public static final double launchWheelRadius = 0.050165; // meters
    public static final double fuelMass = 0.2268; // kg
    public static final double fuelRadius = 0.075; // meters
    public static final double kFuelMomentOfInertia =
        2.0 / 5.0 * fuelMass * fuelRadius * fuelRadius;
    public static final double kFlywheelMomentOfInertia = 0.0004926; // kg * m^2
    public static final double kWheelRadius = 0.050165; // m
  }
}
