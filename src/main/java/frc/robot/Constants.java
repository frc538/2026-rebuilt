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
// CanID List
// ClimberID 13
// Hopper, includes Feed 3 and Spindex 4
// Launcher 5, may need futher implementation
// Intake RotatoRight 1, RotatoLeft 6
// Intake MovementMotor 2

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

  public static class Features {
    public static final boolean ClimberEnabled = true;
    public static final boolean DriveEnabled = true;
    public static final boolean HopperEnabled = true;
    public static final boolean LauncherEnabled = true;
    public static final boolean VisionEnabled = true;
    public static final boolean IntakeEnabled = true;
  }

  public static class navigationConstants {
    // NAV MAP
    public static final Pose2d leftBlue = new Pose2d(3, 5.5, Rotation2d.fromDegrees(0));
    public static final Pose2d rightBlue = new Pose2d(3, 2.5, Rotation2d.fromDegrees(0));
    public static final Pose2d centerRightBlue = new Pose2d(6.3, 2.5, Rotation2d.fromDegrees(0));
    public static final Pose2d centerLeftBlue = new Pose2d(6.3, 5.5, Rotation2d.fromDegrees(0));

    // degrees for these two are inverted because thats the way your going to wanna face when you
    // reach the point
    public static final Pose2d topCenter = new Pose2d(8.7, 7, Rotation2d.fromDegrees(90));
    public static final Pose2d bottomCenter = new Pose2d(8.7, 1, Rotation2d.fromDegrees(270));

    public static final Pose2d rightRed = new Pose2d(13.8, 5.5, Rotation2d.fromDegrees(180));
    public static final Pose2d leftRed = new Pose2d(13.8, 2.5, Rotation2d.fromDegrees(180));

    public static final Pose2d centerRightRed = new Pose2d(10.5, 5.5, Rotation2d.fromDegrees(180));
    public static final Pose2d centerLeftRed = new Pose2d(10.5, 2.5, Rotation2d.fromDegrees(180));

    public static final double distanceThreshold = 0.5; // meters
  }

  public static class ClimberConstants {
    public static final int ClimberMotorCANId = 13;

    public static final int CurrentLimit = 50;

    public static final double ClimberPositionConversionFactor =
        90.0 / 480.0; // 480 revolutions gives us 90 degrees on the climber.
    public static final int ClimberVelocityConversionFactor = /* put real value here--> */ 1;

    public static final int ClimberWheelFreeSpeedMetersPerSecond = /* put real value here--> */ 1;
  }

  public final class Hopper {

    public static final int FeedCanId = 4;
    public static final int SpindexCanId = 3;

    public static final double SpindexGearing = 1.0;
    public static final double FeedGearing = 1.0;

    public static final double SDConversionFactor = 1.0;
    public static final double FDConversionFactor = 1.0;

    public static final double SpindexSpeed = -.25;
    public static final double FeedSpeed = -.5;

    public static final double TestSpindexSpeed = -0.3;
    public static final double TestFeedSpeed = -0.3;
  }

  public final class launcherConstants {
    public static final int launchMotorCanId = 37;
    public static final int turnMotorCanId = 62;
    public static final int CurrentLimit = 50;

    public static final double turretCalibrationOffset = 0.0;

    public static final double minWireLimit = 2.75; // TODO: base wire limits
    public static final double maxWireLimit = 4;

    public static final Pose2d hubBlue = new Pose2d(4.625, 4.030, new Rotation2d());
    public static final Pose2d hubRed = new Pose2d(11.915, 4.030, new Rotation2d());
    public static final Pose2d leftRed = new Pose2d(14.000, 1.7, new Rotation2d());
    public static final Pose2d rightRed = new Pose2d(14.000, 6.250, new Rotation2d());
    public static final Pose2d leftBlue = new Pose2d(2.500, 6.250, new Rotation2d());
    public static final Pose2d rightBlue = new Pose2d(2.5000, 1.7, new Rotation2d());

    public static final double hubHeight = 1.8288; // meters
    public static final double launcherHeight = 0.508; // meters estimate
    public static final double launcherAngle = 0.6981317; // radians

    public static final double launchWheelRadius = 0.050165; // meters
    public static final double fuelMass = 0.2268; // kg
    public static final double fuelRadius = 0.075; // meters
    public static final double kFuelMomentOfInertia =
        2.0 / 5.0 * fuelMass * fuelRadius * fuelRadius;
    public static final double kFlywheelMomentOfInertia = 0.0004926; // kg * m^2
    public static final double kWheelRadius = 0.050165; // m

    public static final double TurnPositionConversionFactor = 2 * Math.PI / 10.0; // 200 : 20 gear ratio, measure in radians
    public static final double TurnVelocityConversionFactor = TurnPositionConversionFactor / 60; // Radians per second

    public static final double minrange = -(3 * Math.PI) / 4;
    public static final double maxrange = (3 * Math.PI) / 4;
    public static final double maxV = 1;
    public static final double maxA = 1;

    public static double allowedClosedLoopError = Math.PI / 6;

    public static double turnP = 1;
    public static double turnI = 0;
    public static double turnD = 0;
    public static double turnVelocityFFGain = maxV * 1;
  }

  public final class Intake {

    public static final int RightRotatoCanId = 1;
    public static final int LeftRotatoCanId = 6;
    public static final int MovMotorCanId = 2;

    public static final double UprightPos =
        Math.toRadians(-1); // Measured upright intake position at 1 degrees past 90
    public static final double ReadyPos = 1.594898;
    public static final double HalfPos = Math.toRadians(50);

    public static final double IntakeKg = -3.5;

    public static final double IntakeSpeed = 0.1;
    public static final double MovementMotorVelocity = 1;

    public static final double RotatoPosConFac = 2 * Math.PI;
    public static final double RotatoVelConFac = Math.PI / 30;

    public static final double ArmPosConFac = 2 * Math.PI / 9;
    public static final double ArmVelConFac = ArmPosConFac / 60;

    public static final double ArmkP = 1.5;
    public static final double ArmkI = 0.00;
    public static final double ArmkD = 0.1;

    public static final double RotatoThresholdRAD = Math.PI / 4.0;
    public static final double RotatoRPM = .35;

    public static final double testRotatoRPM = .2;

    public static final double IntakePositionConversionFactor = Math.PI * 2 / 15; // is in radians
    public static final double IntakeVelocityConversionFactor =
        Math.PI * 2 / 60; // radians per second

    public static final double alpha = Math.toRadians(15); // angle of center of gravitas

    public static final int ExtensionCurrentLimit = 1;
    // public static final int IntakeCurrentLimit = 1;
    // public static final int IntakePositionConversionFactor = 1;
    // public static final int IntakeVelocityConversionFactor = 1;

    public static final double MaxV =
        3 * Math.PI / 2; // 4 * Math.PI / 2 up and down in a quarter second
    public static final double MaxA = 7;
  }
}
