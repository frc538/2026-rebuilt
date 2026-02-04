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
    public static final double hubHeight = 1.8288; // meters
    public static final double launcherHeight = 0.508; // meters estimate
    public static final double launcherAngle = 60; // degrees
  }
}
