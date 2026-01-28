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

  public static class navigationConstants {
    // NAV MAP
    // Blue
    public static final Pose2d centerPointBlue = new Pose2d(2.5, 4.0, Rotation2d.fromDegrees(0));
    public static final Pose2d topCenterPointBlue =
        new Pose2d(2.5, 6.5, Rotation2d.fromDegrees(90));
    public static final Pose2d bottomCenterPointBlue =
        new Pose2d(2.5, 1.5, Rotation2d.fromDegrees(270));
    // Center
    public static final Pose2d centerPoint = new Pose2d(8.25, 4.0, Rotation2d.fromDegrees(45));
    public static final Pose2d topCenterPoint = new Pose2d(8.25, 6.5, Rotation2d.fromDegrees(90));
    public static final Pose2d bottomCenterPoint =
        new Pose2d(8.25, 1.5, Rotation2d.fromDegrees(270));
    // Red
    public static final Pose2d centerPointRed = new Pose2d(14, 4.0, Rotation2d.fromDegrees(45));
    public static final Pose2d topCenterPointRed = new Pose2d(14, 6.5, Rotation2d.fromDegrees(90));
    public static final Pose2d bottomCenterPointRed =
        new Pose2d(14, 1.5, Rotation2d.fromDegrees(270));
  }
}
