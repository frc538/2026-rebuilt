// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import org.opencv.core.Point;

import edu.wpi.first.math.geometry.CoordinateSystem;
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
    //NAV MAP
    // NAMING CONVENTION IS color first, under/over if related to hump or no, side towards center or driverstation or center, Y coordinate side relati+654\]
    //center points
    public static final Point centerPointBlue = new Point(2.5, 4);
    public static final Point centerPointRed = new Point(14, 4);

    //Red Side Navigation
    public static final Point redUnderDriverstation = new Point(13.1, 7.4);
    public static final Point redUnderCenter = new Point(10.8, 7.4);

  }
}
