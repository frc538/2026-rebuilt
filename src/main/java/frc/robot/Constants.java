// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

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

  public final class Intake {

    public static final int RightRotatoCanId = 1;
    public static final int LeftRotatoCanId = 2;
    public static final int MovMotorCanId = 5;

    public static final double UprightPos = Math.PI / 2;
    public static final double ReadyPos = 0;

    public static final double IntakeSpeed = 1;
    public static final double MovementMotorVelocity = 1;

    public static final double RotatoPosConFac = 2 * Math.PI;
    public static final double RotatoVelConFac = Math.PI / 30;

    public static final double ArmPosConFac = 2 * Math.PI;
    public static final double ArmVelConFac = Math.PI / 30;

    public static final double IntakekP = 0.1;
    public static final double IntakekI = 0.01;
    public static final double IntakekD = 0.0;

    public static final double IntakePositionConversionFactor = Math.PI * 2 / 15; // is in radians
    public static final double IntakeVelocityConversionFactor =
        Math.PI * 2 / 60; // radians per second

    public static final int ExtensionCurrentLimit = 1;
    // public static final int IntakeCurrentLimit = 1;
    // public static final int IntakePositionConversionFactor = 1;
    // public static final int IntakeVelocityConversionFactor = 1;
  }
}
