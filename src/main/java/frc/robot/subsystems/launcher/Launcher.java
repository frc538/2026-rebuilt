package frc.robot.subsystems.launcher;

import static frc.robot.Constants.launcherConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.launcherConstants;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class Launcher extends SubsystemBase {
  private Pose2d robotPose = new Pose2d();
  private ChassisSpeeds robotVelocity = new ChassisSpeeds();
  private Pose2d aimPoint = new Pose2d();
  private double distanceX;
  private double distanceY;
  private double endDistance;
  private double timeFlight;
  private double targetAzimuth = Math.PI;
  private double launchSpeed;
  private boolean shootSide = false;
  private boolean disableShoot = true;
  private double testRadPerS = 0.0;
  private Pose2d aimPointComp = new Pose2d(0, 0, new Rotation2d());
  private double finalWheelRotationVelocity;
  private double initialWheelRotVelocity;
  private Constraints profileConstraints;
  private TrapezoidProfile turnProfile;
  private TrapezoidProfile.State mCurrentState;
  private TrapezoidProfile.State mDesiredState;
  private boolean aimGood;
  private boolean TurretSpeedGood;
  private boolean autoRotate = false;
  private boolean stopTurret = false;
  private boolean autoTurnRobot = false;

  TurretEncoder turretEncoder = new TurretEncoder();

  private double currentAimTrim = 0;
  private double currentSpeedTrim = 0;

  LauncherIO io;
  LauncherIOInputsAutoLogged inputs = new LauncherIOInputsAutoLogged();
  LauncherConsumer thingy;

  public Launcher(LauncherIO IO, LauncherConsumer consumer) {
    io = IO;

    profileConstraints = new Constraints(maxV, maxA);
    turnProfile = new TrapezoidProfile(profileConstraints);

    thingy = consumer;

    mCurrentState = new TrapezoidProfile.State(Math.PI, 0);
    mDesiredState = new TrapezoidProfile.State(Math.PI, 0);

    io.TurretDisable();
  }

  public Command toggleShoot() {
    return Commands.runOnce(() -> disableShoot = !disableShoot);
  }

  public Command testFullSpeed() {
    return Commands.run(
            () -> {
              io.setVoltage(12.0);
            })
        .finallyDo(() -> io.setVoltage(0));
  }

  public Command testLowSpeed() {
    return Commands.run(
            () -> {
              io.setVoltage(5.0);
            })
        .finallyDo(() -> io.setVoltage(0));
  }

  public Command testOff() {
    return Commands.run(
        () -> {
          io.setVoltage(0.0);
        });
  }

  public Command testTurn() {
    return Commands.run(
            () -> {
              io.testTurn(2.0);
            })
        .finallyDo(() -> io.testTurn(0));
  }

  public Command invertTestTurn() {
    return Commands.run(
            () -> {
              io.testTurn(-2.0);
            })
        .finallyDo(() -> io.testTurn(0));
  }

  public Command simFeed() {
    return Commands.runOnce(
        () -> {
          io.simLaunch();
        });
  }

  public Command testRPS(DoubleSupplier RPMcontrol) {
    return Commands.runOnce(
            () -> {
              testRadPerS = 0.0;
            })
        .andThen(
            Commands.run(
                    () -> {
                      double input = MathUtil.applyDeadband(RPMcontrol.getAsDouble(), 0.1);
                      testRadPerS = testRadPerS + (input * ((2 * Math.PI) * 10) / 50);
                      io.setRadPerS(testRadPerS);
                    })
                .finallyDo(
                    () -> {
                      io.setRadPerS(0);
                      io.setVoltage(0);
                    }));
  }

  public Command testTurretRotateClockwise() {
    return Commands.run(
            () -> {
              if (autoRotate == false) {
                io.turretVoltage(-1.0);
              }
            })
        .finallyDo(() -> io.turretVoltage(0));
  }

  public Command testTurretRotateCounterclockwise() {
    return Commands.run(
            () -> {
              if (autoRotate == false) {
                io.turretVoltage(1.0);
              }
            })
        .finallyDo(() -> io.turretVoltage(0));
  }

  public Command testTurretRotateToggleAuto() {
    return Commands.runOnce(
        () -> {
          autoRotate = !autoRotate;
        });
  }

  public Command testTurretPosition(DoubleSupplier target) {
    return Commands.runOnce(
        () -> {
          targetAzimuth = target.getAsDouble();
        });
  }

  public void updateOdometry(Pose2d robotPose, ChassisSpeeds robotVelocity) {
    this.robotPose = robotPose;
    this.robotVelocity = robotVelocity;
    io.updateRobotInfo(robotPose, robotVelocity);
  }

  public void getAzimuth() {
    double x1 = robotPose.getX();
    double y1 = robotPose.getY();
    double x2 = aimPoint.getX();
    double y2 = aimPoint.getY();

    double deltaY = y2 - y1;
    double deltaX = x2 - x1;

    double targetGlobalAzimuth = Math.atan2(deltaY, deltaX);
    Logger.recordOutput("Launcher/targetGlobalAzimuth", targetGlobalAzimuth);
    // i.e. Pose2D defines the rotation as a mathematical one... 0 degrees toward
    // positive x,
    // increases counter-clockwise
    if (!DriverStation.isTest()) {
      targetAzimuth = targetGlobalAzimuth - robotPose.getRotation().getRadians() + currentAimTrim;
      targetAzimuth = 2 * Math.PI - targetAzimuth;
      if (targetAzimuth < 0) {
        targetAzimuth += 2 * Math.PI;
      }
      if (targetAzimuth > 2 * Math.PI) {
        targetAzimuth -= 2 * Math.PI;
      }
    }
  }

  public void aimDownSights() {
    if (DriverStation.getAlliance().isPresent()) {
      if (DriverStation.getAlliance().get() == Alliance.Blue) {
        if (robotPose.getX() >= hubBlue.getX()) {
          if (robotPose.getY() >= 4.030) {
            aimPoint = leftBlue;
          } else {
            aimPoint = rightBlue;
          }
        } else {
          aimPoint = hubBlue;
        }
      } else {
        if (robotPose.getX() <= hubRed.getX()) {
          if (robotPose.getY() >= 4.030) {
            aimPoint = rightRed;
          } else {
            aimPoint = leftRed;
          }
        } else {
          aimPoint = hubRed;
        }
      }
    }
  }

  // Need to call this function at some point, and then command the IO layer
  public void getShootSpeed() {

    finalWheelRotationVelocity = (2 * launchSpeed) / launchWheelRadius;

    initialWheelRotVelocity =
        (launchSpeed
            * (kFuelMomentOfInertia * Math.pow(launchWheelRadius, 2)
                + 4 * kFlywheelMomentOfInertia * (Math.pow(fuelRadius, 2))
                + fuelMass * (Math.pow(fuelRadius, 2)) * (Math.pow(kWheelRadius, 2)))
            / (2 * kFlywheelMomentOfInertia * (Math.pow(fuelRadius, 2)) * kWheelRadius));
  }

  private void aimComp() {
    double aimPointX = aimPoint.getX() - robotVelocity.vxMetersPerSecond * timeFlight;
    double aimPointY = aimPoint.getY() - robotVelocity.vyMetersPerSecond * timeFlight;

    aimPointComp = new Pose2d(aimPointX, aimPointY, new Rotation2d());

    distanceX = aimPointComp.getX() - robotPose.getX();
    distanceX = Math.pow(distanceX, 2);
    distanceY = aimPointComp.getY() - robotPose.getY();
    distanceY = Math.pow(distanceY, 2);

    double calculatedDistance = Math.sqrt(distanceX + distanceY);

    double distanceModified = Math.max(1, (0.45 * calculatedDistance) + 5.5);

    Logger.recordOutput("Launcher/calculatedDistance", calculatedDistance);
    Logger.recordOutput("Launcher/distanceModified", distanceModified);
    // Add Trim + currentSpeedTrim;
    endDistance = distanceModified + currentSpeedTrim;

    timeFlight =
        Math.sqrt((hubHeight - launcherHeight - endDistance * Math.tan(launcherAngle) / -9.81));

    launchSpeed = endDistance / (Math.cos(launcherAngle) * timeFlight);
  }

  private void shoot() {
    if (!DriverStation.isTest() || autoRotate) {
      if (disableShoot == false) {
        io.setRadPerS(initialWheelRotVelocity);
      } else {
        io.setRadPerS(0);
      }
    }
  }

  private void DetermineFirePermit() { // target rpm
    if (
    /*abs = absolute value*/ Math.abs(mDesiredState.position - inputs.turnEncoderPosition)
        <= Math.PI * 2 / 72) {
      // make varie with range, not my job though
      aimGood = true;
    } else {
      aimGood = false;
    }
    if (Math.abs(inputs.launcherVelocity / initialWheelRotVelocity - 1) <= 0.05) {
      TurretSpeedGood = true;
    } else {
      TurretSpeedGood = false;
    }
    thingy.accept(aimGood, TurretSpeedGood);
  }

  private void setAz() {
    if (stopTurret == false) {
      mDesiredState.position =
          MathUtil.clamp(
              targetAzimuth, launcherConstants.minWireLimit, launcherConstants.maxWireLimit);

      mCurrentState = turnProfile.calculate(0.02, mCurrentState, mDesiredState);

      if (!DriverStation.isTest() || autoRotate) {
        io.pointAt(mCurrentState.position, mCurrentState.velocity);
      }
    } else {
      mDesiredState.position = Math.PI;
    }
  }

  private void handleDisabled() {
    if (DriverStation.isDisabled() == true) {
      if (Constants.Features.isPotentiometerBroken == false) {
        mDesiredState.position = inputs.turnPotentiometer;
        targetAzimuth = inputs.turnPotentiometer;
        mCurrentState.position = inputs.turnPotentiometer;
        io.TurretCalibrate(inputs.turnPotentiometer);
      } else if (Constants.Features.useChineseRemainderTheorem == true) {
        double position = turretEncoder.read();
        mDesiredState.position = position;
        targetAzimuth = position;
        mCurrentState.position = position;
        io.TurretCalibrate(position);
      }
      io.TurretDisable();
    }
  }

  public Command calibrateCRTEncoders() {
    return Commands.runOnce(() -> {
      turretEncoder.read();
    });
  }

  @Override
  public void periodic() {

    io.updateInputs(inputs);
    Logger.processInputs("Launcher", inputs);

    handleDisabled();

    if (DriverStation.isEnabled()) {
      aimDownSights(); // sets the target
      getAzimuth(); // sets the target rotation
      aimComp(); // compensates for robot velocity
      setAz(); // point turret
      getShootSpeed(); // flywheel speed
      shoot();
      DetermineFirePermit();
    }

    Logger.recordOutput("Launcher/TurretSpeedGood", TurretSpeedGood);
    Logger.recordOutput("Launcher/aimGood", aimGood);

    Logger.recordOutput("Launcher/autoRotate", autoRotate);
    Logger.recordOutput("Launcher/aimPoint", aimPoint);
    Logger.recordOutput("Launcher/aimPointComp", aimPointComp);
    Logger.recordOutput("Launcher/targetAzimuth", targetAzimuth);
    Logger.recordOutput("Launcher/endDistance", endDistance);
    Logger.recordOutput("Launcher/timeFlight", timeFlight);
    Logger.recordOutput("Launcher/launchSpeed", launchSpeed);
    Logger.recordOutput("Launcher/initialWheelRotVelocity", initialWheelRotVelocity);
    Logger.recordOutput("Launcher/finalWheelRotationVelocity", finalWheelRotationVelocity);
    Logger.recordOutput("Launcher/mDesiredState", mDesiredState);
    Logger.recordOutput("Launcher/mCurrentState", mCurrentState);
    Logger.recordOutput("Launcher/testRPSCmd", testRadPerS);
    Logger.recordOutput("Launcher/currentSpeedTrim", currentSpeedTrim);
    Logger.recordOutput("Launcher/currentAimTrim", currentAimTrim);
  }

  @FunctionalInterface
  public interface LauncherConsumer {
    public void accept(boolean AimCorrect, boolean SpeedGood);
  }

  public void TrimSpeed(double speedtrim) {
    currentSpeedTrim =
        currentSpeedTrim
            + Constants.launcherConstants.SpeedStep
                * speedtrim; // the double is just so it works for both back and forward
  }

  public void TrimAim(double aimtrim) {
    currentAimTrim = currentAimTrim - Math.toRadians(aimtrim);
  }

  public void TrimReset() {
    currentAimTrim = 0;
    currentSpeedTrim = 0;
  }

  public Command ResetTrim() {
    return (runOnce(
        () -> {
          TrimReset();
        }));
  } // resets all trims to 0

  public Command trimLeft() {
    // trim aim in degrees, i do convert to radians later assume left is negative?
    return (run(
        () -> {
          TrimAim(-1 * Constants.launcherConstants.AimTrim);
        }));
  }

  public Command trimRight() {
    return (run(
        () -> {
          TrimAim(Constants.launcherConstants.AimTrim);
        }));
  }

  public Command trimBack() {
    return (run(
        () -> {
          TrimSpeed(-1);
        }));
  } // steps of amount of meters?

  public Command trimForward() {
    return (run(
        () -> {
          TrimSpeed(1);
        }));
  }
}
