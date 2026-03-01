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
  private double targetAzimuth;
  private double launchSpeed;
  private boolean shootSide = false;
  private double testRadPerS = 0.0;
  private Pose2d aimPointComp = new Pose2d(0, 0, new Rotation2d());
  private double finalWheelRotationVelocity;
  private double initialWheelRotVelocity;
  private Constraints profileConstraints;
  private TrapezoidProfile turnProfile;
  private TrapezoidProfile.State mCurrentState;
  private TrapezoidProfile.State mDesiredState;

  LauncherIO io;
  LauncherIOInputsAutoLogged inputs = new LauncherIOInputsAutoLogged();

  public Launcher(LauncherIO IO) {
    io = IO;

    profileConstraints = new Constraints(maxV, maxA);
    turnProfile = new TrapezoidProfile(profileConstraints);

    mCurrentState = new TrapezoidProfile.State();
    mDesiredState = new TrapezoidProfile.State();
  }

  public Command testFullSpeed() {
    return Commands.run(
            () -> {
              io.setVoltage(12.0);
              Logger.recordOutput("Launcher/flywheelVoltageCmd", 12.0);
            })
        .finallyDo(() -> io.setVoltage(0));
  }

  public Command testLowSpeed() {
    return Commands.run(
            () -> {
              io.setVoltage(5.0);
              Logger.recordOutput("Launcher/flywheelVoltageCmd", 5.0);
            })
        .finallyDo(() -> io.setVoltage(0));
  }

  public Command testOff() {
    return Commands.run(
        () -> {
          io.setVoltage(0.0);
          Logger.recordOutput("Launcher/flywheelVoltageCmd", 0.0);
        });
  }

  public Command testTurn() {
    return Commands.run(
            () -> {
              io.testTurn(3.0);
              Logger.recordOutput("Launcher/testTurn", 3.0);
            })
        .finallyDo(() -> io.testTurn(0));
  }

  public Command invertTestTurn() {
    return Commands.run(
            () -> {
              io.testTurn(-3.0);
              Logger.recordOutput("Launcher/testTurn", -3.0);
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
                      io.setVoltage(0);
                    }));
  }

  public void updateOdometry(Pose2d robotPose, ChassisSpeeds robotVelocity) {
    this.robotPose = robotPose;
    this.robotVelocity = robotVelocity;
  }

  public void getAzimuth() {
    double x1 = robotPose.getX();
    double y1 = robotPose.getY();
    double x2 = aimPoint.getX();
    double y2 = aimPoint.getY();

    double deltaY = y2 - y1;
    double deltaX = x2 - x1;

    // i.e. Pose2D defines the rotation as a mathematical one... 0 degrees toward
    // positive x,
    // increases counter-clockwise
    targetAzimuth =
        Math.toDegrees(Math.atan2(deltaY, deltaX))
            + ((90 - robotPose.getRotation().getDegrees()) + 360);
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

    endDistance = Math.sqrt(distanceX + distanceY);

    timeFlight =
        Math.sqrt((hubHeight - launcherHeight - endDistance * Math.tan(launcherAngle) / -9.81));

    launchSpeed = endDistance / (Math.cos(launcherAngle) * timeFlight);
  }

  private void shoot() {
    if (!DriverStation.isTest()) {
      io.setRadPerS(initialWheelRotVelocity);
    }
  }

  private void setAz() {
    mDesiredState.position = targetAzimuth;

    mCurrentState = turnProfile.calculate(0.02, mCurrentState, mDesiredState);

    if (!DriverStation.isTest()) {
      io.pointAt(mCurrentState.position);
    }
  }

  @Override
  public void periodic() {

    io.updateInputs(inputs);
    Logger.processInputs("Launcher", inputs);

    aimDownSights(); // sets the target
    getAzimuth(); // sets the target rotation
    aimComp(); // compensates for robot velocity
    setAz(); // point turret
    getShootSpeed(); // flywheel speed
    shoot();

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
  }
}
