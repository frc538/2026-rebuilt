package frc.robot.subsystems.launcher;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

public class Launcher extends SubsystemBase {
  private Pose2d robotPose = new Pose2d();
  private ChassisSpeeds robotVelocity = new ChassisSpeeds();
  private Pose2d aimPoint;
  private double distanceX;
  private double distanceY;
  private double endDistance;
  private double timeFlight;
  private double targetAzimuth;
  private double launchSpeed;

  LauncherIO io;
  LauncherIOInputsAutoLogged inputs = new LauncherIOInputsAutoLogged();

  public Launcher(LauncherIO IO) {
    io = IO;
  }

  public Command fullSpeed() {
    return Commands.runOnce(
        () -> {
          io.setVoltage(12.0);
          Logger.recordOutput("Launcher/voltageCmd", 12.0);
        });
  }

  public Command lowSpeed() {
    return Commands.runOnce(
        () -> {
          io.setVoltage(3.0);
          Logger.recordOutput("Launcher/voltageCmd", 3.0);
        });
  }

  public Command off() {
    return Commands.runOnce(
        () -> {
          io.setVoltage(0.0);
          Logger.recordOutput("Launcher/voltageCmd", 0.0);
        });
  }

  public Command feed() {
    return Commands.runOnce(
        () -> {
          io.simLaunch();
        });
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

    targetAzimuth =
        Math.toDegrees(Math.atan2(deltaY, deltaX) + robotPose.getRotation().getDegrees());
  }

  public void aimDownSights() {
    if (DriverStation.getAlliance().get() == Alliance.Blue) {
      aimPoint = Constants.launcherConstants.hubBlue;
    } else {
      aimPoint = Constants.launcherConstants.hubRed;
    }
    double aimPointX = aimPoint.getX() - robotVelocity.vxMetersPerSecond;
    double aimPointY = aimPoint.getY() - robotVelocity.vyMetersPerSecond;

    aimPoint = new Pose2d(aimPointX, aimPointY, null);
  }

  @Override
  public void periodic() {

    aimDownSights();
    getAzimuth();

    io.updateInputs(inputs);
    Logger.processInputs("Launcher", inputs);

    if (DriverStation.getAlliance().get() == Alliance.Blue) {
      aimPoint = Constants.launcherConstants.hubBlue;
    } else {
      aimPoint = Constants.launcherConstants.hubRed;
    }

    distanceX = aimPoint.getX() - robotPose.getX();
    distanceX = Math.pow(distanceX, 2);
    distanceY = aimPoint.getY() - robotPose.getY();
    distanceY = Math.pow(distanceY, 2);

    endDistance = Math.sqrt(distanceX + distanceY);

    timeFlight =
        Math.sqrt(
            (Constants.launcherConstants.hubHeight
                - Constants.launcherConstants.launcherHeight
                - endDistance * Math.tan(Constants.launcherConstants.launcherAngle) / -9.81));

    launchSpeed = endDistance/Math.cos(Constants.launcherConstants.launcherAngle*timeFlight);
    
    Logger.recordOutput("aimpoint", aimPoint);
    Logger.recordOutput("azimuth", targetAzimuth);
    Logger.recordOutput("distance", endDistance);
    Logger.recordOutput("time flight", timeFlight);
    Logger.recordOutput("launch speed", launchSpeed);
  }
}