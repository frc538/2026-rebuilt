package frc.robot.subsystems.launcher;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

public class Launcher extends SubsystemBase {

  LauncherIO io;
  LauncherIOInputsAutoLogged inputs = new LauncherIOInputsAutoLogged();

  private Pose2d robotPose = new Pose2d();
  private Pose2d aimPoint = new Pose2d();
  private double distanceX;
  private double distanceY;
  private double endDistance;

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

  public void updateOdometry(Pose2d robotPose) {
    this.robotPose = robotPose;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Launcher", inputs);

    if (DriverStation.getAlliance().get() == Alliance.Blue) {
        aimPoint = Constants.launcherConstants.hubBlue;
    } else {
        aimPoint = Constants.launcherConstants.hubRed;
    }

    distanceX = aimPoint.getX()-robotPose.getX();
    distanceX = Math.pow(distanceX, 2);
    distanceY = aimPoint.getY()-robotPose.getY();
    distanceY = Math.pow(distanceY, 2);
    
    endDistance = Math.sqrt(distanceX+distanceY);
  }

}
