package frc.robot.subsystems.launcher;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Launcher extends SubsystemBase {
    private Pose2d robotPose = new Pose2d();
    private Pose2d aimPoint = new Pose2d();
    private double distanceX;
    private double distanceY;
    private double endDistance;
    public Launcher() {

    }

    public void updateOdometry(Pose2d robotPose) {
        this.robotPose = robotPose;
    }

    @Override
    public void periodic() {
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
