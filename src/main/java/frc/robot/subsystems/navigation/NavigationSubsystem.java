package frc.robot.subsystems.navigation;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class NavigationSubsystem extends SubsystemBase {

  public Field2d m_field = new Field2d();
  public PathPlannerPath path;

  // Create the constraints to use while pathfinding
  private final PathConstraints constraints =
      new PathConstraints(3.0, 4.0, Units.degreesToRadians(540), Units.degreesToRadians(720));

  public Command generatePath(Pose2d endPoint) {

    // represents the goal holonomic rotation
    Pose2d targetPose = endPoint;

    // Since AutoBuilder is configured, we can use it to build pathfinding commands

    return AutoBuilder.pathfindToPose(
        targetPose, constraints, 0.0 // Goal end velocity in meters/sec
        // Rotation delay distance in meters. This is how far the robot should travel
        // before attempting to rotate.
        );
  }
  /*
  public Command showPath() {
    return Commands.runOnce(
        () -> {
          private GoalEndState endState = new GoalEndState(0, new Rotation2d(90));
          List<Pose2d> pathPlannerPath =
              Pathfinding

              .getCurrentPath(constraints, endState).getPathPoses();
          m_field.getObject("Path").setPoses(pathPlannerPath);
        });
    // return m_field.setRobotPose(AutoBuilder.getCurrentPose());
  }
    */
}
