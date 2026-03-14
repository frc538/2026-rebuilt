package frc.robot.subsystems.navigation;

import java.util.ArrayList;
import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.navigationConstants;

public class NavigationSubsystem extends SubsystemBase {

  public Field2d m_field = new Field2d();
  public PathPlannerPath path;
  public List <Pose2d> positions = new ArrayList<>();
  public Pose2d selectedPose;
  public double targetAngle = 0;

  public NavigationSubsystem () {
    super();
    positions.add(navigationConstants.leftBlue);
    positions.add(navigationConstants.rightBlue);
    positions.add(navigationConstants.centerLeftBlue);
    positions.add(navigationConstants.centerRightBlue);
    positions.add(navigationConstants.centerRightRed);
    positions.add(navigationConstants.centerLeftRed);
    positions.add(navigationConstants.rightRed);
    positions.add(navigationConstants.leftRed);
  }

  // Create the constraints to use while pathfinding
  private final PathConstraints constraints =
      new PathConstraints(3.0, 4.0, Units.degreesToRadians(540), Units.degreesToRadians(720));

  public Command pathFindUp(Pose2d robotPose) {
    if (DriverStation.getAlliance().get() == Alliance.Blue) {
      Pose2d closestPose;
      closestPose = robotPose.nearest(positions);
      if (closestPose == navigationConstants.centerLeftBlue) {
        selectedPose = navigationConstants.centerRightRed;
      } else if (closestPose == navigationConstants.centerRightBlue) {
        selectedPose = navigationConstants.centerLeftRed;
      } else if (closestPose == navigationConstants.rightBlue) {
        selectedPose = navigationConstants.centerRightBlue;
      } else if (closestPose == navigationConstants.leftBlue) {
        selectedPose = navigationConstants.centerLeftBlue;
      } else if (closestPose == navigationConstants.centerRightRed) {
        selectedPose = navigationConstants.rightRed;
      } else if (closestPose == navigationConstants.centerLeftRed) {
        selectedPose = navigationConstants.leftRed;
      } else if (closestPose == navigationConstants.rightRed) {
        System.out.println("Max Forward");
      } else{
        //leftred
        System.out.println("Max Forward");
      }
    } else {
      Pose2d closestPose;
      closestPose = robotPose.nearest(positions);
      if (closestPose == navigationConstants.centerLeftBlue) {
        selectedPose = navigationConstants.leftBlue;
      } else if (closestPose == navigationConstants.centerRightBlue) {
        selectedPose = navigationConstants.rightBlue;
      } else if (closestPose == navigationConstants.rightBlue) {
        System.out.println("Max Forward");
      } else if (closestPose == navigationConstants.leftBlue) {
        System.out.println("Max Forward");
      } else if (closestPose == navigationConstants.centerRightRed) {
        selectedPose = navigationConstants.centerLeftBlue;
      } else if (closestPose == navigationConstants.centerLeftRed) {
        selectedPose = navigationConstants.centerLeftBlue;
      } else if (closestPose == navigationConstants.rightRed) {
        selectedPose = navigationConstants.centerRightRed;
      } else{
        //leftred
        selectedPose = navigationConstants.centerLeftRed;
      }
    }
        return generatePath(selectedPose);
  }

  public Command pathFindDown(Pose2d robotPose) {
    if (DriverStation.getAlliance().get() == Alliance.Blue) {
      Pose2d closestPose;
      closestPose = robotPose.nearest(positions);
      if (closestPose == navigationConstants.centerLeftBlue) {
        selectedPose = navigationConstants.leftBlue;
      } else if (closestPose == navigationConstants.centerRightBlue) {
        selectedPose = navigationConstants.rightBlue;
      } else if (closestPose == navigationConstants.rightBlue) {
        System.out.println("Max Backward");
      } else if (closestPose == navigationConstants.leftBlue) {
        System.out.println("Max Backward");
      } else if (closestPose == navigationConstants.centerRightRed) {
        selectedPose = navigationConstants.centerLeftBlue;
      } else if (closestPose == navigationConstants.centerLeftRed) {
        selectedPose = navigationConstants.centerRightBlue;
      } else if (closestPose == navigationConstants.rightRed) {
        selectedPose = navigationConstants.centerRightRed;
      } else{
        //leftred
        selectedPose = navigationConstants.centerRightRed;
      }
    } else {
      Pose2d closestPose;
      closestPose = robotPose.nearest(positions);
      if (closestPose == navigationConstants.centerLeftBlue) {
        selectedPose = navigationConstants.centerRightRed;
      } else if (closestPose == navigationConstants.centerRightBlue) {
        selectedPose = navigationConstants.centerLeftRed;
      } else if (closestPose == navigationConstants.rightBlue) {
        selectedPose = navigationConstants.centerRightBlue;
      } else if (closestPose == navigationConstants.leftBlue) {
        selectedPose = navigationConstants.centerLeftBlue;
      } else if (closestPose == navigationConstants.centerRightRed) {
        selectedPose = navigationConstants.rightRed;
      } else if (closestPose == navigationConstants.centerLeftRed) {
        selectedPose = navigationConstants.leftRed;
      } else if (closestPose == navigationConstants.rightRed) {
        System.out.println("Max Backward");
      } else{
        //leftred
        System.out.println("Max Backward");
      }
    }
        return generatePath(selectedPose);
  }
  
  public Command pathFindLeft(Pose2d robotPose) {
    if (DriverStation.getAlliance().get() == Alliance.Blue) {
      Pose2d closestPose;
      closestPose = robotPose.nearest(positions);
      if (closestPose == navigationConstants.centerLeftBlue) {
        System.out.println("Max Left");
      } else if (closestPose == navigationConstants.centerRightBlue) {
        selectedPose = navigationConstants.centerRightBlue;
      } else if (closestPose == navigationConstants.rightBlue) {
        selectedPose = navigationConstants.leftBlue;
      } else if (closestPose == navigationConstants.leftBlue) {
        System.out.println("Max Left");
      } else if (closestPose == navigationConstants.centerRightRed) {
        System.out.println("Max Left");
      } else if (closestPose == navigationConstants.centerLeftRed) {
        selectedPose = navigationConstants.centerRightRed;
      } else if (closestPose == navigationConstants.rightRed) {
        System.out.println("Max Left");
      } else{
        //leftred
        selectedPose = navigationConstants.rightRed;
      }
    } else {
      Pose2d closestPose;
      closestPose = robotPose.nearest(positions);
      if (closestPose == navigationConstants.centerLeftBlue) {
        selectedPose = navigationConstants.centerRightBlue;
      } else if (closestPose == navigationConstants.centerRightBlue) {
        System.out.println("Max Left");
      } else if (closestPose == navigationConstants.rightBlue) {
        System.out.println("Max Left");
      } else if (closestPose == navigationConstants.leftBlue) {
        selectedPose = navigationConstants.rightBlue;
      } else if (closestPose == navigationConstants.centerRightRed) {
        selectedPose = navigationConstants.centerLeftRed;
      } else if (closestPose == navigationConstants.centerLeftRed) {
        System.out.println("Max Left");
      } else if (closestPose == navigationConstants.rightRed) {
        selectedPose = navigationConstants.leftRed;
      } else{
        //leftred
        System.out.println("Max Left");
    }
  }
  return generatePath(selectedPose);
  }

  public Command pathFindRight(Pose2d robotPose) {
    if (DriverStation.getAlliance().get() == Alliance.Blue) {
      Pose2d closestPose;
      closestPose = robotPose.nearest(positions);
      if (closestPose == navigationConstants.centerLeftBlue) {
        selectedPose = navigationConstants.centerRightBlue;
      } else if (closestPose == navigationConstants.centerRightBlue) {
        System.out.println("Max Right");
      } else if (closestPose == navigationConstants.rightBlue) {
        System.out.println("Max Right");
      } else if (closestPose == navigationConstants.leftBlue) {
        selectedPose = navigationConstants.rightBlue;
      } else if (closestPose == navigationConstants.centerRightRed) {
        selectedPose = navigationConstants.centerLeftRed;
      } else if (closestPose == navigationConstants.centerLeftRed) {
        System.out.println("Max Right");
      } else if (closestPose == navigationConstants.rightRed) {
        selectedPose = navigationConstants.leftRed;
      } else{
        //leftred
        System.out.println("Max Right");
      }
    } else {
      Pose2d closestPose;
      closestPose = robotPose.nearest(positions);
      if (closestPose == navigationConstants.centerLeftBlue) {
        System.out.println("Max Right");
      } else if (closestPose == navigationConstants.centerRightBlue) {
        selectedPose = navigationConstants.centerLeftBlue;
      } else if (closestPose == navigationConstants.rightBlue) {
        selectedPose = navigationConstants.leftBlue;
      } else if (closestPose == navigationConstants.leftBlue) {
        System.out.println("Max Right");
      } else if (closestPose == navigationConstants.centerRightRed) {
        System.out.println("Max Right");
      } else if (closestPose == navigationConstants.centerLeftRed) {
        selectedPose = navigationConstants.centerRightRed;
      } else if (closestPose == navigationConstants.rightRed) {
        System.out.println("Max Right");
      } else{
        //leftred
        selectedPose = navigationConstants.rightRed;
      }
    }
        return generatePath(selectedPose);
  }

  public Command rightCenter(Pose2d robotPose) {
    if (DriverStation.getAlliance().get() == Alliance.Blue) {
      double x1 = robotPose.getX();
      double y1 = robotPose.getY();
      double x2 = navigationConstants.bottomCenter.getX();
      double y2 = navigationConstants.bottomCenter.getY();

      double deltaX = x2-x1;
      double deltaY = y2-y1;

      double targetGlobalAngle = Math.atan2(deltaY, deltaX);

      targetAngle = targetGlobalAngle-robotPose.getRotation().getRadians();
      if (targetAngle < -Math.PI) {
        targetAngle += 2 * Math.PI;
      }
      if (targetAngle > Math.PI) {
        targetAngle -= 2 * Math.PI;
      }
      
      //set robot angle to targetangle
      return generatePath(navigationConstants.bottomCenter);
    } else {
      double x1 = robotPose.getX();
      double y1 = robotPose.getY();
      double x2 = navigationConstants.topCenter.getX();
      double y2 = navigationConstants.topCenter.getY();

      double deltaX = x2-x1;
      double deltaY = y2-y1;

      double targetGlobalAngle = Math.atan2(deltaY, deltaX);

      targetAngle = targetGlobalAngle-robotPose.getRotation().getRadians();
      if (targetAngle < -Math.PI) {
        targetAngle += 2 * Math.PI;
      }
      if (targetAngle > Math.PI) {
        targetAngle -= 2 * Math.PI;
      }
      
      //set robot angle to targetangle
      return generatePath(navigationConstants.topCenter);
    }
  }

  
  
  public Command leftCenter(Pose2dGetter poseGetter) {
    if (DriverStation.getAlliance().get() == Alliance.Blue) {
      double x1 = robotPose.getX();
      double y1 = robotPose.getY();
      double x2 = navigationConstants.topCenter.getX();
      double y2 = navigationConstants.topCenter.getY();

      double deltaX = x2-x1;
      double deltaY = y2-y1;

      double targetGlobalAngle = Math.atan2(deltaY, deltaX);

      targetAngle = targetGlobalAngle-robotPose.getRotation().getRadians();
      if (targetAngle < -Math.PI) {
        targetAngle += 2 * Math.PI;
      }
      if (targetAngle > Math.PI) {
        targetAngle -= 2 * Math.PI;
      }
      
      //set robot angle to targetangle
      return generatePath(navigationConstants.topCenter);
    } else {
      double x1 = robotPose.getX();
      double y1 = robotPose.getY();
      double x2 = navigationConstants.bottomCenter.getX();
      double y2 = navigationConstants.bottomCenter.getY();

      double deltaX = x2-x1;
      double deltaY = y2-y1;

      double targetGlobalAngle = Math.atan2(deltaY, deltaX);

      //set robot angle to targetangle
      Pose2d endpoint = navigationConstants.bottomCenter;
      endpoint.rotateBy(Rotation2d.fromRadians(targetGlobalAngle));
      
      return generatePath(endpoint);
    }
  }
  
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
