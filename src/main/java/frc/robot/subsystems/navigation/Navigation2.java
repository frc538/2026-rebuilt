package frc.robot.subsystems.navigation;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.RotationTarget;
import com.pathplanner.lib.path.Waypoint;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.navigationConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.navigation.NavigationSubsystem.Pose2dSupplier;
import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.List;
import org.littletonrobotics.junction.Logger;

public class Navigation2 extends SubsystemBase {

  private boolean isPathing = false;

  private Command thePath = null;

  private Drive theDrive;

  class edge {
    final node m_start;
    final node m_end;

    // Constraints
    final Rotation2d
        m_orientation; // How should we point when crossing this edge - relative to blue alliance
    final double m_maxSpeed; // What is our max speed m/s

    public edge(node start, node end, Rotation2d orientation, double maxSpeed) {
      m_start = start;
      m_end = end;
      m_orientation = orientation;
      m_maxSpeed = maxSpeed;
    }

    public String print() {
      return String.format("{}->{}:", m_start.m_index, m_end.m_index);
    }
  }

  class node {
    // target point in box
    final Pose2d m_pose;

    int m_index;

    public edge up = null;
    public edge down = null;
    public edge right = null;
    public edge left = null;

    public node(Pose2d pose, int index) {
      m_pose = pose;
      m_index = index;
    }
  }

  // even on right, low is closest
  node[] nodes = {
    new node(navigationConstants.rightBlue, 0),
    new node(navigationConstants.leftBlue, 1),
    new node(navigationConstants.centerRightBlue, 2),
    new node(navigationConstants.centerLeftBlue, 3),
    new node(navigationConstants.centerLeftRed, 4),
    new node(navigationConstants.centerRightRed, 5),
    new node(navigationConstants.leftRed, 6),
    new node(navigationConstants.rightRed, 7)
  };

  ArrayDeque<edge> currentEdgeList = new ArrayDeque<>();

  // Create the constraints to use while pathfinding
  private final PathConstraints constraints =
      new PathConstraints(3.0, 4.0, Units.degreesToRadians(540), Units.degreesToRadians(720));

  private double upAngle = 0;
  private double downAngle = Math.PI;
  private double rightAngle = 3 * Math.PI / 2;
  private double leftAngle = Math.PI / 2;

  public Navigation2(Drive drive) {
    theDrive = drive;
    nodes[0].up = new edge(nodes[0], nodes[2], new Rotation2d(upAngle), 3);
    nodes[0].left = new edge(nodes[0], nodes[1], new Rotation2d(leftAngle), 3);

    nodes[1].up = new edge(nodes[1], nodes[3], new Rotation2d(upAngle), 3);
    nodes[1].right = new edge(nodes[1], nodes[0], new Rotation2d(rightAngle), 3);

    nodes[2].up = new edge(nodes[2], nodes[4], new Rotation2d(upAngle), 3);
    nodes[2].left = new edge(nodes[2], nodes[3], new Rotation2d(leftAngle), 3);
    nodes[2].down = new edge(nodes[2], nodes[0], new Rotation2d(downAngle), 3);

    nodes[3].up = new edge(nodes[3], nodes[5], new Rotation2d(upAngle), 3);
    nodes[3].right = new edge(nodes[3], nodes[2], new Rotation2d(rightAngle), 3);
    nodes[3].down = new edge(nodes[3], nodes[1], new Rotation2d(downAngle), 3);

    nodes[4].up = new edge(nodes[4], nodes[6], new Rotation2d(upAngle), 3);
    nodes[4].left = new edge(nodes[4], nodes[5], new Rotation2d(leftAngle), 3);
    nodes[4].down = new edge(nodes[4], nodes[2], new Rotation2d(downAngle), 3);

    nodes[5].up = new edge(nodes[5], nodes[7], new Rotation2d(upAngle), 3);
    nodes[5].right = new edge(nodes[5], nodes[4], new Rotation2d(rightAngle), 3);
    nodes[5].down = new edge(nodes[5], nodes[3], new Rotation2d(downAngle), 3);

    nodes[6].left = new edge(nodes[6], nodes[7], new Rotation2d(leftAngle), 3);
    nodes[6].down = new edge(nodes[6], nodes[4], new Rotation2d(downAngle), 3);

    nodes[7].right = new edge(nodes[7], nodes[6], new Rotation2d(rightAngle), 3);
    nodes[7].down = new edge(nodes[7], nodes[5], new Rotation2d(downAngle), 3);
  }

  node findStartingNode(Pose2d robotPose) {
    int index = -1;
    if (DriverStation.getAlliance().get() == Alliance.Red) {
      Translation2d centerPoint = new Translation2d(8.27, 4);

      robotPose = robotPose.rotateAround(centerPoint, new Rotation2d(Math.PI));
    }
    if (robotPose.getY() < 4) {
      if (robotPose.getX() < 4.63) {
        index = 0;
      } else if (robotPose.getX() < 8.27) {
        index = 2;
      } else if (robotPose.getX() < 11.915) {
        index = 4;
      } else {
        index = 6;
      }
    } else {
      if (robotPose.getX() < 4.63) {
        index = 1;
      } else if (robotPose.getX() < 8.27) {
        index = 3;
      } else if (robotPose.getX() < 11.915) {
        index = 5;
      } else {
        index = 7;
      }
    }
    Logger.recordOutput("Guidance/StartingNode", index);
    return nodes[index];
  }

  void addEdge(edge newEdge) {
    if (newEdge == null) {
      return;
    }
    currentEdgeList.addLast(newEdge);
    findEdgePath();
  }

  void findEdgePath() {
    List<Waypoint> waypoints;

    // if (currentEdgeList.size() < 2) {
    //   thePath = AutoBuilder.pathfindToPose(currentEdgeList.getLast().m_end.m_pose, constraints);
    // } else {
    ArrayList<Pose2d> poses = new ArrayList<>();
    edge[] edges = currentEdgeList.toArray(new edge[0]);
    poses.add(new Pose2d(theDrive.getPose().getTranslation(), edges[0].m_orientation));
    for (int i = 0; i < edges.length; i++) {
      Pose2d pose;
      if (i < edges.length - 1) {
        pose = new Pose2d(edges[i].m_end.m_pose.getTranslation(), edges[i + 1].m_orientation);
      } else {
        pose = edges[i].m_end.m_pose;
      }
      poses.add(pose);
    }
    waypoints = PathPlannerPath.waypointsFromPoses(poses);
    GoalEndState endState = new GoalEndState(0, currentEdgeList.getLast().m_orientation);
    PathPlannerPath path = new PathPlannerPath(waypoints, constraints, null, endState);
    thePath = AutoBuilder.followPath(path);
    // }

    CommandScheduler.getInstance().schedule(thePath);
  }

  public Command navUp(Pose2dSupplier robotPose) {
    return Commands.runOnce(
        () -> {
          node selNode;
          if (thePath == null || thePath.isFinished() || currentEdgeList.size() == 0) {
            selNode = findStartingNode(robotPose.getPose2d());
          } else {
            selNode = currentEdgeList.getLast().m_end;
          }

          addEdge(selNode.up);
        },
        this);
  }

  public Command navDown(Pose2dSupplier robotPose) {
    return Commands.runOnce(
        () -> {
          node selNode;
          if (thePath == null || thePath.isFinished() || currentEdgeList.size() == 0) {
            selNode = findStartingNode(robotPose.getPose2d());
          } else {
            selNode = currentEdgeList.getLast().m_end;
          }

          addEdge(selNode.down);
        },
        this);
  }

  public Command navRight(Pose2dSupplier robotPose) {
    return Commands.runOnce(
        () -> {
          node selNode;
          if (thePath == null || thePath.isFinished() || currentEdgeList.size() == 0) {
            selNode = findStartingNode(robotPose.getPose2d());
          } else {
            selNode = currentEdgeList.getLast().m_end;
          }

          addEdge(selNode.right);
        },
        this);
  }

  public Command navLeft(Pose2dSupplier robotPose) {
    return Commands.runOnce(
        () -> {
          node selNode;
          if (thePath == null || thePath.isFinished() || currentEdgeList.size() == 0) {
            selNode = findStartingNode(robotPose.getPose2d());
          } else {
            selNode = currentEdgeList.getLast().m_end;
          }

          addEdge(selNode.left);
        },
        this);
  }

  public Command cancelPath() {
    return Commands.runOnce(
        () -> {
          if (thePath != null) {
            thePath.cancel();
            currentEdgeList.clear();
            thePath = null;
          }
        },
        this);
  }

  @Override
  public void periodic() {
    if (thePath != null) {
      if (thePath.isFinished()) {
        currentEdgeList.clear();
      } else {
        if (currentEdgeList.size() > 0) {
          if (theDrive
                  .getPose()
                  .getTranslation()
                  .getDistance(currentEdgeList.getFirst().m_end.m_pose.getTranslation())
              < navigationConstants.distanceThreshold) {
            currentEdgeList.removeFirst();
          }
        }
      }
    }

    String path = "";
    for (edge e : currentEdgeList) {
      path.concat(e.print());
    }
    Logger.recordOutput("Guidance/currentEdgeList", path);
  }

  public Command rightCenter(Pose2dSupplier poseSupplier) {
    return Commands.runOnce(
        () -> {
          Pose2d thePose = poseSupplier.getPose2d();
          if (DriverStation.getAlliance().get() == Alliance.Blue) {
            double x1 = thePose.getX();
            double y1 = thePose.getY();
            double x2 = navigationConstants.bottomCenter.getX();
            double y2 = navigationConstants.bottomCenter.getY();

            double deltaX = x2 - x1;
            double deltaY = y2 - y1;

            double targetGlobalAngle = Math.atan2(deltaY, deltaX);

            // set robot angle to targetangle
            Pose2d endpoint = navigationConstants.bottomCenter;
            endpoint.rotateBy(Rotation2d.fromRadians(targetGlobalAngle));

            // set robot angle to targetangle
            CommandScheduler.getInstance().schedule(generatePath(endpoint));
          } else {
            double x1 = thePose.getX();
            double y1 = thePose.getY();
            double x2 = navigationConstants.topCenter.getX();
            double y2 = navigationConstants.topCenter.getY();

            double deltaX = x2 - x1;
            double deltaY = y2 - y1;

            double targetGlobalAngle = Math.atan2(deltaY, deltaX);

            // set robot angle to targetangle
            Pose2d endpoint = navigationConstants.topCenter;
            endpoint.rotateBy(Rotation2d.fromRadians(targetGlobalAngle));

            CommandScheduler.getInstance().schedule(generatePath(endpoint));
          }
        });
  }

  public Command leftCenter(Pose2dSupplier poseSupplier) {
    return Commands.runOnce(
        () -> {
          Pose2d thePose = poseSupplier.getPose2d();
          if (DriverStation.getAlliance().get() == Alliance.Blue) {
            double x1 = thePose.getX();
            double y1 = thePose.getY();
            double x2 = navigationConstants.topCenter.getX();
            double y2 = navigationConstants.topCenter.getY();

            double deltaX = x2 - x1;
            double deltaY = y2 - y1;

            double targetGlobalAngle = Math.atan2(deltaY, deltaX);

            // set robot angle to targetangle
            Pose2d endpoint = navigationConstants.topCenter;
            endpoint.rotateBy(Rotation2d.fromRadians(targetGlobalAngle));

            // set robot angle to targetangle
            CommandScheduler.getInstance().schedule(generatePath(endpoint));
          } else {
            double x1 = thePose.getX();
            double y1 = thePose.getY();
            double x2 = navigationConstants.bottomCenter.getX();
            double y2 = navigationConstants.bottomCenter.getY();

            double deltaX = x2 - x1;
            double deltaY = y2 - y1;

            double targetGlobalAngle = Math.atan2(deltaY, deltaX);

            // set robot angle to targetangle
            Pose2d endpoint = navigationConstants.bottomCenter;
            endpoint.rotateBy(Rotation2d.fromRadians(targetGlobalAngle));

            CommandScheduler.getInstance().schedule(generatePath(endpoint));
          }
        });
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
}
