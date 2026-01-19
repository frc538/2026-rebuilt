package frc.robot.subsystems.navigation;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class nav {
    public class navigationSubsystem extends SubsystemBase {     
        public Command generatePath(Pose2d endPoint, double endRotation) {
            // Create a list of waypoints from poses. Each pose represents one waypoint.
            // The rotation component of the pose should be the direction of travel. Do not
            // use holonomic rotation.
            List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
            //1.0, 1.0, Rotation2d.fromDegrees(0)
                    endPoint
                    
                    //can add on waypoints this can be for queuing or smt ig
                    //, new Pose2d(3.0, 1.0, Rotation2d.fromDegrees(0)),
                    //new Pose2d(5.0, 3.0, Rotation2d.fromDegrees(90))
            );

            PathConstraints constraints = new PathConstraints(3.0, 3.0, 2 * Math.PI, 4 * Math.PI); // The constraints
            // for this path.
            // PathConstraints constraints = PathConstraints.unlimitedConstraints(12.0); //
            // You can also use unlimited constraints, only limited by motor torque and
            // nominal battery voltage

            // Create the path using the waypoints created above

            PathPlannerPath path = new PathPlannerPath(
                    waypoints,
                    constraints,
                    null, // The ideal starting state, this is only relevant for pre-planned paths, so can
                          // be null for on-the-fly paths.
                    new GoalEndState(0.0, Rotation2d.fromDegrees(endRotation)) // Goal end state. You can set a holonomic
                    // rotation here. If using a differential
                    // drivetrain, the rotation will have no effect.
            );

            // Prevent the path from being flipped if the coordinates are already correct
            path.preventFlipping = false;

            

            return AutoBuilder.followPath(path);
        }
    }
}