package frc.robot.subsystems.navigation;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.navigationConstants;

public class Navigation2 extends SubsystemBase {

  class edge {
    final node m_start;
    final node m_end;

    // Constraints
    final Rotation2d m_orientation; // How should we point when crossing this edge - relative to blue alliance
    final double m_maxSpeed;        // What is our max speed m/s

    public edge (node start, node end, Rotation2d orientation, double maxSpeed) {
      m_start = start;
      m_end = end;
      m_orientation = orientation;
      m_maxSpeed = maxSpeed;
    }
  }

  class node {
    // target point in box
    final Pose2d m_pose;

    public edge up = null;
    public edge down = null;
    public edge right = null;
    public edge left = null;

    public node(Pose2d pose) {
      m_pose = pose;
    }
  }

  // even on right, low is closest
  node[] nodes = {
    new node(navigationConstants.rightBlue),
    new node(navigationConstants.leftBlue),
    new node(navigationConstants.centerRightBlue),
    new node(navigationConstants.centerLeftBlue),
    new node(navigationConstants.centerRightRed),
    new node(navigationConstants.centerLeftRed),
    new node(navigationConstants.rightRed),
    new node(navigationConstants.leftRed)
  };
  

  public Navigation2() {
    nodes[0].up = new edge(nodes[0], nodes[2], new Rotation2d(), 3);
    nodes[0].left = new edge(nodes[0], nodes[1], new Rotation2d(), 3);

    nodes[1].up = new edge(nodes[1], nodes[3], new Rotation2d(), 3);
    nodes[1].right = new edge(nodes[1], nodes[0], new Rotation2d(), 3);

    nodes[2].up = new edge(nodes[2], nodes[4], new Rotation2d(), 3);
    nodes[2].left = new edge(nodes[2], nodes[3], new Rotation2d(), 3);
    nodes[2].down = new edge(nodes[2], nodes[0], new Rotation2d(), 3);

    nodes[3].up = new edge(nodes[3], nodes[5], new Rotation2d(), 3);
    nodes[3].right = new edge(nodes[3], nodes[2], new Rotation2d(), 3);
    nodes[3].down = new edge(nodes[3], nodes[1], new Rotation2d(), 3);

    // TODO: More nodes
  }

  // TODO: Use the nodes
}
