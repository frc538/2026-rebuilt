import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.hopper.HopperIO;
import frc.robot.subsystems.launcher.Launcher;
import frc.robot.subsystems.launcher.LauncherIO;
import frc.robot.subsystems.navigation.Navigation2;
import frc.robot.subsystems.navigation.Navigation2.directorator;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

public class navTest {
  Drive drive = null;
  Hopper hopper = null;
  Launcher launcher = null;
  Navigation2 nav = null;

  @BeforeEach
  void setup() {
    if (nav == null) {

      hopper = new Hopper(new HopperIO() {});
      launcher = new Launcher(new LauncherIO() {}, hopper::FirePermit);
      drive =
          new Drive(
              launcher::updateOdometry,
              new GyroIO() {},
              new ModuleIOSim(TunerConstants.FrontLeft),
              new ModuleIOSim(TunerConstants.FrontRight),
              new ModuleIOSim(TunerConstants.BackLeft),
              new ModuleIOSim(TunerConstants.BackRight));
      nav = new Navigation2(drive);
    }
  }

  @AfterEach
  void shutdown() throws Exception {
    // nav.();
  }

  @Test
  void testAngles() {
    Translation2d initialPose = new Translation2d(0, 0);
    Pose2d result;
    double expected, resultD;

    result = nav.calculateOffset(initialPose, directorator.up, directorator.right, 0.8);
    resultD = result.getRotation().getDegrees();
    expected = 0 + 0.8 * -90;
    System.out.printf("expect: %f, got: %f\n", expected, resultD);
    assertEquals(expected, resultD, 0.001);

    result = nav.calculateOffset(initialPose, directorator.right, directorator.down, 0.8);
    resultD = result.getRotation().getDegrees();
    expected = Math.toDegrees(MathUtil.angleModulus(Math.toRadians(270 + 0.8 * -90)));
    System.out.printf("expect: %f, got: %f\n", expected, resultD);
    assertEquals(expected, resultD);

    result = nav.calculateOffset(initialPose, directorator.down, directorator.left, 0.8);
    resultD = result.getRotation().getDegrees();
    expected = Math.toDegrees(MathUtil.angleModulus(Math.toRadians(180 + 0.8 * -90)));
    System.out.printf("expect: %f, got: %f\n", expected, resultD);
    assertEquals(expected, resultD);

    result = nav.calculateOffset(initialPose, directorator.left, directorator.up, 0.8);
    resultD = result.getRotation().getDegrees();
    expected = Math.toDegrees(MathUtil.angleModulus(Math.toRadians(90 + 0.8 * -90)));
    System.out.printf("expect: %f, got: %f\n", expected, resultD);
    assertEquals(expected, resultD);

    result = nav.calculateOffset(initialPose, directorator.up, directorator.left, 0.8);
    resultD = result.getRotation().getDegrees();
    expected = 0 + 0.8 * 90;
    System.out.printf("expect: %f, got: %f\n", expected, resultD);
    assertEquals(expected, resultD);

    result = nav.calculateOffset(initialPose, directorator.left, directorator.down, 0.8);
    resultD = result.getRotation().getDegrees();
    expected = Math.toDegrees(MathUtil.angleModulus(Math.toRadians(90 + 0.8 * 90)));
    System.out.printf("expect: %f, got: %f\n", expected, resultD);
    assertEquals(expected, resultD);

    result = nav.calculateOffset(initialPose, directorator.down, directorator.right, 0.8);
    resultD = result.getRotation().getDegrees();
    expected = Math.toDegrees(MathUtil.angleModulus(Math.toRadians(180 + 0.8 * 90)));
    System.out.printf("expect: %f, got: %f\n", expected, resultD);
    assertEquals(expected, resultD);

    result = nav.calculateOffset(initialPose, directorator.right, directorator.up, 0.8);
    resultD = result.getRotation().getDegrees();
    expected = Math.toDegrees(MathUtil.angleModulus(Math.toRadians(270 + 0.8 * 90)));
    System.out.printf("expect: %f, got: %f\n", expected, resultD);
    assertEquals(expected, resultD);
  }
}
