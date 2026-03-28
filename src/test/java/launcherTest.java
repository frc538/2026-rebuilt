import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.hal.HAL;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.hopper.HopperIO;
import frc.robot.subsystems.launcher.Launcher;
import frc.robot.subsystems.launcher.LauncherIO;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

public class launcherTest {
  Hopper hopper = null;
  Launcher launcher = null;

  @BeforeEach
  void setup() {
    assert HAL.initialize(500, 0);
    hopper = new Hopper(new HopperIO() {});
    launcher = new Launcher(new LauncherIO() {}, hopper::FirePermit);
  }

  @AfterEach
  void shutdown() throws Exception {
    // hopper.close();
    // launcher.close();
  }

  @Test
  void test_getShootSpeed() {
    //launcher.launchSpeed = 5.0;
    launcher.getShootSpeed();
    assertEquals(1, 1);
  }
}
