package frc.robot.subsystems.hopper;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;

public class Hopper {
  public class Arm extends SubsystemBase {
    HopperIO io;
    private double sdSpeed = 0.0;
    private double fdSpeed = 0.0;

    public Command HopperON(DoubleSupplier speedSupplier) {
      return run(
          () -> {
            sdSpeed = speedSupplier.getAsDouble();
            fdSpeed = speedSupplier.getAsDouble();
            fdSpeed = MathUtil.applyDeadband(fdSpeed, 0.1);
            sdSpeed = MathUtil.applyDeadband(sdSpeed, 0.1);
            io.SpindexSpeedCommand(fdSpeed);
            io.FeedSpeedCommand(sdSpeed);
          });
    }
  }
}
