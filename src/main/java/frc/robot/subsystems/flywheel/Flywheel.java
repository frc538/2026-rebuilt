package frc.robot.subsystems.flywheel;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Flywheel extends SubsystemBase {

  FlywheelIO io;
  FlywheelIOInputsAutoLogged inputs = new FlywheelIOInputsAutoLogged();

  public Flywheel(FlywheelIO IO) {
    io = IO;
  }

  public Command fullSpeed() {
    // Commands.run()
    return Commands.run(() -> {});
  }

  public Command lowSpeed() {
    return Commands.run(() -> {});
  }
}
