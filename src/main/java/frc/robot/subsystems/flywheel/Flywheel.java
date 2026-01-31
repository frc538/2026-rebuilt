package frc.robot.subsystems.flywheel;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Flywheel extends SubsystemBase {

  FlywheelIO io;
  FlywheelIOInputsAutoLogged inputs = new FlywheelIOInputsAutoLogged();

  public Flywheel(FlywheelIO IO) {
    io = IO;
  }

  public Command fullSpeed() {
    return Commands.runOnce(
        () -> {
          io.setVoltage(12.0);
          Logger.recordOutput("Flywheel/voltageCmd", 12.0);
        });
  }

  public Command lowSpeed() {
    return Commands.runOnce(
        () -> {
          io.setVoltage(3.0);
          Logger.recordOutput("Flywheel/voltageCmd", 3.0);
        });
  }

  public Command off() {
    return Commands.runOnce(
        () -> {
          io.setVoltage(0.0);
          Logger.recordOutput("Flywheel/voltageCmd", 0.0);
        });
  }

  public Command feed() {
    return Commands.runOnce(
        () -> {
          io.simLaunch();
        });
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Flywheel", inputs);
  }
}
