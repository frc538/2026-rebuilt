package frc.robot.subsystems.launcher;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Launcher extends SubsystemBase {

  LauncherIO io;
  LauncherIOInputsAutoLogged inputs = new LauncherIOInputsAutoLogged();

  public Launcher(LauncherIO IO) {
    io = IO;
  }

  public Command fullSpeed() {
    return Commands.runOnce(
        () -> {
          io.setVoltage(12.0);
          Logger.recordOutput("Launcher/voltageCmd", 12.0);
        });
  }

  public Command lowSpeed() {
    return Commands.runOnce(
        () -> {
          io.setVoltage(3.0);
          Logger.recordOutput("Launcher/voltageCmd", 3.0);
        });
  }

  public Command off() {
    return Commands.runOnce(
        () -> {
          io.setVoltage(0.0);
          Logger.recordOutput("Launcher/voltageCmd", 0.0);
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
    Logger.processInputs("Launcher", inputs);
  }
}
