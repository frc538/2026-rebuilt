package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class ClimberSubsystem extends SubsystemBase {
  private final ClimberIO io;
  private final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();

  private double motorSpeed = 0;

  public ClimberSubsystem(ClimberIO IO) {
    io = IO;
  }

  // Main/Base Commands
  public Command climberRetract() {
    return runEnd(
        () -> {
          motorSpeed = -1;
          io.setOutput(motorSpeed);
        },
        () -> {
          motorSpeed = 0;
          io.setOutput(motorSpeed);
        });
  }

  public Command climberExtend() {
    return runEnd(
        () -> {
          motorSpeed = 1;
          io.setOutput(motorSpeed);
        },
        () -> {
          motorSpeed = 0;
          io.setOutput(motorSpeed);
        });
  }

  public Command TestClimberExtend() {
    return runEnd(
        () -> {
          motorSpeed = 0.2;
          io.setOutput(motorSpeed);
        },
        () -> {
          motorSpeed = 0;
          io.setOutput(motorSpeed);
        });
  }

  public Command TestClimberRetract() {
    return runEnd(
        () -> {
          motorSpeed = -0.2;
          io.setOutput(motorSpeed);
        },
        () -> {
          motorSpeed = 0;
          io.setOutput(motorSpeed);
        });
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("climber subsystem", inputs);
    Logger.recordOutput("climber subsystem/motorSpeed", motorSpeed);
  }
}
