package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import org.littletonrobotics.junction.Logger;

public class ClimberSubsystem extends SubsystemBase {
  private final ClimberIO io;
  private final ClimberIOInputsAutoLogged inputs;

  private double motorSpeed;

  public ClimberSubsystem(ClimberIO IO) {
    io = IO;
    inputs = new ClimberIOInputsAutoLogged();
    motorSpeed = 0;
  }

  // Main/Base Commands
  public Command climberRetract() {
    return runEnd(
        () -> {
          motorSpeed = -Constants.ClimberConstants.climberSpeed;
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
          motorSpeed = Constants.ClimberConstants.climberSpeed;
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
          motorSpeed = Constants.ClimberConstants.testclimberSpeed;
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
          motorSpeed = -Constants.ClimberConstants.testclimberSpeed;
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
