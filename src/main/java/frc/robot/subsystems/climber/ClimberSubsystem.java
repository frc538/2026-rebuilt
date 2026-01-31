package frc.robot.subsystems.climber;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class ClimberSubsystem extends SubsystemBase {
  private final ClimberIO io;
  private final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();

  double winchSpeed = 0; //y-axis?
  double actuatorSpeed = 0; //x-axis?

  public ClimberSubsystem(ClimberIO IO) {
    io = IO;
  }

  //Main/Base Commands
  public Command climberExtend() {
    return runOnce(() -> {
      pushUpClimber();
      pushAwayClimber();
    });
  }

  public Command climberRetract() {
    return runOnce(() -> {
      pullInClimber();
      pullDownClimber();
    });
  }

  //Block Commands
  public Command pullDownClimber() {  //Pulls the climber down (y-axis)
    return runEnd(() -> {
      winchSpeed = -1;
      io.setOutput(winchSpeed);
    }, () -> {
      winchSpeed = 0;
      io.setOutput(winchSpeed);
    });
  }

  public Command pullInClimber() { //Pulls the climber in (x-axis)
    return runEnd(() -> {
      actuatorSpeed = -1;
      io.setOutput(actuatorSpeed);
    }, () -> {
      actuatorSpeed = 0;
      io.setOutput(actuatorSpeed);
    });
  }

  public Command pushAwayClimber() { //Pushes the climber away (x-axis)
    return runEnd(() -> {
      actuatorSpeed = 1;
      io.setOutput(actuatorSpeed);
    }, () -> {
      actuatorSpeed = 0;
      io.setOutput(actuatorSpeed);
    });
  }

  public Command pushUpClimber() { //Pushes the climber up (y-axis)
    return runEnd(() -> {
      winchSpeed = 1;
      io.setOutput(winchSpeed);
    }, () -> {
      winchSpeed = 0;
      io.setOutput(winchSpeed);
    });
  }

  public Command climberWinchSpeed(DoubleSupplier speedSupplier) {
    return run(
        () -> {
          winchSpeed = speedSupplier.getAsDouble();
          io.setOutput(winchSpeed);
        });
  }

  public Command climberActuatorSpeed(DoubleSupplier speedSupplier) {
    return run(
        () -> {
          actuatorSpeed = speedSupplier.getAsDouble();
          io.setOutput(actuatorSpeed);
        });
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("climber subsystem", inputs);
    Logger.recordOutput("climber subsystem/winchSpeed", winchSpeed);
    Logger.recordOutput("climber subsystem/actuatorSpeed", actuatorSpeed);
  }
}
