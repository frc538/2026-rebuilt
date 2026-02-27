package frc.robot.subsystems.Intake;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {

  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
  public boolean FlipFlop = false;

  public Intake(IntakeIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);
    Logger.recordOutput("Intake/FlipFlop", FlipFlop);
    Logger.recordOutput("Intake/Sim/", inputs.MovEncoderValue);
    Logger.recordOutput("Intake/Sim/", inputs.positionRad);
    Logger.recordOutput("Intake/Sim/", inputs.MovementMotorRPM);
    Logger.recordOutput("Intake/Sim/", inputs.MovementMotorRotation);

    if (inputs.positionRad > Constants.Intake.RotatoThresholdRAD && !DriverStation.isTest()) {
      io.runRotato(0);
    } else if (!DriverStation.isTest()) {
      io.runRotato(Constants.Intake.RotatoRPM);
    }

    if (inputs.positionRad > Constants.Intake.RotatoThresholdRAD && DriverStation.isTest()) {
      io.runRotato(0);
    } else if (DriverStation.isTest()) {
      io.runRotato(Constants.Intake.testRotatoRPM);
    }
  }

  public void FlipFlop() {
    if (FlipFlop == true) {
      io.setIntakePosition(Constants.Intake.ReadyPos);
      FlipFlop = false;
    } else {
      io.setIntakePosition(Constants.Intake.UprightPos);
      FlipFlop = true;
    }
  }

  public Command runIntake(double speed) {
    return run(
        () -> {
          io.runRotato(speed);
          Logger.recordOutput("Intake/rotato command", speed);
        });
  }

  public Command togglePosition() {
    return runOnce(
        () -> {
          FlipFlop();
        });
  }
}
