package frc.robot.subsystems.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {

  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
  public boolean FlipFlop = false;
  public double SpeedThingy;

  public Intake(IntakeIO io) {
    this.io = io;
    SpeedThingy = (Constants.Intake.RotatoRPM);
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

    if (inputs.positionRad > Constants.Intake.RotatoThresholdRAD) {
      io.runRotato(0);
    } else {
      io.runRotato(SpeedThingy);
    }
  }

  public Command runIntake(double speed) {
    return run(
        () -> {
          io.runRotato(speed);
          Logger.recordOutput("Intake/rotato command", speed);
        });
  }

  public Command testIntake() {
    return run(
        () -> {
          io.runRotato(.1);
        });
  }

  public Command changeIntakePosition(double radians) {
    return runOnce(
        () -> {
          io.setIntakePosition(radians);
          Logger.recordOutput("IntakeArm/Set Intake Position Command", radians);
        });
  }

  public Command togglePosition() {
    return runOnce(
        () -> {
          if (FlipFlop == true) {
            io.setIntakePosition(Constants.Intake.ReadyPos);
            FlipFlop = false;
          } else if (FlipFlop = false) {
            io.setIntakePosition(Constants.Intake.UprightPos);
            FlipFlop = true;
          }
        });
  }

  public Command testToggle() {
    return runOnce(
        () -> {
          if (FlipFlop == true) {
            changeIntakePosition(Constants.Intake.ReadyPos);

            FlipFlop = false;
          } else {
            changeIntakePosition(Constants.Intake.UprightPos);
            FlipFlop = true;
          }
          SpeedThingy = Constants.Intake.TestRotatoRPM;
        });
  }
}
