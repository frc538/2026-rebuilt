package frc.robot.subsystems.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {

  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
  public boolean FlipFlop = false;
  public double Pos;

  public Intake(IntakeIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);
  }

  public Command runIntake(double speed) {
    return run(
        () -> {
          if (Pos == Constants.Intake.ReadyPos) {
          io.runRotato(speed);
          Logger.recordOutput("Intake/rotato command", speed);
        }          
        });
  }

  public Command setIntakePosition(double radians) {
    return runOnce(
        () -> {
          io.setIntakePosition(radians);
          Logger.recordOutput("IntakeArm/Set Intake Position Command", radians);
        });
  }

  public Command togglePosition() {
    return runOnce(
        () -> {
          if (FlipFlop = true) {
            Pos = Constants.Intake.UprightPos;
            FlipFlop = false;
          } else if (FlipFlop = false) {
            Pos = Constants.Intake.ReadyPos;
            FlipFlop = true;
          }
        });
  }

}
