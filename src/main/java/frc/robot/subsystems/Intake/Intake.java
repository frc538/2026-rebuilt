package frc.robot.subsystems.Intake;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {

  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
  public boolean FlipFlop = false;

  public TrapezoidProfile.State mCurrentState =
      new TrapezoidProfile.State(Constants.Intake.UprightPos, 0);
  public TrapezoidProfile.State mDesiredState =
      new TrapezoidProfile.State(Constants.Intake.UprightPos, 0);
  public TrapezoidProfile mTrapezoidProfile;

  public Constraints mConstraints;

  public Intake(IntakeIO io) {
    this.io = io;
    mConstraints = new Constraints(Constants.Intake.MaxV, Constants.Intake.MaxA);
    mTrapezoidProfile = new TrapezoidProfile(mConstraints);
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

    if (!DriverStation.isTest()) {
      mCurrentState = mTrapezoidProfile.calculate(0.02, mCurrentState, mDesiredState);
      io.setIntakePosition(mCurrentState.position, inputs.positionRad);

      if (inputs.positionRad > Constants.Intake.RotatoThresholdRAD) {
        io.runRotato(0);
      } else {
        io.runRotato(Constants.Intake.RotatoRPM);
      }
    }
  }

  public Command runIntake(double speed) {
    return run(() -> {
          io.runRotato(speed);
          Logger.recordOutput("Intake/rotato command", speed);
        })
        .finallyDo(() -> io.runRotato(0));
  }

  public Command testIntake() {
    return run(() -> {
          io.runRotato(Constants.Intake.testRotatoRPM);
          Logger.recordOutput("Intake/Rightrotato command", inputs.RightrotatoRpm);
          Logger.recordOutput("Intake/Leftrotato command", inputs.LeftrotatoRpm);
        })
        .finallyDo(() -> io.runRotato(0));
  }

  public Command testIntakeUp() {
    return runOnce(
        () -> {
          io.setIntakePosition(Constants.Intake.UprightPos, inputs.positionRad);
        });
  }

  public Command testIntakeDown() {
    return runOnce(
        () -> {
          io.setIntakePosition(Constants.Intake.ReadyPos, inputs.positionRad);
        });
  }

  public Command togglePosition() {
    return runOnce(
        () -> {
          if (FlipFlop == true) {
            SetReference(Constants.Intake.ReadyPos);
            FlipFlop = false;
          } else {
            SetReference(Constants.Intake.UprightPos);
            FlipFlop = true;
          }
        });
  }

  public void SetReference(double position) {
    mDesiredState = new TrapezoidProfile.State(position, 0);
    Logger.recordOutput("Intake/Commanded Position", position);
  }
}
