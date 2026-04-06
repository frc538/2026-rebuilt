package frc.robot.subsystems.Intake;

import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

public class Intake2 extends SubsystemBase {

  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
  private boolean intakerToggle = false;

  //   public TrapezoidProfile.State mCurrentState =
  //       new TrapezoidProfile.State(Constants.Intake.UprightPos, 0);
  //   public TrapezoidProfile.State mDesiredState =
  //       new TrapezoidProfile.State(Constants.Intake.UprightPos, 0);
  //   public TrapezoidProfile mTrapezoidProfile;

  public Constraints mConstraints;

  public Intake2(IntakeIO io) {
    this.io = io;
    // mConstraints = new Constraints(Constants.Intake.MaxV, Constants.Intake.MaxA);
    // mTrapezoidProfile = new TrapezoidProfile(mConstraints);
  }

  public Command forceIntake() {
    return Commands.runOnce(
        () -> {
          intakerToggle = !intakerToggle;
        },
        this);
  }

  public Command runIntake(double speed) {
    return run(() -> {
          io.runRotato(speed);
          Logger.recordOutput("Intake/rotato command", speed);
        })
        .finallyDo(() -> io.runRotato(0));
  }

  public Command intakeUpVoltage() {
    return run(() -> {
          io.armRunVolt(Constants.Intake2.upVoltage);
        })
        .finallyDo(() -> io.armRunVolt(0));
  }

  public Command intakeDownVoltage() {
    return run(() -> {
          io.armRunVolt(Constants.Intake2.downVoltage);
        })
        .finallyDo(() -> io.armRunVolt(0));
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
}
}