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
          Logger.recordOutput("Intake2/armRunVolt", Constants.Intake2.upVoltage);
        })
        .finallyDo(
            () -> {
              io.armRunVolt(0);
              Logger.recordOutput("Intake2/armRunVolt", 0);
            });
  }

  private int rotatoStallCounter = 0;
  private boolean isStall = false;

  public Command goDownButDontWhenStall() {
    return run(
        () -> {
          if (inputs.RightrotatoCurrent > 45) {
            rotatoStallCounter = rotatoStallCounter + 1;
            if (rotatoStallCounter > 25) {
              isStall = true;
            }
          } else {
            rotatoStallCounter = 0;
            isStall = false;
          }

          if (isStall == true) {
            io.armRunVolt(Constants.Intake2.upVoltage);
            Logger.recordOutput("Intake2/armRunVolt", Constants.Intake2.upVoltage);
          } else {
            io.armRunVolt(Constants.Intake2.downVoltage);
            Logger.recordOutput("Intake2/armRunVolt", Constants.Intake2.downVoltage);
          }
        });
  }

  // public Command intakeDownVoltage() {
  //       return run(() -> {
  //             io.armRunVolt(Constants.Intake2.downVoltage);
  //             Logger.recordOutput("Intake2/armRunVolt", Constants.Intake2.downVoltage);
  //           })
  //           .finallyDo(
  //               () -> {
  //                 io.armRunVolt(0);
  //                 Logger.recordOutput("Intake2/armRunVolt", 0);
  //               });
  // }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake2", inputs);

    Logger.recordOutput("Intake2/isStall", isStall);
    Logger.recordOutput("Intake2/stall count", rotatoStallCounter);

    if (intakerToggle == true) {
      io.runRotato(Constants.Intake.RotatoRPM);
    } else {
      io.runRotato(0);
    }
  }
}
