package frc.robot.subsystems.Intake;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {

  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
  public boolean FlipFlop = true;
  private boolean intakerToggle = false;

  private boolean doboble = false;
  private double targetPosition = Math.PI / 4;
  private double timestamp;
  private double armPos;
  private double posErr;
  private double errorDelta;

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

  public Command bobble() {
    return Commands.runOnce(
        () -> {
          timestamp = Timer.getTimestamp();
          doboble = !doboble;
          armPos = inputs.positionRad;
          targetPosition = Math.PI / 4;
          posErr = targetPosition - armPos;
          errorDelta = posErr / 50;
        });
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

    mCurrentState = mTrapezoidProfile.calculate(0.02, mCurrentState, mDesiredState);
    if (doboble == false) {
      io.setIntakePosition(mCurrentState.position, inputs.positionRad);
    }
    Logger.recordOutput("Intake/PosProfile", mCurrentState.position);

    if (doboble == true) {
      double timeDelta = Timer.getTimestamp() - timestamp;
      double mag = Math.PI / 10;
      double period = 2;
      io.setIntakePosition(
          Math.PI + mag * Math.sin(timeDelta / period) - posErr, inputs.positionRad);
      posErr = posErr - errorDelta;
      if (Math.abs(posErr) < 2 * Math.abs(errorDelta)) {
        posErr = 0;
      }
    }

    if (intakerToggle == true || inputs.positionRad > Constants.Intake.RotatoThresholdRAD) {
      io.runRotato(Constants.Intake.RotatoRPM);
    } else {
      io.runRotato(0);
    }
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

  public Command testIntake() {
    return run(() -> {
          io.runRotato(Constants.Intake.testRotatoRPM);
          Logger.recordOutput("Intake/Rightrotato command", inputs.RightrotatoRpm);
          Logger.recordOutput("Intake/Leftrotato command", inputs.LeftrotatoRpm);
        })
        .finallyDo(() -> io.runRotato(0));
  }

  public Command testIntakeUp() {
    return run(() -> {
          io.testArmRun(1 * 0.2);
        })
        .finallyDo(() -> io.testArmRun(0));
  }

  public Command testIntakeDown() {
    return run(() -> {
          io.testArmRun(0.2 * -1);
        })
        .finallyDo(() -> io.testArmRun(0));
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

  public Command HumpAvoid() {
    return (runOnce(() -> SetReference(Constants.Intake.HalfPos)));
  }

  public void SetReference(double position) {
    mDesiredState = new TrapezoidProfile.State(position, 0);
    Logger.recordOutput("Intake/Commanded Position", position);
  }
}
