package frc.robot.subsystems.intake;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.sim.SparkRelativeEncoderSim;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkRelativeEncoder;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import frc.robot.Constants;

public class IntakeIOSim implements IntakeIO {
  SparkMax MovMotor;
  SparkRelativeEncoder MovMotorEncoder;
  SparkMaxConfig movConfig;

  TrapezoidProfile commandProfile;
  Constraints profileConstraints;
  TrapezoidProfile.State mCurrentState;
  TrapezoidProfile.State mDesiredState;

  private boolean firstFrame = true;

  double mReferencePosition = 0.0;
  double mArbFF = 0.0;
  double kS = 0.0;
  double kG = 0.0;
  double kV = 0.0;
  double kA = 0.0;
  double maxV = 0.3;
  double maxA = 0.3;
  double kP = 0.0;
  double kI = 0.0;
  double kD = 0.0;

  double kPLast = 0;
  double kILast = 0;
  double kDLast = 0;

  private SparkClosedLoopController movMotorController;

  SparkMaxSim MovMotorSim;
  SparkRelativeEncoderSim movMotorEncoderSim;

  public IntakeIOSim(int MovMotorCanId) {
    MovMotor = new SparkMax(MovMotorCanId, MotorType.kBrushless);
    MovMotorEncoder = (SparkRelativeEncoder) MovMotor.getEncoder();

    MovMotorSim = new SparkMaxSim(MovMotor, null);
    movMotorEncoderSim = MovMotorSim.getRelativeEncoderSim();

    movConfig 
    .idleMode(IdleMode.kBrake)
        .inverted(false)
        .smartCurrentLimit(Constants.Intake.ExtensionCurrentLimit);
    movConfig
    .encoder
        .positionConversionFactor(Constants.Intake.IntakePositionConversionFactor)
        .velocityConversionFactor(Constants.Intake.IntakeVelocityConversionFactor);
    movConfig
    .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(0, 0, 0)
        .outputRange(-1, 1);
        MovMotor.configure(movConfig,ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        movMotorController =  MovMotor.getClosedLoopController(); 
        profileConstraints = new Constraints(maxV, maxA);
        mCurrentState = new TrapezoidProfile.State();
  }
  //I barely know what im doing D:
  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.armMotorBusVoltage = MovMotorSim.getAppliedOutput() * RoboRioSim.getVInVoltage();
    inputs.armMotorCurrent = MovMotorSim.getMotorCurrent();
    inputs.armMotorOutput = MovMotorSim.getAppliedOutput();
    // inputs.height = m_elevatorSim.getPositionMeters();

    inputs.MovEncoderValue = MovMotorEncoder.getPosition();
  }
}
