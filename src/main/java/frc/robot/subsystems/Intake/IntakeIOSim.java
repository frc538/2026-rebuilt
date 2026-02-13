package frc.robot.subsystems.intake;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.sim.SparkRelativeEncoderSim;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkRelativeEncoder;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

public class IntakeIOSim implements IntakeIO {
  SparkMax MovMotor;
  SparkRelativeEncoder MovMotorEncoder;
  SparkMaxConfig movConfig;

  SingleJointedArmSim TooLongName;

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
    movConfig = new SparkMaxConfig();

    MovMotorSim = new SparkMaxSim(MovMotor, DCMotor.getNEO(1));
    movMotorEncoderSim = MovMotorSim.getRelativeEncoderSim();

    TooLongName =
        new SingleJointedArmSim(
            DCMotor.getNEO(1), 15, 0.01, 0.25, 0, Math.PI / 2.0, true, Math.PI / 2.0);

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
        .pid(0.1, 0, 0)
        .outputRange(-1, 1);
    MovMotor.configure(movConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    movMotorController = MovMotor.getClosedLoopController();
  }
  // I barely know what im doing D:
  @Override
  public void updateInputs(IntakeIOInputs inputs) {

    TooLongName.update(0.02);
    MovMotorSim.iterate(TooLongName.getVelocityRadPerSec(), 12, 0.02);
    movMotorEncoderSim.iterate(TooLongName.getVelocityRadPerSec(), 0.02);

    inputs.armMotorBusVoltage = MovMotorSim.getAppliedOutput() * RoboRioSim.getVInVoltage();
    inputs.armMotorCurrent = MovMotorSim.getMotorCurrent();
    inputs.armMotorOutput = MovMotorSim.getAppliedOutput();
    // inputs.height = m_elevatorSim.getPositionMeters();

    inputs.MovEncoderValue = MovMotorEncoder.getPosition();
  }

  public void setIntakePosition(double radians) {
    movMotorController.setSetpoint(radians, ControlType.kPosition);

    Logger.recordOutput("Intake/Sim/PositionalValue", MovMotorSim.getPosition());
    Logger.recordOutput("Intake/Sim/VelocityValue", MovMotorSim.getVelocity());
    
  }
}
