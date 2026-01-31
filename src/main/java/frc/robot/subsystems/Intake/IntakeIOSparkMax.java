package frc.robot.subsystems.Intake;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.Constants;

public class IntakeIOSparkMax implements IntakeIO {

  private final SparkMax movementMotor =
      new SparkMax(Constants.Intake.movementMotorCANid, MotorType.kBrushless);
  private final SparkFlex rotato =
      new SparkFlex(Constants.Intake.rotatoMotorCANid, MotorType.kBrushless);

  private final RelativeEncoder armEncoder = movementMotor.getEncoder();
  private final RelativeEncoder rotatoEncoder = rotato.getEncoder();
  private final SparkClosedLoopController pid = movementMotor.getClosedLoopController();

  public IntakeIOSparkMax() {
    SparkMaxConfig config = new SparkMaxConfig();
    SparkFlexConfig RotatoConfig = new SparkFlexConfig();
    double radiansPerRotation = 2 * Math.PI;

    config
        .encoder
        .positionConversionFactor(radiansPerRotation)
        .velocityConversionFactor(radiansPerRotation / 60.0);

    RotatoConfig.encoder
        .positionConversionFactor(radiansPerRotation)
        .velocityConversionFactor(radiansPerRotation / 60.0);

    config.closedLoop.p(0.1).i(0.0).d(0.01).outputRange(-1.0, 1.0);

    movementMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    rotato.configure(RotatoConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {

    inputs.rotatoOutput = rotato.getAppliedOutput();
    inputs.rotatoBusVoltage = rotato.getBusVoltage();
    inputs.rotatoCurrent = rotato.getOutputCurrent();

    inputs.armMotorOutput = movementMotor.getAppliedOutput();
    inputs.armMotorBusVoltage = movementMotor.getBusVoltage();
    inputs.armMotorCurrent = movementMotor.getOutputCurrent();

    inputs.positionRad = armEncoder.getPosition();
    inputs.rotatoRpm = rotatoEncoder.getVelocity();
    inputs.MovementMotorRPM = armEncoder.getVelocity();
    inputs.MovementMotorRotation = armEncoder.getPosition();
  }

  @Override
  public void runRotato(double speed) {
    rotato.set(speed);
  }

  @Override
  public void setIntakePosition(double radians) {
    pid.setSetpoint(radians, ControlType.kPosition);
  }
}
