package frc.robot.subsystems.Intake;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.Constants;


public class IntakeIOSparkMax implements IntakeIO {
  SparkMax movementMotor;
  SparkMax rotato;
  RelativeEncoder armEncoder;
  RelativeEncoder rotatoEncoder;
  SparkClosedLoopController pid;

  public IntakeIOSparkMax(int MovMotorCanId, int RotatoCanId) {
    movementMotor = new SparkMax(MovMotorCanId, MotorType.kBrushless);
    rotato = new SparkMax(RotatoCanId, MotorType.kBrushless);
    armEncoder = movementMotor.getEncoder();
    rotatoEncoder = rotato.getEncoder();
    pid = movementMotor.getClosedLoopController();

    SparkMaxConfig config = new SparkMaxConfig();
    SparkFlexConfig RotatoConfig = new SparkFlexConfig();
    
    config
        .encoder
        .positionConversionFactor(Constants.Intake.ArmPosConFac)
        .velocityConversionFactor(Constants.Intake.ArmVelConFac);

    RotatoConfig.encoder
        .positionConversionFactor(Constants.Intake.RotatoPosConFac)
        .velocityConversionFactor(Constants.Intake.RotatoVelConFac);

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
