package frc.robot.subsystems.intake;

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

public class IntakeIOSpark implements IntakeIO {
  SparkFlex LeftMovMotor;
  SparkFlex RightMovMotor;
  SparkMax rotato;
  RelativeEncoder armEncoder;
  RelativeEncoder rotatoEncoder;
  SparkClosedLoopController Lpid;
  SparkClosedLoopController Rpid;

  public IntakeIOSpark(int RightMovMotorCanId, int LeftMovMotorCanId, int RotatoCanId) {
    RightMovMotor = new SparkFlex(RightMovMotorCanId, MotorType.kBrushless);
    LeftMovMotor = new SparkFlex(LeftMovMotorCanId, MotorType.kBrushless);

    rotato = new SparkMax(RotatoCanId, MotorType.kBrushless);

    armEncoder = LeftMovMotor.getEncoder();
    armEncoder = RightMovMotor.getEncoder();
    rotatoEncoder = rotato.getEncoder();

    Lpid = LeftMovMotor.getClosedLoopController();
    Rpid = RightMovMotor.getClosedLoopController();

    SparkFlexConfig config = new SparkFlexConfig();
    SparkMaxConfig RotatoConfig = new SparkMaxConfig();

    config
        .encoder
        .positionConversionFactor(Constants.Intake.ArmPosConFac)
        .velocityConversionFactor(Constants.Intake.ArmVelConFac);

    RotatoConfig.encoder
        .positionConversionFactor(Constants.Intake.RotatoPosConFac)
        .velocityConversionFactor(Constants.Intake.RotatoVelConFac);

    config
        .closedLoop
        .p(Constants.Intake.IntakekP)
        .i(Constants.Intake.IntakekI)
        .d(Constants.Intake.IntakekD)
        .outputRange(-1.0, 1.0);

    LeftMovMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    RightMovMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    rotato.configure(RotatoConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {

    inputs.rotatoOutput = rotato.getAppliedOutput();
    inputs.rotatoBusVoltage = rotato.getBusVoltage();
    inputs.rotatoCurrent = rotato.getOutputCurrent();

    inputs.armMotorOutput = LeftMovMotor.getAppliedOutput();
    inputs.armMotorBusVoltage = LeftMovMotor.getBusVoltage();
    inputs.armMotorCurrent = LeftMovMotor.getOutputCurrent();

    inputs.armMotorOutput = RightMovMotor.getAppliedOutput();
    inputs.armMotorBusVoltage = RightMovMotor.getBusVoltage();
    inputs.armMotorCurrent = RightMovMotor.getOutputCurrent();

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
    Lpid.setSetpoint(radians, ControlType.kPosition);
    Rpid.setSetpoint(radians, ControlType.kPosition);
  }
}
