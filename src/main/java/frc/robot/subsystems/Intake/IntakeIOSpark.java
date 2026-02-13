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

public class IntakeIOSpark implements IntakeIO {
  SparkFlex LeftRotato;
  SparkFlex RightRotato;
  SparkMax MovMotor;
  RelativeEncoder armEncoder;
  RelativeEncoder RightRotatoEncoder;
  RelativeEncoder LeftRotatoEncoder;
  SparkClosedLoopController Lpid;
  SparkClosedLoopController Rpid;

  public IntakeIOSpark(int RightRotatoCanId, int LeftRotatoCanId, int MovMovCanId) {
    RightRotato = new SparkFlex(RightRotatoCanId, MotorType.kBrushless);
    LeftRotato = new SparkFlex(LeftRotatoCanId, MotorType.kBrushless);

    MovMotor = new SparkMax(MovMovCanId, MotorType.kBrushless);

    armEncoder = MovMotor.getEncoder();
    RightRotatoEncoder = LeftRotato.getEncoder();
    LeftRotatoEncoder = RightRotato.getEncoder();

    Lpid = LeftRotato.getClosedLoopController();
    Rpid = RightRotato.getClosedLoopController();

    SparkFlexConfig RotatoConfig = new SparkFlexConfig();
    SparkMaxConfig config = new SparkMaxConfig();

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

    LeftRotato.configure(
        RotatoConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    RightRotato.configure(
        RotatoConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    MovMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {

    inputs.RightrotatoOutput = LeftRotato.getAppliedOutput();
    inputs.RightrotatoBusVoltage = LeftRotato.getBusVoltage();
    inputs.RightrotatoCurrent = LeftRotato.getOutputCurrent();

    inputs.LeftrotatoOutput = LeftRotato.getAppliedOutput();
    inputs.LeftrotatoBusVoltage = LeftRotato.getBusVoltage();
    inputs.LeftrotatoCurrent = LeftRotato.getOutputCurrent();

    inputs.armMotorOutput = MovMotor.getAppliedOutput();
    inputs.armMotorBusVoltage = MovMotor.getBusVoltage();
    inputs.armMotorCurrent = MovMotor.getOutputCurrent();

    inputs.positionRad = armEncoder.getPosition();
    inputs.RightrotatoRpm = LeftRotatoEncoder.getVelocity();
    inputs.LeftrotatoRpm = LeftRotatoEncoder.getVelocity();
    inputs.MovementMotorRPM = armEncoder.getVelocity();
    inputs.MovementMotorRotation = armEncoder.getPosition();
  }

  @Override
  public void runRotato(double speed) {
    MovMotor.set(speed);
  }

  @Override
  public void setIntakePosition(double radians) {
    Lpid.setSetpoint(radians, ControlType.kPosition);
    Rpid.setSetpoint(radians, ControlType.kPosition);
  }
}
