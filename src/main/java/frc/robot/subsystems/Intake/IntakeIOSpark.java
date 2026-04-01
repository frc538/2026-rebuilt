package frc.robot.subsystems.Intake;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.Constants;

public class IntakeIOSpark implements IntakeIO {

  private final SparkMax movementMotor;
  SparkMaxConfig config;
  private final SparkFlex Rotato;

  private final RelativeEncoder armEncoder;
  private final RelativeEncoder rotatoEncoder;
  private final SparkClosedLoopController pid;

  public IntakeIOSpark() {
    Rotato = new SparkFlex(Constants.CanIds.LeftRotatoCanId, MotorType.kBrushless);
    rotatoEncoder = Rotato.getEncoder();
    SparkFlexConfig RotatoConfig = new SparkFlexConfig();

    movementMotor = new SparkMax(Constants.CanIds.MovMotorCanId, MotorType.kBrushless);
    armEncoder = movementMotor.getEncoder();
    pid = movementMotor.getClosedLoopController();

    config
        .encoder
        .positionConversionFactor(Constants.Intake.ArmPosConFac)
        .velocityConversionFactor(Constants.Intake.ArmVelConFac);

    config.idleMode(IdleMode.kBrake);

    RotatoConfig.encoder
        .positionConversionFactor(Constants.Intake.RotatoPosConFac)
        .velocityConversionFactor(Constants.Intake.RotatoVelConFac);

    RotatoConfig.smartCurrentLimit(Constants.CurrentLimit.rotatoLimit);
    config.smartCurrentLimit(Constants.CurrentLimit.armLimit);

    config
        .closedLoop
        .p(Constants.Intake.ArmkP)
        .i(Constants.Intake.ArmkI)
        .d(Constants.Intake.ArmkD)
        .outputRange(-1.0, 1.0);

    movementMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    Rotato.configure(
        RotatoConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    armEncoder.setPosition(Constants.Intake.UprightPos);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {


    inputs.LeftrotatoOutput = Rotato.getAppliedOutput();
    inputs.LeftrotatoBusVoltage = Rotato.getBusVoltage();
    inputs.LeftrotatoCurrent = Rotato.getOutputCurrent();

    inputs.armMotorOutput = movementMotor.getAppliedOutput();
    inputs.armMotorBusVoltage = movementMotor.getBusVoltage();
    inputs.armMotorCurrent = movementMotor.getOutputCurrent();

    inputs.positionRad = armEncoder.getPosition();
    inputs.LeftrotatoRpm = rotatoEncoder.getVelocity();
    inputs.MovementMotorRPM = armEncoder.getVelocity();
    inputs.MovementMotorRotation = armEncoder.getPosition();
  }

  @Override
  public void runRotato(double speed) {
    Rotato.set(speed);
  }

  @Override
  public void setIntakePosition(double radians, double CurrentRads) {
    pid.setSetpoint(
        radians,
        ControlType.kPosition,
        ClosedLoopSlot.kSlot0,
        Constants.Intake.IntakeKg * Math.sin(CurrentRads - Constants.Intake.alpha));
  }

  @Override
  public void testArmRun(double speed) {
    movementMotor.setVoltage(speed);
  }
}
