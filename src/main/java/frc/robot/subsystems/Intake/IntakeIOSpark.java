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
  private final SparkFlex Rightrotato;
  private final SparkFlex Leftrotato;

  private final RelativeEncoder armEncoder;
  private final RelativeEncoder RightrotatoEncoder;
  private final RelativeEncoder LeftrotatoEncoder;
  private final SparkClosedLoopController pid;
  SparkFlexConfig LeftRotatoConfig;
  SparkFlexConfig RightRotatoConfig;

  public IntakeIOSpark() {
    Rightrotato = new SparkFlex(Constants.CanIds.RightRotatoCanId, MotorType.kBrushless);
    RightrotatoEncoder = Rightrotato.getEncoder();
    LeftRotatoConfig = new SparkFlexConfig();

    Leftrotato = new SparkFlex(Constants.CanIds.LeftRotatoCanId, MotorType.kBrushless);
    LeftrotatoEncoder = Leftrotato.getEncoder();
    SparkFlexConfig RightRotatoConfig = new SparkFlexConfig();

    movementMotor = new SparkMax(Constants.CanIds.MovMotorCanId, MotorType.kBrushless);
    armEncoder = movementMotor.getEncoder();
    pid = movementMotor.getClosedLoopController();
    SparkMaxConfig config = new SparkMaxConfig();

    config
        .encoder
        .positionConversionFactor(Constants.Intake.ArmPosConFac)
        .velocityConversionFactor(Constants.Intake.ArmVelConFac);

    config.idleMode(IdleMode.kBrake);

    
    LeftRotatoConfig.encoder
        .positionConversionFactor(Constants.Intake.RotatoPosConFac)
        .velocityConversionFactor(Constants.Intake.RotatoVelConFac);

    LeftRotatoConfig.smartCurrentLimit(Constants.CurrentLimit.rotatoLimit);

    RightRotatoConfig.encoder
        .positionConversionFactor(Constants.Intake.RotatoPosConFac)
        .velocityConversionFactor(Constants.Intake.RotatoVelConFac);

    RightRotatoConfig.smartCurrentLimit(Constants.CurrentLimit.rotatoLimit);
    RightRotatoConfig.follow(Constants.CanIds.LeftRotatoCanId,true  );
    config.smartCurrentLimit(Constants.CurrentLimit.armLimit);

    config
        .closedLoop
        .p(Constants.Intake.ArmkP)
        .i(Constants.Intake.ArmkI)
        .d(Constants.Intake.ArmkD)
        .outputRange(-1.0, 1.0);

    movementMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    Rightrotato.configure(
        RightRotatoConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    Leftrotato.configure(
        LeftRotatoConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    armEncoder.setPosition(Constants.Intake.UprightPos);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {

    inputs.RightrotatoOutput = Rightrotato.getAppliedOutput();
    inputs.RightrotatoBusVoltage = Rightrotato.getBusVoltage();
    inputs.RightrotatoCurrent = Rightrotato.getOutputCurrent();

    inputs.LeftrotatoOutput = Leftrotato.getAppliedOutput();
    inputs.LeftrotatoBusVoltage = Leftrotato.getBusVoltage();
    inputs.LeftrotatoCurrent = Leftrotato.getOutputCurrent();

    inputs.armMotorOutput = movementMotor.getAppliedOutput();
    inputs.armMotorBusVoltage = movementMotor.getBusVoltage();
    inputs.armMotorCurrent = movementMotor.getOutputCurrent();

    inputs.positionRad = armEncoder.getPosition();
    inputs.RightrotatoRpm = RightrotatoEncoder.getVelocity();
    inputs.LeftrotatoRpm = LeftrotatoEncoder.getVelocity();
    inputs.MovementMotorRPM = armEncoder.getVelocity();
    inputs.MovementMotorRotation = armEncoder.getPosition();
  }

  @Override
  public void runRotato(double speed) {
    Leftrotato.set(speed);
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
