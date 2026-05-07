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
import org.littletonrobotics.junction.Logger;

public class IntakeIOSpark implements IntakeIO {

  private final SparkMax movementMotor;
  private final SparkFlex Rightrotato;
  // private final SparkFlex Leftrotato =
  //     new SparkFlex(Constants.CanIds.LeftRotatoCanId, MotorType.kBrushless);

  private final RelativeEncoder armEncoder;
  private final RelativeEncoder RightrotatoEncoder;
  // private final RelativeEncoder LeftrotatoEncoder = Leftrotato.getEncoder();
  private final SparkClosedLoopController pid;

  private double m_kgTrim;
  private double m_alphaTrim;

  public IntakeIOSpark() {
     movementMotor = new SparkMax(Constants.CanIds.MovMotorCanId, MotorType.kBrushless);
     armEncoder = movementMotor.getEncoder();
     pid = movementMotor.getClosedLoopController();

    Rightrotato = new SparkFlex(Constants.CanIds.RightRotatoCanId, MotorType.kBrushless);
    RightrotatoEncoder = Rightrotato.getEncoder();

    SparkMaxConfig config = new SparkMaxConfig();
    SparkFlexConfig RotatoConfig = new SparkFlexConfig();
    
    m_kgTrim = 0;
    m_alphaTrim = 0;

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

    config.inverted(true);

    movementMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    Rightrotato.configure(
        RotatoConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    // Leftrotato.configure(
    //     RotatoConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    armEncoder.setPosition(Constants.Intake.UprightPos);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {

    inputs.RightrotatoOutput = Rightrotato.getAppliedOutput();
    inputs.RightrotatoBusVoltage = Rightrotato.getBusVoltage();
    inputs.RightrotatoCurrent = Rightrotato.getOutputCurrent();

    // inputs.LeftrotatoOutput = Leftrotato.getAppliedOutput();
    // inputs.LeftrotatoBusVoltage = Leftrotato.getBusVoltage();
    // inputs.LeftrotatoCurrent = Leftrotato.getOutputCurrent();

    inputs.armMotorOutput = movementMotor.getAppliedOutput();
    inputs.armMotorBusVoltage = movementMotor.getBusVoltage();
    inputs.armMotorCurrent = movementMotor.getOutputCurrent();

    inputs.positionRad = armEncoder.getPosition();
    inputs.RightrotatoRpm = RightrotatoEncoder.getVelocity();
    // inputs.LeftrotatoRpm = LeftrotatoEncoder.getVelocity();
    inputs.MovementMotorRPM = armEncoder.getVelocity();
    inputs.MovementMotorRotation = armEncoder.getPosition();
  }

  @Override
  public void runRotato(double speed) {
    Rightrotato.set(speed * -1);
    // Leftrotato.set(speed);
  }

  @Override
  public void armRunVolt(double voltage) {
    movementMotor.setVoltage(voltage);
  }

  @Override
  public void setIntakePosition(double radians, double CurrentRads) {
    double effectiveKg = Constants.Intake.IntakeKg + m_kgTrim;
    double effectiveAlpha = CurrentRads - Constants.Intake.alpha + m_alphaTrim;
    Logger.recordOutput("Intake/kgTrim", m_kgTrim);
    Logger.recordOutput("Intake/alphaTrim", m_alphaTrim);
    Logger.recordOutput("Intake/effectiveKg", effectiveKg);
    Logger.recordOutput("Intake/effectiveAlpha", effectiveAlpha);
    pid.setSetpoint(
        radians,
        ControlType.kPosition,
        ClosedLoopSlot.kSlot0,
        effectiveKg * Math.cos(effectiveAlpha));
    // movementMotor.setVoltage(m_kgTrim);
  }

  @Override
  public void testArmRun(double speed) {
    movementMotor.setVoltage(speed);
  }

  @Override
  public void setTrim(double kgtrim, double alphatrim) {
    m_kgTrim = kgtrim;
    m_alphaTrim = alphatrim;
  }

  public void disable() {
    pid.setIAccum(0);
  }
}
