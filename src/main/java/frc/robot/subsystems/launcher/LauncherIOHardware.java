package frc.robot.subsystems.launcher;

import static edu.wpi.first.units.Units.Amps;
import static frc.robot.Constants.launcherConstants.TurnPositionConversionFactor;
import static frc.robot.Constants.launcherConstants.TurnVelocityConversionFactor;
import static frc.robot.Constants.launcherConstants.turnD;
import static frc.robot.Constants.launcherConstants.turnI;
import static frc.robot.Constants.launcherConstants.turnP;
import static frc.robot.Constants.launcherConstants.turretCalibrationOffset;

import com.ctre.phoenix6.configs.ClosedLoopGeneralConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkRelativeEncoder;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

public class LauncherIOHardware implements LauncherIO {
  private FlywheelSim flywheelSim;
  private SingleJointedArmSim turretSim;
  private final TalonFXConfiguration launcherMotorConfig;
  private final TalonFX launcherMotor;
  private final SparkMax turnMotor =
      new SparkMax(Constants.CanIds.turnMotorCanId, MotorType.kBrushless);
  private final SparkMaxConfig turnConfig = new SparkMaxConfig();
  private final Slot0Configs launcherSlot0 = new Slot0Configs();
  private final SparkClosedLoopController turnController;
  private final SparkRelativeEncoder turnEncoder;
  AnalogPotentiometer m_potentiometer = new AnalogPotentiometer(3, 2 * Math.PI, 0);

  public LauncherIOHardware() {
    launcherMotor = new TalonFX(Constants.CanIds.launchMotorCanId);
    launcherMotorConfig =
        new TalonFXConfiguration()
            .withMotorOutput(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake))
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(Amps.of(Constants.CurrentLimit.turretLimit))
                    .withStatorCurrentLimitEnable(true))
            .withFeedback(
                new FeedbackConfigs()
                    .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor))
            .withClosedLoopGeneral(new ClosedLoopGeneralConfigs().withContinuousWrap(true));

    launcherMotor.getConfigurator().apply(launcherMotorConfig);

    launcherSlot0.kP = 0.3;
    launcherSlot0.kI = 0.01;
    launcherSlot0.kD = 0.0;

    launcherMotor.getConfigurator().apply(launcherSlot0);

    turnConfig
        .encoder
        .positionConversionFactor(TurnPositionConversionFactor)
        .velocityConversionFactor(TurnVelocityConversionFactor);

    turnConfig.smartCurrentLimit(Constants.CurrentLimit.turnLimit);

    turnConfig.idleMode(IdleMode.kBrake);
    turnConfig
        .closedLoop
        .pid(turnP, turnI, turnD)
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .outputRange(-1, 1)
        .positionWrappingEnabled(false);

    turnMotor.configure(turnConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    turnController = turnMotor.getClosedLoopController();

    turnEncoder = (SparkRelativeEncoder) turnMotor.getEncoder();
  }

  @Override
  public void updateInputs(LauncherIOInputs inputs) {
    inputs.launcherMotorVoltage = launcherMotor.getMotorVoltage().getValueAsDouble();
    inputs.launcherStatorCurrent = launcherMotor.getStatorCurrent().getValueAsDouble();
    inputs.launcherTorqueCurrent = launcherMotor.getTorqueCurrent().getValueAsDouble();

    // Give accel in rad / sec / sec
    inputs.launcherAcceleration =
        Units.rotationsToRadians(launcherMotor.getAcceleration().getValueAsDouble());

    // Give error in radians per second
    inputs.launcherClosedLoopError =
        Units.rotationsToRadians(launcherMotor.getClosedLoopError().getValueAsDouble());

    // getVelocity returns rotations per second
    // Convert rotations per second to radians per second
    inputs.launcherVelocity =
        Units.rotationsToRadians(launcherMotor.getVelocity().getValueAsDouble());

    inputs.launcherSupplyCurrent = launcherMotor.getSupplyCurrent().getValueAsDouble();
    inputs.launcherSupplyVoltage = launcherMotor.getSupplyVoltage().getValueAsDouble();

    inputs.turnMotorAppliedOutput = turnMotor.getAppliedOutput();
    inputs.turnMotorBusVoltage = turnMotor.getBusVoltage();
    inputs.turnMotorOutputCurrent = turnMotor.getOutputCurrent();

    inputs.turnEncoderVelocity = turnEncoder.getVelocity();
    inputs.turnEncoderPosition = turnEncoder.getPosition();

    inputs.turnPotentiometer = m_potentiometer.get();
  }

  @Override
  public void TurretDisable() {
    turnController.setIAccum(0);
  }

  @Override
  public void TurretCalibrate(double rads) {
    turnEncoder.setPosition(rads + turretCalibrationOffset);
  }

  VelocityVoltage launchCommand = new VelocityVoltage(0);
  double launcherKV = .019;

  @Override
  public void setRadPerS(double RPS) {
    Logger.recordOutput("Launcher/CommandedRadPerSec", RPS);
    // Radians per second to rotations per second
    launchCommand.FeedForward = launcherKV * RPS;
    RPS = Units.radiansToRotations(RPS);
    launchCommand.Velocity = RPS;

    launcherMotor.setControl(launchCommand);
  }

  @Override
  public void pointAt(double radians, double radiansPerSec) {
    double FFTurret = radiansPerSec * Constants.launcherConstants.turnVelocityFFGain;
    Logger.recordOutput("Launcher/FFTurret", FFTurret);
    var result =
        turnController.setSetpoint(radians, ControlType.kPosition, ClosedLoopSlot.kSlot0, FFTurret);
    Logger.recordOutput("Launcher/turnControlResult", result);

    // turnController.setSetpoint(
    //    radiansPerSec, ControlType.kVelocity, ClosedLoopSlot.kSlot0, FFTurret);
  }

  @Override
  public void testTurn(double voltage) {
    Logger.recordOutput("Launcher/testTurnVoltage", voltage);
    turnMotor.setVoltage(voltage);
  }

  @Override
  public void testFlyWheelTurn(double voltage) {
    launcherMotor.setVoltage(voltage);
    Logger.recordOutput("Launcher/flywheelVoltageCmd", voltage);
  }

  @Override
  public void invertTurn() {
    turnConfig.inverted(true);
  }

  @Override
  public void deinvertTurn() {
    turnConfig.inverted(false);
  }

  public void resetEncoder() {
    turnEncoder.setPosition(0);
  }
}
