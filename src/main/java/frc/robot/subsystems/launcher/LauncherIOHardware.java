package frc.robot.subsystems.launcher;

import static edu.wpi.first.units.Units.Amps;
import static frc.robot.Constants.launcherConstants.turnD;
import static frc.robot.Constants.launcherConstants.turnI;
import static frc.robot.Constants.launcherConstants.turnP;

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
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants;

public class LauncherIOHardware implements LauncherIO {
  private FlywheelSim flywheelSim;
  private SingleJointedArmSim turretSim;
  private final TalonFXConfiguration launcherMotorConfig;
  private final TalonFX launcherMotor;
  private final SparkMax turnMotor =
      new SparkMax(Constants.launcherConstants.turnMotorCanId, MotorType.kBrushless);
  private final SparkMaxConfig turnConfig = new SparkMaxConfig();
  private final Slot0Configs launcherSlot0 = new Slot0Configs();
  private final SparkClosedLoopController turnController;
  private final SparkRelativeEncoder turnEncoder;
  private static double fuelLinearVelocity = 0.0; // m/s
  private static double fuelRotationalVelocity = 0.0; // radians/sec

  public LauncherIOHardware() {
    launcherMotor = new TalonFX(Constants.launcherConstants.launchMotorCanId);
    launcherMotorConfig =
        new TalonFXConfiguration()
            .withMotorOutput(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake))
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(Amps.of(120))
                    .withStatorCurrentLimitEnable(true))
            .withFeedback(
                new FeedbackConfigs()
                    .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor))
            .withClosedLoopGeneral(new ClosedLoopGeneralConfigs().withContinuousWrap(true));

    launcherMotor.getConfigurator().apply(launcherMotorConfig);

    turnConfig.idleMode(IdleMode.kBrake);
    turnConfig.smartCurrentLimit(Constants.launcherConstants.CurrentLimit);
    turnConfig.closedLoop.pid(turnP, turnI, turnD).feedbackSensor(FeedbackSensor.kPrimaryEncoder);

    turnMotor.configure(turnConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    turnController = turnMotor.getClosedLoopController();

    turnEncoder = (SparkRelativeEncoder) turnMotor.getEncoder();

    launcherSlot0.kP = 0.3;
    launcherSlot0.kI = 0;
    launcherSlot0.kD = 0.0;

    launcherMotor.getConfigurator().apply(launcherSlot0);
  }

  @Override
  public void updateInputs(LauncherIOInputs inputs) {

    inputs.rpm = flywheelSim.getAngularVelocityRPM();
    inputs.projectileRotationalSpeed = fuelRotationalVelocity;
    inputs.projectileSpeed = fuelLinearVelocity;

    inputs.turretAngle = Math.toDegrees(turretSim.getAngleRads());
    inputs.turretSpeed = Math.toDegrees(turretSim.getVelocityRadPerSec());

    inputs.launcherMotorVoltage = launcherMotor.getMotorVoltage().getValueAsDouble();
    inputs.launcherStatorCurrent = launcherMotor.getStatorCurrent().getValueAsDouble();
    inputs.launcherTorqueCurrent = launcherMotor.getTorqueCurrent().getValueAsDouble();
    inputs.launcherAcceleration = launcherMotor.getAcceleration().getValueAsDouble();
    inputs.launcherClosedLoopError = launcherMotor.getClosedLoopError().getValueAsDouble();
    inputs.launcherVelocity = launcherMotor.getVelocity().getValueAsDouble() / (2 * Math.PI);
    inputs.launcherSupplyCurrent = launcherMotor.getSupplyCurrent().getValueAsDouble();
    inputs.launcherSupplyVoltage = launcherMotor.getSupplyVoltage().getValueAsDouble();

    inputs.turnMotorAppliedOutput = turnMotor.getAppliedOutput();
    inputs.turnMotorBusVoltage = turnMotor.getBusVoltage();
    inputs.turnMotorOutputCurrent = turnMotor.getOutputCurrent();

    inputs.turnEncoderVelocity = turnEncoder.getVelocity();
    inputs.turnEncoderPosition = turnEncoder.getPosition();
  }

  @Override
  public void setRadPerS(double RPS) {
    launcherMotor.setControl(new VelocityVoltage(RPS / (2 * Math.PI)).withSlot(0));
  }

  @Override
  public void pointAt(double radians) {
    turnController.setSetpoint(radians, ControlType.kPosition, ClosedLoopSlot.kSlot0);
  }

  @Override
  public void testTurn(double voltage) {
    turnMotor.setVoltage(voltage);
  }

  @Override
  public void testFlyWheelTurn(double voltage) {
    launcherMotor.setVoltage(voltage);
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
