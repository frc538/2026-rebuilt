package frc.robot.subsystems.launcher;

import static edu.wpi.first.units.Units.Amps;
import static frc.robot.Constants.launcherConstants.TurnPositionConversionFactor;
import static frc.robot.Constants.launcherConstants.TurnVelocityConversionFactor;
import static frc.robot.Constants.launcherConstants.ks;
import static frc.robot.Constants.launcherConstants.turnD;
import static frc.robot.Constants.launcherConstants.turnI;
import static frc.robot.Constants.launcherConstants.turnP;
import static frc.robot.Constants.launcherConstants.turnVelocityFFGain;
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
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
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
  private SimpleMotorFeedforward turretFF;

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
        .positionWrappingEnabled(false)
        .iMaxAccum(0.25);

    turnMotor.configure(turnConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    turnController = turnMotor.getClosedLoopController();

    turnEncoder = (SparkRelativeEncoder) turnMotor.getEncoder();

    // We are setting ks to 0 here because the ks value will be applied manually
    turretFF = new SimpleMotorFeedforward(0.0, turnVelocityFFGain);
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

    inputs.iAccum = turnController.getIAccum();
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
    // In motor voltage
    double FFTurret = turretFF.calculate(radiansPerSec);
    Logger.recordOutput("Launcher/FFTurret", FFTurret);

    // In rotations error
    double turretError = (radians - turnEncoder.getPosition()) / (2 * Math.PI);

    // Emulate the sparkmax stuff
    double pCommand = turnP * turretError;
    double iCommand = turnController.getIAccum();
    double dCommand = 0; // Ignore this for now

    Logger.recordOutput("Launcher/pCommand", pCommand);
    Logger.recordOutput("Launcher/iCommand", iCommand);

    // Rotations to duty-cycle percentage feedback command
    double PID = pCommand + iCommand + dCommand;

    // In motor voltage
    double totalVoltageUncompensated = PID / turnMotor.getBusVoltage() + FFTurret;

    double CommandSgn = 0;
    if (Math.abs(totalVoltageUncompensated) > 0.0001) {
      CommandSgn = Math.signum(totalVoltageUncompensated);
    }

    // In motor voltage
    double armFFCommand = FFTurret + ks * CommandSgn;

    double estimatedTotalApplied = armFFCommand / turnMotor.getBusVoltage() + PID;

    var result =
        turnController.setSetpoint(
            radians, ControlType.kPosition, ClosedLoopSlot.kSlot0, armFFCommand);
    Logger.recordOutput("Launcher/turnControlResult", result);
    Logger.recordOutput("Launcher/armFFCommand", armFFCommand);
    Logger.recordOutput("Launcher/PID (emulated)", PID);
    Logger.recordOutput("Launcher/turretError", turretError);
    Logger.recordOutput("Launcher/estimatedTotalApplied", estimatedTotalApplied);

    // turnController.setSetpoint(
    //    radiansPerSec, ControlType.kVelocity, ClosedLoopSlot.kSlot0, FFTurret);
  }

  @Override
  public void updateRobotInfo(Pose2d robotPose, ChassisSpeeds robotVelocity, double rotationRate) {}

  @Override
  public void turretVoltage(double voltage) {
    Logger.recordOutput("Launcher/turretVoltage", voltage);
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
