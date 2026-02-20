package frc.robot.subsystems.launcher;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkRelativeEncoder;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.Constants;

public class LauncherIOHardware implements LauncherIO {
  private final SparkMax turnMotor;
  private final SparkMax yeetMotor;
  private final SparkClosedLoopController pid;
  SparkMaxConfig turnConf = new SparkMaxConfig();
  SparkMaxConfig yeetConf = new SparkMaxConfig();
  SparkRelativeEncoder turnCoder;
  SparkRelativeEncoder yeetCoder;

  public LauncherIOHardware(int yeetID, int turnID) {
    turnMotor = new SparkMax(yeetID, MotorType.kBrushless);
    yeetMotor = new SparkMax(turnID, MotorType.kBrushless);
    turnCoder = (SparkRelativeEncoder) turnMotor.getEncoder();
    yeetCoder = (SparkRelativeEncoder) yeetMotor.getEncoder();
    pid = turnMotor.getClosedLoopController();
    turnConf
        .idleMode(IdleMode.kBrake)
        // .smartCurrentLimit(Constants.ArmConstants.CurrentLimit)
        .inverted(false);
    yeetConf
        .idleMode(IdleMode.kBrake)
        // .smartCurrentLimit(Constants.ArmConstants.CurrentLimit)
        .inverted(false);
    turnConf
        .encoder
        .positionConversionFactor(Constants.Hopper.FDConversionFactor)
        .velocityConversionFactor(Constants.Hopper.FDConversionFactor);
    yeetConf
        .encoder
        .positionConversionFactor(Constants.Hopper.SDConversionFactor)
        .velocityConversionFactor(Constants.Hopper.SDConversionFactor);
    turnConf.closedLoop.p(0.1).i(0.0).d(0.01).outputRange(-1.0, 1.0);

    turnMotor.configure(
        turnConf, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    yeetMotor.configure(
        yeetConf, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  public void updateInputs(LauncherIOInputs inputs) {
    // Launcher Flywheel
    inputs.rpm = yeetMotor.get();
    inputs.projectileSpeed = 0;
    inputs.projectileRotationalSpeed = 0;

    // Turret (will hopefully get named yeeter mc yeeterson)
    inputs.turretAngle = 0;
    inputs.turretSpeed = 0;
  }

  @Override
  // Need to update this IO interface to add an RPM or rad/s function that sets the flywheel speed
  public void setRadPerS(double rps) {
    turnMotor.set(rps);
  }

  @Override
  // Set launcher voltage
  public void setVoltage(double voltage) {
    yeetMotor.setVoltage(voltage);
  }

  // Command to point the launcher at an angle in degrees
  @Override
  public void pointAt(double angle) {
    pid.setSetpoint(angle, ControlType.kPosition);
  }

  // Sets the minimum and maximum accepted angles in degrees
  @Override
  public void setLockout(double minAngle, double maxAngle) {}

  // Simulate feeding a projectile into the launcher
  @Override
  public void simLaunch() {}
}
