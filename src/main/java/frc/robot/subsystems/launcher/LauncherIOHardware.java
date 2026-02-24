package frc.robot.subsystems.launcher;

import static edu.wpi.first.units.Units.Amps;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.trajectory.TrapezoidProfile;

import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.Constants;

public class LauncherIOHardware implements LauncherIO {
  private final TalonFXConfiguration launcherMotorConfig =
      new TalonFXConfiguration()
          .withMotorOutput(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake))
          .withCurrentLimits(
              new CurrentLimitsConfigs()
                  .withStatorCurrentLimit(Amps.of(120))
                  .withStatorCurrentLimitEnable(true));
  private final TalonFX launcherMotor;
  private final SparkMax turnMotor =
      new SparkMax(Constants.launcherConstants.launchMotorCanId, null);
  private final SparkMaxConfig turnConfig = new SparkMaxConfig();
  private final Slot0Configs launcherSlot0 = new Slot0Configs();

  public LauncherIOHardware() {
    launcherMotor = new TalonFX(Constants.launcherConstants.launchMotorCanId);
    launcherMotor.getConfigurator().apply(launcherMotorConfig);

    turnConfig.idleMode(IdleMode.kBrake);
    turnConfig.smartCurrentLimit(Constants.launcherConstants.CurrentLimit);

    turnMotor.configure(turnConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    launcherSlot0.kP = 2.4;
    launcherSlot0.kI = 0;
    launcherSlot0.kD = 0.1;

    launcherMotor.getConfigurator().apply(launcherSlot0);
  }

  @Override
  public void setRadPerS(double RPS) {
    launcherMotor.setControl(new VelocityVoltage(RPS/(2*Math.PI)).withSlot(0));
  }

  @Override
  public void pointAt(double Angle) {
    
  }
}
