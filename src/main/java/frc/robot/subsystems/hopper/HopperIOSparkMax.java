package frc.robot.subsystems.hopper;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkRelativeEncoder;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.Constants;

public class HopperIOSparkMax implements HopperIO {

  private final SparkMax fdSparkMax;
  private final SparkMax sdSparkMax;

  SparkMaxConfig sdConfig = new SparkMaxConfig();
SparkMaxConfig fdConfig = new SparkMaxConfig();

  SparkRelativeEncoder sdEncoder;
  SparkRelativeEncoder fdEncoder;

  public HopperIOSparkMax(int feedcanid, int spindexcanid) { // feed can id
    sdSparkMax = new SparkMax(feedcanid, MotorType.kBrushless);
    fdSparkMax = new SparkMax(spindexcanid, MotorType.kBrushless);
    sdEncoder = (SparkRelativeEncoder) sdSparkMax.getEncoder();
    fdEncoder = (SparkRelativeEncoder) fdSparkMax.getEncoder();
    
    fdConfig
        .idleMode(IdleMode.kBrake)
        // .smartCurrentLimit(Constants.ArmConstants.CurrentLimit)
        .inverted(false);
    sdConfig
        .idleMode(IdleMode.kBrake)
        // .smartCurrentLimit(Constants.ArmConstants.CurrentLimit)
        .inverted(false);
    fdConfig
        .encoder
        .positionConversionFactor(Constants.Hopper.FDConversionFactor)
        .velocityConversionFactor(Constants.Hopper.FDConversionFactor);
    sdConfig
        .encoder
        .positionConversionFactor(Constants.Hopper.SDConversionFactor)
        .velocityConversionFactor(Constants.Hopper.SDConversionFactor);

     sdSparkMax.configure(sdConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
     fdSparkMax.configure(fdConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  public void updateInputs(HopperIOInputs inputs) {
    inputs.FDAppliedOutput = fdSparkMax.getAppliedOutput();
    inputs.FDBusVoltage = fdSparkMax.getBusVoltage();
    inputs.FDOutputCurrent = fdSparkMax.getOutputCurrent();
    inputs.FDMotorTemperature = fdSparkMax.getMotorTemperature();
    inputs.SDAppliedOutput = sdSparkMax.getAppliedOutput();
    inputs.SDBusVoltage = sdSparkMax.getBusVoltage();
    inputs.SDOutputCurrent = sdSparkMax.getOutputCurrent();
    inputs.SDMotorTemperature = sdSparkMax.getMotorTemperature();

    inputs.SpindexRPM = sdEncoder.getVelocity();
    inputs.FeedRPM = fdEncoder.getVelocity();
  }

  public void FeedSpeedCommand(double percent) {
    fdSparkMax.set(percent);
  }

  public void SpindexSpeedCommand(double percent) {
    sdSparkMax.set(percent);
  }
}
