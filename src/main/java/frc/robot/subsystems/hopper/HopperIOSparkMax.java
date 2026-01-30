package frc.robot.subsystems.hopper;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkRelativeEncoder;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;

public class HopperIOSparkMax implements HopperIO {
  MotorController Spindex = null;
  MotorController Feed = null;
  private final SparkMax fdSparkMax;
  private final SparkMax sdSparkMax;

  SparkMaxConfig mConfig = new SparkMaxConfig();

  SparkRelativeEncoder mEncoder;

  public HopperIOSparkMax(int feedcanid, int spindexcanid) { // feed can id
    sdSparkMax = new SparkMax(feedcanid, MotorType.kBrushless);
    fdSparkMax = new SparkMax(spindexcanid, MotorType.kBrushless);

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
  }

  public void FeedSpeedCommand(double speed) {
    Feed.set(speed);
  }

  public void SpindexSpeedCommand(double speed) {
    Spindex.set(speed);
  }
}
