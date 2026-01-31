package frc.robot.subsystems.hopper;

// import com.revrobotics.spark.SparkBase.PersistMode;
// import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkRelativeEncoder;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

public class HopperIOSparkMax implements HopperIO {

  private final SparkMax fdSparkMax;
  private final SparkMax sdSparkMax;

  SparkMaxConfig mConfig = new SparkMaxConfig();

  SparkRelativeEncoder mEncoder;

  public HopperIOSparkMax(int feedcanid, int spindexcanid) { // feed can id
    sdSparkMax = new SparkMax(feedcanid, MotorType.kBrushless);
    fdSparkMax = new SparkMax(spindexcanid, MotorType.kBrushless);
    mConfig
        .idleMode(IdleMode.kBrake)
        // .smartCurrentLimit(Constants.ArmConstants.CurrentLimit)
        .inverted(false);

    // sdSparkMax.configure(mConfig, ResetMode.kResetSafeParameters,
    // PersistMode.kPersistParameters);
    // fdSparkMax.configure(mConfig, ResetMode.kResetSafeParameters,
    // PersistMode.kPersistParameters);
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
    fdSparkMax.set(speed);
  }

  public void SpindexSpeedCommand(double speed) {
    sdSparkMax.set(speed);
  }
}
