package frc.robot.subsystems.hopper;

import org.littletonrobotics.junction.AutoLog;

public interface HopperIO {

  @AutoLog
  public static class HopperIOInputs {
    public double SDBusVoltage = 0.0;
    public double SDOutputCurrent = 0.0;
    public double SDMotorTemperature = 0.0;
    public double SDAppliedOutput = 0.0;
    public double FDBusVoltage = 0.0;
    public double FDOutputCurrent = 0.0;
    public double FDMotorTemperature = 0.0;
    public double FDAppliedOutput = 0.0;
  }

  public default void updateInputs(HopperIOInputs inputs) {}

  public default void FeedSpeedCommand(double speed) {}

  public default void SpindexSpeedCommand(double speed) {}
}
