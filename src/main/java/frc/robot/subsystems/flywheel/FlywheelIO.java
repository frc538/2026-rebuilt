public interface FlywheelIO {
  @AutoLog
  public static class FlywheelIOInputs {
    public double rpm;
  }

  public default void updateInputs(FlywheelIOInputs inputs) {}

  public default void setRPM(double rpm) {}
}
