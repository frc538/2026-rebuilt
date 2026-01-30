package frc.robot.subsystems.Intake;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

public class IntakeIOSparkMax implements IntakeIO {

  private final SparkMax movementMotor = new SparkMax(1, MotorType.kBrushless);
  private final SparkFlex rotato = new SparkFlex(2, MotorType.kBrushless);

  private final RelativeEncoder encoder = movementMotor.getEncoder();
  private final SparkClosedLoopController pid = movementMotor.getClosedLoopController();

  public IntakeIOSparkMax() {
    SparkMaxConfig config = new SparkMaxConfig();
    double radiansPerRotation = 2 * Math.PI;

    config.encoder.positionConversionFactor(radiansPerRotation);
    config.encoder.velocityConversionFactor(radiansPerRotation / 60.0);

    config.closedLoop.p(0.1).i(0.0).d(0.01).outputRange(-1.0, 1.0);

    movementMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.positionRad = encoder.getPosition();
  }

  @Override
  public void runIntake(double speed) {
    rotato.set(speed);
  }

  @Override
  public void setVortexVelocity(double radiansPerSecond) {
    rotato.getClosedLoopController().setSetpoint(radiansPerSecond, ControlType.kVelocity);
  }

  @Override
  public void setIntakePosition(double radians) {
    pid.setSetpoint(radians, ControlType.kPosition);
  }
}
