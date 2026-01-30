package frc.robot.subsystems.Intake;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  private final SparkMax movementMotor = new SparkMax(1, MotorType.kBrushless);
  private final SparkFlex Rotato = new SparkFlex(2, MotorType.kBrushless);

  private final RelativeEncoder m_encoder = movementMotor.getEncoder();

  private final SparkClosedLoopController m_pidController = movementMotor.getClosedLoopController();

  public Intake() {
    SparkMaxConfig config = new SparkMaxConfig();
    double radiansPerRotation = 2*Math.PI;

    config.encoder.positionConversionFactor(2*Math.PI);
    config.encoder.velocityConversionFactor(radiansPerRotation/60.0);

    config.closedLoop
        .p(0.1)
        .i(0.0)
        .d(0.01)
        .outputRange(-1.0, 1.0);

    movementMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
}

  public double getPosInRadians(){
    return m_encoder.getPosition();
  }

  public void runIntake(double speed) {
    Rotato.set(speed);
  }

  public void setVortexVelocity(double radiansPerSecond) {
    Rotato.getClosedLoopController().setSetpoint(radiansPerSecond, ControlType.kVelocity);
  }

  public void IntakeExtend_Retract(double radians) {
    m_pidController.setSetpoint(radians, ControlType.kPosition);

    }
  }
