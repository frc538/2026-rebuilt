package frc.robot.subsystems.launcher;

import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

public class TurretEncoder {
  CANcoder can1;
  CANcoder can2;

  MagnetSensorConfigs can1MagnetSensorConfigs;
  MagnetSensorConfigs can2MagnetSensorConfigs;

  public TurretEncoder() {
    can1 = new CANcoder(Constants.CanIds.turretEncoder23t);
    can2 = new CANcoder(Constants.CanIds.turretEncoder24t);

    can1MagnetSensorConfigs = new MagnetSensorConfigs();
    can2MagnetSensorConfigs = new MagnetSensorConfigs();

    can1.getConfigurator().refresh(can1MagnetSensorConfigs);
    can2.getConfigurator().refresh(can2MagnetSensorConfigs);
  }

  double read() {
    double encoder1Rotations = can1.getAbsolutePosition().getValueAsDouble();
    double encoder2Rotations = can2.getAbsolutePosition().getValueAsDouble();

    Logger.recordOutput("Turret/encoder 1 rotations", encoder1Rotations);
    Logger.recordOutput("Turret/encoder 2 rotations", encoder2Rotations);

    for (int i = 0; i < Constants.turretConstants.gear2Teeth; i++) {
      double a1 = (i + encoder1Rotations) * Constants.turretConstants.gear1Ratio;
      for (int j = 0; j < Constants.turretConstants.gear1Teeth; j++) {
        double a2 = (i + encoder2Rotations) * Constants.turretConstants.gear2Ratio;
        if (Math.abs(a1 - a2) < Constants.turretConstants.minDistance) {
          double turretAngle = (a1 + a2) / 2 - Constants.turretConstants.halfTravel;
          turretAngle = (turretAngle * (Math.PI * 2)) + Math.PI;
          Logger.recordOutput("Turret/position", turretAngle);
          Logger.recordOutput("Turret/encoder read valid", true);
          Logger.recordOutput("Turret/23t Rotations", i);
          Logger.recordOutput("Turret/24t Rotations", j);
          Logger.recordOutput("Turret/A1", a1);
          Logger.recordOutput("Turret/A2", a2);
          Logger.recordOutput("Turret/Abs of A1 - A2", Math.abs(a1 - a2));
          return turretAngle;
        }
      }
    }
    Logger.recordOutput("Turret/encoder read valid", false);
    return 0;
  }

  public Command calibrate() {
    return Commands.runOnce(
        () -> {
          double encoder1Rotations = can1.getAbsolutePosition().getValueAsDouble();
          double encoder2Rotations = can2.getAbsolutePosition().getValueAsDouble();

          // double encoder1Offset = can1.getConfigurator().refresh(can1Config);
          double EncoderRaw = encoder1Rotations - can1MagnetSensorConfigs.MagnetOffset;
          double newEncoder1Offset = 0 - EncoderRaw;

          if (newEncoder1Offset < 0) {
            newEncoder1Offset = newEncoder1Offset + 1;
          }

          EncoderRaw = encoder2Rotations - can2MagnetSensorConfigs.MagnetOffset;
          double newEncoder2Offset = 0.5 - EncoderRaw;

          if (newEncoder2Offset < 0) {
            newEncoder2Offset = newEncoder2Offset + 1;
          }

          can1MagnetSensorConfigs.MagnetOffset = newEncoder1Offset;
          can2MagnetSensorConfigs.MagnetOffset = newEncoder2Offset;

          can1MagnetSensorConfigs.AbsoluteSensorDiscontinuityPoint = 1;
          can2MagnetSensorConfigs.AbsoluteSensorDiscontinuityPoint = 1;

          can1.getConfigurator().apply(can1MagnetSensorConfigs);
          can2.getConfigurator().apply(can2MagnetSensorConfigs);

          // System.out.printf("Encoder 1 Bias = %f\n", - encoder1Rotations);
          // System.out.printf("Encoder 2 Bias = %f\n", 180 - encoder2Rotations);
        });
  }
}
