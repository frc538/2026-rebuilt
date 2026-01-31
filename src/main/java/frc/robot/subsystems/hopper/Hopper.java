package frc.robot.subsystems.hopper;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

public class Hopper extends SubsystemBase {

  HopperIO io;
  HopperIOInputsAutoLogged inputs = new HopperIOInputsAutoLogged();

  public Hopper(HopperIO IO) {
    io = IO;
  }

  public Command HopperON() {
    return run(
        () -> {
          io.SpindexSpeedCommand(Constants.Hopper.SpindexSpeed);
          io.FeedSpeedCommand(Constants.Hopper.FeedSpeed);
        });
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Hopper", inputs);
  }
}
