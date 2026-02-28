package frc.robot.subsystems.hopper;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

public class Hopper extends SubsystemBase {

  HopperIO io;
  HopperIOInputsAutoLogged inputs = new HopperIOInputsAutoLogged();
  Boolean HopperActivated = false;

  public Hopper(HopperIO IO) {
    io = IO;
  }

  public Command HopperON() {
    return run(
        () -> {
          io.SpindexSpeedCommand(Constants.Hopper.SpindexSpeed);
          io.FeedSpeedCommand(Constants.Hopper.FeedSpeed);
          HopperActivated = true;
        });
  }

  public void HopperToggler(double SpinSpeed, double FeedSpeed) {
    if (HopperActivated) {
      // HopperOFF();
      io.SpindexSpeedCommand(0);
      io.FeedSpeedCommand(0);
      HopperActivated = false;
    } else {
      // HopperON();
      io.SpindexSpeedCommand(SpinSpeed);
      io.FeedSpeedCommand(FeedSpeed);
      HopperActivated = true;
    }
  }

  public Command HopperToggle() {
    return runOnce(
        () -> {
          HopperToggler(Constants.Hopper.SpindexSpeed, Constants.Hopper.FeedSpeed);
        });
  }

  public Command testFeed() {
    return run(
    () -> {
    io.FeedSpeedCommand(Constants.Hopper.TestFeedSpeed);
    }).finallyDo(() -> io.FeedSpeedCommand(0));
  }
  public Command testSpindex() {
    return run(
      () -> {
    io.SpindexSpeedCommand(Constants.Hopper.TestSpindexSpeed);
      }).finallyDo(() -> io.SpindexSpeedCommand(0));
  }


  public Command HopperOFF() {
    return run(
        () -> {
          io.SpindexSpeedCommand(0);
          io.FeedSpeedCommand(0);
          HopperActivated = false;
        });
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Hopper", inputs);
    Logger.recordOutput("Hopper/Activated", HopperActivated);
  }
}
