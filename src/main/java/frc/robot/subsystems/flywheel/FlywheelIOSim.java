package frc.robot.subsystems.flywheel;

import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class FlywheelIOSim implements FlywheelIO {
  private FlywheelSim flywheelSim;
  private final Encoder m_encoder = new Encoder(0, 1);

  private static final double kFlywheelMomentOfInertia = 0.00032; // kg * m^2

  // Reduction between motors and encoder, as output over input. If the flywheel spins slower than
  // the motors, this number should be greater than one.
  private static final double kFlywheelGearing = 1.0;

  // The plant holds a state-space model of our flywheel. This system has the following properties:
  //
  // States: [velocity], in radians per second.
  // Inputs (what we can "put in"): [voltage], in volts.
  // Outputs (what we can measure): [velocity], in radians per second.
  private final LinearSystem<N1, N1, N1> m_flywheelPlant =
      LinearSystemId.createFlywheelSystem(
          DCMotor.getNEO(2), kFlywheelMomentOfInertia, kFlywheelGearing);

  public FlywheelIOSim() {
    flywheelSim = new FlywheelSim(m_flywheelPlant, DCMotor.getNEO(2));
  }

  public void updateInputs(FlywheelIOInputs inputs) {
    // Update the sim model based on most recent commands. The standard loop time is 20ms.
    flywheelSim.update(0.020);

    inputs.rpm = flywheelSim.getAngularVelocityRPM();
  }

  public void setVoltage(double voltage) {
    // In this method, we update our simulation of what our arm is doing
    // First, we set our "inputs" (voltages)
    flywheelSim.setInput(voltage);
  }
}
