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

  // MOI calculated from prototype launcher
  private static final double kFlywheelMomentOfInertia = 0.0004926; // kg * m^2

  // Reduction between motors and encoder, as output over input. If the flywheel spins slower than
  // the motors, this number should be greater than one.
  private static final double kFlywheelGearing = 1.0;

  // Fuel launching state
  private static boolean isFuel = false;
  private static double fuelPosition = 0.0; // radians
  private static double fuelLinearVelocity = 0.0; // m/s
  private static double fuelRotationalVelocity = 0.0; // radians/sec
  private static final double fuelMass = 0.5 * 0.4535924; // kg
  private static final double fuelRadius = 0.075; // meters
  private static final double kFuelMomentOfInertia = 2 / 5 * fuelMass * fuelRadius * fuelRadius;
  private static final double kWheelRadius = 0.050165; // m

  private static final double hoodFraction = 0.125; // 1/8th of the wheel to accelerate
  private static final double hoodDistance = 2 * 3.14 * (kWheelRadius + fuelRadius) * hoodFraction;

  // The plant holds a state-space model of our flywheel. This system has the following properties:
  //
  // States: [velocity], in radians per second.
  // Inputs (what we can "put in"): [voltage], in volts.
  // Outputs (what we can measure): [velocity], in radians per second.
  private final LinearSystem<N1, N1, N1> m_flywheelPlant =
      LinearSystemId.createFlywheelSystem(
          DCMotor.getKrakenX60(1), kFlywheelMomentOfInertia, kFlywheelGearing);

  public FlywheelIOSim() {
    flywheelSim = new FlywheelSim(m_flywheelPlant, DCMotor.getKrakenX60(1));
  }

  public void updateInputs(FlywheelIOInputs inputs) {
    // Update the sim model based on most recent commands. The standard loop time is 20ms.
    flywheelSim.update(0.020);

    // Compensate for a piece of fuel if required
    if (isFuel) {
      // Calculate tangential velocity of the wheel (m/s)
      double vw = flywheelSim.getAngularVelocityRadPerSec() * kWheelRadius;
      // Calculate the angular velocity of the fuel
      double wf = vw / fuelRadius;
      // Calculate rotational acceleration of the fuel assuming no slippage
      double ap = (wf - fuelRotationalVelocity) / 0.02;
      // Calculate torque on the fuel
      double Tp = kFuelMomentOfInertia * ap;
      // Calculate force on the fuel
      double fp = Tp / fuelRadius;
      // Calculate torque on the wheel
      double Tw = -fp * kWheelRadius;
      // Calculate acceleration impact on the wheel
      double aw = Tw / kFlywheelMomentOfInertia;
      // Calculate change in velocity of the wheel
      double vwNew = vw + aw * 0.02;

      // Set the outputs
      flywheelSim.setAngularVelocity(vwNew);
      fuelRotationalVelocity = fuelRotationalVelocity + ap * 0.02;
      fuelLinearVelocity = fuelRotationalVelocity * fuelRadius;
      fuelPosition = fuelPosition + fuelLinearVelocity * 0.02;

      // Stop worrying about the fuel anymore, it's gone baby
      if (fuelPosition > hoodDistance) {
        isFuel = false;

        // TODO: Generate a fuel simulated projectile with the right parameters
      }
    }

    inputs.rpm = flywheelSim.getAngularVelocityRPM();
    inputs.projectileRotationalSpeed = fuelRotationalVelocity;
    inputs.projectileSpeed = fuelLinearVelocity;
  }

  public void setVoltage(double voltage) {
    // In this method, we update our simulation of what our arm is doing
    // First, we set our "inputs" (voltages)
    flywheelSim.setInput(voltage);
  }

  private void launchFuel() {
    // Start a piece of fuel on the flywheel-hood system
    if (isFuel == true) {
      return;
    }

    isFuel = true;
    fuelPosition = 0.0;
    fuelLinearVelocity = 0.0;
    fuelRotationalVelocity = 0.0;
  }

  public void simLaunch() {
    launchFuel();
  }
}
