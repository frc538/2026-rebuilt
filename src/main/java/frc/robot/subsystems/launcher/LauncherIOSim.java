package frc.robot.subsystems.launcher;

import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class LauncherIOSim implements LauncherIO {
  private FlywheelSim flywheelSim;

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
  private static final double fuelMass = 0.2268; // kg
  private static final double fuelRadius = 0.075; // meters
  private static final double kFuelMomentOfInertia = 2.0 / 5.0 * fuelMass * fuelRadius * fuelRadius;
  private static final double kWheelRadius = 0.050165; // m

  private static final double hoodFraction = 0.125; // 1/8th of the wheel to accelerate
  private static final double hoodDistance =
      2.0 * 3.14 * (kWheelRadius + fuelRadius) * hoodFraction;

  // The plant holds a state-space model of our flywheel. This system has the following properties:
  //
  // States: [velocity], in radians per second.
  // Inputs (what we can "put in"): [voltage], in volts.
  // Outputs (what we can measure): [velocity], in radians per second.
  private final LinearSystem<N1, N1, N1> m_flywheelPlant =
      LinearSystemId.createFlywheelSystem(
          DCMotor.getKrakenX60(1), kFlywheelMomentOfInertia, kFlywheelGearing);

  public LauncherIOSim() {
    flywheelSim = new FlywheelSim(m_flywheelPlant, DCMotor.getKrakenX60(1));
  }

  public void updateInputs(LauncherIOInputs inputs) {
    // Update the sim model based on most recent commands. The standard loop time is 20ms.
    flywheelSim.update(0.020);

    // Compensate for a piece of fuel if required
    if (isFuel) {
      double wwi = flywheelSim.getAngularVelocityRadPerSec();
      double wwf =
          kFlywheelMomentOfInertia
              * wwi
              / (kWheelRadius
                      * kWheelRadius
                      * 0.25
                      * (fuelMass + (kFuelMomentOfInertia / (fuelRadius * fuelRadius)))
                  + kFlywheelMomentOfInertia);

      System.out.println(String.format("Launch, wwi=%f, wwf=%f", wwi, wwf));
      flywheelSim.setAngularVelocity(wwf);

      // Stop worrying about the fuel anymore, it's gone baby

      isFuel = false;

      // TODO: Generate a fuel simulated projectile with the right parameters

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
