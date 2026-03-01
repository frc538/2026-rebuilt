package frc.robot.subsystems.launcher;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants;

public class LauncherIOSim implements LauncherIO {
  private FlywheelSim flywheelSim;

  // MOI calculated from prototype launcher
  private static final double kFlywheelMomentOfInertia = 0.0004926; // kg * m^2

  // Reduction between motors and encoder, as output over input. If the flywheel spins slower than
  // the motors, this number should be greater than one.
  private static final double kFlywheelGearing = 1.0;

  // Fuel launching state
  private static boolean isFuel = false;
  private static final double fuelMass = 0.2268; // kg
  private static final double fuelRadius = 0.075; // meters
  private static final double kFuelMomentOfInertia = 2.0 / 5.0 * fuelMass * fuelRadius * fuelRadius;
  private static final double kWheelRadius = 0.050165; // m

  // The plant holds a state-space model of our flywheel. This system has the following properties:
  //
  // States: [velocity], in radians per second.
  // Inputs (what we can "put in"): [voltage], in volts.
  // Outputs (what we can measure): [velocity], in radians per second.
  private final LinearSystem<N1, N1, N1> m_flywheelPlant =
      LinearSystemId.createFlywheelSystem(
          DCMotor.getKrakenX60(1), kFlywheelMomentOfInertia, kFlywheelGearing);

  // Turret azimuth state
  private SingleJointedArmSim turretSim;
  private static double commandedAzimuth = 0.0;
  private static final double kTurretMomentOfInertia = 0.005; // kg * m^2
  private static final double kTurretGearing = 50; // Total guess at the gearing for motor to turret
  private SparkMax m_turretSparkMax =
      new SparkMax(Constants.launcherConstants.launchMotorCanId, MotorType.kBrushless);
  private SparkMaxConfig m_turretConfig = new SparkMaxConfig();
  private SparkMaxSim m_turretSparkMaxSim = new SparkMaxSim(m_turretSparkMax, DCMotor.getNEO(1));
  private final SparkClosedLoopController turretClosedLoopController;

  private final LinearSystem<N2, N1, N2> m_turretPlant =
      LinearSystemId.createSingleJointedArmSystem(
          DCMotor.getNEO(1), kTurretMomentOfInertia, kTurretGearing);

  public LauncherIOSim() {
    flywheelSim = new FlywheelSim(m_flywheelPlant, DCMotor.getKrakenX60(1));

    m_turretConfig.idleMode(IdleMode.kBrake).inverted(false).smartCurrentLimit(50);
    m_turretConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(0.1, 0, 0)
        .outputRange(-1, 1);

    m_turretSparkMax.configure(
        m_turretConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    turretClosedLoopController = m_turretSparkMax.getClosedLoopController();

    turretSim =
        new SingleJointedArmSim(
            m_turretPlant,
            DCMotor.getNEO(1),
            kTurretGearing,
            0.25,
            -3 * Math.PI / 4,
            3 * Math.PI / 4,
            false,
            0.0);
  }

  @Override
  public void updateInputs(LauncherIOInputs inputs) {
    // Update the sim model based on most recent commands. The standard loop time is 20ms.
    flywheelSim.update(0.020);

    turretSim.update(0.020);

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
  }

  @Override
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
  }

  @Override
  public void simLaunch() {
    launchFuel();
  }

  @Override
  public void pointAt(double angle) {
    turretClosedLoopController.setSetpoint(Math.toRadians(angle), ControlType.kPosition);
  }
}
