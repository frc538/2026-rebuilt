package frc.robot.subsystems.launcher;

import static edu.wpi.first.units.Units.Amps;

import com.ctre.phoenix6.configs.ClosedLoopGeneralConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.sim.SparkRelativeEncoderSim;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkRelativeEncoder;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.util.CircularBuffer;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants;
import java.util.Arrays;
import org.littletonrobotics.junction.Logger;

public class LauncherIOSim implements LauncherIO {
  private FlywheelSim flywheelSim;
  private StructArrayPublisher<Translation3d> fuelPublisher;
  private CircularBuffer<Translation3d[]> fuelTrajectory = new CircularBuffer<>(30);
  private CircularBuffer<Integer> trajectoryAge = new CircularBuffer<>(30);
  private Translation3d[] theTrajectories = new Translation3d[30 * 30];
  private int ageValue = 0;
  private final int maxTrajectoryAge = 2 * 50; // 2 seconds
  private Pose2d robotPose = new Pose2d();
  private ChassisSpeeds robotVelocity;

  private final TalonFXConfiguration launcherMotorConfig;
  private final TalonFX launcherMotor;
  private final Slot0Configs launcherSlot0 = new Slot0Configs();
  private TalonFXSimState launcherMotorSim;

  // MOI calculated from prototype launcher
  private static final double kFlywheelMomentOfInertia = 0.0004926; // kg * m^2

  // Reduction between motors and encoder, as output over input. If the flywheel spins slower than
  // the motors, this number should be greater than one.
  private static final double kFlywheelGearing = 1.0;

  // Fuel launching state
  private boolean isFuel = false;
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
  private static final double kTurretMomentOfInertia = 0.05; // kg * m^2
  private static final double kTurretGearing = 20; // Total guess at the gearing for motor to turret
  private SparkMax turretSparkMax =
      new SparkMax(Constants.CanIds.launchMotorCanId + 100, MotorType.kBrushless);
  private final SparkClosedLoopController turretClosedLoopController;
  private SparkMaxConfig m_turretConfig = new SparkMaxConfig();
  private final SparkRelativeEncoder turretEncoder;
  private final SparkRelativeEncoderSim turretEncoderSim;
  private final SparkMaxSim turretSparkMaxSim;

  private final LinearSystem<N2, N1, N2> m_turretPlant =
      LinearSystemId.createSingleJointedArmSystem(
          DCMotor.getNEO(1), kTurretMomentOfInertia, kTurretGearing);

  public LauncherIOSim() {
    launcherMotor = new TalonFX(Constants.CanIds.launchMotorCanId + 100);
    launcherMotorConfig =
        new TalonFXConfiguration()
            .withMotorOutput(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake))
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(Amps.of(120))
                    .withStatorCurrentLimitEnable(true))
            .withFeedback(
                new FeedbackConfigs()
                    .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor))
            .withClosedLoopGeneral(new ClosedLoopGeneralConfigs().withContinuousWrap(true));

    launcherMotor.getConfigurator().apply(launcherMotorConfig);

    launcherSlot0.kP = 0.3;
    launcherSlot0.kI = 0;
    launcherSlot0.kD = 0.0;

    launcherMotor.getConfigurator().apply(launcherSlot0);

    launcherMotorSim = launcherMotor.getSimState();

    flywheelSim = new FlywheelSim(m_flywheelPlant, DCMotor.getKrakenX60(1));

    m_turretConfig.idleMode(IdleMode.kBrake).inverted(false).smartCurrentLimit(50);
    m_turretConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(0.1, 0, 0)
        .outputRange(-1, 1);
    m_turretConfig
        .encoder
        .positionConversionFactor(1.0 / kTurretGearing * 2 * Math.PI) // Radians
        .velocityConversionFactor(1.0 / kTurretGearing * 2 * Math.PI * 60.0); // Radians per second

    turretSparkMax.configure(
        m_turretConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    turretClosedLoopController = turretSparkMax.getClosedLoopController();

    turretEncoder = (SparkRelativeEncoder) turretSparkMax.getEncoder();
    turretSparkMaxSim = new SparkMaxSim(turretSparkMax, DCMotor.getNEO(1));
    turretEncoderSim = turretSparkMaxSim.getRelativeEncoderSim();

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

    fuelPublisher =
        NetworkTableInstance.getDefault()
            .getStructArrayTopic("/AdvantageKit/Launcher/Fuels", Translation3d.struct)
            .publish();
  }

  @Override
  public void updateInputs(LauncherIOInputs inputs) {
    // Update the sim model based on most recent commands. The standard loop time is 20ms.

    launcherMotorSim.setSupplyVoltage(RobotController.getBatteryVoltage());

    flywheelSim.setInputVoltage(launcherMotorSim.getMotorVoltage());
    flywheelSim.update(0.020);

    launcherMotorSim.setRotorVelocity(flywheelSim.getAngularVelocityRadPerSec() / (2 * Math.PI));

    turretSparkMaxSim.setBusVoltage(RobotController.getBatteryVoltage());

    turretSim.setInput(turretSparkMaxSim.getAppliedOutput() * RoboRioSim.getVInVoltage());
    turretSim.update(0.020);
    Logger.recordOutput("Launcher/turret sim pos", turretSim.getAngleRads());

    turretSparkMaxSim.iterate(
        turretSim.getVelocityRadPerSec(), RobotController.getBatteryVoltage(), 0.020);

    turretEncoderSim.setPosition(turretSim.getAngleRads());

    // Just bypass the turret sim stuff to make it point reasonably
    turretEncoderSim.setPosition(turretClosedLoopController.getSetpoint());
    turretEncoder.setPosition(turretClosedLoopController.getSetpoint());
    turretSim.setState(turretClosedLoopController.getSetpoint(), 0);

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

      double projectileVelocity = wwf * kWheelRadius / 2.0;
      double horizontalVelocity =
          projectileVelocity * Math.cos(Constants.launcherConstants.launcherAngle);
      double verticalVelocity =
          projectileVelocity * Math.sin(Constants.launcherConstants.launcherAngle);
      double launchAzimuthRad = turretSim.getAngleRads() + robotPose.getRotation().getRadians();
      double xVelocityProjectile = horizontalVelocity * Math.cos(launchAzimuthRad);
      double yVelocityProjectile = horizontalVelocity * Math.sin(launchAzimuthRad);

      Translation3d[] shot = new Translation3d[30];
      for (int i = 0; i < 30; i++) {
        // Generate 3 seconds worth of trajectory info at 10Hz
        double time = (double) i * 2.0 / 30.0;

        double pointX =
            robotPose.getX() + robotVelocity.vxMetersPerSecond + xVelocityProjectile * time;
        double pointY =
            robotPose.getY() + robotVelocity.vyMetersPerSecond + yVelocityProjectile * time;
        double pointZ =
            Constants.launcherConstants.launcherHeight
                + verticalVelocity * time
                - 9.81 * time * time;
        shot[i] = (new Translation3d(pointX, pointY, pointZ));
      }

      if (fuelTrajectory.size() < 30) {
        fuelTrajectory.addLast(shot);
        trajectoryAge.addLast(ageValue);
      }
    }

    // Handle old trajectories
    for (int i = 0; i < 30; i++) {
      if (trajectoryAge.size() > 0) {
        if (ageValue - trajectoryAge.getFirst().intValue() > maxTrajectoryAge) {
          trajectoryAge.removeFirst();
          fuelTrajectory.removeFirst();
        } else {
          // Stop iterating
          break;
        }
      }
    }
    ageValue++;

    // Iterate through the ring buffer, pulling from the front, placing at the end, until all
    // trajectories are plotted
    Translation3d[] traj = new Translation3d[0];
    int age;
    for (int i = 0; i < fuelTrajectory.size(); i++) {

      traj = fuelTrajectory.removeFirst();
      age = trajectoryAge.removeFirst();

      System.arraycopy(traj, 0, theTrajectories, i * 30, 30);

      fuelTrajectory.addLast(traj);
      trajectoryAge.addLast(age);
    }

    Translation3d[] loggedTrajectories = Arrays.copyOf(theTrajectories, fuelTrajectory.size() * 30);
    fuelPublisher.set(loggedTrajectories);

    if (fuelTrajectory.size() == 0) {
      traj = new Translation3d[0];
      fuelPublisher.set(traj);
    }

    inputs.launcherMotorVoltage = launcherMotor.getMotorVoltage().getValueAsDouble();
    inputs.launcherStatorCurrent = launcherMotor.getStatorCurrent().getValueAsDouble();
    inputs.launcherTorqueCurrent = launcherMotor.getTorqueCurrent().getValueAsDouble();
    inputs.launcherAcceleration = launcherMotor.getAcceleration().getValueAsDouble();
    inputs.launcherClosedLoopError = launcherMotor.getClosedLoopError().getValueAsDouble();
    inputs.launcherVelocity =
        Units.rotationsToRadians(launcherMotor.getVelocity().getValueAsDouble());
    inputs.launcherSupplyCurrent = launcherMotor.getSupplyCurrent().getValueAsDouble();
    inputs.launcherSupplyVoltage = launcherMotor.getSupplyVoltage().getValueAsDouble();

    inputs.turnMotorAppliedOutput = turretSparkMax.getAppliedOutput();
    inputs.turnMotorBusVoltage = turretSparkMax.getBusVoltage();
    inputs.turnMotorOutputCurrent = turretSparkMax.getOutputCurrent();

    inputs.turnEncoderVelocity = turretEncoder.getVelocity();
    inputs.turnEncoderPosition = turretEncoder.getPosition();
  }

  @Override
  public void setVoltage(double voltage) {
    // In this method, we update our simulation of what our arm is doing
    // First, we set our "inputs" (voltages)
    launcherMotor.setVoltage(voltage);
    Logger.recordOutput("Launcher/flywheelVoltageCmd", voltage);
  }

  @Override
  public void setRadPerS(double RPS) {
    Logger.recordOutput("Launcher/testRPS", RPS);
    RPS = Units.radiansToRotations(RPS);
    launcherMotor.setControl(new VelocityVoltage(RPS).withSlot(0));
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
  public void pointAt(double angle, double radPerSec) {
    double FFTurret = radPerSec * Constants.launcherConstants.turnVelocityFFGain;
    Logger.recordOutput("Launcher/FFTurret", FFTurret);
    turretClosedLoopController.setSetpoint(
        angle, ControlType.kPosition, ClosedLoopSlot.kSlot0, FFTurret);
  }

  @Override
  public void turretVoltage(double voltage) {
    Logger.recordOutput("Launcher/testTurnVoltage", voltage);
    turretSparkMax.setVoltage(voltage);
  }

  @Override
  public void updateRobotInfo(Pose2d robotPose, ChassisSpeeds robotVelocity) {
    this.robotPose = robotPose;
    this.robotVelocity = robotVelocity;
  }
}
