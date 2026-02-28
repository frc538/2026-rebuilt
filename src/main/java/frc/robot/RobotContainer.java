// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import static frc.robot.subsystems.vision.VisionConstants.*;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Intake.IntakeIO;
import frc.robot.subsystems.Intake.IntakeIOSim;
import frc.robot.subsystems.Intake.IntakeIOSpark;
import frc.robot.subsystems.climber.ClimberIO;
import frc.robot.subsystems.climber.ClimberIOSparkMax;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.hopper.HopperIO;
import frc.robot.subsystems.hopper.HopperIOSparkMax;
import frc.robot.subsystems.launcher.Launcher;
import frc.robot.subsystems.launcher.LauncherIO;
import frc.robot.subsystems.launcher.LauncherIOHardware;
import frc.robot.subsystems.launcher.LauncherIOSim;
import frc.robot.subsystems.navigation.NavigationSubsystem;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;
  private final NavigationSubsystem navSys;
  private final Vision vision;
  private final Hopper hopper;
  private final Intake intake;
  private final ClimberSubsystem climberSubsystem;

  private final Launcher launcher;

  // Controller
  private final CommandXboxController pilotController = new CommandXboxController(0);
  private final CommandXboxController navController = new CommandXboxController(1);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        // ModuleIOTalonFX is intended for modules with TalonFX drive, TalonFX turn, and
        // a CANcoder
        if (Constants.Features.LauncherEnabled) {
          launcher = new Launcher(new LauncherIOHardware());
        } else {
          launcher = new Launcher(new LauncherIO() {});
        }
        navSys = new NavigationSubsystem();

        if (Constants.Features.DriveEnabled) {
          drive =
              new Drive(
                  launcher::updateOdometry,
                  new GyroIOPigeon2(),
                  new ModuleIOTalonFX(TunerConstants.FrontLeft),
                  new ModuleIOTalonFX(TunerConstants.FrontRight),
                  new ModuleIOTalonFX(TunerConstants.BackLeft),
                  new ModuleIOTalonFX(TunerConstants.BackRight));
        } else {
          drive =
              new Drive(
                  launcher::updateOdometry,
                  new GyroIO() {},
                  new ModuleIO() {},
                  new ModuleIO() {},
                  new ModuleIO() {},
                  new ModuleIO() {});
        }
        if (Constants.Features.VisionEnabled) {
          vision =
              new Vision(
                  drive::addVisionMeasurement,
                  new VisionIOLimelight(camera0Name, drive::getRotation),
                  new VisionIOLimelight(camera1Name, drive::getRotation));
        } else {
          vision =
              new Vision(
                  drive::addVisionMeasurement,
                  new VisionIO() {},
                  new VisionIO() {},
                  new VisionIO() {},
                  new VisionIO() {});
        }

        // new VisionIOLimelight(camera2Name, drive::getRotation),
        // new VisionIOLimelight(camera3Name, drive::getRotation));
        if (Constants.Features.HopperEnabled) {
          hopper =
              new Hopper(
                  new HopperIOSparkMax(Constants.Hopper.FeedCanId, Constants.Hopper.SpindexCanId));
        } else {
          hopper = new Hopper(new HopperIO() {});
        }
        if (Constants.Features.IntakeEnabled) {
          intake = new Intake(new IntakeIOSpark() {});
        } else {
          intake = new Intake(new IntakeIO() {});
        }
        if (Constants.Features.ClimberEnabled) {
          climberSubsystem =
              new ClimberSubsystem(
                  new ClimberIOSparkMax(Constants.ClimberConstants.ClimberMotorCANId, 5, 6));
        } else {
          climberSubsystem = new ClimberSubsystem(new ClimberIO() {});
        }

        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        launcher = new Launcher(new LauncherIOSim());
        drive =
            new Drive(
                launcher::updateOdometry,
                new GyroIO() {},
                new ModuleIOSim(TunerConstants.FrontLeft),
                new ModuleIOSim(TunerConstants.FrontRight),
                new ModuleIOSim(TunerConstants.BackLeft),
                new ModuleIOSim(TunerConstants.BackRight));
        navSys = new NavigationSubsystem();
        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOPhotonVisionSim(camera0Name, robotToCamera0, drive::getPose),
                new VisionIOPhotonVisionSim(camera1Name, robotToCamera1, drive::getPose));
        // new VisionIOPhotonVisionSim(camera2Name, robotToCamera2, drive::getPose),
        // new VisionIOPhotonVisionSim(camera3Name, robotToCamera3, drive::getPose));
        hopper = new Hopper(new HopperIO() {});
        intake = new Intake(new IntakeIOSim(Constants.Intake.MovMotorCanId) {});
        climberSubsystem = new ClimberSubsystem(new ClimberIO() {});
        break;

      default:
        // Replayed robot, disable IO implementations
        launcher = new Launcher(new LauncherIO() {});
        drive =
            new Drive(
                launcher::updateOdometry,
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});

        navSys = new NavigationSubsystem();
        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIO() {},
                new VisionIO() {},
                new VisionIO() {},
                new VisionIO() {});
        hopper = new Hopper(new HopperIO() {});
        intake = new Intake(new IntakeIO() {});
        climberSubsystem = new ClimberSubsystem(new ClimberIO() {});
        break;
    }

    SmartDashboard.putData(navSys.m_field);

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    //////////////////////////////////////////////////////////////
    /// Drive Commands
    //////////////////////////////////////////////////////////////

    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -pilotController.getLeftY(),
            () -> -pilotController.getLeftX(),
            () -> -pilotController.getRightX()));

    // Switch to X pattern when X button is pressed
    pilotController
        .x()
        .and(
            () -> {
              return !DriverStation.isTest();
            })
        .onTrue(Commands.runOnce(drive::stopWithX, drive));

    // Blue
    navController.y().onTrue(navSys.generatePath(Constants.navigationConstants.topCenterPointBlue));
    navController.b().onTrue(navSys.generatePath(Constants.navigationConstants.centerPointBlue));
    navController
        .a()
        .onTrue(navSys.generatePath(Constants.navigationConstants.bottomCenterPointBlue));
    // Red
    navController
        .povLeft()
        .onTrue(navSys.generatePath(Constants.navigationConstants.centerPointRed));
    navController
        .povUp()
        .onTrue(navSys.generatePath(Constants.navigationConstants.bottomCenterPointRed));
    navController
        .povDown()
        .onTrue(navSys.generatePath(Constants.navigationConstants.topCenterPointRed));
    // Center
    navController
        .rightBumper()
        .onTrue(navSys.generatePath(Constants.navigationConstants.topCenterPoint));
    navController.start().onTrue(navSys.generatePath(Constants.navigationConstants.centerPoint));
    navController
        .leftBumper()
        .onTrue(navSys.generatePath(Constants.navigationConstants.bottomCenterPoint));
    // controller.rightBumper().onTrue(navSys.showPath());

    // Reset gyro to 0° when B button is pressed
    pilotController
        .b()
        .and(
            () -> {
              return !DriverStation.isTest();
            })
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), Rotation2d.kZero)),
                    drive)
                .ignoringDisable(true));

    //////////////////////////////////////////////////////////////
    /// Climber Commands
    //////////////////////////////////////////////////////////////

    pilotController.leftBumper().and(this::isNotTest).whileTrue((climberSubsystem.climberRetract()));
    pilotController.rightBumper().and(this::isNotTest).whileTrue((climberSubsystem.climberExtend()));

    pilotController
        .leftBumper()
        .and(DriverStation::isTest)
        .whileTrue(climberSubsystem.TestClimberRetract());
    pilotController
        .rightBumper()
        .and(DriverStation::isTest)
        .whileTrue(climberSubsystem.TestClimberExtend());

    //////////////////////////////////////////////////////////////
    /// Launcher Commands
    //////////////////////////////////////////////////////////////

    /// Teleop Commands

    /// Test mode commands

    pilotController.a().and(DriverStation::isTest).whileTrue(launcher.testFullSpeed());
    pilotController.b().and(DriverStation::isTest).whileTrue(launcher.testLowSpeed());
    pilotController.x().and(DriverStation::isTest).whileTrue(launcher.testTurn());
    pilotController.y().and(DriverStation::isTest).whileTrue(launcher.invertTestTurn());

    //////////////////////////////////////////////////////////////
    /// Hopper Commands (Drives spindexer and feeds the launcher)
    //////////////////////////////////////////////////////////////

    pilotController.b().and(this::isNotTest).onTrue(hopper.HopperToggle());
    pilotController.b().and(DriverStation::isTest).whileTrue(hopper.testFeed());
    pilotController.a().and(DriverStation::isTest).whileTrue(hopper.testSpindex());

    //////////////////////////////////////////////////////////////
    /// Intake Commands
    //////////////////////////////////////////////////////////////

    pilotController.leftTrigger().and(DriverStation::isTest).onTrue(intake.testIntakeDown());
    pilotController.rightTrigger().and(DriverStation::isTest).onTrue(intake.testIntakeUp());
    pilotController.x().and(DriverStation::isTest).whileTrue(intake.testIntake());
  
  }
  private boolean isNotTest() {
    return (!DriverStation.isTest());
  }
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
