package frc.robot.subsystems.launcher;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkRelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.Constants;

public class LauncherIOHardware implements LauncherIO {
    private final SparkMax turnMotor;
    private final SparkMax yeetMotor;
    private final SparkClosedLoopController pid = turnMotor.GetClosedLoopController();
    SparkMaxConfig turnConf = new SparkMaxConfig();
    SparkMaxConfig yeetConf = new SparkMaxConfig();
    SparkRelativeEncoder turnCoder;
    SparkRelativeEncoder yeetCoder;

    public LauncherIOHardware(int yeetID, int turnID){
        turnMotor = new SparkMax(yeetID, MotorType.kBrushless);
        yeetMotor = new SparkMax(turnID, MotorType.kBrushless);
        turnCoder = (SparkRelativeEncoder) turnMotor.getEncoder();
        yeetCoder = (SparkRelativeEncoder) yeetMotor.getEncoder();

        turnCoder
        .idleMode(IdleMode.kBrake)
        // .smartCurrentLimit(Constants.ArmConstants.CurrentLimit)
        .inverted(false);
        yeetCoder
            .idleMode(IdleMode.kBrake)
            // .smartCurrentLimit(Constants.ArmConstants.CurrentLimit)
            .inverted(false);
        turnCoder
            .encoder
            .positionConversionFactor(Constants.Hopper.FDConversionFactor)
            .velocityConversionFactor(Constants.Hopper.FDConversionFactor);
        yeetCoder
            .encoder
            .positionConversionFactor(Constants.Hopper.SDConversionFactor)
            .velocityConversionFactor(Constants.Hopper.SDConversionFactor);
        turnCoder.closedLoop.p(0.1).i(0.0).d(0.01).outputRange(-1.0,1.0);
        

    turnMotor.configure(
        turnCoder, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    yeetMotor.configure(
        yeetCoder, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    public void updateInputs(LauncherIOInputs inputs){
        // Launcher Flywheel
        inputs.rpm;
        inputs.projectileSpeed;
        inputs.projectileRotationalSpeed;

        // Turret
        inputs.turretAngle;
        inputs.turretSpeed;
    }
    @Override
    // Need to update this IO interface to add an RPM or rad/s function that sets the flywheel speed
    public default void setRadPerS(double rps) {
        turnMotor.set(rps);
    }

    @Override
    // Set launcher voltage
    public default void setVoltage(double voltage) {
        yeetMotor.setVoltage(voltage);
    }

    // Command to point the launcher at an angle in degrees
    @Override
    public default void pointAt(double angle) {
        pid.setSetpoint(angle, ControlType.kPosition);
    }

    // Sets the minimum and maximum accepted angles in degrees
    @Override
    public default void setLockout(double minAngle, double maxAngle) {
        return
    }

    // Simulate feeding a projectile into the launcher
    @Override
    public default void simLaunch() {
        
    }


}
