package frc.robot.subsystems.intake;

import com.revrobotics.sim.SparkRelativeEncoderSim;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;

public class IntakeIOSim implements IntakeIO{ 
    SparkMax MovMotor;
    SparkRelativeEncoderSim MovMotorEncoder;
    TrapezoidProfile commandProfile;
    Constraints profileConstraints;
    TrapezoidProfile.State mCurrentState;
    TrapezoidProfile.State mDesiredState;

    private boolean firstFrame = true;

    double mReferencePosition = 0.0;
    double mArbFF = 0.0;
    double kS = 0.0;
    double kG = 0.0;
    double kV = 0.0;
    double kA = 0.0;
    double maxV = 0.3;
    double maxA = 0.3;
    double kP = 0.0;
    double kI = 0.0;
    double kD = 0.0;

    double kPLast = 0;
    double kILast = 0;
    double kDLast = 0;

    //private final SparkClosedLoopController movMotorController;
}
