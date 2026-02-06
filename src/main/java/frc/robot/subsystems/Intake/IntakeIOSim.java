package frc.robot.subsystems.Intake;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;

public class IntakeIOSim implements IntakeIO {
    SparkMax Movement;
    EncoderSim MovementEncoderSim;
    TrapezoidProfile MovementTrapProf;
    Constraints profileConstraints;
    TrapezoidProfile.State MovementCurrentState ;
    TrapezoidProfile.State MovementDesiredState ;
    
    private boolean FirstFrame = true;

    public ElevatorIOSim (int MovementId) {
        Movement = new SparkMax(MovementId, MotorType.kBrushless);
        MovementEncoderSim = Movement.get

    }
    
    public void updateInputs(IntakeIOInputs inputs) {
        
    }
    public void setIntakePosition(double radians) {

    }
    public void runRotato(double speed) {
        //nothing
    }
}
