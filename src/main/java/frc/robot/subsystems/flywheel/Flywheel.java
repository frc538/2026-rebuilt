public class Flywheel extends SubsystemBase {
    
    FlywheelIO io;
    FlywheelIOInputsAutoLogged inputs = new FlywheelIOInputsAutoLogged();

    public Flywheel(FlywheelIO IO) {
        io = IO;
    }

    

}
