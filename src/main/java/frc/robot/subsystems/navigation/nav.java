package frc.robot.subsystems.navigation;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class nav {
    public class navigationSubsystem extends SubsystemBase {
        private final navIO io;
        private final navIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged;

        public navigationSubsystem(navIO io) {
            io = IO;
        }

        public Command navigateTo() {
            
        }
    }
}