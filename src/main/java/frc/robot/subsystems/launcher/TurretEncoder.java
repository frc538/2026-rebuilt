package frc.robot.subsystems.launcher;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CANcoderConfigurator;
import com.ctre.phoenix6.configs.CustomParamsConfigs;
import com.ctre.phoenix6.hardware.CANcoder;

import frc.robot.Constants;

public class TurretEncoder {
    CANcoder can1 = new CANcoder(Constants.launcherConstants.turretEncoder1);
    CANcoder can2 = new CANcoder(Constants.launcherConstants.turretEncoder2);

    CANcoderConfigurator can1Config = can1.getConfigurator();
    CANcoderConfigurator can2Config = can2.getConfigurator();


    
    public TurretEncoder() {
        
    }
    
    double read() {

    }
}
