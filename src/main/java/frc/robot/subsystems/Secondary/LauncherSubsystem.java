package frc.robot.subsystems.Secondary;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LauncherSubsystem extends SubsystemBase {

    public CANSparkMax LauncherMotorMaster;
    public CANSparkMax LauncherMotorFollower;

    public LauncherSubsystem() {
        LauncherMotorMaster =  new CANSparkMax(Constants.LauncherConstants.kLauncherL, MotorType.kBrushless);
        LauncherMotorFollower =  new CANSparkMax(Constants.LauncherConstants.kLauncherR, MotorType.kBrushless);
        LauncherMotorFollower.follow(LauncherMotorMaster, true);
    }
    
    public Command LauncherCmd() {
        // implicitly require `this`
        return this.runOnce(() -> LauncherMotorMaster.set(1)); //needs to change
        
        //armSubsystem.intakeMotorR.set(Constants.ArmConstants.gIntakeSpeed););
    }

}