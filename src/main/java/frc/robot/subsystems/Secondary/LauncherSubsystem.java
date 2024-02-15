package frc.robot.subsystems.Secondary;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LauncherSubsystem extends SubsystemBase {

    public CANSparkMax LauncherMotorTop;
    public CANSparkMax LauncherMotorBot;

    public LauncherSubsystem() {

        LauncherMotorTop =  new CANSparkMax(Constants.LauncherConstants.kLauncherT, MotorType.kBrushless);
        LauncherMotorBot =  new CANSparkMax(Constants.LauncherConstants.kLauncherB, MotorType.kBrushless);

        LauncherMotorBot.follow(LauncherMotorTop, true);
    }
    
    public Command LauncherCmd() {
        // implicitly require `this`

        return this.runOnce(() -> LauncherMotorTop.set(ArmIntakeSetpoint));

        
        //armSubsystem.intakeMotorR.set(Constants.ArmConstants.gIntakeSpeed););
    }

}