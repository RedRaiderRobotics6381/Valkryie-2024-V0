package frc.robot.subsystems.Secondary;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {

    public CANSparkMax intakeMotor;
    public CANSparkMax indexerMotor;
    public CANSparkMax launcherIndexerMotor;
    public static SparkPIDController intakePIDController;

    public IntakeSubsystem() {
        intakeMotor = new CANSparkMax(Constants.IntakeConstants.kIntakeMotor, MotorType.kBrushless);
        indexerMotor = new CANSparkMax(Constants.IntakeConstants.kIndexerMotor, MotorType.kBrushless);
        launcherIndexerMotor = new CANSparkMax(Constants.IntakeConstants.kLauncherIndexerMotor, MotorType.kBrushless);
        
        indexerMotor.follow(intakeMotor);
        launcherIndexerMotor.follow(intakeMotor);

                
        /**
         * The RestoreFactoryDefaults method can be used to reset the configuration parameters
         * in the SPARK MAX to their factory default state. If no argument is passed, these
         * parameters will not persist between power cycles
         */
        intakeMotor.restoreFactoryDefaults();  //Remove this when we remove the burnFlash() call below
    
        // initialze PID controller and encoder objects
        intakeMotor.enableVoltageCompensation(12.0);
        intakeMotor.setSmartCurrentLimit(25);
        intakeMotor.burnFlash();  //Remove this after everything is up and running to save flash wear
        
        intakePIDController.setOutputRange(0, 1.0); //ArmConstants.armRotatekMinOutput, ArmConstants.armRotatekMaxOutput);
    
        /**
         * Smart Motion coefficients are set on a SparkMaxPIDController object
         * 
         * - setSmartMotionMaxVelocity() will limit the velocity in RPM of
         * the pid controller in Smart Motion mode
         * - setSmartMotionMinOutputVelocity() will put a lower bound in
         * RPM of the pid controller in Smart Motion mode
         * - setSmartMotionMaxAccel() will limit the acceleration in RPM^2
         * of the pid controller in Smart Motion mode
         * - setSmartMotionAllowedClosedLoopError() will set the max allowed
         * error for the pid controller in Smart Motion mode
         */
        intakePIDController.setSmartMotionMaxVelocity(5000.0,0); //ArmConstants.armRotateMaxVel, ArmConstants.armRotateSmartMotionSlot);
        intakePIDController.setSmartMotionMinOutputVelocity(0.0, 0); //ArmConstants.armRotateMinVel, ArmConstants.armRotateSmartMotionSlot);
        intakePIDController.setSmartMotionMaxAccel(3000.0,0); //ArmConstants.armRotateMaxAcc, ArmConstants.armRotateSmartMotionSlot);
        intakePIDController.setSmartMotionAllowedClosedLoopError(0.01, 0); //ArmConstants.armRotateAllowedErr, ArmConstants.armRotateSmartMotionSlot);  
    }
    
    public Command IntakeCmd() {
        // implicitly require `this`
        return this.runOnce(() -> intakeMotor.set(0.5));
        //set to final speed once tested
        
    }

    public Command IntakeReverseCmd() {
        // implicitly require `this`
        return this.runOnce(() -> intakeMotor.set(-0.5));
        //set to final speed once tested
        
    }

}