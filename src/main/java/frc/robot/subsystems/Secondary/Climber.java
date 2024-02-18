package frc.robot.subsystems.Secondary;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
//import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.ClimberConstants;

//is this working?


public class Climber extends SubsystemBase{

    public static CANSparkMax m_climberMotorL;  
    public static CANSparkMax m_climberMotorR;

    public static RelativeEncoder m_climberEncoderR;
    public static RelativeEncoder m_climberEncoderL;
// =======
//     public static RelativeEncoder m_climberEncoder;

// >>>>>>> main
    public static ProfiledPIDController m_climberPIDController;
    //public static Encoder ClimberEncoder;
    private static double kS = 0.0;
    private static double kG = 0.0;
    private static double kV = 0.0;
    private static double kP = 0.0;
    private static double kI = 0.0;
    private static double kD = 0.0;
    private static double kDt  = 0.0;
    private static double kMaxVelocity = 1000;
    private static double kMaxAcceleration = 500;
    private final TrapezoidProfile.Constraints m_constraints = new TrapezoidProfile.Constraints(kMaxVelocity, kMaxAcceleration);
    public static ElevatorFeedforward m_climberFF;
    

    /**
    * @param Climber
    */
    public Climber(){
        // Declare the motors

        //ClimberEncoder = new Encoder(1, 2);
        m_climberMotorR = new CANSparkMax(ClimberConstants.kClimberMotorR, MotorType.kBrushless);
        m_climberMotorL = new CANSparkMax(ClimberConstants.kClimberMotorL, MotorType.kBrushless);
        m_climberEncoderR = m_climberMotorR.getEncoder();
        m_climberEncoderL = m_climberMotorL.getEncoder();
        //m_climberFF = new ElevatorFeedforward(kS, kG, kV);
        //m_climberMotorL.setInverted(true);
        //m_climberMotorL.follow(m_climberMotorR);
        //m_climberPIDController = new ProfiledPIDController(kP, kI, kD, m_constraints, kDt);

        /**
         * The RestoreFactoryDefaults method can be used to reset the configuration parameters
         * in the SPARK MAX to their factory default state. If no argument is passed, these
         * parameters will not persist between power cycles
         */

        m_climberMotorR.restoreFactoryDefaults();  //Remove this when we remove the burnFlash() call below
        m_climberMotorL.restoreFactoryDefaults();  //Remove this when we remove the burnFlash() call below



        // ClimberEncoder = m_climberMotorRight.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);

        // ClimberEncoder.setPositionConversionFactor(360);
        // ClimberEncoder.setZeroOffset(72.5);
        m_climberEncoderR.setPositionConversionFactor(2.356); //TODO set this value to the amount the arm moves with each rotation
        m_climberEncoderL.setPositionConversionFactor(2.356); //TODO set this value to the amount the arm moves with each rotation

        // initialze PID controller and encoder objects

        // m_climberPIDController = m_climberMotorRight.getPIDController();
        // m_climberPIDController.setFeedbackDevice(ClimberEncoder);

        // m_climberMotorR.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);
        // m_climberMotorR.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
        // m_climberMotorR.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, 0); //TODO change this value
        // m_climberMotorR.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, 6);//TODO change this value
// =======

//         m_climberMotorR.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);
//         m_climberMotorR.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
//         m_climberMotorR.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, 0); //TODO change this value
//         m_climberMotorR.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, 241);//TODO change this value
// >>>>>>> main
        m_climberMotorR.enableVoltageCompensation(12.0);
        m_climberMotorR.setSmartCurrentLimit(25);
        m_climberMotorR.burnFlash(); //Remove this after everything is up and running to save flash wear
        m_climberMotorL.enableVoltageCompensation(12.0);
        m_climberMotorL.setSmartCurrentLimit(25);
        m_climberMotorL.burnFlash(); //Remove this after everything is up and running to save flash wear


        // set PID coefficients
        //TODO tune the PID
        //m_climberPIDController.setIZone(0.0);
        

        //Set the maximum speeds

        //m_climberPIDController.setOutputRange(-1.0, 1.0);


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
        // m_climberPIDController.setSmartMotionMaxVelocity(5000.0,0); 
        // m_climberPIDController.setSmartMotionMinOutputVelocity(0.0, 0);
        // m_climberPIDController.setSmartMotionMaxAccel(3000.0,0); 
        // m_climberPIDController.setSmartMotionAllowedClosedLoopError(0.01, 0);

    }
@Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("RClimber Enc Val", m_climberEncoderR.getPosition());
    SmartDashboard.putNumber("LClimber Enc Val", m_climberEncoderL.getPosition());
    // if (RobotContainer.engineerXbox.getRawButton(2) == true){
    //   m_climberMotorL.set(-.25);
    //   m_climberMotorR.set(.25);
    // } else if (RobotContainer.engineerXbox.getRawButton(3) == true){
    //   m_climberMotorL.set(.25);
    //   m_climberMotorR.set(-.25);
    // } else{
    //   m_climberMotorL.set(0);
    //   m_climberMotorR.set(0);
    // }

  }


  //   public Command climberHeightCommand(double ClimberHeightPos) {
  //   // implicitly require `this`
  //   return this.runOnce(() ->{
  //   m_climberPIDController.setGoal(ClimberHeightPos);

  //   m_climberMotorR.setVoltage(
  //       m_climberPIDController.calculate(m_climberEncoderR.getPosition())
  //           + m_climberFF.calculate(m_climberPIDController.getSetpoint().velocity));
  //   });
  // }

    // public Command climberInCmd() {
    //   // implicitly require `this`
    //   return this.runOnce(() ->{m_climberMotorL.set(.25);
    //   });
    // }

// <<<<<<< Jason
// =======

//     m_climberMotorR.setVoltage(
//         m_climberPIDController.calculate(m_climberEncoder.getPosition())

//             + m_climberFF.calculate(m_climberPIDController.getSetpoint().velocity));
//   }
// >>>>>>> main


}