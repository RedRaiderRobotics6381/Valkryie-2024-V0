package frc.robot.subsystems.Secondary;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;




public class Climber extends SubsystemBase{
    public static CANSparkMax m_climberMotor1;
    public static CANSparkMax m_climberMotor2;
    public static ProfiledPIDController m_climberPIDController;
    public static Encoder ClimberEncoder;
    private static double kS = 0.0;
    private static double kG = 0.0;
    private static double kV = 0.0;
    private static double kP = 0.0;
    private static double kI = 0.0;
    private static double kD = 0.0;
    private static double kDt  = 0.0;
    private static double kMaxVelocity = 0.0;
    private static double kMaxAcceleration = 0.0;
    private final TrapezoidProfile.Constraints m_constraints = new TrapezoidProfile.Constraints(kMaxVelocity, kMaxAcceleration);
    public static ElevatorFeedforward m_climberFF;
    

    /**
    * @param Climber
    */
    public Climber(){
        // Declare the motors
        //TODO set motor ID
        ClimberEncoder = new Encoder(1, 2);
        m_climberMotor1 = new CANSparkMax(ClimberConstants.kClimberRotateMotor1, MotorType.kBrushless);
        m_climberMotor2 = new CANSparkMax(ClimberConstants.kClimberRotateMotor2, MotorType.kBrushless);
        m_climberFF = new ElevatorFeedforward(kS, kG, kV);
        m_climberMotor2.setInverted(true);
        m_climberMotor2.follow(m_climberMotor1);
        m_climberPIDController = new ProfiledPIDController(kP, kI, kD, m_constraints, kDt);

        /**
         * The RestoreFactoryDefaults method can be used to reset the configuration parameters
         * in the SPARK MAX to their factory default state. If no argument is passed, these
         * parameters will not persist between power cycles
         */
        m_climberMotor1.restoreFactoryDefaults();  //Remove this when we remove the burnFlash() call below
        // ClimberEncoder = m_climberMotor1.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
        // ClimberEncoder.setPositionConversionFactor(360);
        // ClimberEncoder.setZeroOffset(72.5);


        // initialze PID controller and encoder objects

        // m_climberPIDController = m_climberMotor1.getPIDController();
        // m_climberPIDController.setFeedbackDevice(ClimberEncoder);
        m_climberMotor1.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);
        m_climberMotor1.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
        m_climberMotor1.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, 85); //TODO change this value
        m_climberMotor1.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, 241);//TODO change this value
        m_climberMotor1.enableVoltageCompensation(12.0);
        m_climberMotor1.setSmartCurrentLimit(25);
        m_climberMotor1.burnFlash(); //Remove this after everything is up and running to save flash wear


        // set PID coefficients
        //TODO tune the PID
        m_climberPIDController.setIZone(0.0);
        

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
    SmartDashboard.putNumber("Climber Enc Val", ClimberEncoder.getDistance());
  }


    public void climberHeightCommand(double ClimberHeightPos) {
    // implicitly require `this`
    m_climberPIDController.setGoal(ClimberHeightPos);

    m_climberMotor1.setVoltage(
        m_climberPIDController.calculate(ClimberEncoder.getDistance())
            + m_climberFF.calculate(m_climberPIDController.getSetpoint().velocity));
  }


}