package frc.robot.commands.Vision;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.common.hardware.VisionLEDMode;
import org.photonvision.targeting.PhotonTrackedTarget;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;


import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.RobotContainer;
import frc.robot.Constants.LauncherConstants;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.subsystems.Secondary.LauncherRotateSubsystem;
import edu.wpi.first.math.util.Units;
/**
 * Auto Balance command using a simple PID controller. Created by Team 3512
 * https://github.com/frc3512/Robot-2023/blob/main/src/main/java/frc3512/robot/commands/AutoBalance.java
 */
public class PVAim extends Command
{
  private double visionObject;
  CANSparkMax m_LauncherRotateMotor = new CANSparkMax(LauncherConstants.kLauncherRotate, MotorType.kBrushless);
  PhotonCamera camera = new PhotonCamera("OV5647");
  public static double Launcher_Pitch;
 
  public PVAim(double visionObject)
  {

    // each subsystem used by the command must be passed into the
    // addRequirements() method (which takes a vararg of Subsystem)
    this.visionObject = visionObject;
  }

  /**
   * The initial subroutine of a command.  Called once when the command is initially scheduled.
   */
  @Override
  public void initialize()
  {
    camera.setLED(VisionLEDMode.kOn);
    camera.setPipelineIndex((int)visionObject);
    camera.setDriverMode(false);

  }

  /**
   * The main body of a command.  Called repeatedly while the command is scheduled. (That is, it is called repeatedly
   * until {@link #isFinished()}) returns true.)
   */
  @Override
  public void execute()
  {
    var result = camera.getLatestResult();  // Get the latest result from PhotonVision
    boolean hasTargets = result.hasTargets(); // Check if the latest result has any targets.
    PhotonTrackedTarget target = result.getBestTarget();
    //int targetID = result.
    
    while (hasTargets == true) {
      RobotContainer.driverXbox.setRumble(XboxController.RumbleType.kLeftRumble, 0.25);
      Double TZ = target.getPitch();
      SmartDashboard.putNumber("Angle to Target", TZ);

      Double ID_HEIGHT = Units.inchesToMeters(57.13) - LauncherConstants.HEIGHT_TO_ROTATE_MOTOR;
      //TODO which camera is this?
      Double LAUNCHER_TO_TOWER = PhotonUtils.calculateDistanceToTargetMeters(LauncherConstants.CAMERA_HEIGHT_METERS,
                                                                             LauncherConstants.TARGET_Height_Meters, 
                                                                             LauncherConstants.Camera1_pitch, 
                                                                             Units.degreesToRadians(result.getBestTarget().getPitch())) + LauncherConstants.PV_TO_ROTATE_MOTOR;

     Launcher_Pitch = Math.asin(ID_HEIGHT / Math.sqrt((ID_HEIGHT * ID_HEIGHT) + (LAUNCHER_TO_TOWER * LAUNCHER_TO_TOWER)));
      

      

      if (visionObject == 0) {
          RobotContainer.driverXbox.setRumble(XboxController.RumbleType.kLeftRumble, 0.25);
        //LauncherRotateSubsystem.m_LauncherRotatePIDController.setReference(1, CANSparkMax.ControlType.kSmartMotion);
        LauncherRotateSubsystem.m_LauncherRotateMotor.set(LauncherConstants.ROTATE_MAX_SPEED);
      

          
        }

      if (visionObject == 1) {
          RobotContainer.driverXbox.setRumble(XboxController.RumbleType.kRightRumble, 0.25);

        }
    }
    
      // double translationVal = MathUtil.clamp(controller.calculate(swerveSubsystem.getPitch().getDegrees(), 0.0), -0.5,
    //                                        0.5);
    // swerveSubsystem.drive(new Translation2d(translationVal, 0.0), 0.0, true, false);
  }

  /**
   * <p>
   * Returns whether this command has finished. Once a command finishes -- indicated by this method returning true --
   * the scheduler will call its {@link #end(boolean)} method.
   * </p><p>
   * Returning false will result in the command never ending automatically. It may still be cancelled manually or
   * interrupted by another command. Hard coding this command to always return true will result in the command executing
   * once and finishing immediately. It is recommended to use *
   * {@link edu.wpi.first.wpilibj2.command.InstantCommand InstantCommand} for such an operation.
   * </p>
   *
   * @return whether this command has finished.
   */
  @Override
  public boolean isFinished()
  {
    return false;
  }

  /**
   * The action to take when the command ends. Called when either the command finishes normally -- that is it is called
   * when {@link #isFinished()} returns true -- or when  it is interrupted/canceled. This is where you may want to wrap
   * up loose ends, like shutting off a motor that was being used in the command.
   *
   * @param interrupted whether the command was interrupted/canceled
   */
  @Override
  public void end(boolean interrupted)
  {
    //swerveSubsystem.lock();
    RobotContainer.driverXbox.setRumble(XboxController.RumbleType.kBothRumble, 0);
  }
}
