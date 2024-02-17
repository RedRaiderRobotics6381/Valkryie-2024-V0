package frc.robot.commands.Vision;

import org.photonvision.PhotonUtils;
//import org.photonvision.common.hardware.VisionLEDMode;
import org.photonvision.targeting.PhotonTrackedTarget;
// import com.revrobotics.CANSparkLowLevel.MotorType;
// import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.Constants.AprilTagConstants;
import frc.robot.Constants.LauncherConstants;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.subsystems.Secondary.IntakeSubsystem;
import frc.robot.subsystems.Secondary.LauncherRotateSubsystem;
import frc.robot.subsystems.Secondary.LauncherSubsystem;
import edu.wpi.first.math.util.Units;

public class PVAim extends Command
{
  //private double visionObject;
  //CANSparkMax m_LauncherRotateMotor = new CANSparkMax(LauncherConstants.kLauncherRotate, MotorType.kBrushless);
  public static double Launcher_Pitch;
  private final LauncherRotateSubsystem launcherRotateSubsystem;
  private final LauncherSubsystem launcherSubsystem;
  private final IntakeSubsystem intakeSubsystem;
  private PhotonTrackedTarget lastTarget;

  public PVAim(LauncherRotateSubsystem launcherRotateSubsystem, LauncherSubsystem launcherSubsystem , IntakeSubsystem intakeSubsystem)
  {

    // each subsystem used by the command must be passed into the
    // addRequirements() method (which takes a vararg of Subsystem)
    this.launcherRotateSubsystem = launcherRotateSubsystem;
    this.launcherSubsystem = launcherSubsystem;
    this.intakeSubsystem = intakeSubsystem;
  }

  /**
   * The initial subroutine of a command.  Called once when the command is initially scheduled.
   */
  @Override
  public void initialize()
  {
    lastTarget = null;
  }

  /**
   * The main body of a command.  Called repeatedly while the command is scheduled. (That is, it is called repeatedly
   * until {@link #isFinished()}) returns true.)
   */
  @Override
  public void execute()
  {
    var photonRes = Robot.camAprTgLow.getLatestResult();
    //System.out.println(photonRes.hasTargets());
    if (photonRes.hasTargets()) {
      //Find the tag we want to chase
      var targetOpt = photonRes.getTargets().stream()
        .filter(t -> t.getFiducialId() == AprilTagConstants.speakerID) //4 Red & 7 Blue
        .filter(t -> !t.equals(lastTarget) && t.getPoseAmbiguity() != -1)
        .findFirst();
      if (targetOpt.isPresent()) {
        var target = targetOpt.get();
        Double TZ = target.getPitch();
        SmartDashboard.putNumber("Angle to Target", TZ);
        // var result = Robot.camAprTgLow.getLatestResult();  // Get the latest result from PhotonVision
        // boolean hasTargets = result.hasTargets(); // Check if the latest result has any targets.
        // PhotonTrackedTarget target = result.getBestTarget();
        //int targetID = result
        Double ID_HEIGHT = Units.inchesToMeters(57.13) - LauncherConstants.HEIGHT_TO_ROTATE_MOTOR;
        Double LAUNCHER_TO_TOWER = PhotonUtils.calculateDistanceToTargetMeters(LauncherConstants.CAMERA_HEIGHT_METERS,
                                                                              LauncherConstants.TARGET_Height_Meters, 
                                                                              LauncherConstants.Camera1_pitch, 
                                                                              Units.degreesToRadians(TZ))
                                                                              + LauncherConstants.PV_TO_ROTATE_MOTOR;

        Launcher_Pitch = Math.asin(ID_HEIGHT / Math.sqrt((ID_HEIGHT * ID_HEIGHT) + (LAUNCHER_TO_TOWER * LAUNCHER_TO_TOWER)));
          
        launcherRotateSubsystem.rotatePosCommand(Launcher_Pitch);
        RobotContainer.driverXbox.setRumble(XboxController.RumbleType.kLeftRumble, 0.25);
        RobotContainer.engineerXbox.setRumble(XboxController.RumbleType.kLeftRumble, 0.25);
        
        if (RobotContainer.engineerXbox.getRawButton(2) == true) {
          launcherSubsystem.LauncherCmd(5000);
          if (launcherSubsystem.m_launcherMotorTop.getEncoder().getVelocity() >= 4950) {
            intakeSubsystem.LaunchCmd();
          }
        }
        // This is new target data, so recalculate the goal
        lastTarget = target;
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
