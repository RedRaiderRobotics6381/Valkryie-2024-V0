// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.LauncherConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Secondary.IntakeCmd;
import frc.robot.commands.Secondary.OuttakeCmd;
import frc.robot.commands.Vision.DriveToAmpCmd;
import frc.robot.commands.Vision.DriveToObjectCmd;
import frc.robot.commands.Vision.DriveToSpeakerCmd;
import frc.robot.commands.Vision.DriveToStageCmd;
import frc.robot.subsystems.Secondary.LauncherSubsystem;
import frc.robot.subsystems.Secondary.Climber;
import frc.robot.subsystems.Secondary.IntakeSubsystem;
import frc.robot.subsystems.Secondary.LauncherRotateSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.io.File;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{

  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                         "swerve/neo"));

  public static XboxController driverXbox = new XboxController(0);
  public static XboxController engineerXbox = new XboxController(1);
  //private final Supplier<Pose2d> poseProvider = drivebase::getPose;

  private final SendableChooser<Command> autoChooser;
  static double lastTime = -1;

  LauncherSubsystem launcherSubsystem = new LauncherSubsystem();
  LauncherRotateSubsystem launcherRotateSubsystem = new LauncherRotateSubsystem();
  IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  Climber climberSubsystem = new Climber();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer()
  {
    // Configure the trigger bindings
    configureBindings();
    
    // Register Named Commands

    //NamedCommands.registerCommand("Intake", intakeSubsystem.IntakeCmd());
    NamedCommands.registerCommand("Shoot", launcherSubsystem.LauncherCmd(5000));
    // Build an auto chooser. This will use Commands.none() as the default option.
    autoChooser = AutoBuilder.buildAutoChooser();
    
    SmartDashboard.putData("Auto Chooser", autoChooser);

    // Applies deadbands and inverts controls because joysticks
    // are back-right positive while robot
    // controls are front-left positive
    // left stick controls translation
    // right stick controls the angular velocity of the robot
    Command driveFieldOrientedAnglularVelocity = drivebase.driveCommand(
        () -> MathUtil.applyDeadband(-driverXbox.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND) *
                                                             Constants.Drivebase.Max_Speed_Multiplier,
        () -> MathUtil.applyDeadband(-driverXbox.getLeftX(), OperatorConstants.LEFT_X_DEADBAND) *
                                                             Constants.Drivebase.Max_Speed_Multiplier,
        () -> MathUtil.applyDeadband(-driverXbox.getRawAxis(4), OperatorConstants.RIGHT_X_DEADBAND) );

    Command driveFieldOrientedDirectAngleSim = drivebase.simDriveCommand(
        () -> MathUtil.applyDeadband(-driverXbox.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND) *
                                                             Constants.Drivebase.Max_Speed_Multiplier,
        () -> MathUtil.applyDeadband(-driverXbox.getLeftX(), OperatorConstants.LEFT_X_DEADBAND) *
                                                             Constants.Drivebase.Max_Speed_Multiplier,
        () -> MathUtil.applyDeadband(-driverXbox.getRawAxis(4), OperatorConstants.RIGHT_X_DEADBAND) *
                                                                     Constants.Drivebase.Max_Speed_Multiplier);
    
    drivebase.setDefaultCommand(!RobotBase.isSimulation() ? driveFieldOrientedAnglularVelocity : driveFieldOrientedDirectAngleSim);
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary predicate, or via the
   * named factories in {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
   */
  private void configureBindings()
  {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    
    //Button 1 is "A" on xbox controller
    //Button 2 is "B" on xbox controller
    //Button 3 is "X" on xbox controller  
    //Button 4 is "Y" on xbox controller
    //Button 5 is "Left Bumper" on xbox controller
    //Button 6 is "Right Bumper" on xbox controller
    //Button 7 is "Back" on xbox controller
    //Button 8 is "Start" on xbox controller
    //Button 9 is "Left Joystick" on xbox controller
    //Button 10 is "Right Joystick" on xbox controller
    //Axis 0 is left joystick x side to side
    //Axis 1 is left joystick y forward and back
    //Axis 2 is left trigger 
    //Axis 3 is right trigger
    //Axis 4 is right joystick x side to side
    //Axis 5 is right joystick y forward and back


    new JoystickButton(driverXbox, 8).onTrue((new InstantCommand(drivebase::zeroGyro)));
    new JoystickButton(driverXbox, 2).whileTrue(new DriveToObjectCmd(drivebase)); //changed to 1 from zero. 
    new JoystickButton(driverXbox, 3).whileTrue(new DriveToSpeakerCmd(drivebase));
    new JoystickButton(driverXbox, 1).whileTrue(new DriveToAmpCmd(drivebase));
    new JoystickButton(driverXbox, 4).whileTrue(new DriveToStageCmd(drivebase));


    new JoystickButton(engineerXbox, 1).onTrue(launcherRotateSubsystem.rotatePosCommand(LauncherConstants.posOuttake)); //190.0 // DO NOT RUN AT 190. LAUNCHER WILL BREAK!!
    new JoystickButton(engineerXbox, 4).onTrue(launcherRotateSubsystem.rotatePosCommand(LauncherConstants.posDefault)); //60.0
    //new JoystickButton(engineerXbox, 6).onTrue(intakeSubsystem.InitialIntakeCmd());
    new JoystickButton(engineerXbox, 6).onTrue(new IntakeCmd(intakeSubsystem, launcherRotateSubsystem));
    //new JoystickButton(engineerXbox, 5).onTrue(intakeSubsystem.LaunchIntakeCmd());
    new JoystickButton(engineerXbox, 5).onTrue(new OuttakeCmd(intakeSubsystem, launcherSubsystem));
    //new JoystickButton(engineerXbox, 5).whileTrue(launcherSubsystem.LauncherCmd(.75));
    //new JoystickButton(engineerXbox, 3).whileTrue(climberSubsystem.climberInCmd()); 
    
    
    //new JoystickButton(engineerXbox, 7).onTrue(LauncherRotateSubsystem.rotateAutoPosCommand());

    //new JoystickButton(engineerXbox,3 ).whileTrue(new ArmIntakeInCmd(armIntakeSubsystem));
    // new JoystickButton(engineerXbox,3 ).whileTrue(LauncherSubsystem.ArmIntakeCmd(LauncherConstants.intakeSpeedIn));
    // new JoystickButton(engineerXbox, 3).onFalse(LauncherSubsystem.ArmIntakeCmd(LauncherConstants.intakeSpeedHold));
    // new JoystickButton(engineerXbox,2 ).whileTrue(LauncherSubsystem.ArmIntakeCmd(LauncherConstants.intakeSpeedOut));
    // new JoystickButton(engineerXbox, 2).onFalse(LauncherSubsystem.ArmIntakeCmd(0));

    // new JoystickButton(driverXbox,
    //                    2).whileTrue(
    //     Commands.deferredProxy(() -> drivebase.driveToPose(
    //                                new Pose2d(new Translation2d(4, 4), Rotation2d.fromDegrees(0)))
    //                           ));
//    new JoystickButton(driverXbox, 3).whileTrue(new RepeatCommand(new InstantCommand(drivebase::lock, drivebase)));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand()
  {
    // An example command will be run in autonomous
    return autoChooser.getSelected();
  }

  //   public Command setMaxDriveSpeed(double speedModifier)
  // {
  // }

  public void setDriveMode()
  {
    //drivebase.setDefaultCommand();
  }

  public void setMotorBrake(boolean brake)
  {
    drivebase.setMotorBrake(brake);
  }

  public void spencerButtons(){
    if (driverXbox.getRawButton(5) == true && driverXbox.getRawButton(6) == true){
      System.out.println("HighSpd");
      Constants.Drivebase.Max_Speed_Multiplier = 1;
    }
    if (driverXbox.getRawButton(5) == true && driverXbox.getRawButton(6) == false){
      System.out.println("MedSpd");
      Constants.Drivebase.Max_Speed_Multiplier = 0.85;
    }
    if (driverXbox.getRawButton(5) == false && driverXbox.getRawButton(6) == true){
      System.out.println("MedSpd");
      Constants.Drivebase.Max_Speed_Multiplier = 0.85;
    }

    if (driverXbox.getRawButton(5) == false && (driverXbox.getRawButton(6) == false)){
      //System.out.println("LowSpd");
      Constants.Drivebase.Max_Speed_Multiplier = 0.69;
    }
  }

  public static void pulseRumble(){
      if (lastTime != -1 && Timer.getFPGATimestamp() - lastTime <= 0.25) {
        driverXbox.setRumble(XboxController.RumbleType.kBothRumble, .25);
        //System.err.println("Rumble On");
      }
      else if (Timer.getFPGATimestamp() - lastTime <= 0.5) {
        driverXbox.setRumble(XboxController.RumbleType.kBothRumble, 0);
        //System.err.println("Rumble Off");
      }      
      else {
        lastTime = Timer.getFPGATimestamp();
      }
  }
}