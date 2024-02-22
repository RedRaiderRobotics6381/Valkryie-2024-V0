// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Secondary;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Secondary.ClimberSubsystem;

public class ClimberCmd extends Command {
  
  private final ClimberSubsystem climber;
  private boolean down = false;
  private boolean up = true;
  private boolean done = false;

  public ClimberCmd(ClimberSubsystem climber) {
    this.climber = climber;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    done = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (down == false){
      climber.m_climberMotorR.set(.25);
      if(climber.m_climberEncoderR.getPosition() >= 15){
        down = true;
        up = false;
        done = true;
      } 

  } else if (up == false){
    climber.m_climberMotorR.set(-.25);
    if(climber.m_climberEncoderR.getPosition() <= 0){
      down = false;
      up = true;
      done = true;
    }
  }
}
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climber.m_climberMotorR.set(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return done;
  }
}
