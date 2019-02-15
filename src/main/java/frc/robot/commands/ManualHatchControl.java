/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class ManualHatchControl extends Command {

  boolean beenPressed = false;
  boolean reachedFront = false;

  public ManualHatchControl(){
    requires(Robot.hatch);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
   // Called repeatedly when this Command is scheduled to run
   @Override
   protected void execute() {

    if(Robot.hatch.hatchMotor.getSensorCollection().isRevLimitSwitchClosed() && beenPressed)
    {
     beenPressed = false;
     reachedFront = false;
     Robot.hatch.set(0.0);
    }

     if(Robot.oi.getArmYButton())
     {
       beenPressed = true;
     } 
     if(beenPressed)
     {
       if(Robot.hatch.hatchMotor.getSensorCollection().isFwdLimitSwitchClosed())
       {
         reachedFront = true;
       }
       if(!reachedFront)
       {
         Robot.hatch.set(1.0);
       }
       else
       {
         Robot.hatch.set(-0.1);
       }
     }

   }
 

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false; //change to getRevLimitSwitch if needed later
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
