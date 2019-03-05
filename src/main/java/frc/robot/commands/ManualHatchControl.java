/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;

public class ManualHatchControl extends Command {

  public static boolean beenPressed = false;
  private boolean servoUp = true;
  boolean reachedFront = false;

  public ManualHatchControl(){
    requires(Robot.hatch);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    //servoUp = true;
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    
    if (Robot.oi.getMainBButtonPressed()) {
      servoUp = !servoUp;
      /*if (position == 0.6 + 0.35) {
        position = 0.6;
      }
      else {
        position = 0.6 + 0.35;
      }*/
    }
    Robot.hatch.setServoUp(servoUp);
    SmartDashboard.putBoolean("servoUp", servoUp);
    if (Robot.hatch.hatchMotor.getSensorCollection().isRevLimitSwitchClosed() && beenPressed) {
      beenPressed = false;
      reachedFront = false;
      Robot.hatch.set(0.0);
    }

    if (Robot.oi.getMainLeftBumper()) {
      beenPressed = true;
    } 
    if (beenPressed) {
      if (Robot.hatch.hatchMotor.getSensorCollection().isFwdLimitSwitchClosed()) {
        reachedFront = true;
      }
      if (!reachedFront && !Robot.climber.getClimbTime()) {
        Robot.hatch.setRamp(0.0);
        Robot.hatch.set(1.0);
      }
      else if (!Robot.climber.getClimbTime()) {
        Robot.hatch.setRamp(0.6);
        Robot.hatch.set(-0.3);
      }
    }
    if (Robot.climber.getClimbTime() && !Robot.hatch.hatchMotor.getSensorCollection().isFwdLimitSwitchClosed()) {
      Robot.hatch.setRamp(0.3);
      Robot.hatch.set(0.35);
    }
    else if (Robot.climber.getClimbTime()) {
      Robot.hatch.set(0.0);
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
