package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class ManualClimberControl extends Command {

  private double speed;

  public ManualClimberControl(){
    requires(Robot.climber);
    speed = 0.5;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {

    if (Robot.oi.getArmBPressed()){
      Robot.climber.changeClimbTime();
    }

    if (Robot.oi.getMainYButton() && Robot.climber.getClimbTime()){
      Robot.climber.set(speed);
    }
    else if(Robot.oi.getMainAButton() && Robot.climber.getClimbTime()){
        Robot.climber.set(-speed);
    }
    else
    {
        Robot.climber.set(0.0);
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