package frc.robot.commands;


import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;

public class ManualClimberControl extends Command {

  private final double speed;
  private double output;

  public ManualClimberControl(double speed){
    requires(Robot.climber);
    this.speed = speed;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {

    SmartDashboard.putNumber("Elevator Position", Robot.climber.getPos());
    SmartDashboard.putBoolean("TopLimit Elevator", Robot.climber.getTopLimit());

    if (Robot.climber.getTopLimit()) {
      Robot.climber.resetEnc();
    }
    
    Robot.climber.set(speed);
    
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