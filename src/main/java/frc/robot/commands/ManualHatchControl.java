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

  private boolean servoUp = true;
  private static boolean hatchUp = false;

  public ManualHatchControl() {
    requires(Robot.hatch);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    servoUp = true;
    hatchUp = false;
  }

  public static void hatchDown() {
    hatchUp = false;
  }
  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    boolean hatchActuallyUp = Robot.hatch.hatchMotor.getSensorCollection().isFwdLimitSwitchClosed();
    boolean hatchActuallyDown = Robot.hatch.hatchMotor.getSensorCollection().isRevLimitSwitchClosed();

    if (Robot.oi.getMainBButtonPressed()) {
      servoUp = !servoUp;
    }
    if (Robot.oi.getMainLeftBumperPressed()) {
      hatchUp = !hatchUp;
    }

    if (hatchActuallyUp) hatchUp = false;
    if (Robot.climber.getClimbTime()) {
      Robot.hatch.setRamp(0.3);
      Robot.hatch.set(hatchActuallyUp ? 0.0 : 0.35);
    } else if (hatchUp) {
      servoUp = false;
      Robot.hatch.setRamp(0.0);
      Robot.hatch.set(1.0);
    } else if (hatchActuallyDown) {
      Robot.hatch.set(0.0);
    } else {
      Robot.hatch.setRamp(0.6);
      Robot.hatch.set(-0.3);
    }

    Robot.hatch.setServoUp(servoUp);

    SmartDashboard.putBoolean("Reverse Limit", hatchActuallyUp);
    SmartDashboard.putBoolean("Forward Limit", hatchActuallyDown);
    SmartDashboard.putBoolean("servoUp", servoUp);
    SmartDashboard.putBoolean("hatchUp", hatchUp);
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
