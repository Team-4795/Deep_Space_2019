/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;

public class DriveForward extends Command {

  private double feet;
  boolean inErrorZone;
  boolean isFinished;
  int count;

  public DriveForward(double feet, double timeout) {
    requires(Robot.drivebase);
    this.feet = feet;
    setTimeout(timeout);
  }

  @Override
  protected void initialize() {
    Robot.drivebase.DriveFeet(feet);
  }

  @Override
  protected void execute() {
    /*
     * // check how close we are to the target angle, if we are within the tolerance
     * for 10 roboRio // ticks then end the command double error =
     * Robot.drivebase.leftMotorOne.getClosedLoopError(0); inErrorZone =
     * Math.abs(error) < Robot.drivebase.allowableError ? true : false;
     * 
     * if (inErrorZone) { count++; if (count >= 6) { isFinished = true; } else {
     * isFinished = false; } } else { count = 0; }
     */

    SmartDashboard.putNumber("Left Encoder Count", Robot.drivebase.getLeftEncoderCount());
    SmartDashboard.putNumber("Right Encoder Count", Robot.drivebase.getRightEncoderCount());

    SmartDashboard.putNumber("Left Encoder Feet", Robot.drivebase.getLeftEncoderFeet());
    SmartDashboard.putNumber("Right Encoder Feet", Robot.drivebase.getRightEncoderFeet());

    SmartDashboard.putNumber("Error", Robot.drivebase.leftMotorOne.getClosedLoopError());
    SmartDashboard.putNumber("Setpoint", Robot.drivebase.leftMotorOne.getClosedLoopTarget());

  }

  @Override
  protected boolean isFinished() {
    return isTimedOut() || isFinished;
  }

  @Override
  protected void end() {
    Robot.drivebase.setMotors(0.0, 0.0);
  }

  @Override
  protected void interrupted() {
  }
}
