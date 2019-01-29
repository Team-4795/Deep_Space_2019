/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;

public class DriveForward extends Command {
  double Feet;
  double Speed;

  double distanceInTicks;
  double leftTarget;
  double rightTarget;

  boolean isFinished = false;

  public DriveForward(double feet, double speed) {
      requires(Robot.drivebase);
      Feet = feet;
      Speed = speed;
  }
  
  public DriveForward(double feet, double speed, double timeout) {
      requires(Robot.drivebase);
      Feet = feet;
      Speed = speed;
      setTimeout(timeout);
  }
  
  protected void initialize() {
    Robot.drivebase.DriveFeet(Feet);
    /*
      distanceInTicks = Feet * Robot.drivebase.ENCODER_COUNTS_PER_FT;
      leftTarget = (int) (Robot.drivebase.getLeftEncoderCount() + distanceInTicks);
      rightTarget = (int) (Robot.drivebase.getRightEncoderCount() + distanceInTicks);
      SmartDashboard.putNumber("left", Feet);
      */
  }

  protected void execute() {
      double leftSpeed =
              Math.pow((leftTarget - Robot.drivebase.getLeftEncoderCount()) / distanceInTicks, 2)
                      * Speed;
      double rightSpeed =
              Math.pow((rightTarget - Robot.drivebase.getRightEncoderCount()) / distanceInTicks, 2)
                      * Speed;
      if (Math.abs(leftSpeed) < 0.3 || Math.abs(rightSpeed) < 0.3) {
          leftSpeed = 0;
          rightSpeed = 0;
          isFinished = true;
      }
      if (Feet > 0) {
          Robot.drivebase.setMotors(leftSpeed, rightSpeed);
      } else
          Robot.drivebase.setMotors(-leftSpeed, -rightSpeed);

  }

  protected boolean isFinished() {
      return isFinished || isTimedOut();
  }
}