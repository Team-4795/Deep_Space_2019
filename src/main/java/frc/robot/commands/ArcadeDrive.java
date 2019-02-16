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
import frc.robot.ColorSensor;

public class ArcadeDrive extends Command {

  /*private double maxVel;
  private double pastVel;
  private double maxAccel;*/
  //ColorSensor cs;
  //private Boolean beenPressed;

  public ArcadeDrive() {
    requires(Robot.drivebase);
   // cs = ColorSensor.getInstanceOnboard();
  }

  @Override
  protected void initialize() {
    Robot.drivebase.resetEncoders();
  }


  @Override
  protected void execute() {

    /*if (Robot.oi.getRightBumperPress()){
      beenPressed = !beenPressed;
    }*/

    double throttle = 1.0 - (0.65 * Robot.oi.getMainRightTrigger());
    double turn = Robot.oi.getMainLeftJoyY() == 0.0 ? Robot.oi.getMainRightJoyX() * .8 : Robot.oi.getMainRightJoyX() * 0.5;

    /*if (beenPressed) {
      throttle *= -1.0;
    }*/

    Robot.drivebase.setMotors((Robot.oi.getMainLeftJoyY() - turn) * throttle, (Robot.oi.getMainLeftJoyY() + turn) * throttle);

    /*if (Robot.oi.getMainAButton()) {
      Robot.drivebase.resetEncoders();
    }*/
    SmartDashboard.putNumber("Left Encoder Count", Robot.drivebase.getLeftEncoderCount());
    SmartDashboard.putNumber("Right Encoder Count", Robot.drivebase.getRightEncoderCount());
    
    SmartDashboard.putNumber("Left Encoder Feet", Robot.drivebase.getLeftEncoderFeet());
    SmartDashboard.putNumber("Right Encoder Feet", Robot.drivebase.getRightEncoderFeet());
    /*
    SmartDashboard.putNumber("Left Velocity", Robot.drivebase.getLeftVelocity());
    SmartDashboard.putNumber("Right Velocity", Robot.drivebase.getRightVelocity());
    SmartDashboard.putNumber("Clear", cs.getColor().clear);
    SmartDashboard.putNumber("Red", cs.getColor().red);
    SmartDashboard.putNumber("Green", cs.getColor().green);
    SmartDashboard.putNumber("Blue", cs.getColor().blue);
    
    maxVel = Math.abs(Robot.drivebase.getLeftVelocity()) > Math.abs(maxVel) ? Robot.drivebase.getLeftVelocity() : maxVel;
    SmartDashboard.putNumber("Max Velocity", maxVel);
    maxAccel = Math.abs(Math.abs(Robot.drivebase.getLeftVelocity()) - Math.abs(pastVel)) > Math.abs(maxAccel) ? (Math.abs(Robot.drivebase.getLeftVelocity()) - Math.abs(pastVel)) / 0.05 : maxAccel;
    pastVel = Robot.drivebase.getLeftVelocity();*/
  }

  @Override
  protected boolean isFinished() {
    return false;
  }


  @Override
  protected void end() {
  }

  @Override
  protected void interrupted() {
  }
}
