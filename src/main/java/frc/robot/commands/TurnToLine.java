/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.ColorSensor;
import frc.robot.ColorSensor.ColorData;

public class TurnToLine extends Command {

  ColorSensor csLeft = ColorSensor.getInstanceMXP();
  ColorSensor csRight = ColorSensor.getInstanceOnboard();
  
  double threshold = 7;
  public TurnToLine(double timeout) {
    requires(Robot.drivebase);
    setTimeout(timeout);
  }

  @Override
  protected void initialize() {
  }

  @Override
  protected void execute() {
    ColorData dataLeft = csLeft.getColor();
    ColorData dataRight = csRight.getColor();

    /*
    double leftClear = dataLeft.clear * 300;
    double rightClear = dataRight.clear * 300;
    double left = leftClear - rightClear;
    SmartDashboard.putNumber("Color Sensor Turn Output", left);
    Robot.drivebase.setMotors(-0.0 + left, -0.0 - left);
    */
    double leftClear = dataLeft.clear * 10000;
    double rightClear = dataRight.clear * 10000;  
    SmartDashboard.putNumber("leftClear", leftClear);
    SmartDashboard.putNumber("rightClear", rightClear);

    if(leftClear > threshold || rightClear > threshold)
    {
      if(leftClear > rightClear)
        curveLeft();
      if(leftClear < rightClear)
        curveRight();
    }
  }

  public void curveRight()
  {
    Robot.drivebase.setMotors(-0.05, -0.25);
  }

  
  public void curveLeft()
  {
    Robot.drivebase.setMotors(-0.25, -0.05);
  }

  @Override
  protected boolean isFinished() {
    return isTimedOut();
  }


  @Override
  protected void end() {
    Robot.drivebase.setMotors(0.0, 0.0);
  }

  @Override
  protected void interrupted() {
  }
}
