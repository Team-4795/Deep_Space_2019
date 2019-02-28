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
    double whiteLeft = dataLeft.clear * 30;
    double whiteRight = dataRight.clear * 30;
    double left = whiteLeft - whiteRight;
    SmartDashboard.putNumber("Color Sensor Turn Output", left);
    Robot.drivebase.setMotors(0.1 + left, 0.1 - left);
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
