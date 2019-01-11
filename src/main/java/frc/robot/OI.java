/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;

public class OI {

  private static final double DEADZONE = 0.1;

  private Joystick MAIN_CONTROLLER;
  private Joystick ARM_CONTROLLER;

  public OI() { 
  MAIN_CONTROLLER = new Joystick(RobotMap.MAIN_CONTROLLER.value);
  ARM_CONTROLLER =new Joystick(RobotMap.MAIN_CONTROLLER.value);
  }

  public double getMainLeftJoyY() {
    return Math.abs(MAIN_CONTROLLER.getRawAxis(1)) > DEADZONE ? MAIN_CONTROLLER.getRawAxis(1) : 0.0;
  } 

  public double getMainRightJoyX() {
    return Math.abs(MAIN_CONTROLLER.getRawAxis(4)) > DEADZONE ? MAIN_CONTROLLER.getRawAxis(4) : 0.0;
  }

  public double getMainRightTrigger() {
		return Math.abs(MAIN_CONTROLLER.getRawAxis(3)) < DEADZONE ? 0.0 : MAIN_CONTROLLER.getRawAxis(3);
  }

  public double getArmLeftJoyY() {
    return Math.abs(ARM_CONTROLLER.getRawAxis(1)) > DEADZONE ? ARM_CONTROLLER.getRawAxis(1) : 0.0;
  }

}
