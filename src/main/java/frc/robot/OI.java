/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

// test comment
package frc.robot;

import java.awt.Button;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.buttons.POVButton;
import edu.wpi.first.wpilibj.buttons.Trigger;
import frc.robot.commands.ArmPIDBalance;
import frc.robot.commands.AutoClimb;
import frc.robot.commands.ClimberPIDControl;
import frc.robot.commands.ManualArmControl;
import frc.robot.commands.ManualClimberControl;
import frc.robot.commands.ToggleClimbTime;
import frc.robot.commands.AutoPositionArm;
import frc.robot.commands.CameraToggle;

public class OI {

  private static final double DEADZONE = 0.15 ;

  private final Joystick MAIN_CONTROLLER, ARM_CONTROLLER;
  private final JoystickButton XButton, YButton, AButton, BButton, ArmBButton;
  private double value;
  private final TriggerManualArm ArmOverride;
  private final POVButton ArmDPadUp, ArmDPadDown, MainDPadDown;

  public OI() { 
    MAIN_CONTROLLER = new Joystick(RobotMap.MAIN_CONTROLLER.value);
    ARM_CONTROLLER = new Joystick(RobotMap.ARM_CONTROLLER.value);

    YButton = new JoystickButton(MAIN_CONTROLLER, 4);
    AButton = new JoystickButton(MAIN_CONTROLLER, 1);
    XButton = new JoystickButton(MAIN_CONTROLLER, 3);
    BButton = new JoystickButton(MAIN_CONTROLLER, 2);
    ArmBButton = new JoystickButton(ARM_CONTROLLER, 2);
    ArmOverride = new TriggerManualArm();
    ArmDPadUp = new POVButton(ARM_CONTROLLER, 0);
    ArmDPadDown = new POVButton(ARM_CONTROLLER, 180);

    MainDPadDown = new POVButton(ARM_CONTROLLER, 90);

    MainDPadDown.whenPressed(new CameraToggle());
    ArmBButton.whenPressed(new ToggleClimbTime());
    
    YButton.whileHeld(new ManualClimberControl(.4));
    AButton.whileHeld(new ManualClimberControl(-.4));
    XButton.whenPressed(new AutoClimb());
    //XButton.whenPressed(new ClimberPIDControl(218));
    //BButton.whenPressed(new ClimberPIDControl(9));
    ArmDPadDown.whenPressed(new AutoPositionArm(35.0));
    ArmDPadUp.whenPressed(new AutoPositionArm(15.0));

    ArmOverride.whileActive(new ManualArmControl());
  }

  //Drivebase control
  public double getMainLeftJoyY() {
    double value = MAIN_CONTROLLER.getRawAxis(1);
    return Math.abs(value) > DEADZONE ? ((Math.abs(value) - DEADZONE) * Math.abs(value) / (0.85 * value)) : 0.0;
  }
  
  //For tankdrive control
  public double getMainRightJoyY() {
    double value = MAIN_CONTROLLER.getRawAxis(5);
    return Math.abs(value) > DEADZONE ? ((Math.abs(value) - DEADZONE) * Math.abs(value) / (0.85 * value)) : 0.0;
  }

  //Drivebase control
  public double getMainRightJoyX() {
    double value = MAIN_CONTROLLER.getRawAxis(4);
    return Math.abs(value) > DEADZONE ? ((Math.abs(value) - DEADZONE) * Math.abs(value) / (0.85 * value)) : 0.0;
  }

  //Drivebase throttle
  public double getMainRightTrigger() {
    double value = MAIN_CONTROLLER.getRawAxis(3);
    return Math.abs(value) > DEADZONE ? ((Math.abs(value) - DEADZONE) * Math.abs(value) / (0.85 * value)) : 0.0;
    //return Math.abs(value) > DEADZONE ? value : 0.0;
  }

  //Climber wheel actuation
  public double getMainLeftTrigger() {
    double value = MAIN_CONTROLLER.getRawAxis(2);
    return Math.abs(value) > DEADZONE ? ((Math.abs(value) - DEADZONE) * Math.abs(value) / (0.85 * value)) : 0.0;
    //return Math.abs(value) > DEADZONE ? value : 0.0;
  }

  public boolean getMainLeftBumper() {
    return MAIN_CONTROLLER.getRawButton(5);
  }

  public boolean getArmLeftBumper() {
    return ARM_CONTROLLER.getRawButton(5);
  }

  public boolean getArmRightBumper() {
    return ARM_CONTROLLER.getRawButton(6);
  }

  //toggles which way is "forward" for drivebase
  public boolean getMainRightBumperPressed() {
    return MAIN_CONTROLLER.getRawButtonPressed(6);
  }

  //Cargo outtake
  public boolean getArmXButton() {
    return ARM_CONTROLLER.getRawButton(3);
  }

  //Cargo intake
  public boolean getArmAButton() {
    return ARM_CONTROLLER.getRawButton(1);
  }

  //Hatch control
  public boolean getArmYButton() {
    return ARM_CONTROLLER.getRawButton(4);
  }

  //Arm control
  public double getArmLeftJoyY() {
    double value = ARM_CONTROLLER.getRawAxis(1);
    return Math.abs(value) > DEADZONE ? ((Math.abs(value) - DEADZONE) * Math.abs(value) / (0.85 * value)) : 0.0;
  }

  //Arm throttle
  public double getArmRightTrigger() {
    double value = ARM_CONTROLLER.getRawAxis(3);
    return Math.abs(value) > DEADZONE ? ((Math.abs(value) - DEADZONE) * Math.abs(value) / (0.85 * value)) : 0.0;
  }

  public double getArmLeftTrigger() {
    double value = ARM_CONTROLLER.getRawAxis(2);
    return Math.abs(value) > DEADZONE ? ((Math.abs(value) - DEADZONE) * Math.abs(value) / (0.85 * value)) : 0.0;
  }
}
