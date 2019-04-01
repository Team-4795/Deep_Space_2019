/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

// test comment
package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.buttons.POVButton;
import frc.robot.commands.ArcadeDrive;
import frc.robot.commands.ArmBalance;
import frc.robot.commands.AutoClimb;
import frc.robot.commands.ClimberToPosition;
import frc.robot.commands.DriveForward;
import frc.robot.commands.IntakeCargo;
import frc.robot.commands.ManualArmControl;
import frc.robot.commands.ManualClimberControl;
import frc.robot.commands.OuttakeCargo;
import frc.robot.commands.SlowRoll;
import frc.robot.commands.ToggleClimbTime;
import frc.robot.commands.TurnToAngle;
import frc.robot.triggers.DrivetrainOverride;
import frc.robot.triggers.IntakeTrigger;
import frc.robot.triggers.LeftTriggerPressed;
import frc.robot.triggers.ManualArmTrigger;
import frc.robot.commands.ArmToPosition;
import frc.robot.commands.TurnToLine;
import frc.robot.commands.ToTarget;

public class OI {

  private static final double DEADZONE = 0.15 ;

  public Joystick MAIN_CONTROLLER, ARM_CONTROLLER;
  private JoystickButton XButton, YButton, AButton, BButton, ArmBButton, RightBumper, ArmLeftBumper, ArmRightBumper;
  private double value;
  public ManualArmTrigger ArmOverride;
  private IntakeTrigger SlowMode;
  private LeftTriggerPressed CargoOuttake;
  private POVButton ArmDPadUp, ArmDPadDown, MainDPadDown, MainDPadUp, ArmDPadRight;
  public DrivetrainOverride drivetrainOverride;

  public OI() { 
    
  }

  public void init() {
    MAIN_CONTROLLER = new Joystick(RobotMap.MAIN_CONTROLLER.value);
    ARM_CONTROLLER = new Joystick(RobotMap.ARM_CONTROLLER.value);

    YButton = new JoystickButton(MAIN_CONTROLLER, 4);
    AButton = new JoystickButton(MAIN_CONTROLLER, 1);
    XButton = new JoystickButton(MAIN_CONTROLLER, 3);
    BButton = new JoystickButton(MAIN_CONTROLLER, 2);
    ArmBButton = new JoystickButton(ARM_CONTROLLER, 2);
    ArmOverride = new ManualArmTrigger();
    ArmDPadUp = new POVButton(ARM_CONTROLLER, 0);
    ArmDPadDown = new POVButton(ARM_CONTROLLER, 180);
    RightBumper = new JoystickButton(MAIN_CONTROLLER, 6);
    MainDPadUp = new POVButton(MAIN_CONTROLLER, 0);
    MainDPadDown = new POVButton(MAIN_CONTROLLER, 180);
    ArmDPadRight = new POVButton(ARM_CONTROLLER, 90);
    ArmLeftBumper = new JoystickButton(ARM_CONTROLLER, 5);
    ArmRightBumper = new JoystickButton(ARM_CONTROLLER, 6);
    SlowMode = new IntakeTrigger();
    CargoOuttake = new LeftTriggerPressed();
    drivetrainOverride = new DrivetrainOverride();

    ArmBButton.whenPressed(new ToggleClimbTime());
    
    MainDPadUp.whileHeld(new ManualClimberControl(.8));
    MainDPadDown.whileHeld(new ManualClimberControl(-.8));
    XButton.whenPressed(new AutoClimb());

    ArmDPadDown.whenPressed(new ArmToPosition(-78.5));
    ArmDPadUp.whenPressed(new ArmToPosition(-17.38));
    ArmDPadRight.whenPressed(new ArmToPosition(-42.3));

    ArmOverride.whileActive(new ManualArmControl());
    ArmRightBumper.whileHeld(new IntakeCargo());
    CargoOuttake.whileActive(new OuttakeCargo(.9));
    SlowMode.whileActive(new SlowRoll());
    //drivetrainOverride.whileActive(new ArcadeDrive());

    //AButton.whenPressed(new TurnToLine(5));
  }

  public void rumbleMain() {
    MAIN_CONTROLLER.setRumble(RumbleType.kLeftRumble, 0.6);
    MAIN_CONTROLLER.setRumble(RumbleType.kRightRumble,  0.6);
  }

  public void rumbleArm() {
    ARM_CONTROLLER.setRumble(RumbleType.kLeftRumble,  0.6);
    ARM_CONTROLLER.setRumble(RumbleType.kRightRumble,  0.6);
  }

  public void stopRumble(boolean mainStop, boolean armStop) {
    if (mainStop) {
      MAIN_CONTROLLER.setRumble(RumbleType.kLeftRumble, 0.0);
      MAIN_CONTROLLER.setRumble(RumbleType.kRightRumble,  0.0);
    }
    if (armStop) {
      ARM_CONTROLLER.setRumble(RumbleType.kLeftRumble,  0.0);
      ARM_CONTROLLER.setRumble(RumbleType.kRightRumble,  0.0);
    }
  }

  //Drivebase control
  public double getMainLeftJoyY() {
    double value = MAIN_CONTROLLER.getRawAxis(1);
    return Math.abs(value) > DEADZONE ? ((Math.abs(value) - DEADZONE) * Math.abs(value) / (0.85 * value)) : 0.0;
  }
  
  //For tankdrive control (unused)
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
  }

  //Climber wheel actuation & outtaking
  public double getMainLeftTrigger() {
    double value = MAIN_CONTROLLER.getRawAxis(2);
    return Math.abs(value) > DEADZONE ? ((Math.abs(value) - DEADZONE) * Math.abs(value) / (0.85 * value)) : 0.0;
  }

  public boolean getMainBButtonPressed() {
    return MAIN_CONTROLLER.getRawButtonPressed(2);
  }

  public boolean getMainBButton() {
    return MAIN_CONTROLLER.getRawButton(2);
  }

  public boolean getMainLeftBumperPressed() {
    return MAIN_CONTROLLER.getRawButtonPressed(5);
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

  public boolean getArmXButton() {
    return ARM_CONTROLLER.getRawButton(3);
  }

  public boolean getArmAButton() {
    return ARM_CONTROLLER.getRawButton(1);
  }

  public boolean getArmYButton() {
    return ARM_CONTROLLER.getRawButton(4);
  }

  public boolean getMainYButtonPressed() {
    return MAIN_CONTROLLER.getRawButtonPressed(4);
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
