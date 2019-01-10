/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.commands.ArcadeDrive;

/**
 * An example subsystem.  You can replace me with your own Subsystem.
 */
public class Drivebase extends Subsystem {
  
  private final TalonSRX leftMotorOne;
  private final TalonSRX leftMotorTwo;
  private final TalonSRX leftMotorThree;
  private final TalonSRX rightMotorOne;
  private final TalonSRX rightMotorTwo;
  private final TalonSRX rightMotorThree;
  
  public Drivebase () {

    leftMotorOne = new TalonSRX(RobotMap.LEFT_MOTOR_ONE.value);
    leftMotorTwo = new TalonSRX(RobotMap.LEFT_MOTOR_TWO.value);
    leftMotorThree = new TalonSRX(RobotMap.LEFT_MOTOR_THREE.value);
    rightMotorOne = new TalonSRX(RobotMap.RIGHT_MOTOR_ONE.value);
    rightMotorTwo = new TalonSRX(RobotMap.RIGHT_MOTOR_TWO.value);
    rightMotorThree = new TalonSRX(RobotMap.RIGHT_MOTOR_THREE.value);

    Robot.masterTalon(leftMotorOne);
    Robot.masterTalon(rightMotorOne);

    Robot.initTalon(leftMotorTwo);
    Robot.initTalon(leftMotorThree);
    Robot.initTalon(rightMotorTwo);
    Robot.initTalon(rightMotorThree);

    leftMotorTwo.follow(leftMotorOne);
    leftMotorThree.follow(leftMotorOne);

    rightMotorTwo.follow(rightMotorOne);
    rightMotorThree.follow(rightMotorOne);
    
  }

  public void setMotors(double left, double right) {
    leftMotorOne.set(ControlMode.PercentOutput, left);
    rightMotorOne.set(ControlMode.PercentOutput, right);
  }

  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new ArcadeDrive());
  }
}
