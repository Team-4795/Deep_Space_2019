/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.commands.ManualArmControl;

public class Arm extends Subsystem {
  
  private final CANSparkMax leftArmMotor;
  private final CANSparkMax rightArmMotor;

  public Arm() {
    leftArmMotor = new CANSparkMax(RobotMap.ARM_MOTOR.value, MotorType.kBrushless);
    rightArmMotor = new CANSparkMax(RobotMap.ARM_MOTOR.value, MotorType.kBrushless);

    leftArmMotor.setIdleMode(IdleMode.kBrake);
    rightArmMotor.setIdleMode(IdleMode.kBrake);

    rightArmMotor.follow(leftArmMotor);
  }
  public void actuate(double output) {
    leftArmMotor.set(output);
  }

  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new ManualArmControl());
  }
}
