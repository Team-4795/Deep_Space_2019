/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
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
import frc.robot.commands.Climb;

public class Arm extends Subsystem {
  
  private final TalonSRX armMotor;

  public Arm() {
    armMotor = new TalonSRX(RobotMap.ARM_MOTOR.value);

    Robot.masterTalon(armMotor);
  }
  public void actuate(double output) {
    armMotor.set(ControlMode.PercentOutput, output);
  }

  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new Climb());
  }
}
