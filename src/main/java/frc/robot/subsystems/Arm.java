/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.ConfigParameter;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.commands.ManualArmControl;

public class Arm extends Subsystem{
  
  private final CANSparkMax leftArmMotor;
  //private final CANSparkMax rightArmMotor;

  //private final CANPIDController armController;
  private final static double P = -0.08;
  private final static double I = 0.0;
  private final static double D = -0.00;
  private final static double F = 0.00;
  private final static double Tolerance = 6.0f;

  public Arm() {
    leftArmMotor = new CANSparkMax(RobotMap.ARM_MOTOR.value, MotorType.kBrushless);
    //rightArmMotor = new CANSparkMax(RobotMap.ARM_MOTOR.value, MotorType.kBrushless);

    leftArmMotor.setIdleMode(IdleMode.kBrake);
    //rightArmMotor.setIdleMode(IdleMode.kBrake);

    //rightArmMotor.follow(leftArmMotor);
    /*
    armController = new CANPIDController(leftArmMotor);
    armController.setOutputRange(-0.6, 0.6);
    armController.setP(P);
    armController.setI(I);
    armController.setD(D);
    armController.setFF(F);
    */
  }
  public void actuate(double output) {
    leftArmMotor.set(output);
  }

  public void setPosition(double position)
  {
    //armController.setReference(position, ControlType.kPosition);
  }

  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new ManualArmControl());
  }
}
