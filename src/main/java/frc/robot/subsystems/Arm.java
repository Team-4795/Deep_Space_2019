/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANDigitalInput.LimitSwitchPolarity;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.ConfigParameter;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.commands.ManualArmControl;

public class Arm extends Subsystem implements PIDOutput, PIDSource{
  
  private final CANSparkMax leftArmMotor;
  private final CANSparkMax rightArmMotor;

  DigitalInput lowerLimit;

  //private final CANPIDController armController;
  private final static double P = -0.01;
  private final static double I = 0.0;
  private final static double D = 0.00;
  private final static double F = 0.00;
  private final static double Tolerance = 6.0f;
  private final PIDController armController;

  public Arm() {

    leftArmMotor = new CANSparkMax(RobotMap.ARM_MOTOR.value, MotorType.kBrushless);
    rightArmMotor = new CANSparkMax(RobotMap.ARM_MOTOR_FOLLOWER.value, MotorType.kBrushless);

    lowerLimit = new DigitalInput(0);

    leftArmMotor.setIdleMode(IdleMode.kBrake);
    leftArmMotor.setOpenLoopRampRate(0.5);
    leftArmMotor.setClosedLoopRampRate(0.5);
    leftArmMotor.setParameter(ConfigParameter.kHardLimitRevEn, true);
    rightArmMotor.setIdleMode(IdleMode.kBrake);
    rightArmMotor.follow(leftArmMotor, true);

    armController = new PIDController(P, I, D, this, this);
    armController.setInputRange(-180.0f, 180.0f);
    armController.setOutputRange(-0.4, 0.4);
    armController.setAbsoluteTolerance(Tolerance);
    armController.setContinuous();
    /*
    armController = new CANPIDController(leftArmMotor);
    armController.setOutputRange(-0.6, 0.6);
    armController.setP(P);
    armController.setI(I);
    armController.setD(D);
    armController.setFF(F);
    */
  }

  public void balance(){
    if (Robot.climber.getClimbTime()) {
    SmartDashboard.putNumber("Angle error", armController.getError());
    armController.setSetpoint(0.0f);
    armController.enable();
    } else {
      armController.disable();
    }
  }

  public void actuate(double output) {
    leftArmMotor.set(-output);
    if(output > 0 && !lowerLimit.get())
    {
      leftArmMotor.set(0);
    }
  }

  /*public void setPosition(double position)
  {
    armController.setReference(position, ControlType.kPosition);
  }*/

  @Override
  public void pidWrite(double output) {
    leftArmMotor.set(output);
  }

  @Override
  public double pidGet() {
    return Robot.ahrs.getRoll();
  }

  @Override
  public PIDSourceType getPIDSourceType() {
    return PIDSourceType.kDisplacement;
  }

  @Override
  public void setPIDSourceType(PIDSourceType pidSource) {
    
  }

  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new ManualArmControl());
  }
}
