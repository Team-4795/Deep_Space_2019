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
import com.revrobotics.CANDigitalInput;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANDigitalInput.LimitSwitch;
import com.revrobotics.CANDigitalInput.LimitSwitchPolarity;
import com.revrobotics.CANPIDController.AccelStrategy;
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

  //soft limit for arm in encoder ticks
  private final double lowerLimit = -77.69;

  private final CANPIDController armController;

  //PIDF values for balancing when climbing
  private static double Pb = 0.018;
  private static double Ib = 0.0;
  private static double Db = 0.00;

  //PID values for moving arm to position
  private static double P = 0.00055;
  private static double I = 0.00000;
  private static double D = 0.00;
  private static double F = 0.00;

  private final static double Tolerance = 6.0f;
  private final PIDController armBalancer;
  private final CANEncoder armEnc;
  private final CANDigitalInput topLimit;

  public Arm() {

    leftArmMotor = new CANSparkMax(RobotMap.ARM_MOTOR.value, MotorType.kBrushless);
    rightArmMotor = new CANSparkMax(RobotMap.ARM_MOTOR_FOLLOWER.value, MotorType.kBrushless);

    armEnc = new CANEncoder(leftArmMotor);
    topLimit = new CANDigitalInput(leftArmMotor, LimitSwitch.kReverse, LimitSwitchPolarity.kNormallyOpen);

    leftArmMotor.setIdleMode(IdleMode.kBrake);
    leftArmMotor.setOpenLoopRampRate(0.5);
    leftArmMotor.setClosedLoopRampRate(0.5);
    leftArmMotor.setParameter(ConfigParameter.kHardLimitRevEn, true);
    leftArmMotor.setParameter(ConfigParameter.kCanID, RobotMap.ARM_MOTOR.value);
    leftArmMotor.setInverted(true);

    rightArmMotor.setParameter(ConfigParameter.kCanID, RobotMap.ARM_MOTOR_FOLLOWER.value);
    rightArmMotor.setIdleMode(IdleMode.kBrake);
    rightArmMotor.follow(leftArmMotor, true);

    armBalancer = new PIDController(Pb, Ib, Db, this, this);
    armBalancer.setInputRange(-180.0f, 180.0f);
    armBalancer.setOutputRange(-0.45, 0.45);
    armBalancer.setAbsoluteTolerance(Tolerance);
    armBalancer.setContinuous();

    armController = new CANPIDController(leftArmMotor);
    armController.setP(P, 0);
    armController.setI(I, 0);
    armController.setIZone(0.0, 0);
    armController.setD(D, 0);
    armController.setFF(F, 0);
    armController.setOutputRange(-0.45, 0.45, 0);
    armController.setSmartMotionMaxVelocity(1200, 0);
    armController.setSmartMotionMaxAccel(800, 0); 
    armController.setSmartMotionAccelStrategy(AccelStrategy.kTrapezoidal, 0);
    armController.setSmartMotionAllowedClosedLoopError(0.8, 0); 
  }

  public void balance(){
    if (Robot.climber.getClimbTime()) {
    SmartDashboard.putNumber("Angle error", armBalancer.getError());
    armBalancer.setSetpoint(0.0f);
    armBalancer.enable();
    } else {
      armBalancer.disable();
    }
  }

  public double getPos() {
    return armEnc.getPosition();
  }

  public Boolean getTopLimit() {
    return topLimit.get();
  }

  public void resetEnc() {
    armEnc.setPosition(0.0);
  }

  public void actuate(double output) {
    leftArmMotor.set(output);
    
    if(output < 0 && armEnc.getPosition() < lowerLimit && !Robot.climber.getClimbTime())
    {
      leftArmMotor.set(0);
    } 
    else if (output > 0 && armEnc.getPosition() > -3.0 && !Robot.climber.getClimbTime())
    {
      leftArmMotor.set(0);
    } 
  }

  public void autoActuate(double position) {
    if (!Robot.climber.getClimbTime()) {
    armController.setReference(position, ControlType.kSmartMotion, 0);
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
