/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.command.PIDSubsystem;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.commands.ArcadeDrive;

public class Drivebase extends Subsystem implements PIDOutput {
  
  private final TalonSRX leftMotorOne;
  private final TalonSRX leftMotorTwo;
  private final TalonSRX leftMotorThree;
  private final TalonSRX rightMotorOne;
  private final TalonSRX rightMotorTwo;
  private final TalonSRX rightMotorThree;
  private final AHRS ahrs;
  private final PIDController turnController;

  private final static double P = 0.1;
  private final static double I = 0.0;
  private final static double D = 0.0;
  private final static double Tolerance = 5.0f;

  private final static double kP = 0.1;
  private final static double kI = 0.0;
  private final static double kD = 0.0;
  private final static int allowableError = 5;

  public final double WHEEL_DIAMETER_IN = 8.0;
  public final int ENCODER_COUNTS_PER_REV = 4096;
  public final double ENCODER_COUNTS_PER_FT = (ENCODER_COUNTS_PER_REV * 12) / (Math.PI * WHEEL_DIAMETER_IN);
  
  public Drivebase () {

    ahrs = new AHRS(SPI.Port.kMXP);
    turnController = new PIDController(P, I, D, ahrs, this);
    turnController.setInputRange(-180.0f, 180.0f);
    turnController.setOutputRange(-0.6, 0.6);
    turnController.setAbsoluteTolerance(Tolerance);
    turnController.setContinuous();

    leftMotorOne = new TalonSRX(RobotMap.LEFT_MOTOR_ONE.value);
    leftMotorTwo = new TalonSRX(RobotMap.LEFT_MOTOR_TWO.value);
    leftMotorThree = new TalonSRX(RobotMap.LEFT_MOTOR_THREE.value);
    rightMotorOne = new TalonSRX(RobotMap.RIGHT_MOTOR_ONE.value);
    rightMotorTwo = new TalonSRX(RobotMap.RIGHT_MOTOR_TWO.value);
    rightMotorThree = new TalonSRX(RobotMap.RIGHT_MOTOR_THREE.value);
    
    //config PID
    leftMotorOne.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, 0, 10);
    rightMotorOne.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, 0, 10);

    leftMotorOne.config_kP(0, kP);
    leftMotorOne.config_kI(0, kI);
    leftMotorOne.config_kD(0, kD);

    rightMotorOne.config_kP(0, kP);
    rightMotorOne.config_kI(0, kI);
    rightMotorOne.config_kD(0, kD);

    leftMotorOne.configAllowableClosedloopError(allowableError, 0, 10);
    rightMotorOne.configAllowableClosedloopError(allowableError, 0, 10);
    
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

  public double getYaw() {
    return ahrs.getYaw();
  }

  public double getLeftEncoderCount() {
    return leftMotorOne.getSensorCollection().getQuadraturePosition();
  }

  public double getRightEncoderCount() {
    return rightMotorOne.getSensorCollection().getQuadraturePosition();
  }

  public double getLeftEncoderFeet() {
    return leftMotorOne.getSensorCollection().getQuadraturePosition() / ENCODER_COUNTS_PER_FT;
  }

  public double getRightEncoderFeet() {
    return rightMotorOne.getSensorCollection().getQuadraturePosition() / ENCODER_COUNTS_PER_FT;
  }

  //should give velocity in ft per second
  public double getLeftVelocity() {
    return leftMotorOne.getSensorCollection().getQuadratureVelocity() / (10 * ENCODER_COUNTS_PER_FT);
  }
  public double getRightVelocity() {
    return rightMotorOne.getSensorCollection().getQuadratureVelocity() / (10 * ENCODER_COUNTS_PER_FT);
  }

  public void TurnToAngle(double angle) {
    ahrs.reset();
    turnController.reset();
    turnController.setSetpoint(angle);
    turnController.enable();
  }

  public void driveFeet(int feet)
  {
    leftMotorOne.set(ControlMode.Position, feet * ENCODER_COUNTS_PER_FT);
    rightMotorOne.set(ControlMode.Position, feet * ENCODER_COUNTS_PER_FT);
  }
  @Override
  public void pidWrite(double output) {
    setMotors(output, -output);
  }

  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new ArcadeDrive());
  }
}
