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
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.kauailabs.navx.frc.AHRS;
import com.kauailabs.navx.frc.AHRS.BoardAxis;
import com.kauailabs.navx.frc.AHRS.BoardYawAxis;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Joystick.AxisType;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.command.PIDSubsystem;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.commands.ArcadeDrive;
import frc.robot.commands.TankDrive;

public class Drivebase extends Subsystem implements PIDOutput{
  
  private final TalonSRX leftMotorOne;
  private final VictorSPX leftMotorTwo;
  private final VictorSPX leftMotorThree;
  private final TalonSRX rightMotorOne;
  private final TalonSRX rightMotorTwo;
  private final TalonSRX rightMotorThree;
  private final AHRS ahrs;
  private final PIDController turnController;

  private final static double P = -0.08;
  private final static double I = 0.0;
  private final static double D = -0.00;
  private final static double Tolerance = 6.0f;

  private final double WHEEL_DIAMETER_IN = 8.0;
  private final int ENCODER_COUNTS_PER_REV = 4096;
  private final double ENCODER_COUNTS_PER_FT = 15689.8;//(ENCODER_COUNTS_PER_REV * 12) / (Math.PI * WHEEL_DIAMETER_IN)
  
  public Drivebase () {

    ahrs = new AHRS(SPI.Port.kMXP);
    turnController = new PIDController(P, I, D, ahrs, this);
    turnController.setInputRange(-180.0f, 180.0f);
    turnController.setOutputRange(-0.6, 0.6);
    turnController.setAbsoluteTolerance(Tolerance);
    turnController.setContinuous();

    leftMotorOne = new TalonSRX(RobotMap.LEFT_MOTOR_ONE.value);
    leftMotorTwo = new VictorSPX(RobotMap.LEFT_MOTOR_TWO.value);
    leftMotorThree = new VictorSPX(RobotMap.LEFT_MOTOR_THREE.value);
    rightMotorOne = new TalonSRX(RobotMap.RIGHT_MOTOR_ONE.value);
    rightMotorTwo = new TalonSRX(RobotMap.RIGHT_MOTOR_TWO.value);
    rightMotorThree = new TalonSRX(RobotMap.RIGHT_MOTOR_THREE.value);
    
    Robot.masterTalon(leftMotorOne);
    Robot.masterTalon(rightMotorOne);

    Robot.initVictor(leftMotorTwo);
    Robot.initVictor(leftMotorThree);
    Robot.initTalon(rightMotorTwo);
    Robot.initTalon(rightMotorThree);


    rightMotorOne.setInverted(true);

    rightMotorTwo.setInverted(true);
    rightMotorThree.setInverted(true);

    leftMotorTwo.follow(leftMotorOne);
    leftMotorThree.follow(leftMotorOne);

    rightMotorTwo.follow(rightMotorOne);
    rightMotorThree.follow(rightMotorOne);
    
    rightMotorOne.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
    leftMotorOne.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);

    rightMotorOne.setSelectedSensorPosition(0);
    leftMotorOne.setSelectedSensorPosition(0);

    ahrs.reset();
  }

  public void setMotors(double left, double right) {
    leftMotorOne.set(ControlMode.PercentOutput, left);
    rightMotorOne.set(ControlMode.PercentOutput, right);
  }

  public double getYaw() {
    return ahrs.getYaw();
  }

  public double getPitch() {
    return ahrs.getPitch();
  }

  public double getRoll() {
    return ahrs.getRoll();
  }
  public double getLeftEncoderCount() {
    return leftMotorOne.getSelectedSensorPosition();
  }

  public double getRightEncoderCount() {
    return rightMotorOne.getSelectedSensorPosition();
  }

  public double getLeftEncoderFeet() {
    return leftMotorOne.getSelectedSensorPosition() / ENCODER_COUNTS_PER_FT;
  }

  public double getRightEncoderFeet() {
    return rightMotorOne.getSelectedSensorPosition() / ENCODER_COUNTS_PER_FT;
  }

  //should give velocity in ft per second
  public double getLeftVelocity() {
    return leftMotorOne.getSelectedSensorVelocity() * 10 / ENCODER_COUNTS_PER_FT;
  }

  public double getRightVelocity() {
    return rightMotorOne.getSelectedSensorVelocity() * 10 / ENCODER_COUNTS_PER_FT;
  }

  public void TurnToAngle(double angle) {
    ahrs.reset();
    turnController.reset();
    SmartDashboard.putString("Axis", ahrs.getBoardYawAxis().toString());
    turnController.setSetpoint(angle);
    turnController.enable();
  }

  public void resetEncoders() {
    rightMotorOne.setSelectedSensorPosition(0);
    leftMotorOne.setSelectedSensorPosition(0);
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
