/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.cscore.VideoSink;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.TimedRobot;

import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivebase;
import frc.robot.subsystems.Hatch;
import frc.robot.subsystems.CargoIntake;

import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.kauailabs.navx.frc.AHRS;

public class Robot extends TimedRobot {

  public static OI oi;
  public static Drivebase drivebase;
  public static Arm arm;
  public static CargoIntake intake;
  public static ColorSensor colorsensor;
  public static Hatch hatch;
  public static Climber climber;
  public static AHRS ahrs;
  public static UsbCamera hatchCam;
  public static UsbCamera cargoCam;
  public static VideoSink switcher;
  public static ColorSensor csLeft = ColorSensor.getInstanceMXP();
  public static ColorSensor csRight = ColorSensor.getInstanceOnboard();
  public static PowerDistributionPanel pdp;

  @Override
  public void robotInit() {

    pdp = new PowerDistributionPanel();
    ahrs = new AHRS(SPI.Port.kMXP);
    drivebase = new Drivebase();
    arm = new Arm();
    climber = new Climber();
    intake = new CargoIntake();
    hatch = new Hatch();
    oi = new OI();
    oi.init();
  }

  @Override
  public void disabledInit() {
    Robot.climber.setClimbTime(false);
    Robot.hatch.armUp = true;
    Robot.hatch.kickerUp = false;
  }

  public void disabledPeriodic() {
    SmartDashboard.putNumber("Roll", ahrs.getRoll());
  }

  public void autonomousInit() {
    Scheduler.getInstance().enable();
    ahrs.reset();
    Robot.climber.resetEnc();
    Robot.hatch.resetArmEncoder();
  }

  public void autonomousPeriodic() {
    Scheduler.getInstance().run();
  }

  public void teleopInit() {
  }

  public void teleopPeriodic() {
    SmartDashboard.putNumber("current (0)", pdp.getCurrent(0));
    SmartDashboard.putNumber("current (1)", pdp.getCurrent(1));
    SmartDashboard.putNumber("current (2)", pdp.getCurrent(2));
    SmartDashboard.putNumber("current (3)", pdp.getCurrent(3));
    SmartDashboard.putNumber("current (4)", pdp.getCurrent(4));
    SmartDashboard.putNumber("current (5)", pdp.getCurrent(5));
    SmartDashboard.putNumber("current (6)", pdp.getCurrent(6));
    SmartDashboard.putNumber("current (7)", pdp.getCurrent(7));
    SmartDashboard.putNumber("current (8)", pdp.getCurrent(8));
    SmartDashboard.putNumber("current (9)", pdp.getCurrent(9));
    SmartDashboard.putBoolean("withinTolerance", Robot.arm.withinTolerance());
    Scheduler.getInstance().run();
    SmartDashboard.putBoolean("ClimbTime", Robot.climber.getClimbTime());
    SmartDashboard.putNumber("Arm Encoder", Robot.arm.getPos());
    //SmartDashboard.putBoolean("zero", Robot.intake.getLimitZero());
    //SmartDashboard.putBoolean("one", Robot.intake.getLimitOne());
    //SmartDashboard.putBoolean("two", Robot.intake.getLimitTwo());
    //SmartDashboard.putBoolean("three", Robot.intake.getLimitThree());
    //SmartDashboard.putBoolean("Arm Top Limit", Robot.arm.getTopLimit());
    //SmartDashboard.putBoolean("Climber Top Limit", Robot.climber.getTopLimit());
    //SmartDashboard.putBoolean("Climber Bot Limit", Robot.climber.getBotLimit());
    SmartDashboard.putNumber("Color Sensor Left", csLeft.getColor().clear);
    SmartDashboard.putNumber("Color Sensor Right", csRight.getColor().clear);
    SmartDashboard.putBoolean("Has Ball", Robot.intake.hasBall());
    SmartDashboard.putNumber("Roll", ahrs.getRoll());
    //SmartDashboard.putBoolean("Arm Trigger", Robot.oi.ArmOverride.get());
    if (Robot.arm.getTopLimit()) {
      Robot.arm.resetEnc();
    }
    if (Robot.climber.getClimbTime()) {
      Robot.oi.rumbleArm();
      Robot.oi.rumbleMain();
    }
    else {
      Robot.oi.stopRumble(true, true);
    }
  }

  public void testPeriodic() {
  }

  public static void masterTalon(TalonSRX motor) {
    motor.setNeutralMode(NeutralMode.Brake);
    motor.configContinuousCurrentLimit(12, 0);
    motor.configPeakCurrentLimit(14, 0);
    motor.configPeakCurrentDuration(50, 0);
    motor.enableCurrentLimit(true);
    motor.configOpenloopRamp(0.2, 0);
    motor.configClosedloopRamp(0.2, 0);
  }

  public static void initTalon(TalonSRX motor) {
    motor.setNeutralMode(NeutralMode.Brake);
    motor.neutralOutput();
    motor.setSensorPhase(false);
    motor.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen, 0);
    motor.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen, 0);
    motor.configNominalOutputForward(0.0, 0);
    motor.configNominalOutputReverse(0.0, 0);
  }

  public static void initVictor(VictorSPX motor) {
    motor.setNeutralMode(NeutralMode.Brake);
    motor.neutralOutput();
    motor.setSensorPhase(false);
    motor.configNominalOutputForward(0.0, 0);
    motor.configNominalOutputReverse(0.0, 0);
  }

}
