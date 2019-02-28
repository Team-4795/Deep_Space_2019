/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.cscore.VideoSink;
import edu.wpi.cscore.VideoSource.ConnectionStrategy;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.TimedRobot;

import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.ArcadeDrive;
import frc.robot.commands.DriveForward;
import frc.robot.commands.TurnToAngle;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivebase;
import frc.robot.subsystems.Hatch;
import frc.robot.subsystems.Intake;

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
  public static Intake intake;
  public static ColorSensor colorsensor;
  public static Hatch hatch;
  public static Climber climber;
  public static AHRS ahrs;
  public static UsbCamera hatchCam;
  public static UsbCamera cargoCam;
  public static VideoSink switcher;

  @Override
  public void robotInit() {

    ahrs = new AHRS(SPI.Port.kMXP);
    drivebase = new Drivebase();
    arm = new Arm();
    climber = new Climber();
    intake = new Intake();
    hatch = new Hatch();
    oi = new OI();
    //hatchCam = new UsbCamera("hatch", 0);
    hatchCam = CameraServer.getInstance().startAutomaticCapture("hatchCam", 0);
    //CameraServer.getInstance().addCamera(hatchCam);
    //CameraServer.getInstance().startAutomaticCapture(hatchCam);
    //CameraServer.getInstance().removeCamera("hatchCam");
    cargoCam = CameraServer.getInstance().startAutomaticCapture("cargoCam", 1);

    hatchCam.setFPS(15);
    hatchCam.setResolution(780, 640);
    hatchCam.setConnectionStrategy(ConnectionStrategy.kKeepOpen);
    hatchCam.setExposureAuto();
    hatchCam.setWhiteBalanceAuto();
    
    cargoCam.setFPS(15);
    cargoCam.setResolution(780, 640);
    cargoCam.setConnectionStrategy(ConnectionStrategy.kKeepOpen);
    cargoCam.setExposureAuto();
    cargoCam.setWhiteBalanceAuto();

    switcher = CameraServer.getInstance().addSwitchedCamera("Switched Camera");
  }

  @Override
  public void disabledInit() {
    Robot.climber.setClimbTime(false);
  }

  public void disabledPeriodic() {
    Scheduler.getInstance().run();
  }

  public void autonomousInit() {
    // takes argument: angle, timeout
    double dist = -1 * SmartDashboard.getNumber("Z", 3) - 0.5;
    Scheduler.getInstance().add(new DriveForward(dist, 2));
    ahrs.reset();
    Robot.climber.resetEnc();
    
  }

  public void autonomousPeriodic() {
    Scheduler.getInstance().run();
  }

  public void teleopInit() {

  }

  public void teleopPeriodic() {
    Scheduler.getInstance().run();
    SmartDashboard.putBoolean("ClimbTime", Robot.climber.getClimbTime());
    SmartDashboard.putNumber("Arm Encoder", Robot.arm.getPos());
    SmartDashboard.putBoolean("Arm Top Limit", Robot.arm.getTopLimit());
    SmartDashboard.putBoolean("Climber Top Limit", Robot.climber.getTopLimit());
    SmartDashboard.putBoolean("Climber Bot Limit", Robot.climber.getBotLimit());
    SmartDashboard.putBoolean("Has Ball", Robot.intake.hasBall());
    if (Robot.arm.getTopLimit()) {
      Robot.arm.resetEnc();
    }
  }

  public void testPeriodic() {
  }

  public static void masterTalon(TalonSRX motor) {
    motor.setNeutralMode(NeutralMode.Brake);
    motor.configContinuousCurrentLimit(10, 0);
    motor.configPeakCurrentLimit(12, 0);
    motor.configPeakCurrentDuration(20, 0);
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
