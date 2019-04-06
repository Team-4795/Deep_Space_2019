/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.commands.ManualArmControl;
import frc.robot.commands.ManualHatchControl;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Add your docs here.
 */

public class Hatch extends Subsystem {
  public final TalonSRX hatchKicker;
  public final TalonSRX hatchArm;
  public boolean armUp = true;
  public boolean kickerUp = false;
  private boolean stalled = false;

  private final double posDown = -1800.0;
  private final double posUp = 0.0;

  //not used
  private final double kP = 0.005;
  private final double kI = 0.0;
  private final double kD = 0.0;
  private final double kF = 0.0;
  public final int allowableError = 100; //other PID vals to tune later 

  public Hatch() {
    hatchKicker = new TalonSRX(RobotMap.HATCH_KICKER_MOTOR.value);
    hatchArm = new TalonSRX(RobotMap.HATCH_ARM_MOTOR.value);

    Robot.initTalon(hatchKicker);
    hatchKicker.configOpenloopRamp(0.0);

    Robot.initTalon(hatchArm);
    hatchArm.configClosedLoopPeakOutput(0, .3);
    hatchArm.setNeutralMode(NeutralMode.Brake);
    hatchArm.configOpenloopRamp(0.1);
    hatchArm.config_kP(0, kP);
    hatchArm.config_kI(0, kI);
    hatchArm.config_kD(0, kD);
    hatchArm.config_kF(0, kF);
    hatchArm.configAllowableClosedloopError(0, allowableError, 5000);
    hatchArm.configPeakCurrentLimit(8, 10);
    hatchArm.configPeakCurrentDuration(200, 10);
    hatchArm.configContinuousCurrentLimit(7, 10);
  }

  public void setRamp(double rate) {
    hatchKicker.configOpenloopRamp(rate);
  }
  public void setKicker(double speed) {
    hatchKicker.set(ControlMode.PercentOutput, speed);
  }

  public void setArmUp(boolean up) {
    //hatchArm.set(ControlMode.Position, up ? posUp : posDown);
    if (hatchArm.getSensorCollection().getQuadraturePosition() < -2000 && !up) {
      hatchArm.set(ControlMode.PercentOutput, 0.0);
      stalled = false;
    }
    else if (!up) {
      hatchArm.set(ControlMode.PercentOutput, -01);
    }
    else if (up && hatchArm.getSensorCollection().getQuadraturePosition() > -100) {
      hatchArm.set(ControlMode.PercentOutput, 0.3);
    }
    else if (up) {
      hatchArm.set(ControlMode.PercentOutput, 0.7);
    }
    else {
      hatchArm.set(ControlMode.PercentOutput, 0.0);
    }
    if (hatchArm.getOutputCurrent() > 9) {
      stalled = true;
    }
    SmartDashboard.putNumber("hatch current", hatchArm.getOutputCurrent());
    SmartDashboard.putBoolean("stalled", stalled);
  }

  public int getArmEncoder() {
    return hatchArm.getSensorCollection().getQuadraturePosition();
  }
  public void resetArmEncoder() {
    hatchArm.setSelectedSensorPosition(0);
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    setDefaultCommand(new ManualHatchControl());
  }
}
