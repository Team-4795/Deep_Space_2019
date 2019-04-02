/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.commands.ManualArmControl;
import frc.robot.commands.ManualHatchControl;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * Add your docs here.
 */

public class Hatch extends Subsystem implements PIDOutput {
  public final TalonSRX hatchKicker; // TODO: maybe make this private
  private final TalonSRX hatchArm;
  public boolean armUp = true;
  public boolean kickerUp = false;

  public final double posDown = 0; //consistency 
  public final double posUp = 100.0; //tune later --> encoder ticks to go up

  private final double kP = 0.0;
  private final double kI = 0.0;
  private final double kD = 0.0;
  private final double kF = 0.0;
  public final int allowableError = 100; //other PID vals to tune later 

  public Hatch() {
    hatchKicker = new TalonSRX(RobotMap.HATCH_MOTOR.value);
    hatchArm = new TalonSRX(RobotMap.HATCH_ARM.value);

    Robot.initTalon(hatchKicker);
    hatchKicker.configOpenloopRamp(0.0);

    Robot.initTalon(hatchArm);
    hatchArm.configOpenloopRamp(0.0);
    hatchArm.config_kP(0, kP);
    hatchArm.config_kI(0, kI);
    hatchArm.config_kD(0, kD);
    hatchArm.config_kF(0, kF);
    hatchArm.configAllowableClosedloopError(0, allowableError, 5000);
  }

  public void setRamp(double rate) {
    hatchKicker.configOpenloopRamp(rate);
  }
  public void setKicker(double speed) {
    hatchKicker.set(ControlMode.PercentOutput, speed);
  }

  public void setArmUp(boolean up) {
    hatchArm.set(ControlMode.Position, up ? posUp : posDown);
  }

  public int getArmEncoder() {
    return hatchArm.getSensorCollection().getQuadraturePosition();
  }
  @Override
  public void pidWrite(double speed) {
    hatchArm.set(ControlMode.PercentOutput, speed);
  }
  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    setDefaultCommand(new ManualHatchControl());
  }
}
