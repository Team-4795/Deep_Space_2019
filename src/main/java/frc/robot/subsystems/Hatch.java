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
import edu.wpi.first.wpilibj.command.Subsystem;

// import java.lang.System;

/**
 * Add your docs here.
 */
 

public class Hatch extends Subsystem {
  public final TalonSRX hatchMotor;

  // double start;
  // double stop;
  // double inital; 

  public Hatch() {
    hatchMotor = new TalonSRX(RobotMap.HATCH_MOTOR.value);

    Robot.initTalon(hatchMotor);
  }

  public void set(double speed) {
    hatchMotor.set(ControlMode.PercentOutput, speed);
  }

  public boolean getForwardlimit()
  {
    return hatchMotor.getSensorCollection().isFwdLimitSwitchClosed();
  }

  public boolean getReverselimit()
  {
    return hatchMotor.getSensorCollection().isRevLimitSwitchClosed();
  }
  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
