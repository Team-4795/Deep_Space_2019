/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;
import frc.robot.Robot;
/**
 * Add your docs here.
 */
public class Intake extends Subsystem {
  public final TalonSRX wheelMotor;
  public final VictorSPX rollerMotor;

  public Intake() {
    wheelMotor = new TalonSRX(RobotMap.INTAKE_MOTOR.value);
    rollerMotor = new VictorSPX(RobotMap.INTAKE_MOTOR_FOLLOWER.value);

    Robot.masterTalon(wheelMotor);
    Robot.initVictor(rollerMotor);

    rollerMotor.follow(wheelMotor);
  }

  public void set(double speed)
  {
    wheelMotor.set(ControlMode.PercentOutput, speed);
  }

  public boolean getRevLimitSwitch() {
    return wheelMotor.getSensorCollection().isRevLimitSwitchClosed();
  }

  public boolean getFwdLimitSwitch() {
    return wheelMotor.getSensorCollection().isFwdLimitSwitchClosed();
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
