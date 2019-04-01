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

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;
import frc.robot.commands.ManualIntakeControl;
import frc.robot.commands.SlowRoll;
import frc.robot.Robot;
/**
 * Add your docs here.
 */
public class CargoIntake extends Subsystem {

  private final TalonSRX wheelMotor;
  private final TalonSRX rollerMotor;
  private DigitalInput cargoZero, cargoOne, cargoTwo, cargoThree;

  public CargoIntake() {
    wheelMotor = new TalonSRX(RobotMap.CLIMBER_WHEELS.value);
    rollerMotor = new TalonSRX(RobotMap.INTAKE_MOTOR.value);

    Robot.initTalon(wheelMotor);
    Robot.initTalon(rollerMotor);

    wheelMotor.configOpenloopRamp(0);
    rollerMotor.configOpenloopRamp(0);

    cargoZero = new DigitalInput(0);
    cargoOne = new DigitalInput(1);
    cargoTwo = new DigitalInput(2);
    cargoThree = new DigitalInput(3);

  }
  public boolean getLimitZero() {
    return cargoZero.get();
  }
  public boolean getLimitOne() {
    return cargoOne.get();
  }
  public boolean getLimitTwo() {
    return cargoTwo.get();
  }
  public boolean getLimitThree() {
    return cargoThree.get();
  }
  
  public boolean hasBall() {
    return cargoZero.get() || cargoOne.get();
  }

  public void setWheels(double speed)
  {
    wheelMotor.set(ControlMode.PercentOutput, speed);
  }

  public void setRoller(double speed)
  {
    rollerMotor.set(ControlMode.PercentOutput, speed);
  }

  /*
  public boolean getRevLimitSwitch() {
    return wheelMotor.getSensorCollection().isRevLimitSwitchClosed();
  }

  public boolean getFwdLimitSwitch() {
    return wheelMotor.getSensorCollection().isFwdLimitSwitchClosed();
  }
*/
  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    setDefaultCommand(new SlowRoll());
  }
}
