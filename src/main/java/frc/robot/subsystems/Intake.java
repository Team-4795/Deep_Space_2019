/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class Intake extends Subsystem {
  public final Spark motorIn;
  public final boolean limitSwitch;

  public Intake() {
    motorIn = new Spark(RobotMap.INTAKE_MOTOR.value);
    limitSwitch = false;
  }

  public void set(double speed)
  {
    if(!(speed > 0 && !limitSwitch))
    {
      motorIn.set(0);
    }
    motorIn.set(speed);
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
