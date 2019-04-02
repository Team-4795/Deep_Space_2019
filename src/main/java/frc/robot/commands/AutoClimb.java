/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class AutoClimb extends CommandGroup {
  /**
   * Add your docs here.
   */
  public AutoClimb() {
  
    addParallel(new ClimberToPosition(220));
    addParallel(new ArmBalance());
  }
}
