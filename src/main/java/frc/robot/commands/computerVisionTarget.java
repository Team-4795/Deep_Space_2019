/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class computerVisionTarget extends CommandGroup {
  /**
   * Add your docs here.
   */
  public computerVisionTarget() {
    double radangle = SmartDashboard.getNumber("Angle", 0);
    double angle = Math.toDegrees(radangle);

    SmartDashboard.putNumber("fuckduck", angle);
    
    double feet = SmartDashboard.getNumber("Z", 1);

    addSequential(new TurnToAngle(angle, 2));
    addSequential(new DriveTillLine());
  }
}
