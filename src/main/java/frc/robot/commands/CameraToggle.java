/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.cscore.VideoSource.ConnectionStrategy;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.command.InstantCommand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;

/**
 * Add your docs here.
 */
public class CameraToggle extends InstantCommand {
  private boolean toggle = false;
  public CameraToggle() {
    super();
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  // Called once when the command executes
  @Override
  protected void initialize() {
    if (toggle) {
      SmartDashboard.putBoolean("Hatch Cam Active", toggle);
      Robot.hatchCam.setFPS(15);
      Robot.hatchCam.setResolution(240, 360);
      Robot.hatchCam.setConnectionStrategy(ConnectionStrategy.kKeepOpen);
      //Robot.hatchCam.setExposureAuto();
      //Robot.hatchCam.setWhiteBalanceAuto();
      Robot.switcher.setSource(Robot.hatchCam);
      toggle = !toggle;
    }
    else {
      SmartDashboard.putBoolean("Hatch Cam Active", toggle);
      Robot.cargoCam.setFPS(15);
      Robot.cargoCam.setResolution(240, 360);
      Robot.cargoCam.setConnectionStrategy(ConnectionStrategy.kKeepOpen);
      //Robot.cargoCam.setExposureAuto();
      //Robot.cargoCam.setWhiteBalanceAuto();
      Robot.switcher.setSource(Robot.cargoCam);
      toggle = !toggle;
    }
  }

}
