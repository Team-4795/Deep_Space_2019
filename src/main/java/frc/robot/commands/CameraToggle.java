/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.command.InstantCommand;

/**
 * Add your docs here.
 */
public class CameraToggle extends InstantCommand {
  private double cameraCount = 1;
  public CameraToggle() {
    super();
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  // Called once when the command executes
  @Override
  protected void initialize() {
    if (cameraCount == 0) {
      CameraServer.getInstance().removeCamera("cargoCam");
      CameraServer.getInstance().startAutomaticCapture("hatchCam", 0);
      cameraCount = 1;
    }
    else if (cameraCount == 1) {
      CameraServer.getInstance().removeCamera("hatchCam");
      CameraServer.getInstance().startAutomaticCapture("cargoCam", 1);
      cameraCount = 0;
    }
  }

}
