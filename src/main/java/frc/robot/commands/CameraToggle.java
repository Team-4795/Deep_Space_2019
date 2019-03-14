/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.InstantCommand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTableInstance;

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
    /*if (toggle) {
      SmartDashboard.putBoolean("Hatch Cam Active", toggle);
      //Robot.hatchCam.setFPS(15);
      //Robot.hatchCam.setPixelFormat(PixelFormat.kGray);
      Robot.hatchCam.setVideoMode(PixelFormat.kRGB565, 360, 240, 25);
      //SmartDashboard.putBoolean("set hatch res", Robot.hatchCam.setResolution(4, 3));
      //Robot.hatchCam.setResolution(16, 12);
      Robot.hatchCam.setConnectionStrategy(ConnectionStrategy.kKeepOpen);
      Robot.hatchCam.setExposureHoldCurrent();
      Robot.hatchCam.setWhiteBalanceHoldCurrent();
      Robot.switcher.setSource(Robot.hatchCam);
    }
    else {
      SmartDashboard.putBoolean("Hatch Cam Active", toggle);
      

      //Robot.cargoCam.setFPS(15);
      //Robot.cargoCam.setResolution(4, 3);
      SmartDashboard.putBoolean("set cargo res", Robot.cargoCam.setVideoMode(PixelFormat.kMJPEG, 360, 240, 20));
      Robot.cargoCam.setConnectionStrategy(ConnectionStrategy.kKeepOpen);
      Robot.cargoCam.setExposureHoldCurrent();
      Robot.cargoCam.setWhiteBalanceHoldCurrent();
      Robot.switcher.setSource(Robot.cargoCam);
    }*/
    //SmartDashboard.putBoolean("Camera ID", toggle);
    //NetworkTableInstance.getDefault().getEntry("CamID").setDouble(toggle ? 0 : 1);
    toggle = !toggle;
  }

}
