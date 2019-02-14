/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

public enum RobotMap {

  //Controller Mappings
  MAIN_CONTROLLER(0),
  ARM_CONTROLLER(1),
  //CAN Motor Controller Mappings
  LEFT_MOTOR_ONE(1),
  LEFT_MOTOR_TWO(2),
  LEFT_MOTOR_THREE(3),
  RIGHT_MOTOR_ONE(4),
  RIGHT_MOTOR_TWO(5),
  RIGHT_MOTOR_THREE(6),
  ARM_MOTOR(7),
  INTAKE_MOTOR(8),
  INTAKE_MOTOR_FOLLOWER(9),
  HATCH_MOTOR(10);

  public final int value;

  RobotMap(int value) {
    this.value = value;
  }
}
