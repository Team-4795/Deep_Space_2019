package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANDigitalInput.LimitSwitchPolarity;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.commands.ManualArmControl;
import frc.robot.commands.ManualClimberControl;
import edu.wpi.first.wpilibj.command.Subsystem;

public class Climber extends Subsystem {
    public final CANSparkMax climberMotor;
  
    public Climber() {
      climberMotor = new CANSparkMax(RobotMap.CLIMBER_MOTOR.value, MotorType.kBrushless);
      climberMotor.setIdleMode(IdleMode.kBrake);
    }
  
    public void set(double speed) {
      climberMotor.set(speed);
    }
    @Override
    public void initDefaultCommand() {
      // Set the default command for a subsystem here.
      setDefaultCommand(new ManualClimberControl());
    }
  }