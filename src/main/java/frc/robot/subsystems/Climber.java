package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANDigitalInput;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANDigitalInput.LimitSwitchPolarity;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.commands.ManualArmControl;
import frc.robot.commands.ManualClimberControl;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Climber extends Subsystem {
    
    private final CANSparkMax climberMotor;
    private final CANPIDController climbPID;
    private Boolean climbTime;
    private final double GEAR_RATIO = 49.0;
    private final CANEncoder climbEnc;
    private final CANDigitalInput topLimit;
    private final CANDigitalInput botLimit;
      
    public Climber() {
      climberMotor = new CANSparkMax(RobotMap.CLIMBER_MOTOR.value, MotorType.kBrushless);
      climbPID = new CANPIDController(climberMotor);
      topLimit = climberMotor.getReverseLimitSwitch(LimitSwitchPolarity.kNormallyOpen);
      botLimit = climberMotor.getForwardLimitSwitch(LimitSwitchPolarity.kNormallyOpen);
      climbEnc = new CANEncoder(climberMotor);
      climberMotor.setIdleMode(IdleMode.kBrake);
      climberMotor.setOpenLoopRampRate(0.5);
      climberMotor.setClosedLoopRampRate(0.5);
      climbPID.setP(0.0055);
      climbPID.setI(0.0000);
      climbPID.setD(0.0);
      climbPID.setFF(0.0);
      climbPID.setOutputRange(0, 0.65);
      climbTime = false;
    }
  
    //normally open
    public Boolean getTopLimit() {
     return topLimit.get();
    } 

    public Boolean getBotLimit () {
      return botLimit.get();
    }

    public void resetEnc() {
      climberMotor.setEncPosition(0.0);
    }

    public void setPIDPos (double goal) {
      if (climbTime) {
      climbPID.setReference(goal, ControlType.kPosition);
      SmartDashboard.putNumber("Elevator Position (PID)", climbEnc.getPosition());
      SmartDashboard.putNumber("Elevator Output", climberMotor.getAppliedOutput());
      }
      
    }

    public double getPos () {
      return climbEnc.getPosition();
    }

    public void clearIAccum() {
      climbPID.setIAccum(0.0);
    }

    public void setClimbTime(Boolean state) {
      climbTime = state;
    }

    public void set(double speed) {
      if (climbTime) {
      climberMotor.set(speed);
      }
      else {
        climberMotor.set(0.0);
      }
      SmartDashboard.putNumber("Elevator Position", climbEnc.getPosition());
    }

    public void changeClimbTime(){
      climbTime = !climbTime;
    }
    public boolean getClimbTime(){
      return climbTime;
    }

    @Override
    public void initDefaultCommand() {
      // Set the default command for a subsystem here.
      setDefaultCommand(new ManualClimberControl(0.0));
    }
  }