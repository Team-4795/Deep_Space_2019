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
import com.revrobotics.CANPIDController.AccelStrategy;
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
      climberMotor.setClosedLoopRampRate(0.4);
      climbPID.setP(0.0035);
      climbPID.setI(0.0000);
      climbPID.setD(-0.0005);
      climbPID.setFF(0.00325);
      climbPID.setSmartMotionAccelStrategy(AccelStrategy.kTrapezoidal, 0);
      climbPID.setSmartMotionMaxAccel(4000, 0);
      climbPID.setSmartMotionMaxVelocity(5000, 0);
      climbPID.setOutputRange(0, 0.8);
      climbPID.setSmartMotionAllowedClosedLoopError(1.0, 0);
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

    public void setPos (double goal) {
      if (climbTime) {
      climbPID.setReference(goal, ControlType.kSmartMotion);
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
      speed = climbEnc.getPosition() > 40 ? speed : 0.35 * speed;
      if (speed > 0 && climbEnc.getPosition() > 235) {
        speed = 0.0;
      }
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