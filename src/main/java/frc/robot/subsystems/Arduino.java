package frc.robot.subsystems;

import java.util.TimerTask;
import java.util.Arrays;

import frc.robot.Robot;
import frc.robot.RobotMap;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Arduino extends Subsystem {
  
  private static final long THREAD_PERIOD = 30; // ms - max poll rate on arduino.

  private byte[] registers;
  private int registerPosition;
  private SerialPort sp;
  private java.util.Timer executor;

  public Arduino() {
    registers = new byte[32];
    registerPosition = 0;

    sp = new SerialPort(9600, SerialPort.Port.kUSB);

    executor = new java.util.Timer();
    executor.schedule(new ArduinoUpdateTask(this), 0L, THREAD_PERIOD);
  }

  public void setLights(double r, double g, double b) {
    registers[0] = (byte)(r <= 0.0 ? 0 : r >= 1.0 ? 255 : r * 256.0);
    registers[1] = (byte)(g <= 0.0 ? 0 : g >= 1.0 ? 255 : g * 256.0);
    registers[2] = (byte)(b <= 0.0 ? 0 : b >= 1.0 ? 255 : b * 256.0);
  }
  private void update() {
    if (registerPosition == 0) {
      registerPosition += sp.write(registers, 32);
    } else {
      registerPosition += sp.write(Arrays.copyOfRange(registers, registerPosition, 32), 32 - registerPosition);
    }
    registerPosition %= 32;
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    //setDefaultCommand(new ManualClimberControl(0.0));
  }

  private class ArduinoUpdateTask extends TimerTask {
    private Arduino uno;

    private ArduinoUpdateTask(Arduino uno) {
      if (uno == null) {
        throw new NullPointerException("ColorSensor pointer null");
      }
      this.uno = uno;
    }

    public void run() {
      uno.update();
    }
  }
}