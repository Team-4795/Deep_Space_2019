package frc.robot;

import java.util.TimerTask;

import edu.wpi.first.wpilibj.I2C;

public class ProximitySensor {
  private java.util.Timer executor;
  private static final long THREAD_PERIOD = 20; //ms - max poll rate on sensor.
  
  public static final byte PS_ADDRESS = 0x52;

  private static ProximitySensor instanceOnboard;
  private static ProximitySensor instanceMXP;
  private static I2C ps;
  private volatile int distance;

  public enum reg_t {
    //https://github.com/OlivierLD/raspberry-coffee/blob/master/I2C.SPI/src/i2c/sensor/VL53L0X.java#L27
    SYSRANGE_START (0x00),
    RESULT_RANGE_STATUS (0x14);

    private final int val;

    reg_t(int val) {
      this.val = val;
    }

    public int getVal() {
      return val;
    }
  };
  
  private ProximitySensor(I2C.Port port) {
    ps = new I2C(port, PS_ADDRESS);

    write8(reg_t.SYSRANGE_START, (byte) 0);
    write8(reg_t.SYSRANGE_START, (byte) 1);

    executor = new java.util.Timer();
    executor.schedule(new ProximitySensorUpdateTask(this), 0L, THREAD_PERIOD);
  }

  public static ProximitySensor getInstanceOnboard() {
    if (instanceOnboard == null) {
      instanceOnboard = new ProximitySensor(I2C.Port.kOnboard);
    }
    return instanceOnboard;
  }

  public static ProximitySensor getInstanceMXP() {
    if (instanceMXP == null) {
      instanceMXP = new ProximitySensor(I2C.Port.kMXP);
    }
    return instanceMXP;
  }

  private void update() {
    // assumptions: Linearity Corrective Gain is 1000 (default)
    // fractional ranging is not enabled
    distance = this.read16(reg_t.RESULT_RANGE_STATUS);
  }

  public int getMillimeters() {
    return distance;
  }
  
  private boolean write8(reg_t reg, byte value) {
    return ps.write(reg.getVal(), value);
  }

  private byte read8(reg_t reg) {
    byte[] vals = new byte[1];
    readLen(reg, vals);
    return vals[0];
  }

  private short read16(reg_t reg) {
    byte[] vals = new byte[2];
    readLen(reg, vals);
    return (short) ((vals[0] << 8) | vals[1]);
  }

  // return true if successful
  private boolean readLen(reg_t reg, byte[] buffer) {
    if (buffer == null || buffer.length < 1) {
      return false;
    }
    return !ps.read(reg.getVal(), buffer.length, buffer);
  }
  
  private class ProximitySensorUpdateTask extends TimerTask {
    private ProximitySensor ps;

    private ProximitySensorUpdateTask(ProximitySensor ps) {
      if (ps == null) {
        throw new NullPointerException("ProximitySensor pointer null");
      }
      this.ps = ps;
    }

    public void run() {
      ps.update();
    }
  }
}