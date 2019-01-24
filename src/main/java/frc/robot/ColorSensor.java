package frc.robot;

import java.util.TimerTask;

import edu.wpi.first.wpilibj.I2C;

public class ColorSensor {
  //Tread variables
  private java.util.Timer executor;
  private static final long THREAD_PERIOD = 20; //ms - max poll rate on sensor.
  
  public static final byte CS_ADDRESS = 0x39;

  private static ColorSensor instance;
  private static I2C cs;
  private volatile ColorData color;

  public enum reg_t {
    CS_CDATA  (0x14),
    CS_CDATAH (0x15),
    CS_RDATA  (0x16),
    CS_RDATAH (0x17),
    CS_GDATA  (0x18),
    CS_GDATAH (0x19),
    CS_BDATA  (0x1A),
    CS_BDATAH (0x1B),
    CS_PDATA  (0x1C),
    CS_PDATAH (0x1D);

    private final int val;

    reg_t(int val) {
      this.val = val;
    }

    public int getVal() {
      return val;
    }
  };
  
  public class ColorData {
    public short clear;
    public short red;
    public short green;
    public short blue;
    public short proximity;
  }
  
  private ColorSensor(I2C.Port port, byte address) {
    cs = new I2C(port, address);
    
    executor = new java.util.Timer();
    executor.schedule(new ColorSensorUpdateTask(this), 0L, THREAD_PERIOD);
  }

  public static ColorSensor getInstance(I2C.Port port, byte address) {
    if (instance == null) {
      instance = new ColorSensor(port, address);
    }
    return instance;
  }

  public static ColorSensor getInstance() {
    return getInstance(I2C.Port.kOnboard, CS_ADDRESS);
  }

  private void update() {
    ColorData colorData = new ColorData();
    
    colorData.clear = read16(reg_t.CS_CDATA);
    colorData.red = read16(reg_t.CS_RDATA);
    colorData.green = read16(reg_t.CS_GDATA);
    colorData.blue = read16(reg_t.CS_BDATA);
    colorData.proximity = read16(reg_t.CS_PDATA);

    color = colorData;
  }

  public ColorData getColor() {
    return color;
  }
  
  private boolean write8(reg_t reg, byte value) {
    return cs.write(reg.getVal(), value);
  }

  private byte read8(reg_t reg) {
    byte[] vals = new byte[1];
    readLen(reg, vals);
    return vals[0];
  }

  private short read16(reg_t reg) {
    byte[] vals = new byte[2];
    readLen(reg, vals);
    return (short) (vals[0] | (vals[1] << 8));
  }

  // return true if successful
  private boolean readLen(reg_t reg, byte[] buffer) {
    if (buffer == null || buffer.length < 1) {
      return false;
    }
    return !cs.read(reg.getVal(), buffer.length, buffer);
  }
  
  private class ColorSensorUpdateTask extends TimerTask {
    private ColorSensor cs;

    private ColorSensorUpdateTask(ColorSensor cs) {
      if (cs == null) {
        throw new NullPointerException("ColorSensor pointer null");
      }
      this.cs = cs;
    }

    public void run() {
      cs.update();
    }
  }
}