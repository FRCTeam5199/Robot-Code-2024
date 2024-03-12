package frc.robot.LEDs;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Main;
import frc.robot.constants.MainConstants;

public class LEDSubsystem extends SubsystemBase {
    AddressableLED LEDRGB = new AddressableLED(MainConstants.LED_PORT);
    AddressableLEDBuffer LEDBUFFER = new AddressableLEDBuffer(MainConstants.LED_LENGTH);

    boolean powerSaverMode;

    public LEDMode selectedLEDMode;

    public enum LEDMode {
        IDLE,
        NOTE,
        TRACKING
    }

    public LEDSubsystem() {}

    public void init() {
        LEDRGB.setLength(LEDBUFFER.getLength());

        LEDRGB.setData(LEDBUFFER);
        LEDRGB.start();
    }

    public void start() {
        LEDRGB.start();
    }

    public void stop() {
        LEDRGB.stop();
    }

    public void setPowerSaverMode(boolean mode) {
        powerSaverMode = mode;
    }

    public void setMode(LEDMode mode) {
        this.selectedLEDMode = mode;
    }
    
    @Override
    public void periodic() {
        if (powerSaverMode) {
            for(int i = 0; i < MainConstants.LED_LENGTH; i++) {
                if (getRed(i) >= 15) {
                    if (getGreen(i) >= 15) {
                        if (getBlue(i) >= 15) {
                            LEDBUFFER.setRGB(i, LEDBUFFER.getRed(i) - 15, LEDBUFFER.getGreen(i) - 15, LEDBUFFER.getBlue(i) - 15);
                        } else {
                            setBlue(i, 0);
                        }
                    } else {
                        setGreen(i, 0);
                    }
                } else {
                    setRed(i, 0);
                }
            }
        }

        if (selectedLEDMode == LEDMode.IDLE) {
            // setFade(Color.kBlue, Color.kLightBlue);
            setColor(Color.kBlue);
        } else if (selectedLEDMode == LEDMode.NOTE) {
            // setFade(Color.kBlue, Color.kDarkSeaGreen);
            setColor(Color.kViolet);
        } else if (selectedLEDMode == LEDMode.TRACKING) {
            // setFade(Color.kBlue, Color.kBlueViolet);
            setColor(Color.kGreen);
        } else {
            rainbow();
        }

        System.out.println(selectedLEDMode.toString());

        // setRGB(255, 255, 255);
    }

    public int getRed(int LEDindex) {
        return LEDBUFFER.getRed(LEDindex);
    }

    public int getGreen(int LEDindex) {
        return LEDBUFFER.getGreen(LEDindex);
    }

    public int getBlue(int LEDindex) {
        return LEDBUFFER.getBlue(LEDindex);
    }

    public void setRGB(int index, int red, int blue, int green) {
        LEDBUFFER.setRGB(index, red, blue, green);
        LEDRGB.setData(LEDBUFFER);
    }

    public void setRGB(int red, int blue, int green) {
        for(int i = 0; i < MainConstants.LED_LENGTH; i++) { LEDBUFFER.setRGB(i, red, blue, green); }
        LEDRGB.setData(LEDBUFFER);
    }

    public void setColor(int index, Color color) {
        LEDBUFFER.setLED(index, color);
        LEDRGB.setData(LEDBUFFER);
    }

    public void setColor(Color color) {
        for(int i = 0; i < MainConstants.LED_LENGTH; i++) { LEDBUFFER.setLED(i, color); }
        LEDRGB.setData(LEDBUFFER);
    }

    public void setRed(int index, int red) {
        LEDBUFFER.setRGB(index, red, LEDBUFFER.getGreen(index), LEDBUFFER.getBlue(index));
        LEDRGB.setData(LEDBUFFER);
    }

    public void setRed(int red) {
        for(int i = 0; i < MainConstants.LED_LENGTH; i++) { LEDBUFFER.setRGB(i, red, LEDBUFFER.getGreen(i), LEDBUFFER.getBlue(i)); }
        LEDRGB.setData(LEDBUFFER);
    }

    public void setGreen(int index, int green) {
        LEDBUFFER.setRGB(index, LEDBUFFER.getRed(index), green, LEDBUFFER.getBlue(index));
        LEDRGB.setData(LEDBUFFER);
    }

    public void setGreen(int green) {
        for(int i = 0; i < MainConstants.LED_LENGTH; i++) { LEDBUFFER.setRGB(i, LEDBUFFER.getRed(i), green, LEDBUFFER.getBlue(i)); }
        LEDRGB.setData(LEDBUFFER);
    }

    public void setBlue(int index, int blue) {
        LEDBUFFER.setRGB(index, LEDBUFFER.getRed(index), LEDBUFFER.getGreen(index), blue);
        LEDRGB.setData(LEDBUFFER);
    }

    public void setBlue(int blue) {
        for(int i = 0; i < MainConstants.LED_LENGTH; i++) { LEDBUFFER.setRGB(i, LEDBUFFER.getRed(i), LEDBUFFER.getGreen(i), blue); }
        LEDRGB.setData(LEDBUFFER);
    }

    public void setShift(int red1, int green1, int blue1, int red2, int green2, int blue2) {
        for(int i = 0; i < MainConstants.LED_LENGTH; i++) {
            if (i <= (LEDBUFFER.getLength() / 2)) { LEDBUFFER.setRGB(i, red1, green1, blue1);
            } else { LEDBUFFER.setRGB(i, red1, green1, blue1); }
        }
        
        LEDRGB.setData(LEDBUFFER);
    }

    public void setShift(Color color1, Color color2) {
        for(int i = 0; i < MainConstants.LED_LENGTH; i++) {
            if (i <= (LEDBUFFER.getLength() / 2)) { LEDBUFFER.setLED(i, color1);
            } else { LEDBUFFER.setLED(i, color2); }
        }
        
        LEDRGB.setData(LEDBUFFER);
    }

    public void setFade(int red, int green, int blue) {
        for(int i = 0; i < MainConstants.LED_LENGTH; i++) { LEDBUFFER.setRGB(i, red + i, green + i, blue + i); }
        LEDRGB.setData(LEDBUFFER);
    }

    public void setFade(int red, int green, int blue, double strength) {
        for(int i = 0; i < MainConstants.LED_LENGTH; i++) { LEDBUFFER.setRGB(i, red + (int)Math.round(i * strength), green + (int)Math.round(i * strength), blue + (int)Math.round(i * strength)); }
        LEDRGB.setData(LEDBUFFER);
    }

    public void setFade(Color... color) {
        for(int i = 0; i < MainConstants.LED_LENGTH; i++) { LEDBUFFER.setLED(i, color[Math.round(MainConstants.LED_LENGTH / color.length)]); }
        LEDRGB.setData(LEDBUFFER);
    }

    public void rainbow() {
        int rainbowFirstPixelHue = 0;
        // For every pixel
        for (var i = 0; i < LEDBUFFER.getLength(); i++) {
          LEDBUFFER.setHSV(i, (rainbowFirstPixelHue + (i * 180 / LEDBUFFER.getLength())) % 180, 255, 128);
        }

        // Increase by to make the rainbow "move"
        rainbowFirstPixelHue += 3;
        // Check bounds
        rainbowFirstPixelHue %= 180;
        
        LEDRGB.setData(LEDBUFFER);
      }
}