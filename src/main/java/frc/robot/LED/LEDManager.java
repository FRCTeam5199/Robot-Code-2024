package frc.robot.LED;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.MainConstants;


public class LEDManager extends SubsystemBase{
    static AddressableLED LEDRGB = new AddressableLED(MainConstants.LED_PORT);
    static AddressableLEDBuffer LEDBUFFER = new AddressableLEDBuffer(MainConstants.LED_LENGTH);

    Timer timer = new Timer();
    boolean swap = false;
    int firstPixelHue = 0;

    public void init() {
        
    }

    public void red() {
        for(var i = 0; i < LEDBUFFER.getLength(); i++) {
            LEDBUFFER.setRGB(i, 255, 0, 0);
        }
        LEDRGB.setLength(LEDBUFFER.getLength());
        LEDRGB.setData(LEDBUFFER);
        LEDRGB.start();
    }

    public void green() {
        for(var i = 0; i < LEDBUFFER.getLength(); i++) {
            LEDBUFFER.setRGB(i, 0, 255, 0);
        }
        LEDRGB.setLength(LEDBUFFER.getLength());
        LEDRGB.setData(LEDBUFFER);
        LEDRGB.start();

  
    }
}