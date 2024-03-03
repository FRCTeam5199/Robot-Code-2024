package frc.robot.LED;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.MainConstants;

public class LEDManager extends SubsystemBase{
    AddressableLED LEDRGB = new AddressableLED(MainConstants.LED_PORT);
    AddressableLEDBuffer LEDBUFFER = new AddressableLEDBuffer(MainConstants.LED_LENGTH);

    public LEDManager() {
        LEDRGB.setLength(LEDBUFFER.getLength());
    }

    public void red() {
        for(int i = 0; i < LEDBUFFER.getLength(); i++) {
            LEDBUFFER.setRGB(i, 255, 0, 0);
        }
        LEDRGB.setData(LEDBUFFER);
        LEDRGB.start();
    }

    public void green() {
        for(int i = 0; i < LEDBUFFER.getLength(); i++) {
            LEDBUFFER.setRGB(i, 0, 255, 0);
        }
        LEDRGB.setData(LEDBUFFER);
        LEDRGB.start();
    }
}