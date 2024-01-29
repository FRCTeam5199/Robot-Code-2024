package frc.robot.utility;

import edu.wpi.first.wpilibj.DigitalInput;

public class BrakeButton {
    private final DigitalInput brakeButton;
    public BrakeButton(int port){
    brakeButton = new DigitalInput(port);
    }

    public boolean get() {
    return brakeButton.get();
  }
}
