package frc.robot.utility;

import edu.wpi.first.wpilibj.DigitalInput;

public class StopClimbButton {
    private final DigitalInput ClimbButton;
    public StopClimbButton(int port){
    ClimbButton = new DigitalInput(port);
    }

    public boolean get() {
    return ClimbButton.get();
  }
}
