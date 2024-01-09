package frc.robot.abstractMotorInterfaces.followers;

import frc.robot.abstractMotorInterfaces.AbstractMotorController;
import frc.robot.abstractMotorInterfaces.SparkMotorController;

/**
 * This works to wrap Neo's and maybe some other motors
 */
public class SparkFollowerMotorsController extends AbstractFollowerMotorController {
    public SparkFollowerMotorsController(int... ids) {
        motors = new AbstractMotorController[ids.length];
        for (int i = 0; i < ids.length; i++)
            motors[i] = new SparkMotorController(ids[i]);
    }

    @Override
    public void invert(boolean invert) {
        for (AbstractMotorController motor : motors)
            motor.setInverted(invert);
    }
}