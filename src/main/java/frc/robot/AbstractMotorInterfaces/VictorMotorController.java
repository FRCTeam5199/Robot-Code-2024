package frc.robot.abstractMotorInterfaces;

import frc.robot.Robot;
import frc.robot.utility.PID;

/**
 * This works to wrap 775 Pros and maybe some other motors
 */
public class VictorMotorController extends AbstractMotorController {
    public VictorMotorController(int id) {
        super();

        sensorToRealDistanceFactor = 1 / frc.robot.Constants.CTRE_SENSOR_UNITS_PER_ROTATION;
        sensorToRealTimeFactor = 60D * 10D / 1D;
    }

    @Override
    public void setRealFactorFromMotorRPM(double r2rf, double t2tf) {
        sensorToRealDistanceFactor = r2rf / frc.robot.Constants.CTRE_SENSOR_UNITS_PER_ROTATION;
        sensorToRealTimeFactor = t2tf * 60D * 10D / 1D;
    }

    @Override
    public AbstractMotorController setInverted(boolean invert) {
        return this;
    }

    @Override
    public String getName() {
        return "No Victors Exist";
    }

    @Override
    public int getID() {
        return 0;
    }

    @Override
    public void setOutPutRange(double min, double max) {}

    @Override
    public AbstractMotorController follow(AbstractMotorController leader, boolean invert) {
        if (leader instanceof VictorMotorController) {
        } else
            throw new IllegalArgumentException("I cant follow that!");
        setInverted(invert);

        this.sensorToRealTimeFactor = leader.sensorToRealTimeFactor;
        this.sensorToRealDistanceFactor = leader.sensorToRealDistanceFactor;
        return this;
    }

    @Override
    public AbstractMotorController unfollow() {
        return this;
    }

    @Override
    public void resetEncoder() {
        if (!Robot.SECOND_TRY)
            throw new IllegalStateException("Victor Motor Controller could not be reset");
        else
            failureFlag = true;
    }

    @Override
    public AbstractMotorController setPid(PID pid) {
        if (!Robot.SECOND_TRY)
            throw new IllegalStateException("Victor Motor Controller PIDF could not be set");
        else
            failureFlag = true;
        return this;
    }

    @Override
    public void moveAtVelocity(double realVelocity) {}

    @Override
    public void moveAtPosition(double pos) {}

    @Override
    public double getVoltage() {
        return 0;
    }

    @Override
    public double getCurrent() {
        return -1;
    }

    @Override
    public void moveAtVoltage(double voltin) {
    }

    @Override
    public AbstractMotorController setBrake(boolean brake) {
        return this;
    }

    @Override
    public double getRotations() {
        return 0;
    }

    @Override
    public double getSpeed() {
        return 0;
    }

    //TODO make this work lol
    @Override
    public AbstractMotorController setCurrentLimit(int limit) {
        return this;
    }

    @Override
    public AbstractMotorController setCurrentLimit(int stallLimit, int freeLimit) {
        return null;
    }

    @Override
    public int getMaxRPM() {
        return SupportedMotors.VICTOR.MAX_SPEED_RPM;
    }

    @Override
    public void moveAtPercent(double percent) {
    }

    @Override
    public AbstractMotorController setOpenLoopRampRate(double timeToMax) {
        if (!Robot.SECOND_TRY)
            throw new IllegalStateException("Victor Motor Controller with open loop ramp could not be set");
        else
            failureFlag = true;
        return this;
    }

    @Override
    public double getMotorTemperature() {
        return 0;
    }

    @Override
    public boolean isFailed() {
        return false;
    }

    @Override
    public String getSuggestedFix() {
        return null;
    }
}
