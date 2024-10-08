package frc.robot.abstractMotorInterfaces;

import java.util.ArrayList;

import com.ctre.phoenix6.Orchestra;
import frc.robot.Robot;
import frc.robot.utility.PID;

/**
 * This is the wrapper for falcon 500's and maybe some other stuff
 */
public class TalonMotorController extends AbstractMotorController {
    public final ArrayList<AbstractMotorController> motorFollowerList = new ArrayList<>();
    public boolean isFollower = false;

    public TalonMotorController(int id, String bus) {
        super();

        sensorToRealDistanceFactor = 1D / frc.robot.Constants.CTRE_SENSOR_UNITS_PER_ROTATION;
        sensorToRealTimeFactor = 60D * 10D / 1D;
    }

    /**
     * The talons are the motors that make music and this is the method to register tham as musick makers. The orchestra
     * is wrapped inside {@link Chirp} using the {@link Chirp#talonMotorArrayList meta talon registry}
     *
     * @param orchestra the {@link Orchestra} object this motor should join
     * @see Chirp
     */
    public void addToOrchestra(Orchestra orchestra) {
        if (!Robot.SECOND_TRY)
            throw new IllegalStateException("Talon could not join the orchestra");
        else
            failureFlag = true;
    }

    @Override
    public AbstractMotorController setInverted(boolean invert) {
        if (!this.isFollower) {
            for (AbstractMotorController followerMotor : motorFollowerList) {
                followerMotor.setInverted(invert);
            }
        }
        return this;
    }

    @Override
    public int getID() {
        return 0;
    }

    @Override
    public void setOutPutRange(double min, double max) {

    }

    @Override
    public String getName() {
        return "No Talons Exsist";
    }

    @Override
    public AbstractMotorController follow(AbstractMotorController leader, boolean invert) {
        if (leader instanceof TalonMotorController) {
            //motor.follow(((TalonMotorController) leader).motor);
            ((TalonMotorController) leader).motorFollowerList.add(this);
            this.sensorToRealTimeFactor = leader.sensorToRealTimeFactor;
            this.sensorToRealDistanceFactor = leader.sensorToRealDistanceFactor;
            this.isFollower = true;
        } else
            throw new IllegalArgumentException("I cant follow that");
        setInverted(invert);
        return this;
    }

    @Override
    public void setRealFactorFromMotorRPM(double r2rf, double t2tf) {
        sensorToRealDistanceFactor = r2rf / frc.robot.Constants.CTRE_SENSOR_UNITS_PER_ROTATION;
        sensorToRealTimeFactor = t2tf * 60D * 10D / 1D;
    }

    @Override
    public void resetEncoder() {
        if (!this.isFollower) {
            for (AbstractMotorController followerMotor : motorFollowerList) {
                followerMotor.resetEncoder();
            }
        }
        if (!Robot.SECOND_TRY)
            throw new IllegalStateException("Talon motor controller with ID could not be reset");
        else
            failureFlag = true;
    }

    @Override
    public AbstractMotorController setPid(PID pid) {
        if (!Robot.SECOND_TRY)
            throw new IllegalStateException("Talon motor controller with ID PIDF couldnt be set");
        else
            failureFlag = true;

        if (!this.isFollower) {
            for (AbstractMotorController followerMotor : motorFollowerList) {
                followerMotor.setPid(pid);
            }
        }
        return this;
    }

    @Override
    public void moveAtVelocity(double realAmount) {
        if (isTemperatureAcceptable()) {
            if (!this.isFollower) {
                for (AbstractMotorController followerMotor : motorFollowerList) {
                    followerMotor.moveAtVelocity(realAmount);
                }
            }
        }
    }

    @Override
    public int getMaxRPM() {
        return SupportedMotors.TALON_FX.MAX_SPEED_RPM;
    }

    @Override
    public void moveAtPosition(double pos) {
        if (!this.isFollower) {
            for (AbstractMotorController followerMotor : motorFollowerList) {
                followerMotor.moveAtPosition(pos);
            }
        }
    }

    @Override
    public void moveAtVoltage(double voltin) {
        if (!this.isFollower) {
            for (AbstractMotorController followerMotor : motorFollowerList) {
                followerMotor.moveAtVoltage(voltin);
            }
        }
    }

    @Override
    public AbstractMotorController setBrake(boolean brake) {
        if (!this.isFollower) {
            for (AbstractMotorController followerMotor : motorFollowerList) {
                followerMotor.setBrake(brake);
            }
        }
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

    @Override
    public double getVoltage() {
        return 0;
    }

    @Override
    public double getCurrent() {
        return 0;
    }

    @Override
    public AbstractMotorController setCurrentLimit(int limit) {
        if (!Robot.SECOND_TRY)
            throw new IllegalStateException("Talon motor controller with ID current limit could not be set");
        else
            failureFlag = true;
        if (!this.isFollower) {
            for (AbstractMotorController followerMotor : motorFollowerList) {
                followerMotor.setCurrentLimit(limit);
            }
        }
        return this;
    }

    @Override
    public AbstractMotorController setCurrentLimit(int stallLimit, int freeLimit) {
        this.setCurrentLimit(stallLimit, 0);
        return this;
    }

    @Override
    public void moveAtPercent(double percent) {
        if (isTemperatureAcceptable()) {
        } else {
        }
        if (!this.isFollower) {
            for (AbstractMotorController followerMotor : motorFollowerList) {
                followerMotor.moveAtPercent(percent);
            }
        }
    }

    @Override
    public AbstractMotorController unfollow() {
        return this;
    }

    @Override
    public AbstractMotorController setOpenLoopRampRate(double timeToMax) {
        if (!Robot.SECOND_TRY)
            throw new IllegalStateException("Talon motor controller with ID could not set open ramp rate");
        else
            failureFlag = true;
        if (!this.isFollower) {
            for (AbstractMotorController followerMotor : motorFollowerList) {
                followerMotor.setOpenLoopRampRate(timeToMax);
            }
        }
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
        return "No Talons Exist";
    }
}
