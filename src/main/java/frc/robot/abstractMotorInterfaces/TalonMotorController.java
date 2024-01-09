package frc.robot.abstractMotorInterfaces;

import com.ctre.phoenix6.*;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
// import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.Follower;

// import com.ctre.phoenix.motorcontrol.Faults;
// import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
// import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import frc.robot.Robot;
import frc.robot.constants.MainConstants;
import frc.robot.utility.PID;

import java.util.ArrayList;

// import static com.ctre.phoenix.motorcontrol.ControlMode.*;
// import static com.ctre.phoenix.motorcontrol.NeutralMode.Brake;
// import static com.ctre.phoenix.motorcontrol.NeutralMode.Coast;

/**
 * This is the wrapper for falcon 500's and maybe some other stuff
 */
public class TalonMotorController extends AbstractMotorController {
    public final ArrayList<AbstractMotorController> motorFollowerList = new ArrayList<>();
    public MainConstants main = new MainConstants();
    private final TalonFX motor;
    private final Slot0Configs fd = new Slot0Configs();
    public boolean isFollower = false;

    
    public TalonMotorController(int id, String bus) {
        super();
        motor = new TalonFX(id, bus);

        sensorToRealDistanceFactor = 1D / MainConstants.CTRE_SENSOR_UNITS_PER_ROTATION;
        sensorToRealTimeFactor = 60D * 10D / 1D;
    }

    /**
     * The talons are the motors that make music and this is the method to register tham as musick makers. The orchestra
     * is wrapped inside {@link Chirp} using the {@link Chirp#talonMotorArrayList meta talon registry}
     *
     * @param orchestra the {@link Orchestra} object this motor should join
     * @see Chirp
     */

    @Override
    public AbstractMotorController setInverted(boolean invert) {
        motor.setInverted(invert);
        if (!this.isFollower) {
            for (AbstractMotorController followerMotor : motorFollowerList) {
                followerMotor.setInverted(invert);
            }
        }
        return this;
    }

    @Override
    public int getID() {
        return motor.getDeviceID();
    }

    @Override
    public void setOutPutRange(double min, double max) {

    }

    @Override
    public String getName() {
        return "Talon: " + motor.getDeviceID();
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
        sensorToRealDistanceFactor = r2rf / MainConstants.CTRE_SENSOR_UNITS_PER_ROTATION;
        sensorToRealTimeFactor = t2tf * 60D * 10D / 1D;
    }

    @Override
    public void resetEncoder() {
        if (!this.isFollower) {
            for (AbstractMotorController followerMotor : motorFollowerList) {
                followerMotor.resetEncoder();
            }
        }
        if (motor.setPosition(0) != StatusCode.OK)
        //!Robot.SECOND_TRY
            if (true)
                throw new IllegalStateException("Talon motor controller with ID " + motor.getDeviceID() + " could not be reset");
            else
                failureFlag = true;
    }

    @Override
    public AbstractMotorController setPid(PID pid) {
        Slot0Configs slot0Configs = new Slot0Configs();
        slot0Configs.kP = pid.getP();
        slot0Configs.kP = pid.getI();
        slot0Configs.kP = pid.getD();
        slot0Configs.kP = pid.getF();

        if (motor.getConfigurator().apply(slot0Configs) != StatusCode.OK)
            if (!Robot.SECOND_TRY)
                throw new IllegalStateException("Talon motor controller with ID " + motor.getDeviceID() + " PIDF couldnt be set");
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
            motor.set(realAmount / sensorToRealDistanceFactor / sensorToRealTimeFactor);
            if (!this.isFollower) {
                for (AbstractMotorController followerMotor : motorFollowerList) {
                    followerMotor.moveAtVelocity(realAmount);
                }
            }
        } else
            motor.set(0);
    }

    @Override
    public int getMaxRPM() {
        return SupportedMotors.TALON_FX.MAX_SPEED_RPM;
    }

    @Override
    public void moveAtPosition(double pos) {
        motor.setPosition(pos / sensorToRealDistanceFactor);
        if (!this.isFollower) {
            for (AbstractMotorController followerMotor : motorFollowerList) {
                followerMotor.moveAtPosition(pos);
            }
        }
    }

    @Override
    public void moveAtVoltage(double voltin) {
        motor.setVoltage(voltin);
        if (!this.isFollower) {
            for (AbstractMotorController followerMotor : motorFollowerList) {
                followerMotor.moveAtVoltage(voltin);
            }
        }
    }

    @Override
    public AbstractMotorController setBrake(boolean brake) {
        motor.setNeutralMode(brake ? NeutralModeValue.Brake : NeutralModeValue.Coast);
        if (!this.isFollower) {
            for (AbstractMotorController followerMotor : motorFollowerList) {
                followerMotor.setBrake(brake);
            }
        }
        return this;
    }

    @Override
    public double getRotations() {
        return motor.getPosition().getValueAsDouble();
    }

    @Override
    public double getSpeed() {
        return motor.getVelocity().getValueAsDouble();
    }

    @Override
    public double getVoltage() {
        return motor.getMotorVoltage().getValueAsDouble();
    }

    @Override
    public double getCurrent() {
        return motor.getTorqueCurrent().getValueAsDouble();
    }

    @Override
    public AbstractMotorController setCurrentLimit(int limit) {
        CurrentLimitsConfigs config = new CurrentLimitsConfigs();
        config.SupplyCurrentLimit = limit;
        config.SupplyCurrentLimitEnable = true;
        if (motor.getConfigurator().apply(config) != StatusCode.OK)
            if (!Robot.SECOND_TRY)
                throw new IllegalStateException("Talon motor controller with ID " + motor.getDeviceID() + " current limit could not be set");
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
            motor.set(percent);
        } else {
            motor.set(0);
        }
        if (!this.isFollower) {
            for (AbstractMotorController followerMotor : motorFollowerList) {
                followerMotor.moveAtPercent(percent);
            }
        }
    }

    @Override
    public AbstractMotorController unfollow() {
        Follower follow = new Follower(getID(), false);
        motor.setControl(follow);
        motorFollowerList.remove(this);
        return this;
    }

    @Override
    public AbstractMotorController setOpenLoopRampRate(double timeToMax) {
        OpenLoopRampsConfigs openLoopRampsConfigs = new OpenLoopRampsConfigs();
        openLoopRampsConfigs.DutyCycleOpenLoopRampPeriod = timeToMax;
        if (motor.getConfigurator().apply(openLoopRampsConfigs) != StatusCode.OK)
            if (!Robot.SECOND_TRY)
                throw new IllegalStateException("Talon motor controller with ID " + motor.getDeviceID() + " could not set open ramp rate");
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
        return motor.getDeviceTemp().getValueAsDouble();
    }

    @Override
    public boolean isFailed() {
        return motor.getFaultField().getValue() != 0 || failureFlag;
    }

    @Override
    public String getSuggestedFix() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getSuggestedFix'");
    }
}