package frc.robot.abstractMotorInterfaces;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

public class TalonMotorController extends AbstractMotorController{
    
    TalonFX talon;
    TalonFXConfiguration configuration = new TalonFXConfiguration();

    public TalonMotorController(int ID, String bus) {
        super();
        talon = new TalonFX(ID, bus);
        
    }

    public TalonMotorController(int ID) {
        super();
        talon = new TalonFX(ID);
        talon.getConfigurator().apply(configuration);
    }

    @Override
    public AbstractMotorController setInvert(boolean invert) {
        talon.setInverted(invert);
        return this;
    }

    @Override
    public void setVelocity(double Velocity, double Acceleration, boolean FOC, double feed, int PIDSLOT, boolean brake, boolean forwardLimit, boolean backwardLimit) {
        talon.setControl(new VelocityDutyCycle(Velocity, Acceleration, FOC, feed, PIDSLOT, brake, forwardLimit, backwardLimit));
    }

    @Override
    public void set(double Percent) {
        talon.set(Percent);
    }

    @Override
    public void setPosition(double Position, boolean FOC, double feed, int PidSlot, boolean brake, boolean forwardlimit, boolean backwardlimit, double Velocity) {
        talon.setControl(new PositionDutyCycle(Velocity, Position, FOC, feed, PidSlot, brake, forwardlimit, backwardlimit));
    }

    @Override
    public void setPosition(double Position) {
        talon.setControl(new PositionDutyCycle(Position));
    }

    @Override
    public AbstractMotorController setBrake(boolean brake) {
        return null;
    }

    @Override
    public double getVelocity() {
        return talon.getVelocity().getValue();
    }

    @Override
    public void setVelocity(double Velocity) {
        talon.setControl(new VelocityDutyCycle(Velocity));
    }

    @Override
    public double getAngularVelocity() {
        return talon.getVelocity().getValue();
    }

    @Override
    public double getRotations() {
        return talon.getVelocity().getValue();
    }

    @Override
    public double getVoltage() {
        return talon.getVelocity().getValue();
    }

    @Override
    public void setVoltage(double Voltage) {
        talon.setControl(new VoltageOut(Voltage));
    }

    @Override
    public double getCurrent() {
        return talon.getStatorCurrent().getValue();
    }

    @Override
    public int getID() {
        return talon.getDeviceID();
    }

    @Override
    public void setCurrentLimit(int limit) {
        configuration.CurrentLimits.StatorCurrentLimit = limit;
    }

    @Override
    public void setOpenLoopRampRate(double Ramp) {
        configuration.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = Ramp;
    }

    @Override
    public void setRange(double minRange, double maxRange) {
    }

    @Override
    public AbstractMotorController follow(AbstractMotorController leader, boolean invert) {
        talon.setControl(new Follower(leader.getID(), invert));
        return this;
    }

    public void setConfiguration(double currentLimit, double peakVoltage) {
        configuration.CurrentLimits.StatorCurrentLimit = currentLimit;
        configuration.Voltage.PeakForwardVoltage = peakVoltage;
        configuration.Voltage.PeakReverseVoltage = -peakVoltage;
    }
}
