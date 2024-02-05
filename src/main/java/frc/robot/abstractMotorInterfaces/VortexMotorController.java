package frc.robot.abstractMotorInterfaces;

import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.revrobotics.*;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkFlex;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import frc.robot.abstractMotorInterfaces.AbstractMotorController;

public class VortexMotorController extends AbstractMotorController {
    public CANSparkBase vortex;
    public RelativeEncoder encoder;
    public VortexMotorController(int ID){
        super();
        vortex = new CANSparkFlex(ID, CANSparkLowLevel.MotorType.kBrushless);
        encoder = vortex.getEncoder();
    }

    @Override
    public AbstractMotorController setInvert(boolean invert) {
        vortex.setInverted(invert);
        return this;
    }

    public RelativeEncoder getEncoder() {
        return vortex.getEncoder();
    }

    @Override
    public void setVelocity(double Velocity, double Acceleration, boolean FOC, double feed, int PIDSLOT, boolean brake, boolean forwardLimit, boolean backwardLimit) {

    }

    @Override
    public void set(double Percent) {
        vortex.set(Percent);
    }

    @Override
    public void setPosition(double Position) {
        vortex.getEncoder().setPosition(Position);
    }

    @Override
    public void setPosition(double Position, boolean FOC, double feed, int PidSlot, boolean brake, boolean forwardlimit, boolean backwardlimit, double Velocity) {

    }

    @Override
    public AbstractMotorController setBrake(boolean brake) {
        if (brake) {
            vortex.setIdleMode(IdleMode.kBrake);
        } else {
            vortex.setIdleMode(IdleMode.kCoast);
        }
        return this;
    }

    @Override
    public double getVelocity() {
        return encoder.getVelocity();
    }

    @Override
    public void setVelocity(double Velocity) {
        double speed = 0;
        vortex.set(0);
        while (vortex.getEncoder().getVelocity() <= Velocity) {
            speed++;
            vortex.set(speed);
        }
    }

    @Override
    public double getAngularVelocity() {
        return 0;
    }

    @Override
    public double getRotations() {
        return encoder.getPosition() * sensorToRealDistanceFactor;
    }

    @Override
    public double getVoltage() {
        return vortex.getBusVoltage();
    }

    @Override
    public void setVoltage(double Voltage) {
        vortex.setVoltage(Voltage);
    }

    @Override
    public double getCurrent() {
        return vortex.getOutputCurrent();
    }

    @Override
    public int getID() {
        return vortex.getDeviceId();
    }

    @Override
    public void setCurrentLimit(int limit) {
        vortex.setSmartCurrentLimit(limit);
    }

    @Override
    public void setOpenLoopRampRate(double Ramp) {
        vortex.setOpenLoopRampRate(Ramp);
    }

    @Override
    public void setRange(double minRange, double maxRange) {

    }

    public AbstractMotorController follow(CANSparkBase leader, boolean invert) {
        vortex.follow(leader);
        return this;
    }

    @Override
    public AbstractMotorController follow(AbstractMotorController leader, boolean invert) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'follow'");
    }
}