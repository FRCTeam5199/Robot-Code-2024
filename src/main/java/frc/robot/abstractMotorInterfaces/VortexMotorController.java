package frc.robot.abstractMotorInterfaces;

import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.revrobotics.*;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkFlex;
import edu.wpi.first.wpilibj.motorcontrol.Spark;

public class VortexMotorController extends AbstractMotorController{
    CANSparkBase vortex;
    public VortexMotorController(int ID){
        super();
        vortex = new CANSparkFlex(ID, CANSparkLowLevel.MotorType.kBrushless);
    }

    @Override
    public void setVelocity(double Velocity) {
    }

    @Override
    public AbstractMotorController setInvert(boolean invert) {
        vortex.setInverted(invert);
        return this;
    }

    public RelativeEncoder getEncoder(){
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
    public void setVoltage(double Voltage) {
        vortex.setVoltage(Voltage);
    }

    @Override
    public AbstractMotorController setBrake(boolean brake) {
        return null;
    }

    @Override
    public double getVelocity() {
        return 0;
    }

    @Override
    public double getAngularVelocity() {
        return 0;
    }

    @Override
    public double getRotations() {
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
    public int getID() {
        return vortex.getDeviceId();
    }

    @Override
    public void setCurrentLimit(double limit) {

    }

    @Override
    public void setOpenLoopRampRate(double Ramp) {
        vortex.setOpenLoopRampRate(Ramp);
    }

    @Override
    public void setRange(double minRange, double maxRange) {

    }

    @Override
    public AbstractMotorController follow(AbstractMotorController leader, boolean invert) {

        vortex.follow(new CANSparkFlex(leader.getID(), CANSparkLowLevel.MotorType.kBrushless));
        return this;
    }
}