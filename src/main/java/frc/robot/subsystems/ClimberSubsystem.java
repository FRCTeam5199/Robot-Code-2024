package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.abstractMotorInterfaces.VortexMotorController;
import frc.robot.constants.MainConstants;

public class ClimberSubsystem implements Subsystem {
    public VortexMotorController climberMotor1;
    public VortexMotorController climberMotor2;

    public PIDController climberPIDController;

    private boolean isClimbed = false;

    public ClimberSubsystem() {
        // init();
    }

    public void init() {
        motorInit();
        PIDInit();
    }

    public void motorInit() {
        climberMotor1 = new VortexMotorController(MainConstants.IDs.Motors.CLIMBER_JOINT_MOTOR_ID);
        climberMotor2 = new VortexMotorController(MainConstants.IDs.Motors.CLIMBER_CLAW_MOTOR_ID);

        climberMotor1.setInvert(false);
        climberMotor2.setInvert(true);

        climberMotor1.getEncoder().setPosition(0);
        climberMotor2.getEncoder().setPosition(0);
    }

    private void PIDInit() {
        climberPIDController = new PIDController(MainConstants.PIDConstants.CLIMBER_PID.P, MainConstants.PIDConstants.CLIMBER_PID.I, MainConstants.PIDConstants.CLIMBER_PID.D);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        climberMotor1.set(climberPIDController.calculate(climberMotor1.getRotations()));
        climberMotor2.set(climberPIDController.calculate(climberMotor2.getRotations()));
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }

    public Command setClimberSpeed(double percent) {
        return this.runOnce(() -> climberMotor1.set(percent)).andThen((() -> climberMotor2.set(percent)));
    }

    public Command setClimberTarget(double target) {
        return this.runOnce(() -> climberPIDController.setSetpoint(target));
    }

    public Command climbClimber() {
        return this.runOnce(() -> climb());
    }

    public Command storeClimber() {
        return this.runOnce(() -> store());
    }

    public void climb() {
        climberPIDController.setSetpoint(MainConstants.Setpoints.CLIMBER_CLIMB_SETPOINT);
        isClimbed = true;
    }

    public void store() {
        climberPIDController.setSetpoint(MainConstants.Setpoints.CLIMBER_STORE_SETPOINT);
        isClimbed = false;
    }

    public boolean isClimbed() {
        return isClimbed;
    }
}
