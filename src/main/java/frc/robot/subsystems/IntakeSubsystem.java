package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Main;
import frc.robot.abstractMotorInterfaces.VortexMotorController;
import frc.robot.constants.MainConstants;

public class IntakeSubsystem implements Subsystem {
    public VortexMotorController intakeMotor;
    public VortexMotorController intakeAngleMotor;
    public PIDController pidController;
    public double setpoint;

    public IntakeSubsystem() {
        init();
    }

    public void init() {
        motorInit();
        PIDInit();
    }

    @Override
    public void periodic() {
        intakeAngleMotor.set(pidController.calculate(intakeAngleMotor.getRotations(), setpoint));
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }

    public void motorInit() {
        intakeMotor = new VortexMotorController(MainConstants.IDs.Motors.INTAKE_MOTOR_ID);
        intakeAngleMotor = new VortexMotorController(MainConstants.IDs.Motors.INTAKE_ANGLE_MOTOR_ID);

        intakeMotor.setPosition(0);
        intakeAngleMotor.setPosition(0);
    }

    public void PIDInit() {
        pidController.setP(MainConstants.PIDConstants.INTAKE_PID.P);
        pidController.setP(MainConstants.PIDConstants.INTAKE_PID.I);
        pidController.setP(MainConstants.PIDConstants.INTAKE_PID.D);
    }

    public Command setIntakeSpeed(double percent) {
        return this.runOnce(() -> intakeMotor.set(percent));
    }

    public Command stowIntake() {
        return this.runOnce(() -> setpoint = MainConstants.Setpoints.STOW_INTAKE);
    }

    public Command deployIntake() {
        return this.runOnce(() -> setpoint = MainConstants.Setpoints.DEPLOY_INTAKE);
    }
}