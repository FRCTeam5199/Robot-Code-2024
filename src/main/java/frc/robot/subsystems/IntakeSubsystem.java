package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.abstractMotorInterfaces.VortexMotorController;
import frc.robot.constants.MainConstants;

public class IntakeSubsystem implements Subsystem {
    public VortexMotorController intakeMotor;
    public VortexMotorController intakeActuatorMotor;
    public PIDController pidController;
    public double setpoint;

    public IntakeSubsystem() {}

    public void init() {
        motorInit();
        PIDInit();
        
        Shuffleboard.getTab("Status").add("Intake Subsystem Status", true).getEntry();

    }

    @Override
    public void periodic() {
        intakeActuatorMotor.set(pidController.calculate(intakeActuatorMotor.getRotations(), setpoint));
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }

    /**
     * motor init of intake subsystem
     */
    public void motorInit() {
        intakeMotor = new VortexMotorController(MainConstants.IDs.Motors.INTAKE_MOTOR_ID);
        intakeActuatorMotor = new VortexMotorController(MainConstants.IDs.Motors.INTAKE_ACTUATOR_MOTOR_ID);

        intakeMotor.getEncoder().setPosition(0);
        intakeActuatorMotor.getEncoder().setPosition(0);

        intakeMotor.setInvert(false);
        intakeActuatorMotor.setInvert(true);
    }

    /**
     * init of the pidController
     */
    public void PIDInit() {
        pidController = new PIDController(MainConstants.PIDConstants.INTAKE_PID.P, MainConstants.PIDConstants.INTAKE_PID.I, MainConstants.PIDConstants.INTAKE_PID.D);
    }

    /**
     * 
     * @param percent speed to set intake to
     * @return command to spin intake
     */
    public Command setIntakeSpeed(double percent) {
        return this.runOnce(() -> intakeMotor.set(percent));
    }

    public Command setIntakeActuatorSpeed(double percent) {
        return this.runOnce(() -> intakeActuatorMotor.set(percent));
    }

    /**
     * retract the intake 
     * @return command to retract intake
     */
    public Command stowIntake() {
        return this.runOnce(() -> setpoint = MainConstants.Setpoints.STOW_INTAKE);
    }

    /**
     * deploy intake
     * @return command to deploy the intake
     */
    public Command deployIntake() {
        return this.runOnce(() -> setpoint = MainConstants.Setpoints.DEPLOY_INTAKE);
    }
}