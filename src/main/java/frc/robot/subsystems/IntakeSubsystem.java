package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.abstractMotorInterfaces.VortexMotorController;
import frc.robot.constants.MainConstants;

public class IntakeSubsystem extends SubsystemBase {
    private static IntakeSubsystem intakeSubsystem;

    private static boolean subsystemStatus = false;

    public VortexMotorController intakeMotor;
    public VortexMotorController intakeActuatorMotor;
    public PIDController intakeActuatorMotorPIDController;
    public double setpoint;
    public double rotateOffset;
    public SparkPIDController sparkPIDController;
    TrapezoidProfile profile;
    TrapezoidProfile.State current;
    TrapezoidProfile.State goal;

    public IntakeSubsystem() {
    }

    /**
     * Gets the instnace of the Arm Subsystem.
     */
    public static IntakeSubsystem getInstance() {
        if (intakeSubsystem == null) {
            intakeSubsystem = new IntakeSubsystem();
        }

        return intakeSubsystem;
    }

    public void init() {
        try {
            motorInit();
        } catch (Exception exception) {
            System.err.println("One or more issues occured while trying to initalize motors for Intake Subsystem");
            System.err.println("Exception Message:" + exception.getMessage());
            System.err.println("Exception Cause:" + exception.getCause());
            System.err.println("Exception Stack Trace:" + exception.getStackTrace());
        }

        try {
            PIDInit();
        } catch (Exception exception) {
            System.err.println("One or more issues occured while trying to initalize PID for Intake Subsystem");
            System.err.println("Exception Message:" + exception.getMessage());
            System.err.println("Exception Cause:" + exception.getCause());
            System.err.println("Exception Stack Trace:" + exception.getStackTrace());
        }

        profile = new TrapezoidProfile(new TrapezoidProfile.Constraints(125, 250));
        current = new TrapezoidProfile.State();
        goal = new TrapezoidProfile.State();
    }

    public boolean getSubsystemStatus() {
        return subsystemStatus;
    }

    /**
     * motor init of intake subsystem
     */
    public void motorInit() {
        intakeMotor = new VortexMotorController(MainConstants.IDs.Motors.INTAKE_MOTOR_ID);
        intakeMotor.getEncoder().setPosition(0);
        intakeMotor.setInvert(false);
        intakeMotor.setBrake(false);

        intakeActuatorMotor = new VortexMotorController(MainConstants.IDs.Motors.INTAKE_ACTUATOR_MOTOR_ID);
        intakeActuatorMotor.getEncoder().setPosition(0);
        intakeActuatorMotor.setInvert(true);
        intakeActuatorMotor.setBrake(true);

        intakeMotor.setCurrentLimit(40);
        intakeActuatorMotor.setCurrentLimit(60);
    }

    /**
     * init of the pidController
     */
    public void PIDInit() {
        intakeActuatorMotorPIDController = new PIDController(MainConstants.PIDConstants.INTAKE_PID.P, MainConstants.PIDConstants.INTAKE_PID.I, MainConstants.PIDConstants.INTAKE_PID.D);
    }

    public boolean checkMotors() {
        if (intakeMotor != null && intakeActuatorMotor != null) {
            return true;
        } else {
            return false;
        }
    }

    public boolean checkPID() {
        if (intakeActuatorMotorPIDController != null) {
            return true;
        } else {
            return false;
        }
    }

    public RelativeEncoder getIntakeMotorEncoder() {
        return intakeMotor.getEncoder();
    }

    public RelativeEncoder getIntakeActuatorMotorEncoder() {
        return intakeActuatorMotor.getEncoder();
    }

    @Override
    public void periodic() {
        if (checkMotors() && checkPID()) {
            subsystemStatus = true;
        } else {
            subsystemStatus = false;
        }

        if (subsystemStatus) {
            subsystemPeriodic();
        }
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }

    private void subsystemPeriodic() {
        current = profile.calculate(0.2, current, goal);
        intakeActuatorMotor.set(intakeActuatorMotorPIDController.calculate(intakeActuatorMotor.getRotations(), current.position));
    }

    public Command increaseOffset() {
        return this.runOnce(() -> rotateOffset += 1);
    }

    public Command decreaseOffset() {
        return this.runOnce(() -> rotateOffset -= 1);
    }

    /**
     * @param percent speed to set intake to
     * @return command to spin intake
     */
    public Command setIntakeSpeed(double percent) {
        return this.runOnce(() -> intakeMotor.set(percent));
    }

    public Command setIntakeActuatorSpeed(double percent) {
        return this.runOnce(() -> intakeActuatorMotor.set(percent));
    }

    public Command setIntakeActuatorTarget(double target) {
        return this.runOnce(() -> setpoint = target);
    }

    /**
     * retract the intake
     *
     * @return command to retract intake
     */
    public Command stowIntake() {
        return this.runOnce(() -> goal = new TrapezoidProfile.State(MainConstants.Setpoints.STOW_INTAKE_SETPOINT, 0));
    }

    /**
     * deploy intake
     *
     * @return command to deploy the intake
     */
    public Command deployIntake() {
        return this.runOnce(() -> goal = new TrapezoidProfile.State(MainConstants.Setpoints.DEPLOY_INTAKE_SETPOINT, 0));
    }
}