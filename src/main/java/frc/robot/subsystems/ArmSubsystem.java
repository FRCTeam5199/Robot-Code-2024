package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Main;
import frc.robot.abstractMotorInterfaces.VortexMotorController;
import frc.robot.constants.MainConstants;


public class ArmSubsystem extends SubsystemBase {
    private static ArmSubsystem armSubsystem;

    private static boolean subsystemStatus = false;

    private static VortexMotorController armMotorL;
    private static VortexMotorController armMotorR;
    private static boolean isBrakeMode = false;
    public boolean inAuton = false;
    public double encoderValue;
    private CANSparkMax armEncoderMotor;
    private SparkAbsoluteEncoder armEncoder;
    private PIDController rotatePIDController;
    private boolean isAiming = false;
    private double rotateSetpoint = 120;
    private double rotateOffset;

    public ArmSubsystem() {
    }

    /**
     * Gets the instnace of the Arm Subsystem.
     */
    public static ArmSubsystem getInstance() {
        if (armSubsystem == null) {
            armSubsystem = new ArmSubsystem();
        }

        return armSubsystem;
    }

    public static void toggleBrakeMode() {
        isBrakeMode = !isBrakeMode;
        armMotorL.setBrake(isBrakeMode);
    }

    /**
     * init for arm and pid controller
     */
    public void init() {
        motorInit();
        //  try { motorInit(); } catch (Exception exception) {
        // 	System.err.println("One or more issues occured while trying to initalize motors for Arm Subsystem");
        // 	System.err.println("Exception Message:" + exception.getMessage());
        // 	System.err.println("Exception Cause:" + exception.getCause());
        // 	System.err.println("Exception Stack Trace:" + exception.getStackTrace()); }

        try {
            PIDInit();
        } catch (Exception exception) {
            System.err.println("One or more issues occured while trying to initalize PID for Arm Subsystem");
            System.err.println("Exception Message:" + exception.getMessage());
            System.err.println("Exception Cause:" + exception.getCause());
            System.err.println("Exception Stack Trace:" + exception.getStackTrace());
        }
    }

    public boolean getSubsystemStatus() {
        return subsystemStatus;
    }

    public void motorInit() {
        armMotorL = new VortexMotorController(MainConstants.IDs.Motors.ARM_MOTOR_ID);
        armMotorR = new VortexMotorController(MainConstants.IDs.Motors.ARM_MOTOR2_ID);

        armMotorL.setInvert(false);
        armMotorL.setBrake(true);

        armMotorR.setInvert(true);
        armMotorR.setBrake(true);

        armMotorL.setCurrentLimit(40);
        armMotorR.setCurrentLimit(40);

        armEncoderMotor = new CANSparkMax(MainConstants.IDs.Motors.ARM_ENCODER_MOTOR, MotorType.kBrushed);
        armEncoder = armEncoderMotor.getAbsoluteEncoder(Type.kDutyCycle);
    }

    public void PIDInit() {
        rotatePIDController = new PIDController(MainConstants.PIDConstants.ARM_PID.P, MainConstants.PIDConstants.ARM_PID.I, MainConstants.PIDConstants.ARM_PID.D);
    }

    public boolean checkMotors() {
        if (armMotorL != null && armEncoderMotor != null && armEncoder != null) {
            return true;
        } else {
            return false;
        }
    }

    public boolean checkPID() {
        if (rotatePIDController != null) {
            return true;
        } else {
            return false;
        }
    }

    @Override
    public void periodic() {
        if (armEncoder.getPosition() < 175) {
            encoderValue = armEncoder.getPosition();
        } else {
            encoderValue = 160;
        }
        if (inAuton) {
            armMotorL.set(rotatePIDController.calculate(encoderValue, rotateSetpoint));
            armMotorR.set(rotatePIDController.calculate(encoderValue, rotateSetpoint));
        }
        if (isAiming) {
            armMotorL.set(rotatePIDController.calculate(encoderValue, rotateSetpoint + rotateOffset));
            armMotorR.set(rotatePIDController.calculate(encoderValue, rotateSetpoint + rotateOffset));
        } else {
            armMotorL.set(rotatePIDController.calculate(encoderValue, MainConstants.Setpoints.ARM_STABLE_SETPOINT));
            armMotorR.set(rotatePIDController.calculate(encoderValue, MainConstants.Setpoints.ARM_STABLE_SETPOINT));
        }
    }

    private void subsystemPeriodic() {
        if (armEncoder.getPosition() < 175) {
            encoderValue = armEncoder.getPosition();
        }

        // if (encoderValue > 170) {
        // 	rotateSetpoint = 160;
        // } else {
        if (isAiming) {
            armMotorL.set(rotatePIDController.calculate(encoderValue, rotateSetpoint));
            armMotorR.set(rotatePIDController.calculate(encoderValue, rotateSetpoint));
        } else {
            armMotorL.set(rotatePIDController.calculate(encoderValue, MainConstants.Setpoints.ARM_STABLE_SETPOINT));
            armMotorR.set(rotatePIDController.calculate(encoderValue, MainConstants.Setpoints.ARM_STABLE_SETPOINT));
        }
        // }
    }

    public Command increaseOffset() {
        return this.runOnce(() -> rotateOffset += 5);
    }

    public Command decreaseOffset() {
        return this.runOnce(() -> rotateOffset -= 5);
    }

    public AbsoluteEncoder getArmEncoder() {
        if (!subsystemStatus) return null;
        return armEncoder;
    }

    public Command isAiming(boolean bool) {
        return this.runOnce(() -> isAiming = bool);
    }

    /**
     * @param percent move armMotor at a percent(-1 to 1)
     * @return command to spin motor at percent
     */
    public Command moveAtPercent(double percent) {
        return this.run(() -> armMotorL.set(percent));
    }

    /**
     * @param rotations adds this to setPoint variable(setpoint)
     * @return command to move to setPoint
     */
    public Command changeArmSetpoint(double rotations) {
        // System.out.println(rotateSetpoint + rotations);
        return this.runOnce(() -> this.rotateSetpoint += rotations);
    }

    /**
     * @param setpoint set armMotor to setPoint
     * @return command to move armMotor to setPoint
     */
    public Command setArmSetpoint(double setpoint) {
        return this.runOnce(() -> rotateSetpoint = setpoint);
    }

    /**
     * Sets the Arm setpoint to the Arm Stable setpoint
     */
    public Command rotateStable() {
        return this.runOnce(() -> rotateSetpoint = MainConstants.Setpoints.ARM_STABLE_SETPOINT).andThen(isAiming(false));
    }

    /**
     * Sets the Arm setpoint to the Arm Amp setpoint
     */
    public Command rotateAmp() {
        return this.runOnce(() -> rotateSetpoint = MainConstants.Setpoints.ARM_AMP_SETPOINT);
    }

    /**
     * Sets the Arm setpoint to the Arm Back setpoint
     */
    public Command rotateBack() {
        return this.runOnce(() -> rotateSetpoint = MainConstants.Setpoints.ARM_SPEAKER_BACK_SETPOINT);
    }

    /**
     * Sets the Arm setpoint to the Arm Front setpoint
     */
    public Command rotateFront() {
        return this.runOnce(() -> rotateSetpoint = MainConstants.Setpoints.ARM_SPEAKER_FRONT_SETPOINT);
    }

    /**
     * Sets the Arm setpoint to slightly above retracted intake
     */
    public Command rotateIntake() {
        return this.runOnce(() -> rotateSetpoint = MainConstants.Setpoints.ARM_INTAKE_SETPOINT);
    }

    /**
     * Sets the Arm setpoint to the Arm climb setpoint
     */
    public Command rotateClimb() {
        return this.runOnce(() -> rotateSetpoint = MainConstants.Setpoints.ARM_CLIMB_SETPOINT);
    }

    public Command rotateTopPiece() {
        return this.runOnce(() -> rotateSetpoint = MainConstants.Setpoints.ARM_TOP_PIECE_SETPOINT);
    }

    public Command rotateBottomPiece() {
        return this.runOnce(() -> rotateSetpoint = MainConstants.Setpoints.ARM_BOTTOM_PIECE_SETPOINT);
    }

    public Command rotateMiddlePiece() {
        return this.runOnce(() -> rotateSetpoint = MainConstants.Setpoints.ARM_MIDDLE_PIECE_SETPOINT);
    }

    /**
     * Sets the Arm setpoint to the Arm Trap setpoint
     */
    public Command rotateTrap() {
        return this.runOnce(() -> rotateSetpoint = MainConstants.Setpoints.ARM_TRAP_SETPOINT);
    }

    /**
     * Sets the Arm setpoint to the Arm Subwoofer setpoint
     */
    public Command rotateSubwoofer() {
        return this.runOnce(() -> rotateSetpoint = MainConstants.Setpoints.ARM_SUBWOOFER_SETPOINT);
    }

    /**
     * Sets the Arm setpoint to the Arm Podium setpoint
     */
    public Command rotatePodium() {
        return this.runOnce(() -> rotateSetpoint = MainConstants.Setpoints.ARM_PODIUM_SETPOINT);
    }

    public Command rotateRedLine() {
        return this.runOnce(() -> rotateSetpoint = MainConstants.Setpoints.ARM_RED_LINE_SETPOINT);
    }
}