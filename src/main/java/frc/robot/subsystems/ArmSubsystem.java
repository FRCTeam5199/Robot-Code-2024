package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.abstractMotorInterfaces.VortexMotorController;
import frc.robot.constants.MainConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drivetrain.SwerveDrive;


public class ArmSubsystem extends SubsystemBase {
    private static ArmSubsystem armSubsystem;
    private static AprilTagSubsystem aprilTagSubsystem = new AprilTagSubsystem();

    private static boolean subsystemStatus = false;

    private static VortexMotorController armMotorL;
    private static VortexMotorController armMotorR;
    private static boolean isBrakeMode = false;
    public boolean inAuton = false;
    public boolean climbMode = false;
    public double encoderValue;
    public boolean isAiming = true;
    public boolean autoAiming = false;
    public SwerveDrive drive = TunerConstants.DriveTrain;
    private SparkAbsoluteEncoder armEncoder;
    private PIDController rotatePIDController;
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
        armMotorR.setBrake(isBrakeMode);
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

        armEncoder = armMotorL.getAbsoluteEncoder(Type.kDutyCycle);

    }

    public void PIDInit() {

        rotatePIDController = new PIDController(0.0083262, 0.00569673212, 0.003);
        rotatePIDController.setIZone(3);

    }

    public boolean checkMotors() {
        if (armMotorL != null && armEncoder != null) {
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
        if (checkMotors() && checkPID()) {
            subsystemStatus = true;
        } else {
            subsystemStatus = false;
        }

        if (subsystemStatus) {
            subsystemPeriodic();
        }
    }

    private void subsystemPeriodic() {
        if (armEncoder.getPosition() < 170) {
            encoderValue = armEncoder.getPosition();
        } else if (armEncoder.getPosition() > 170 && armEncoder.getPosition() < 200) {
            encoderValue = 170;
        } else if (armEncoder.getPosition() > 200 && armEncoder.getPosition() < 361) {
            encoderValue = 0;
        } else if (armEncoder.getPosition() > 200) {
            armMotorL.set(rotatePIDController.calculate(encoderValue, 0));
            armMotorR.set(rotatePIDController.calculate(encoderValue, 0));
        }
        if (inAuton) {
            goToSetpoint(rotateSetpoint, rotateOffset);
        }
        if (autoAiming == false) {
            if (isAiming) {
                if(rotatePIDController.calculate(encoderValue, rotateSetpoint + rotateOffset)  > 0){
                    rotatePIDController.setPID(0.0081262, 0.00472673212, 0.00);
                    rotatePIDController.setIZone(3);
                }
                if (rotatePIDController.calculate(encoderValue, rotateSetpoint + rotateOffset)  < 0){
                    rotatePIDController = new PIDController(0.0072262, 0.00219673212, 0.00);
                    rotatePIDController.setIZone(3);
                }
                goToSetpoint(rotateSetpoint, rotateOffset);
                //   System.out.println("rotateSetpoint"  + rotateSetpoint);
                // System.out.println("encoder value" + encoderValue);

            } else {
                goToSetpoint(MainConstants.Setpoints.ARM_STABLE_SETPOINT, rotateOffset);
            }
        } else {
            if(rotatePIDController.calculate(encoderValue, aprilTagSubsystem.armSpeakersAligning())  > 0){
                    rotatePIDController.setIZone(2);
                    rotatePIDController.setPID(0.0091262, 0.0089673212, 0.00);
                }
                if (rotatePIDController.calculate(encoderValue, aprilTagSubsystem.armSpeakersAligning())  < 0){
                    rotatePIDController = new PIDController(0.00019262, 0.000309673212, 0.00);
                    rotatePIDController.setIZone(3);
                }
                goToSetpoint(aprilTagSubsystem.armSpeakersAligning(), 0);
            
            // System.out.println("april tag value arm "  + aprilTagSubsystem.armSpeakersAligning());
            // System.out.println("encoder value     " + encoderValue);
        }
    }

    public static void setBrakeTrue(){
        armMotorL.setBrake(true);

        armMotorR.setBrake(true);
    }

    private void goToSetpoint(double rotateSetpoint, double rotateOffset) {
        if (climbMode) {
            armMotorL.set(rotatePIDController.calculate(encoderValue, rotateSetpoint + rotateOffset) * 0.5);
            armMotorR.set(rotatePIDController.calculate(encoderValue, rotateSetpoint + rotateOffset) * 0.5);
        } else {
            armMotorL.set(rotatePIDController.calculate(encoderValue, rotateSetpoint + rotateOffset));
            armMotorR.set(rotatePIDController.calculate(encoderValue, rotateSetpoint + rotateOffset));
        }
    }


    public Command switchAiming() {
        return this.runOnce(() -> autoAiming = !autoAiming).andThen(() -> System.out.println(autoAiming));
    }

    public Command isAutoAiming(boolean bool) {
        return this.runOnce(() -> autoAiming = bool);
    }

    public Command setClimbMode(boolean bool) {
        return this.runOnce(() -> climbMode = bool);
    }

    public Command isAiming(boolean bool) {
        return this.runOnce(() -> isAiming = bool);
    }

    public Command increaseOffset(double amount) {
        return this.runOnce(() -> rotateOffset += amount);
    }

    public Command decreaseOffset(double amount) {
        return this.runOnce(() -> rotateOffset -= amount);
    }

    public AbsoluteEncoder getArmEncoder() {
        if (!subsystemStatus) return null;
        return armEncoder;
    }

    /**
     * @param percent move armMotor at a percent(-1 to 1)
     * @return command to spin motor at percent
     */
    public Command moveAtPercent(double percent) {
        return this.run(() -> armMotorL.set(percent));
    }


    /**
     * @param setpoint set armMotor to setPoint
     * @return command to move armMotor to setPoint
     */
    public Command setArmSetpoint(double setpoint) {
        return this.runOnce(() -> System.out.println("changed to " + setpoint)).andThen(() -> rotateSetpoint = setpoint);
    }


    /**
     * Sets the Arm setpoint to the Arm Stable setpoint
     */
    public Command rotateStable() {
        return this.runOnce(() -> rotateSetpoint = MainConstants.Setpoints.ARM_STABLE_SETPOINT);
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
    public Command rotateTrap() {
        return this.runOnce(() -> rotateSetpoint = MainConstants.Setpoints.ARM_TRAP_SETPOINT);
    }

    public Command rotatePrepClimbP2() {
        return this.runOnce(() -> rotateSetpoint = MainConstants.Setpoints.ARM_TRAP_PREP2_SETPOINT);
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
    public Command rotateTrapPrep() {
        return this.runOnce(() -> rotateSetpoint = MainConstants.Setpoints.ARM_TRAP_PREP_SETPOINT);
    }

    /**
     * Sets the Arm setpoint to the Arm Subwoofer setpoint
     */
    public Command rotateSub() {
        return this.runOnce(() -> rotateSetpoint = MainConstants.Setpoints.ARM_SUBWOOFER_SETPOINT);
    }

    /**
     * Sets the Arm setpoint to the Arm Podium setpoint
     */
    public Command rotateSafe() {
        return this.runOnce(() -> rotateSetpoint = MainConstants.Setpoints.ARM_SAFE_SETPOINT);
    }

    public Command rotateFarShot() {
        return this.runOnce(() -> rotateSetpoint = MainConstants.Setpoints.ARM_FAR_SHOT_SETPOINT);
    }

    public Command rotateHPStation() {
        return this.runOnce(() -> rotateSetpoint = MainConstants.Setpoints.ARM_HP_STATION_SETPOINT);
    }
}