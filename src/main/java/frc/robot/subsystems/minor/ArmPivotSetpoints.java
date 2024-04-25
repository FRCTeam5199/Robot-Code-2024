package frc.robot.subsystems.minor;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.utils.TagalongAngle;
import frc.robot.Main;
import frc.robot.constants.MainConstants;

public enum ArmPivotSetpoints implements TagalongAngle {
    //need offset dont delete
    ZERO(0.0),
    SUB(MainConstants.Setpoints.ARM_SUBWOOFER_SETPOINT + 2),
    MID(MainConstants.Setpoints.ARM_SAFE_SETPOINT + 1.5),
    TWO_PIECE_EXTENDED_RED(MainConstants.Setpoints.ARM_TWO_PIECE_EXTENDED_SHOT_RED + 1.5),
    TWO_PIECE_EXTENDED_BLUE(MainConstants.Setpoints.ARM_TWO_PIECE_EXTENDED_SHOT_BLUE + 1.5),
    AMP(MainConstants.Setpoints.ARM_AMP_SETPOINT + 1.25),
    BACK(MainConstants.Setpoints.ARM_SPEAKER_BACK_SETPOINT + .75),
    STABLE(MainConstants.Setpoints.ARM_STABLE_SETPOINT + 1),
    INTAKE_STEP_UP(50 + 1),
    INTAKE(3),
    VERTICAL(120),
    LONG(80),
    TRAP(MainConstants.Setpoints.ARM_TRAP_SETPOINT + 0.75),
    CLIMB_MIDDLE(MainConstants.Setpoints.ARM_TRAP_PREP_SETPOINT + 2);

    private final Rotation2d _value;


    ArmPivotSetpoints(double degree) {
        _value = Rotation2d.fromDegrees(degree);
    }

    @Override
    public double getDegrees() {
        return _value.getDegrees();
    }

    @Override
    public double getRadians() {
        return _value.getRadians();
    }

    @Override
    public double getRotations() {
        return _value.getRotations();
    }

}