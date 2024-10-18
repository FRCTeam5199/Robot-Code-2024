package frc.robot.subsystems.minor;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.constants.MainConstants;
import frc.robot.utils.TagalongAngle;

public enum ArmPivotSetpoints implements TagalongAngle {
    ZERO(0.0),
    SUB(MainConstants.Setpoints.ARM_SUBWOOFER_SETPOINT),
    MID(MainConstants.Setpoints.ARM_SAFE_SETPOINT + 4),
    TWO_PIECE_EXTENDED_RED(MainConstants.Setpoints.ARM_TWO_PIECE_EXTENDED_SHOT_RED_SETPOINT),
    TWO_PIECE_EXTENDED_BLUE(MainConstants.Setpoints.ARM_TWO_PIECE_EXTENDED_SHOT_BLUE_SETPOINT),
    AMP(MainConstants.Setpoints.ARM_AMP_SETPOINT),
    BACK(MainConstants.Setpoints.ARM_SPEAKER_BACK_SETPOINT),
    STABLE(MainConstants.Setpoints.ARM_STABLE_SETPOINT),
    INTAKE_STEP_UP(51),
    INTAKE(MainConstants.Setpoints.ARM_INTAKE_SETPOINT),
    VERTICAL(120),
    LONG(80),
    TRAP(MainConstants.Setpoints.ARM_TRAP_SETPOINT),
    CLIMB_MIDDLE(MainConstants.Setpoints.ARM_TRAP_PREP_SETPOINT),
    FAR_SHOT(MainConstants.Setpoints.ARM_FAR_SHOT_SETPOINT),
    HP_STATION(MainConstants.Setpoints.ARM_HP_STATION_SETPOINT),
    SHUTTLE(MainConstants.Setpoints.ARM_SHUTTLE_SETPOINT);

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