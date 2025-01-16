package frc.robot.controls;

public enum ButtonPanelButtons {
    CLIMB_MODE(1, 13),
    LEFT_CLIMB_SETUP(1, 4),
    RIGHT_CLIMB_SETUP(1, 5),
    CLIMB_UP(1, 14),
    CLIMB_DOWN(1, 10),
    LEFT_CLIMB_UP(1, 11),
    LEFT_CLIMB_DOWN(1, 19),
    RIGHT_CLIMB_UP(1, 9),
    RIGHT_CLIMB_DOWN(1, 12),
    CLIMB_ARM_TRAP_SETPOINT(1, 6),
    CLIMB_ARM_TRAP_PREP_SETPOINT(1, 8),
    FLIPPY_DO_UP(1, 3),
    FLIPPY_DO_DOWN(1, 7),
    ARM_SUB_SETPOINT(2, 13),
    ARM_BACK_SETPOINT(2, 21),
    ARM_SAFE_SETPOINT(2, 4),
    ARM_AMP_SETPOINT(2, 5),
    ARM_FAR_SHOT_SETPOINT(2, 12),
    ARM_HP_STATION_SETPOINT(2, 7),
    MOVE_INTAKE_SETPOINT_UP(2, 22),
    MOVE_INTAKE_SETPOINT_DOWN(2, 8),
    MOVE_ARM_SETPOINT_UP(2, 6),
    MOVE_ARM_SETPOINT_DOWN(2, 19),
    INCREASE_SHOOTER_SPEED(2, 10),
    DECREASE_SHOOTER_SPEED(2, 3),
    AUX_LEFT_TOP(2, 20),
    AUX_RIGHT_TOP(2, 14),
    AUX_LEFT_BOTTOM(2, 9),
    AUX_RIGHT_BOTTOM(2, 9);
    final int arduinoID;
    final int buttonId;

    ButtonPanelButtons(int arduinoID, int buttonID) {
        this.arduinoID = arduinoID;
        this.buttonId = buttonID;
    }
}
