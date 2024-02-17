package frc.robot.controls;

public enum ButtonPanelButtons {
    CENTER_CLIMB_SETUP(1, 4),
    LEFT_CLIMB_SETUP(1, 9),
    RIGHT_CLIMB_SETUP(1, 10),
    CLIMB_UP(1, 14),
    CLIMB_DOWN(1, 21),
    LEFT_CLIMB_UP(1, 3),
    LEFT_CLIMB_DOWN(1, 20),
    RIGHT_CLIMB_UP(1, 11),
    RIGHT_CLIMB_DOWN(1, 5),
    CLIMB_ARM_UP_SETPOINT(1, 12),
    CLIMB_ARM_TRAP_SETPOINT(1, 6),
    FLIPPY_DO_UP(1, 7),
    FLIPPY_DO_DOWN(1, 8),
    ARM_SUB_SETPOINT(2, 5),
    ARM_BACK_SETPOINT(2, 13),
    ARM_SAFE_SETPOINT(2, 22),
    ARM_AMP_SETPOINT(2, 4),
    ARM_FAR_SHOT_SETPOINT(2, 11),
    ARM_HP_STATION_SETPOINT(2, 6),
    MOVE_INTAKE_SETPOINT_UP(2, 21),
    MOVE_INTAKE_SETPOINT_DOWN(2, 26),
    MOVE_ARM_SETPOINT_UP(2, 12),
    MOVE_ARM_SETPOINT_DOWN(2, 23),
    INCREASE_SHOOTER_SPEED(2, 7),
    DECREASE_SHOOTER_SPEED(2, 20);
    final int arduinoID;
    final int buttonId;

    ButtonPanelButtons(int arduinoID, int buttonID) {
        this.arduinoID = arduinoID;
        this.buttonId = buttonID;
    }
}
