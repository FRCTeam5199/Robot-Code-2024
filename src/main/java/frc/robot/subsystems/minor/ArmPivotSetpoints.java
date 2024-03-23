package frc.robot.subsystems.minor;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.utils.TagalongAngle;

public enum ArmPivotSetpoints implements TagalongAngle {
  //need offset dont delete
  ZERO(0.0),
  VERTICAL(120.0+1),
  TEN(40.0+1),
  SIXTY(60.0+1),
  ONE_FOUR_ONE(141+1);

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