package frc.robot;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.units.measure.Angle;

public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;
  }

  public static class MotorIDConstants {
    public static final int kCLimberMotorLeader = 0;
    public static final int kCLimberMotorFollower = 0;
  }

  public static class ClimberConstants {
    public static final double kClimberP = 1;
    public static final double kClimberI = 0;
    public static final double kClimberD = 0;
    public static final double KGearRatio = 126;
    public static final Angle kClimberExtended = Degrees.of(-180 * KGearRatio);
    public static final Angle kClimberRetracted = Degrees.of(-20 * KGearRatio);
  }
}
