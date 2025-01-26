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
    public static final double kClimberP0 = 1;
    public static final double kClimberI0 = 0;
    public static final double kClimberD0 = 0;
    public static final double kClimberkG0 = 0;
    public static final double kClimberkV0 = 2.5;

    public static final double kClimberP1 = 2;
    public static final double kClimberI1 = 0;
    public static final double kClimberD1 = 0;
    public static final double kClimberkG1 = 0.26;
    public static final double kClimberkV1 = 2.5;

    public static final double KGearRatio = 126;
    public static final Angle kClimberExtended = Degrees.of(-180 * KGearRatio);
    public static final Angle kClimberRetracted = Degrees.of(-20 * KGearRatio);
  }
}
