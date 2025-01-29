package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Pounds;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.wpilibj.RobotBase;

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
    public static final Angle kClimberExtended = Degrees.of(190);
    public static final Angle kClimberCage = Degrees.of(170);
    public static final Angle kClimberRetracted = Degrees.of(0);

    public static class ClimberSimConstants {
      public static final double kClimberGearRatio = 126;
      public static final Distance kClimberArmLength = Inches.of(6);
      public static final Mass kClimberMass = Pounds.of(0.5);
      public static final Angle kClimberArmMinAngle = Degrees.of(-20);
      public static final Angle kClimberArmMaxAngle = Degrees.of(200);
      public static final MomentOfInertia kClimberArmMOI = KilogramSquareMeters.of(0.077);
    }
  }

  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static final Distance kFieldWidth = Inches.of(317);
  public static final Distance kFieldLength = Inches.of(690 + (7 / 8));

  public final class DrivetrainConstants {
    // PathPlanner config constants
    public static final double ROBOT_MASS_KG = 74.088;
    public static final double ROBOT_MOI = 6.883;
    public static final double WHEEL_COF = 1.2;

    // Scoring Poses for PathFinder
    public static final Pose2d[] SCORE_POSES =
        new Pose2d[] {
          new Pose2d(Meters.of(3.173), Meters.of(4.193), new Rotation2d(0)),
          new Pose2d(Meters.of(3.173), Meters.of(3.854), new Rotation2d(0)),
          new Pose2d(Meters.of(3.686), Meters.of(2.973), new Rotation2d(60)),
          new Pose2d(Meters.of(3.974), Meters.of(2.801), new Rotation2d(60)),
          new Pose2d(Meters.of(5.000), Meters.of(2.799), new Rotation2d(120)),
          new Pose2d(Meters.of(5.286), Meters.of(2.966), new Rotation2d(120)),
          new Pose2d(Meters.of(5.803), Meters.of(3.854), new Rotation2d(180)),
          new Pose2d(Meters.of(5.803), Meters.of(4.194), new Rotation2d(180)),
          new Pose2d(Meters.of(5.288), Meters.of(5.082), new Rotation2d(-120)),
          new Pose2d(Meters.of(5.001), Meters.of(5.245), new Rotation2d(-120)),
          new Pose2d(Meters.of(3.972), Meters.of(5.244), new Rotation2d(-60)),
          new Pose2d(Meters.of(3.684), Meters.of(5.078), new Rotation2d(-60)),
        };
  }
}
