package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Hertz;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.InchesPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Second;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.AngularAccelerationUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.subsystems.Elevator;

public final class Constants {
  public static class MotorIDConstants {
    public static final int kCLimberMotorLeader = 12;
    public static final int kCLimberMotorFollower = 13;
    public static final int kScorerMotor = 9;
    public static final int kElevatorMotor1 = 10;
    public static final int kElevatorMotor2 = 11;
    public static final int kIntakeMotor = 9;
  }

  public static class ClimberConstants {
    public static final double kClimberP0 = 2;
    public static final double kClimberI0 = 0;
    public static final double kClimberD0 = 1;
    public static final double kClimberkG0 = 0;
    public static final double kClimberkV0 = 2;

    public static final double kClimberP1 = 2;
    public static final double kClimberI1 = 0;
    public static final double kClimberD1 = 1;
    public static final double kClimberkG1 = 0.5;
    public static final double kClimberkV1 = 2;

    public static final double KGearRatio = 126;
    public static final Angle kClimberExtendedSetpoint = Degrees.of(190);
    public static final Angle kClimberAtCageSetpoint = Degrees.of(170);
    public static final Angle kClimberRetractedSetpoint = Degrees.of(90);
    public static final Angle kClimberSensorError = Degrees.of(1.5);
    // Sim Constants
    public static final Distance kClimberArmLength = Inches.of(6);
    public static final Mass kClimberMass = Pounds.of(0.5);
    public static final Angle kClimberArmMinAngle = Degrees.of(-30);
    public static final Angle kClimberArmMaxAngle = Degrees.of(200);
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

  public static final String CANivoreName = "Default Name";
  public static final String RIOName = "rio";

  // Scorer Constants
  public static class CoralScorerConstants {
    public static final double kSpeed = 0.5;
  }

  // Intake Constants
  public static class IntakeConstants {
    public static final double kSpeed = 0.5;
  }

  public static class ElevatorConstants {
    public static final Distance kL0Height = Meters.of(0.252);
    // L1 needs to be adjusted once it actually is worth it
    public static final Distance kL1Height = Inches.of(15);
    public static final Distance kL2Height = Inches.of(16.62);
    public static final Distance kL3Height = Inches.of(30.9);
    public static final Distance kL4Height = Inches.of(55);

    public static final int elvLimitID = 0;

    public static final double P = 1.5;
    public static final double I = 0.04;
    public static final double D = 0.02;
    public static final double FF = 0.0;
    public static final Distance kBottomLimit = Inches.of(9);
    public static final Distance kTopLimit = Inches.of(75);
    public static final double kElevatorConversion = 1.0;

    // The carriage on the elv effectivly adds a gearing multiplier of 1
    public static final double kCarageFactor = 1;

    // Simulation Constants
    public static final DCMotor kElevatorGearbox = DCMotor.getKrakenX60(2);
    public static final double elevatorOutput = .30;
    public static final double kElevatorGearing = 1.0;
    public static final Mass kCarriageMass = Pounds.of(5.15);
    public static final Distance kElevatorDrumRadius = Inches.of(.75 / 2);
    public static final Distance kMinElevatorHeight = Inches.zero();
    public static final Distance kMaxElevatorHeight = Inches.of(72);
    public static final Distance kElevatorDrumCircumference =
        kElevatorDrumRadius.times(2 * Math.PI);
    public static final AngularVelocity MMVel = Elevator.heightToRotations(InchesPerSecond.of(60));
    public static final AngularAcceleration MMAcc = MMVel.times(Hertz.of(5));
    public static final Velocity<AngularAccelerationUnit> MMJerk =
        RotationsPerSecondPerSecond.per(Second).of(MMAcc.in(RotationsPerSecondPerSecond)).times(10);
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
