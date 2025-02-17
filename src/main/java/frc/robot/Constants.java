package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Hertz;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.InchesPerSecond;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Second;

import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.AngularAccelerationUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.subsystems.Elevator;

// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

public final class Constants {

  /**
   * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when
   * running on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and
   * "replay" (log replay from a file).
   */
  public static class CANIDs {
    // Rev Can Bus
    // 1-8 Motor ID is reserved by the drivebase
    public static final int kClimberMotorFront = 16; // FIXME
    public static final int kClimberMotorBack = 17; // FIXME
    public static final int kScorerMotor = 15;
    public static final int kElevatorMotor1 = 10;
    public static final int kElevatorMotor2 = 11;
    public static final int kIntakeMotor = 13;
    public static final int kPivotMotor = 12;
    public static final int kConveyorMotor = 14;
  }

  public static class DIOConstants {
    public static final int kthroughBoreEncoderID = 10;
    public static final int kClimberFrontEncoderID = 5;
    public static final int kClimberBackEncoderID = 6;
    public static final int gear11ID = 13;
    public static final int gear3ID = 14;
  }

  public static class ClimberConstants {
    public static final double kClimberP0 = 100;
    public static final double kClimberI0 = 0;
    public static final double kClimberD0 = 5;
    public static final double kClimberkG0 = 0;
    public static final double kClimberkV0 = 10;

    public static final double kClimberP1 = 100;
    public static final double kClimberI1 = 0;
    public static final double kClimberD1 = 5;
    public static final double kClimberkG1 = 1;
    public static final double kClimberkV1 = 10;

    public static final double kGearRatio = 126;
    public static final Angle kClimberOffsetAngle = Degrees.of(180);
    public static final Angle kClimberExtendedSetpoint = Degrees.of(225).plus(kClimberOffsetAngle);
    public static final Angle kClimberAtCageSetpoint = Degrees.of(190).plus(kClimberOffsetAngle);
    public static final Angle kClimberRetractedSetpoint = Degrees.of(90).plus(kClimberOffsetAngle);
    public static final Angle kClimberExtendedSetpoint2 = Degrees.of(-10).plus(kClimberOffsetAngle);
    public static final Angle kClimberAtCageSetpoint2 = Degrees.of(10).plus(kClimberOffsetAngle);
    public static final Angle kClimberSensorTolerance = Degrees.of(2.5);
    public static final Angle kExpectedStartAngle = Degrees.of(90);
    public static final InvertedValue kClimberFrontInvert = InvertedValue.CounterClockwise_Positive;
    public static final InvertedValue kClimberBackInvert = InvertedValue.Clockwise_Positive;
    // Sim Constants
    public static final int KClimberMotorsCount = 2;
    public static final Distance kClimberArmLength = Inches.of(6);
    public static final Mass kClimberMass = Pounds.of(0.5);
    public static final Angle kClimberArmMinAngle = Degrees.of(-30).plus(kClimberOffsetAngle);
    public static final Angle kClimberArmMaxAngle = Degrees.of(250).plus(kClimberOffsetAngle);
  }

  // Scorer Constants
  public static class CoralScorerConstants {
    public static final double kSpeed = 0.5;
    public static final double kIntakeSpeed = 0.4;
  }

  // Intake Constants
  public static class IntakeConstants {
    public static final double kIntakeSpeed = -1;
    public static final double kIntakeHandoffSpeed = -0.75;
    public static final double kConveyorSpeed = 0.25;
    public static final double kPivotSpeed = 0.2;
    public static final Angle kPivotRetractAngle = Degrees.of(128);
    public static final Angle kPivotExtendAngle = Degrees.of(-6.25);
    public static final Angle kcoralStation = Degrees.of(101);
    public static final Angle kl1 = Degrees.of(75.5);
    public static final Angle kalgae = Degrees.of(44.5);
    public static final Angle kPivotTolerance = Degrees.of(3);
    public static final double kPivotP = 100.0;
    public static final double kPivotI = 0.0;
    public static final double kPivotD = 0.0;
    public static final double kPivotG = 0.0;

    public static final double kPivotV = 7.29; // 7.20;
    public static final double kPivotA = 0.03; // 0.03;
    public static final GravityTypeValue kPivotGravityType = GravityTypeValue.Arm_Cosine;

    public static final double kGearing = 60;
    public static final double kSensorToMechanism = 60;
    public static final Distance kLength = Feet.of(1);
    public static final Mass kMass = Pounds.of(8);
    public static final MomentOfInertia kMOI =
        KilogramSquareMeters.of(
            SingleJointedArmSim.estimateMOI(kLength.in(Meters), kMass.in(Kilograms)));
    public static final AngularVelocity kMotionMagicCruiseVelocity = DegreesPerSecond.of(450);
    public static final AngularAcceleration kMotionMagicAcceleration =
        kMotionMagicCruiseVelocity.times(Hertz.of(5));
    public static final double kMotionMagicJerk = 80.0;

    public static final double armZero = 0.35;

    public static enum CoralEnum {
      CORAL_TOO_CLOSE,
      CORAL_TOO_FAR,
      NO_CORAL,
      IN_ELEVATOR,
      CORAL_ALIGNED
    }
  }

  // Elevator Constants
  public static class ElevatorConstants {
    public static final Distance kL0Height = Inches.of(0);
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
    public static final Distance kBottomLimit = Inches.of(0);
    public static final Distance kTopLimit = Inches.of(75);
    public static final double kElevatorConversion = 1.0;

    public static final int gear1Toothing = 3;
    public static final int gear2Toothing = 11;

    // Gear for CRT offsets
    // TODO: get actual offset values
    public static final double gear3Offset = 0.0;
    public static final double gear11Offset = 0.0;

    // CRTA - Chinese Remainder Theorem Array
    public static int[][] CRTA = {
      {0, 12, 24, 3, 15, 27, 6, 18, 30, 9, 21},
      {22, 1, 13, 25, 4, 16, 28, 7, 19, 31, 10},
      {11, 23, 2, 14, 26, 5, 17, 29, 8, 20, 32}
    };

    // The carriage on the elv effectivly adds a gearing multiplier of 1
    public static final double kCarageFactor = 1;

    // Simulation Constants
    public static final DCMotor kElevatorGearbox = DCMotor.getKrakenX60(2);
    public static final double elevatorOutput = .10;
    public static final double kElevatorGearing = 1.0;
    public static final Mass kCarriageMass = Pounds.of(4.75);
    public static final Distance kElevatorDrumRadius = Inches.of(.75 / 2);
    public static final Distance kMinElevatorHeight = Inches.zero();
    public static final Distance kMaxElevatorHeight = Inches.of(72);
    public static final Distance kElevatorDrumCircumference =
        kElevatorDrumRadius.times(2 * Math.PI);
    public static final AngularVelocity MMVel = Elevator.heightToRotations(InchesPerSecond.of(100));
    public static final AngularAcceleration MMAcc = MMVel.times(Hertz.of(5));
    public static final Velocity<AngularAccelerationUnit> MMJerk =
        RotationsPerSecondPerSecond.per(Second).of(MMAcc.in(RotationsPerSecondPerSecond)).times(10);
  }

  /**
   * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when
   * running on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and
   * "replay" (log replay from a file).
   */
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

  public static class SensorIDs {
    public static final int kSensor2ID = 2;
    public static final int kSensor3ID = 3;
    public static final int kSensor4ID = 4;
    public static final int kScorerSensorID = 1;
  }
}
