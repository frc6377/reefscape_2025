package frc.robot;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Inch;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import java.util.HashMap;
import org.littletonrobotics.junction.Logger;
import utilities.HowdyMM;
import utilities.HowdyPID;

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
  // Robot Mode
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = Robot.isReal() ? Mode.REAL : simMode;

  public static enum Mode {
    REAL,
    SIM,
    REPLAY
  }

  public static final Alliance kAllianceColor =
      DriverStation.getAlliance().isPresent() ? DriverStation.getAlliance().get() : Alliance.Red;

  public static final String CANivoreName = "Default Name";
  public static final String RIOName = "rio";

  public static final int kStreamDeckTotalButtonCount = 64;
  public static final int kMaxControllerButtonCount = 32;
  public static final String[] kPoleLetters =
      new String[] {"A", "B", "C", "D", "E", "F", "G", "H", "I", "J", "K", "L"};

  public static class CANIDs {
    // Rev Can Bus
    // 1-8 Motor ID is reserved by the drivebase
    public static final int kElevatorMotor1 = 10;
    public static final int kElevatorMotor2 = 11;
    public static final int kPivotMotor = 12;
    public static final int kIntakeMotor = 13;
    public static final int kConveyorMotor = 14;
    public static final int kScorerMotor = 15;
    public static final int kClimberMotorBack = 16;
    public static final int kClimberMotorFront = 17;
    public static final int kAlgeaMotor = 18;
    public static final int kCANdle = 19;
  }

  public static class DIOConstants {
    public static final int elvLimitID = 0;
    public static final int kIntakePivotEncoderID = 1;
    public static final int kAlgeaEncoderID = 2;
    public static final int kClimberFrontEncoderID = 3;
    public static final int kClimberBackEncoderID = 4;
    public static final int kGearID1 = 13;
    public static final int kGearID2 = 14;
  }

  public static class PWMIDs {
    public static final int kFrontClimberServoID = 1;
    public static final int kBackClimberServoID = 0;
  }

  public static class SensorIDs {
    public static final int kScorerSensorID = 1;
    public static final int kSensor2ID = 2;
    public static final int kSensor3ID = 3;
    public static final int kSensor4ID = 4;
    public static final int kAlignmentSensorID = 5;
  }

  public static class ClimberConstants {
    // PIDs
    public static final HowdyPID kClimberFrontPID0 = new HowdyPID();
    public static final HowdyPID kClimberBackPID0 = new HowdyPID();
    public static final HowdyPID kClimberFrontPID1 = new HowdyPID();
    public static final HowdyPID kClimberBackPID1 = new HowdyPID();

    static {
      kClimberFrontPID0.setKP(100);
      kClimberFrontPID0.setKD(0);
      kClimberFrontPID0.setKV(0);

      kClimberBackPID0.setKP(50);
      kClimberBackPID0.setKD(0);
      kClimberBackPID0.setKV(0);

      kClimberFrontPID1.setKP(100);
      kClimberFrontPID1.setKD(5);
      kClimberFrontPID1.setKV(15.12);
      kClimberFrontPID1.setKG(.62);
      kClimberFrontPID1.setGravityType(GravityTypeValue.Arm_Cosine);

      kClimberBackPID1.setKP(75);
      kClimberBackPID1.setKD(5);
      kClimberBackPID1.setKV(15.12);
      kClimberBackPID1.setKG(.62);
      kClimberBackPID1.setGravityType(GravityTypeValue.Arm_Cosine);
    }

    // Mech Constants
    public static final InvertedValue kClimberFrontInvert = InvertedValue.CounterClockwise_Positive;
    public static final InvertedValue kClimberBackInvert = InvertedValue.Clockwise_Positive;
    public static final Current kClimberIdleCurrentLimit = Amps.of(5);
    public static final Current kClimberClimbingCurrentLimit = Amps.of(70);
    public static final double kGearRatio = 126;

    // Motor Setpoints
    // 120 Degrees for climb
    public static final Angle kClimberFrontOffsetAngle = Degrees.of(-106);
    public static final Angle kClimberBackOffsetAngle = Degrees.of(30);
    public static final Angle kClimberOffsetAngle = Degrees.of(180);
    public static final Angle kClimberExtendedSetpoint = Degrees.of(-65).plus(kClimberOffsetAngle);
    public static final Angle kClimberAtCageSetpoint = Degrees.of(-10).plus(kClimberOffsetAngle);
    public static final Angle kClimberRetractedSetpoint = Degrees.of(90).plus(kClimberOffsetAngle);
    public static final Angle kClimberSensorTolerance = Degrees.of(2);
    public static final Angle kClimberDisengageAngle = Degrees.of(-45).plus(kClimberOffsetAngle);
    public static final Angle kClimberDisengageOffset = Degrees.of(5);

    // Servo Setpoints
    public static final Angle kFrontServoEngageAngle = Degrees.of(45);
    public static final Angle kBackServoEngageAngle = Degrees.of(45);
    public static final Angle kFrontServoDisengageAngle = Degree.of(0);
    public static final Angle kBackServoDisengageAngle = Degree.of(90);

    // Sim Constants
    public static final int KClimberMotorsCount = 2;
    public static final Distance kClimberArmLength = Inches.of(6);
    public static final Mass kClimberMass = Pounds.of(0.5);
    public static final Angle kClimberArmMinAngle = Degrees.of(0);
    public static final Angle kClimberArmMaxAngle = Degrees.of(250).plus(kClimberOffsetAngle);
  }

  // Scorer Constants
  public static class CoralScorerConstants {
    public static final double kHandoffSpeed = -0.5;
    public static final double kScoreSpeed = -0.3;
    public static final double kScoreL4Speed = -0.1;
    public static final double kScoreAutoSpeed = -0.1;
    public static final double kReverseSpeed = 0.25;
    public static final double kAlignSpeed = -0.1;

    public static final Distance kSensorDistnace = Inches.of(1.5);

    public static final Distance kAlignSensorDistnace = Inches.of(5.5);
    public static final Time kAlignSensorDebounce = Seconds.of(0);
  }

  // Signaling Constants
  public static class SignalingConstants {
    public static final int kNumLEDs = 200;
    public static final double kLEDBrightness = 0.2;
    public static final double kPatternSpeed = 0.001;
  }

  // Intake Constants
  public static class IntakeConstants {
    public static final boolean kEnableStateMachineSim = false;

    public static final double kIntakeSpeed = -1;
    public static final double kOuttakeSpeed = 0.1;
    public static final double kIntakeHandoffSpeed = -0.75;
    public static final double kConveyorSpeed = 0.45;
    public static final double kPivotSpeed = 0.2;
    public static final double kHoldSpeed = -0.2;

    // Pivot Arm Setpoints
    public static final Angle kPivotZero = Degrees.of(304);
    public static final Angle kPivotRetractAngle = Degrees.of(137);
    public static final Angle kPivotOuttakeAngle = Degrees.of(87);
    public static final Angle kPivotExtendAngle = Degrees.of(10);
    public static final Angle kPivotCoralStationAngle = Degrees.of(110);
    public static final Angle kPivotL1StowedAngle = Degrees.of(120);
    public static final Angle kPivotL1Score = Degrees.of(85);
    public static final Angle kPivotAlgaeIntakeAngle = Degrees.of(55);
    public static final Angle kPivotClimbingAngle = Degrees.of(75.5);
    public static final Angle kPivotEndClimbAngle = Degrees.of(130);
    public static final Angle kPivotTolerance = Degrees.of(5);

    public static final HowdyPID kPivotArmPID = new HowdyPID();

    static {
      kPivotArmPID.setKP(100);
      kPivotArmPID.setKV(7.29);
      kPivotArmPID.setKA(0.03);
      kPivotArmPID.setGravityType(GravityTypeValue.Arm_Cosine);
    }

    public static final HowdyMM kPivotArmMM =
        new HowdyMM(DegreesPerSecond.of(450), DegreesPerSecondPerSecond.of(2250), 80.0);

    public static final Current kHoldPower = Amps.of(40);
    public static final double kGearing = 60;
    public static final double kSensorToMechanism = 60;

    public static enum CoralEnum {
      CORAL_TOO_CLOSE,
      CORAL_TOO_FAR,
      NO_CORAL,
      IN_ELEVATOR,
      CORAL_ALIGNED,
      OTHER
    }

    // Simulation
    public static final Distance kPivotLength = Feet.of(1);
    public static final Mass kPivotMass = Pounds.of(8);
    public static final MomentOfInertia kPivotMOI =
        KilogramSquareMeters.of(
            SingleJointedArmSim.estimateMOI(kPivotLength.in(Meters), kPivotMass.in(Kilograms)));

    // For maplesim Intake
    public static final Distance kIntakeWidth = Meters.of(0.470);
    public static final Distance kIntakeExtension = Meters.of(0.191);
    public static final int kIntakeCapacity = 1;
  }

  // Elevator Constants
  public static class ElevatorConstants {
    public static final Distance kL0Height = Inches.of(0);
    public static final Distance kL2Height = Inches.of(18);
    public static final Distance kL3Height = Inches.of(30.9);
    public static final Distance kL4Height = Inches.of(54);

    public static final HowdyPID kElevatorPID = new HowdyPID();

    static {
      kElevatorPID.setKP(2);
      kElevatorPID.setKI(0.08);
      kElevatorPID.setKD(0.02);
      kElevatorPID.setKS(0.5);
      kElevatorPID.setStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);
    }

    public static final HowdyMM kElevatorMM =
        new HowdyMM(RotationsPerSecond.of(200), RotationsPerSecondPerSecond.of(250));

    public static final Distance kSetpointTolerance = Inches.of(1.75);

    // Mech Constants
    public static final Distance kElevatorDrumRadius = Inches.of(0.375);
    public static final Distance kElevatorDrumCircumference =
        kElevatorDrumRadius.times(2 * Math.PI);
    public static final double kElvRawOutput = .10;
    public static final double kElevatorGearing = 3;
    public static final int kGearToothing1 = 3;
    public static final int kGearToothing2 = 11;

    public static final Distance kBottomLimit = Inches.of(0);
    public static final Distance kTopLimit = Inches.of(75);

    // Gear for CRT offsets
    // TODO: get actual offset values
    public static final double kGearOffset1 = 0.0;
    public static final double kGearOffset2 = 0.0;

    // CRTA - Chinese Remainder Theorem Array
    public static int[][] CRTA = {
      {0, 12, 24, 3, 15, 27, 6, 18, 30, 9, 21},
      {22, 1, 13, 25, 4, 16, 28, 7, 19, 31, 10},
      {11, 23, 2, 14, 26, 5, 17, 29, 8, 20, 32}
    };

    // Simulation Constants
    public static final DCMotor kElevatorGearbox = DCMotor.getKrakenX60(2);
    public static final Mass kCarriageMass = Pounds.of(4.75);
    public static final Distance kMinElevatorHeight = Inches.zero();
    public static final Distance kMaxElevatorHeight = Inches.of(72);
  }

  // Algea Remover Constants
  public static class AlgeaRemoverConstants {
    // PID
    public static final double kAlgeaP = 0.1;
    public static final double kAlgeaI = 0.0;
    public static final double kAlgeaD = 0.0;

    // Mech Constants
    public static final double kAlgeaPercent = 0.1;
    public static final int kAlegeaGearRatio = 16;
    public static final Angle ksetpointTolerance = Degrees.of(10);

    // Algea Arm Setpoints (// 35 Degrees is breaking point)
    public static final Angle kEncoderOffset = Rotations.of(0);
    public static final Angle kAlgeaStowed = Degrees.of(200);
    public static final Angle kRemoveUpAngle = Degrees.of(100);
    public static final Angle kRemoveDownAngle = Degrees.of(50);

    // Simulation Constants
    public static final DCMotor kAlgeaGearbox = DCMotor.getNEO(1);
    public static final Distance kAlgeaArmLength = Inches.of(19);
    public static final Angle kAlgeaStartingAngle = Rotations.of(-0.25);
  }

  @SuppressWarnings("unused")
  public final class DrivetrainConstants {
    public static final Mass kRobotMass = Pounds.of(106.6);
    public static final Distance kBumperSize = Meters.of(0.889);

    // POV Drive Constants
    public static final LinearVelocity kPOVDriveSpeed = MetersPerSecond.of(0.25);

    // Strafe Constants
    public static final Time kStrafeTime = Seconds.of(0.5);
    public static final LinearVelocity kStrafeSpeed = MetersPerSecond.of(0.5);

    // Constants from DriveCommands
    public static final double kWheelRadiusMaxVelocity = 0.25; // Rad/Sec
    public static final double kWheelRadiusRampRate = 0.05; // Rad/Sec^2

    // Path Finder/Planner Stuff
    public static final PathConstraints kPathConstraints =
        new PathConstraints(
            1,
            2,
            DegreesPerSecond.of(300).in(RadiansPerSecond),
            DegreesPerSecond.of(1200).in(RadiansPerSecond));
    public static final PPHolonomicDriveController KPPDriveController =
        new PPHolonomicDriveController(
            new PIDConstants(5.0, 0.0, 0.0), new PIDConstants(5.0, 0.0, 0.0));

    // For Drive At Angle Command
    public static final ProfiledPIDController kRotationController =
        new ProfiledPIDController(5, 0, 0.01, new TrapezoidProfile.Constraints(8, 20));

    public static final Pose3d[] kMechPoses =
        new Pose3d[] {
          // Intake
          new Pose3d(
              Meters.of(0.195051), Meters.of(0.091696), Meters.of(0.241039), new Rotation3d()),
          // Elevator
          new Pose3d(
              Meters.of(-0.0635), Meters.of(-0.236448), Meters.of(0.117475), new Rotation3d()),
          // Coral Scorer
          new Pose3d(
              Meters.of(-0.058686), Meters.of(-0.236449), Meters.of(0.201601), new Rotation3d()),
          // Climber Front
          new Pose3d(
              Meters.of(0.160211), Meters.of(0.010763), Meters.of(0.133263), new Rotation3d()),
          // Climber Back
          new Pose3d(
              Meters.of(-0.160211), Meters.of(0.010763), Meters.of(0.133263), new Rotation3d()),
          // Algae Remover
          new Pose3d(
              Meters.of(-0.116593), Meters.of(-0.329488), Meters.of(0.9525), new Rotation3d()),
        };

    public static final Pose3d kRobotCoralPose =
        new Pose3d(
            Meters.of(0.037449),
            Meters.of(-0.251632),
            Meters.of(0.547541),
            new Rotation3d(Degrees.of(0), Degrees.of(-45), Degrees.of(90)));

    static {
      Logger.recordOutput("Odometry/Mech Poses List", kMechPoses);
    }
  }

  public final class ReefAlignConstants {
    // Target Poses
    public static final Pose2d kLeftReefPose =
        new Pose2d(Inches.of(18.5), Inches.of(-4), new Rotation2d(Degrees.of(-90)));
    public static final Pose2d kRightReefPose =
        new Pose2d(Inches.of(18.5), Inches.of(9), new Rotation2d(Degrees.of(-90)));

    // PID Controllers
    public static final PIDController kTranslationXController = new PIDController(3, 0, 0.1);
    public static final PIDController kTranslationYController = new PIDController(3, 0, 0.1);
    public static final PIDController kRotationController = new PIDController(0.2, 0, 0);
    public static final Angle kSetpointRotTolerance = Degrees.of(1);
    public static final Distance kSetpointTolerance = Inches.of(1);
    public static final Time kAtPoseDebounce = Seconds.of(0);
  }

  public final class FeildConstants {
    public static final Distance kFieldWidth = Inches.of(317);
    public static final Distance kFieldLength = Inches.of(690 + (7 / 8));
  }

  public final class SimulationConstants {
    public static final Distance kScoreDistance = Inch.of(13);

    public static final Pose2d[][] kSourceAreas =
        new Pose2d[][] {
          new Pose2d[] {
            new Pose2d(Feet.of(0), Feet.of(0), new Rotation2d()),
            new Pose2d(Feet.of(6), Feet.of(5), new Rotation2d())
          },
          new Pose2d[] {
            new Pose2d(Feet.of(0), Feet.of(21.5), new Rotation2d()),
            new Pose2d(Feet.of(6), Feet.of(26.5), new Rotation2d())
          },
          new Pose2d[] {
            new Pose2d(Feet.of(51.5), Feet.of(0), new Rotation2d()),
            new Pose2d(Feet.of(57.5), Feet.of(5), new Rotation2d())
          },
          new Pose2d[] {
            new Pose2d(Feet.of(51.5), Feet.of(21.5), new Rotation2d()),
            new Pose2d(Feet.of(57.5), Feet.of(26.5), new Rotation2d())
          },
        };

    // Heights
    public static final HashMap<String, Pose2d> kCoralHeightMap = new HashMap<String, Pose2d>(3);

    static {
      kCoralHeightMap.put(
          "L2", new Pose2d(Inches.of(31.1843), Inches.zero(), new Rotation2d(Degrees.of(35))));
      kCoralHeightMap.put(
          "L3", new Pose2d(Inches.of(47.0542), Inches.zero(), new Rotation2d(Degrees.of(35))));
      kCoralHeightMap.put(
          "L4", new Pose2d(Inches.of(71.9348), Inches.zero(), new Rotation2d(Degrees.of(90))));
    }

    public static final Pose2d[] kBlueStickPoses =
        new Pose2d[] {
          new Pose2d(Inches.of(146.303), Inches.of(152.030), new Rotation2d(Degrees.of(0))),
          new Pose2d(Inches.of(146.348), Inches.of(164.968), new Rotation2d(Degrees.of(0))),
          new Pose2d(Inches.of(156.038), Inches.of(135.246), new Rotation2d(Degrees.of(60))),
          new Pose2d(Inches.of(156.016), Inches.of(181.791), new Rotation2d(Degrees.of(-60))),
          new Pose2d(Inches.of(167.220), Inches.of(128.738), new Rotation2d(Degrees.of(60))),
          new Pose2d(Inches.of(167.243), Inches.of(188.221), new Rotation2d(Degrees.of(-60))),
          new Pose2d(Inches.of(186.624), Inches.of(128.777), new Rotation2d(Degrees.of(120))),
          new Pose2d(Inches.of(186.646), Inches.of(188.260), new Rotation2d(Degrees.of(-120))),
          new Pose2d(Inches.of(197.850), Inches.of(135.207), new Rotation2d(Degrees.of(120))),
          new Pose2d(Inches.of(197.828), Inches.of(181.752), new Rotation2d(Degrees.of(-120))),
          new Pose2d(Inches.of(207.518), Inches.of(152.030), new Rotation2d(Degrees.of(180))),
          new Pose2d(Inches.of(207.563), Inches.of(164.968), new Rotation2d(Degrees.of(180))),
        };
    public static final Pose2d[] kRedStickPoses =
        new Pose2d[] {
          new Pose2d(Inches.of(483.688), Inches.of(152.030), new Rotation2d(Degrees.of(0))),
          new Pose2d(Inches.of(483.733), Inches.of(164.968), new Rotation2d(Degrees.of(0))),
          new Pose2d(Inches.of(493.423), Inches.of(135.246), new Rotation2d(Degrees.of(60))),
          new Pose2d(Inches.of(493.401), Inches.of(181.791), new Rotation2d(Degrees.of(-60))),
          new Pose2d(Inches.of(504.605), Inches.of(128.738), new Rotation2d(Degrees.of(60))),
          new Pose2d(Inches.of(504.627), Inches.of(188.221), new Rotation2d(Degrees.of(-60))),
          new Pose2d(Inches.of(524.008), Inches.of(128.777), new Rotation2d(Degrees.of(120))),
          new Pose2d(Inches.of(524.031), Inches.of(188.260), new Rotation2d(Degrees.of(-120))),
          new Pose2d(Inches.of(535.235), Inches.of(135.207), new Rotation2d(Degrees.of(120))),
          new Pose2d(Inches.of(535.213), Inches.of(181.752), new Rotation2d(Degrees.of(-120))),
          new Pose2d(Inches.of(544.903), Inches.of(152.030), new Rotation2d(Degrees.of(180))),
          new Pose2d(Inches.of(544.948), Inches.of(164.968), new Rotation2d(Degrees.of(180))),
        };
  }
}
