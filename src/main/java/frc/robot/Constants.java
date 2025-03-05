package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import java.util.HashMap;
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
    public static final int kClimberMotorFront = 17; // FIXME
    public static final int kClimberMotorBack = 16; // FIXME
    public static final int kScorerMotor = 15;
    public static final int kElevatorMotor1 = 10;
    public static final int kElevatorMotor2 = 11;
    public static final int kIntakeMotor = 13;
    public static final int kPivotMotor = 12;
    public static final int kConveyorMotor = 14;
    public static final int kAlgeaMotor = 18;
  }

  public static class DIOConstants {
    public static final int elvLimitID = 0;
    public static final int kthroughBoreEncoderID = 1;
    public static final int kAlgeaEncoderID = 2;
    public static final int kClimberFrontEncoderID = 3;
    public static final int kClimberBackEncoderID = 4;
    public static final int gear11ID = 13;
    public static final int gear3ID = 14;
  }

  public static class PWMIDs {
    public static final int kFrontClimberServoID = 1;
    public static final int kBackClimberServoID = 0;
  }

  public static class SensorIDs {
    public static final int kSensor2ID = 2;
    public static final int kSensor3ID = 3;
    public static final int kSensor4ID = 4;
    public static final int kScorerSensorID = 1;
  }

  public static class ClimberConstants {
    public static final HowdyPID kClimberPID0 = new HowdyPID();

    static {
      kClimberPID0.setKP(100);
      kClimberPID0.setKD(5);
      kClimberPID0.setKV(10);
    }

    public static final HowdyPID kClimberPID1 = new HowdyPID();

    static {
      kClimberPID1.setKP(200);
      kClimberPID1.setKD(5);
      kClimberPID1.setKV(10);
      kClimberPID1.setKG(1);
    }

    public static final Current kClimberIdleCurrentLimit = Amps.of(20);
    public static final Current kClimberClimbingCurrentLimit = Amps.of(70);
    public static final double kGearRatio = 126;
    public static final Angle kClimberFrontOffsetAngle = Degrees.of(-333.3);
    public static final Angle kClimberBackOffsetAngle = Degrees.of(148.1);
    public static final Angle kClimberOffsetAngle = Degrees.of(180);
    public static final Angle kClimberExtendedSetpoint = Degrees.of(-50).plus(kClimberOffsetAngle);
    public static final Angle kClimberAtCageSetpoint = Degrees.of(-10).plus(kClimberOffsetAngle);
    public static final Angle kClimberRetractedSetpoint = Degrees.of(90).plus(kClimberOffsetAngle);
    public static final Angle kClimberSensorTolerance = Degrees.of(4);
    public static final Angle kClimberServoDisengageAngle =
        Degrees.of(-45).plus(kClimberOffsetAngle);
    public static final InvertedValue kClimberFrontInvert = InvertedValue.CounterClockwise_Positive;
    public static final InvertedValue kClimberBackInvert = InvertedValue.Clockwise_Positive;

    // Sim Constants
    public static final int KClimberMotorsCount = 2;
    public static final Distance kClimberArmLength = Inches.of(6);
    public static final Mass kClimberMass = Pounds.of(0.5);
    public static final Angle kClimberArmMinAngle = Degrees.of(-30).plus(kClimberOffsetAngle);
    public static final Angle kClimberArmMaxAngle = Degrees.of(250).plus(kClimberOffsetAngle);

    public static final Angle kFrontServoEngageAngle = Degrees.of(45);
    public static final Angle kBackServoEngageAngle = Degrees.of(90);
    public static final Angle kFrontServoDisengageAngle = Degree.of(0);
    public static final Angle kBackServoDisengageAngle = Degree.of(45);
  }

  // Scorer Constants
  public static class CoralScorerConstants {
    public static final double kIntakeSpeed = -0.5;
    public static final double kScoreSpeed = -0.35;

    public static final Current kScoreAMPs = Amps.of(-20);
    public static final Current kIntakeAMPs = Amps.of(-20);

    public static final HowdyPID CoralScorerPID = new HowdyPID();

    static {
      CoralScorerPID.setKP(0.1);
    }
  }

  // Intake Constants
  public static class IntakeConstants {
    public static final boolean kEnableStateMachineSim = false;

    public static final double kIntakeSpeed = -1;
    public static final double kOuttakeSpeed = 0.2;
    public static final double kIntakeHandoffSpeed = -0.75;
    public static final double kConveyorSpeed = 0.45;
    public static final double kPivotSpeed = 0.2;
    public static final double kHoldSpeed = kIntakeSpeed / 5;

    // Pivot Arm Setpoints
    public static final Angle armZero = Degrees.of(76.05);
    public static final Angle kPivotRetractAngle = Degrees.of(128);
    public static final Angle kPivotOuttakeAngle = Degrees.of(87);
    public static final Angle kPivotExtendAngle = Degrees.of(-6.25);
    public static final Angle kPivotCoralStationAngle = Degrees.of(105);
    public static final Angle kPivotL1Score = Degrees.of(85);
    public static final Angle kPivotAlgaeIntakeAngle = Degrees.of(55);
    public static final Angle kClimbingAngle = Degrees.of(75.5);

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
    public static final Distance kLength = Feet.of(1);
    public static final Mass kMass = Pounds.of(8);
    // TODO: We can estimate MOI after doing SysID
    public static final MomentOfInertia kMOI =
        KilogramSquareMeters.of(
            SingleJointedArmSim.estimateMOI(kLength.in(Meters), kMass.in(Kilograms)));

    // For maplesim Intake
    public static final Distance kIntakeWidth = Meters.of(0.470);
    public static final Distance kIntakeExtension = Meters.of(0.191);
    public static final int kIntakeCapacity = 1;
  }

  // Elevator Constants
  public static class ElevatorConstants {
    public static final Distance kL0Height = Inches.of(0);
    public static final Distance kL2Height = Inches.of(16.62);
    public static final Distance kL3Height = Inches.of(30.9);
    public static final Distance kL4Height = Inches.of(55);

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

    public static final Distance kSetpointTolerance = Inches.of(1.25);

    // Mech Constants
    public static final Distance kElevatorDrumRadius = Inches.of(0.375);
    public static final Distance kElevatorDrumCircumference =
        kElevatorDrumRadius.times(2 * Math.PI);

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

    // Simulation Constants
    public static final DCMotor kElevatorGearbox = DCMotor.getKrakenX60(2);
    public static final double elevatorOutput = .10;
    public static final double kElevatorGearing = 3;
    public static final Mass kCarriageMass = Pounds.of(4.75);
    public static final Distance kMinElevatorHeight = Inches.zero();
    public static final Distance kMaxElevatorHeight = Inches.of(72);
  }

  // Algea Remover Constants
  public static class AlgeaRemoverConstants {
    public static final double kAlgeaP = 0.7;
    public static final double kAlgeaI = 0.0;
    public static final double kAlgeaD = 0.0;
    public static final double kAlgeaPercent = .4;
    public static final int kAlegeaGearRatio = 80;
    public static final Angle algeaStowed = Rotations.of(0.46);
    public static final Angle algeaRemove = Rotations.of(0.1);
    public static final double algeaZero = 0.0; // update with actual value
    public static final DCMotor kAlgeaGearbox = DCMotor.getNEO(1);
    public static final Distance algeaArmLength = Inches.of(19);
    public static final Angle kAlgeaStartingAngle = Rotations.of(-.25);
    public static final Angle encoderOffset = Rotations.of(0); // update with actual offset
    public static final Angle ksetpointTolerance = Degrees.of(10);
  }

  public final class DrivetrainConstants {
    // PathPlanner config constants
    public static final Mass ROBOT_MASS = Pounds.of(96.2);
    public static final MomentOfInertia ROBOT_MOI = KilogramSquareMeters.of(49.5459894327);
    public static final double WHEEL_COF = 1.2;

    public static final PathConstraints PATH_CONSTRAINTS =
        new PathConstraints(
            4.3,
            4.3,
            DegreesPerSecond.of(630).in(RadiansPerSecond),
            DegreesPerSecond.of(630).in(RadiansPerSecond));

    // Constants from DriveCommands
    public static final double ANGLE_KP = 5;
    public static final double ANGLE_KD = 0.01;
    public static final double ANGLE_MAX_VELOCITY = 8.0;
    public static final double ANGLE_MAX_ACCELERATION = 20.0;
    public static final double FF_START_DELAY = 2.0; // Secs
    public static final double FF_RAMP_RATE = 0.1; // Volts/Sec
    public static final double WHEEL_RADIUS_MAX_VELOCITY = 1; // Rad/Sec
    public static final double WHEEL_RADIUS_RAMP_RATE = 0.05; // Rad/Sec^2

    // Scoring Poses for PathFinder
    public static final HashMap<String, Pose2d> SCORE_POSES =
        new HashMap<String, Pose2d>(kPoleLetters.length);

    static {
      SCORE_POSES.put(
          kPoleLetters[0],
          new Pose2d(Meters.of(3.183), Meters.of(4.157), new Rotation2d(Degrees.of(90))));
      SCORE_POSES.put(
          kPoleLetters[1],
          new Pose2d(Meters.of(3.184), Meters.of(3.828), new Rotation2d(Degrees.of(90))));
      SCORE_POSES.put(
          kPoleLetters[2],
          new Pose2d(Meters.of(3.726), Meters.of(2.960), new Rotation2d(Degrees.of(150))));
      SCORE_POSES.put(
          kPoleLetters[3],
          new Pose2d(Meters.of(4.004), Meters.of(2.797), new Rotation2d(Degrees.of(150))));
      SCORE_POSES.put(
          kPoleLetters[4],
          new Pose2d(Meters.of(5.029), Meters.of(2.834), new Rotation2d(Degrees.of(-150))));
      SCORE_POSES.put(
          kPoleLetters[5],
          new Pose2d(Meters.of(5.311), Meters.of(2.993), new Rotation2d(Degrees.of(-150))));
      SCORE_POSES.put(
          kPoleLetters[6],
          new Pose2d(Meters.of(5.791), Meters.of(3.898), new Rotation2d(Degrees.of(-90))));
      SCORE_POSES.put(
          kPoleLetters[7],
          new Pose2d(Meters.of(5.790), Meters.of(4.219), new Rotation2d(Degrees.of(-90))));
      SCORE_POSES.put(
          kPoleLetters[8],
          new Pose2d(Meters.of(5.250), Meters.of(5.093), new Rotation2d(Degrees.of(-30))));
      SCORE_POSES.put(
          kPoleLetters[9],
          new Pose2d(Meters.of(4.964), Meters.of(5.259), new Rotation2d(Degrees.of(-30))));
      SCORE_POSES.put(
          kPoleLetters[10],
          new Pose2d(Meters.of(3.942), Meters.of(5.215), new Rotation2d(Degrees.of(30))));
      SCORE_POSES.put(
          kPoleLetters[11],
          new Pose2d(Meters.of(3.667), Meters.of(5.062), new Rotation2d(Degrees.of(30))));
    }

    public static final Pose2d[] SOURSE_POSES =
        new Pose2d[] {
          new Pose2d(Meters.of(1.227), Meters.of(7.045), new Rotation2d(Degrees.of(-54))),
          new Pose2d(Meters.of(1.256), Meters.of(0.955), new Rotation2d(Degrees.of(54))),
        };

    public static final Pose3d kIntakeStartPose =
        new Pose3d(
            Meters.of(0.191591),
            Meters.of(0.091696),
            Meters.of(0.242354),
            new Rotation3d(Degrees.of(0), Degrees.of(-90), Degrees.of(0)));

    public static final Pose3d kElvStage1Pose =
        new Pose3d(Meters.of(-0.0635), Meters.of(-0.236449), Meters.of(0.1016), new Rotation3d());
    public static final Pose3d kElvStage2Pose =
        new Pose3d(
            Meters.of(-0.063479), Meters.of(-0.236448), Meters.of(0.22065), new Rotation3d());

    public static final Pose3d kClimber1Pose =
        new Pose3d(
            Meters.of(0.16021),
            Meters.of(0.004064),
            Meters.of(0.133263),
            new Rotation3d(Degrees.of(79), Degrees.of(0), Degrees.of(-90)));
    public static final Pose3d kClimber2Pose =
        new Pose3d(
            Meters.of(-0.160211),
            Meters.of(0.004064),
            Meters.of(0.133263),
            new Rotation3d(Degrees.of(-100), Degrees.of(0), Degrees.of(-90)));

    public static final Pose3d kCoralScorerPose =
        new Pose3d(
            Meters.of(0.037449),
            Meters.of(-0.251632),
            Meters.of(0.547541),
            new Rotation3d(Degrees.of(0), Degrees.of(-45), Degrees.of(90)));
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
    private static final Distance kL2H = Inches.of(31.1843);
    private static final Distance kL3H = Inches.of(47.0542);
    private static final Distance kL4H = Inches.of(71.9348);

    // X, Y Cordiantes
    private static final Distance[][] kBlueStickPoses =
        new Distance[][] {
          new Distance[] {Inches.of(146.303), Inches.of(152.030)},
          new Distance[] {Inches.of(146.348), Inches.of(164.968)},
          new Distance[] {Inches.of(156.038), Inches.of(135.246)},
          new Distance[] {Inches.of(156.016), Inches.of(181.791)},
          new Distance[] {Inches.of(167.220), Inches.of(128.738)},
          new Distance[] {Inches.of(167.243), Inches.of(188.221)},
          new Distance[] {Inches.of(186.624), Inches.of(128.777)},
          new Distance[] {Inches.of(186.646), Inches.of(188.260)},
          new Distance[] {Inches.of(197.850), Inches.of(135.207)},
          new Distance[] {Inches.of(197.828), Inches.of(181.752)},
          new Distance[] {Inches.of(207.518), Inches.of(152.030)},
          new Distance[] {Inches.of(207.563), Inches.of(164.968)},
        };
    private static final Distance[][] kRedStickPoses =
        new Distance[][] {
          new Distance[] {Inches.of(483.688), Inches.of(152.030)},
          new Distance[] {Inches.of(483.733), Inches.of(164.968)},
          new Distance[] {Inches.of(493.423), Inches.of(135.246)},
          new Distance[] {Inches.of(493.401), Inches.of(181.791)},
          new Distance[] {Inches.of(504.605), Inches.of(128.738)},
          new Distance[] {Inches.of(504.627), Inches.of(188.221)},
          new Distance[] {Inches.of(524.008), Inches.of(128.777)},
          new Distance[] {Inches.of(524.031), Inches.of(188.260)},
          new Distance[] {Inches.of(535.235), Inches.of(135.207)},
          new Distance[] {Inches.of(535.213), Inches.of(181.752)},
          new Distance[] {Inches.of(544.903), Inches.of(152.030)},
          new Distance[] {Inches.of(544.948), Inches.of(164.968)},
        };

    public static final Pose3d[] kBlueCoralScorePoses = {
      // Level 2 (L2 - Lower Branches)
      new Pose3d(
          kBlueStickPoses[0][0],
          kBlueStickPoses[0][1],
          kL2H,
          new Rotation3d(Degrees.zero(), Degrees.of(35), Degrees.zero())),
      new Pose3d(
          kBlueStickPoses[1][0],
          kBlueStickPoses[1][1],
          kL2H,
          new Rotation3d(Degrees.zero(), Degrees.of(35), Degrees.zero())),
      new Pose3d(
          kBlueStickPoses[2][0],
          kBlueStickPoses[2][1],
          kL2H,
          new Rotation3d(Degrees.zero(), Degrees.of(35), Degrees.of(60))),
      new Pose3d(
          kBlueStickPoses[3][0],
          kBlueStickPoses[3][1],
          kL2H,
          new Rotation3d(Degrees.zero(), Degrees.of(35), Degrees.of(-60))),
      new Pose3d(
          kBlueStickPoses[4][0],
          kBlueStickPoses[4][1],
          kL2H,
          new Rotation3d(Degrees.zero(), Degrees.of(35), Degrees.of(60))),
      new Pose3d(
          kBlueStickPoses[5][0],
          kBlueStickPoses[5][1],
          kL2H,
          new Rotation3d(Degrees.zero(), Degrees.of(35), Degrees.of(-60))),
      new Pose3d(
          kBlueStickPoses[6][0],
          kBlueStickPoses[6][1],
          kL2H,
          new Rotation3d(Degrees.zero(), Degrees.of(-35), Degrees.of(-60))),
      new Pose3d(
          kBlueStickPoses[7][0],
          kBlueStickPoses[7][1],
          kL2H,
          new Rotation3d(Degrees.zero(), Degrees.of(-35), Degrees.of(60))),
      new Pose3d(
          kBlueStickPoses[8][0],
          kBlueStickPoses[8][1],
          kL2H,
          new Rotation3d(Degrees.zero(), Degrees.of(-35), Degrees.of(-60))),
      new Pose3d(
          kBlueStickPoses[9][0],
          kBlueStickPoses[9][1],
          kL2H,
          new Rotation3d(Degrees.zero(), Degrees.of(-35), Degrees.of(60))),
      new Pose3d(
          kBlueStickPoses[10][0],
          kBlueStickPoses[10][1],
          kL2H,
          new Rotation3d(Degrees.zero(), Degrees.of(35), Degrees.of(180))),
      new Pose3d(
          kBlueStickPoses[11][0],
          kBlueStickPoses[11][1],
          kL2H,
          new Rotation3d(Degrees.zero(), Degrees.of(35), Degrees.of(180))),

      // Level 3 (L3 - Middle Branches)
      new Pose3d(
          kBlueStickPoses[0][0],
          kBlueStickPoses[0][1],
          kL3H,
          new Rotation3d(Degrees.zero(), Degrees.of(35), Degrees.zero())),
      new Pose3d(
          kBlueStickPoses[1][0],
          kBlueStickPoses[1][1],
          kL3H,
          new Rotation3d(Degrees.zero(), Degrees.of(35), Degrees.zero())),
      new Pose3d(
          kBlueStickPoses[2][0],
          kBlueStickPoses[2][1],
          kL3H,
          new Rotation3d(Degrees.zero(), Degrees.of(35), Degrees.of(60))),
      new Pose3d(
          kBlueStickPoses[3][0],
          kBlueStickPoses[3][1],
          kL3H,
          new Rotation3d(Degrees.zero(), Degrees.of(35), Degrees.of(-60))),
      new Pose3d(
          kBlueStickPoses[4][0],
          kBlueStickPoses[4][1],
          kL3H,
          new Rotation3d(Degrees.zero(), Degrees.of(35), Degrees.of(60))),
      new Pose3d(
          kBlueStickPoses[5][0],
          kBlueStickPoses[5][1],
          kL3H,
          new Rotation3d(Degrees.zero(), Degrees.of(35), Degrees.of(-60))),
      new Pose3d(
          kBlueStickPoses[6][0],
          kBlueStickPoses[6][1],
          kL3H,
          new Rotation3d(Degrees.zero(), Degrees.of(-35), Degrees.of(-60))),
      new Pose3d(
          kBlueStickPoses[7][0],
          kBlueStickPoses[7][1],
          kL3H,
          new Rotation3d(Degrees.zero(), Degrees.of(-35), Degrees.of(60))),
      new Pose3d(
          kBlueStickPoses[8][0],
          kBlueStickPoses[8][1],
          kL3H,
          new Rotation3d(Degrees.zero(), Degrees.of(-35), Degrees.of(-60))),
      new Pose3d(
          kBlueStickPoses[9][0],
          kBlueStickPoses[9][1],
          kL3H,
          new Rotation3d(Degrees.zero(), Degrees.of(-35), Degrees.of(60))),
      new Pose3d(
          kBlueStickPoses[10][0],
          kBlueStickPoses[10][1],
          kL3H,
          new Rotation3d(Degrees.zero(), Degrees.of(35), Degrees.of(180))),
      new Pose3d(
          kBlueStickPoses[11][0],
          kBlueStickPoses[11][1],
          kL3H,
          new Rotation3d(Degrees.zero(), Degrees.of(35), Degrees.of(180))),

      // Level 4 (L4 - Highest Branches)
      new Pose3d(
          kBlueStickPoses[0][0],
          kBlueStickPoses[0][1],
          kL4H,
          new Rotation3d(Degrees.zero(), Degrees.of(90), Degrees.zero())),
      new Pose3d(
          kBlueStickPoses[1][0],
          kBlueStickPoses[1][1],
          kL4H,
          new Rotation3d(Degrees.zero(), Degrees.of(90), Degrees.zero())),
      new Pose3d(
          kBlueStickPoses[2][0],
          kBlueStickPoses[2][1],
          kL4H,
          new Rotation3d(Degrees.zero(), Degrees.of(90), Degrees.zero())),
      new Pose3d(
          kBlueStickPoses[3][0],
          kBlueStickPoses[3][1],
          kL4H,
          new Rotation3d(Degrees.zero(), Degrees.of(90), Degrees.zero())),
      new Pose3d(
          kBlueStickPoses[4][0],
          kBlueStickPoses[4][1],
          kL4H,
          new Rotation3d(Degrees.zero(), Degrees.of(90), Degrees.zero())),
      new Pose3d(
          kBlueStickPoses[5][0],
          kBlueStickPoses[5][1],
          kL4H,
          new Rotation3d(Degrees.zero(), Degrees.of(90), Degrees.zero())),
      new Pose3d(
          kBlueStickPoses[6][0],
          kBlueStickPoses[6][1],
          kL4H,
          new Rotation3d(Degrees.zero(), Degrees.of(90), Degrees.zero())),
      new Pose3d(
          kBlueStickPoses[7][0],
          kBlueStickPoses[7][1],
          kL4H,
          new Rotation3d(Degrees.zero(), Degrees.of(90), Degrees.zero())),
      new Pose3d(
          kBlueStickPoses[8][0],
          kBlueStickPoses[8][1],
          kL4H,
          new Rotation3d(Degrees.zero(), Degrees.of(90), Degrees.zero())),
      new Pose3d(
          kBlueStickPoses[9][0],
          kBlueStickPoses[9][1],
          kL4H,
          new Rotation3d(Degrees.zero(), Degrees.of(90), Degrees.zero())),
      new Pose3d(
          kBlueStickPoses[10][0],
          kBlueStickPoses[10][1],
          kL4H,
          new Rotation3d(Degrees.zero(), Degrees.of(90), Degrees.zero())),
      new Pose3d(
          kBlueStickPoses[11][0],
          kBlueStickPoses[11][1],
          kL4H,
          new Rotation3d(Degrees.zero(), Degrees.of(90), Degrees.zero())),
    };

    public static final Pose3d[] kRedCoralScorePoses = {
      // Level 2 (L2 - Lower Branches)
      new Pose3d(
          kRedStickPoses[0][0],
          kRedStickPoses[0][1],
          kL2H,
          new Rotation3d(Degrees.zero(), Degrees.of(35), Degrees.zero())),
      new Pose3d(
          kRedStickPoses[1][0],
          kRedStickPoses[1][1],
          kL2H,
          new Rotation3d(Degrees.zero(), Degrees.of(35), Degrees.zero())),
      new Pose3d(
          kRedStickPoses[2][0],
          kRedStickPoses[2][1],
          kL2H,
          new Rotation3d(Degrees.zero(), Degrees.of(35), Degrees.of(60))),
      new Pose3d(
          kRedStickPoses[3][0],
          kRedStickPoses[3][1],
          kL2H,
          new Rotation3d(Degrees.zero(), Degrees.of(35), Degrees.of(-60))),
      new Pose3d(
          kRedStickPoses[4][0],
          kRedStickPoses[4][1],
          kL2H,
          new Rotation3d(Degrees.zero(), Degrees.of(35), Degrees.of(60))),
      new Pose3d(
          kRedStickPoses[5][0],
          kRedStickPoses[5][1],
          kL2H,
          new Rotation3d(Degrees.zero(), Degrees.of(35), Degrees.of(-60))),
      new Pose3d(
          kRedStickPoses[6][0],
          kRedStickPoses[6][1],
          kL2H,
          new Rotation3d(Degrees.zero(), Degrees.of(-35), Degrees.of(-60))),
      new Pose3d(
          kRedStickPoses[7][0],
          kRedStickPoses[7][1],
          kL2H,
          new Rotation3d(Degrees.zero(), Degrees.of(-35), Degrees.of(60))),
      new Pose3d(
          kRedStickPoses[8][0],
          kRedStickPoses[8][1],
          kL2H,
          new Rotation3d(Degrees.zero(), Degrees.of(-35), Degrees.of(-60))),
      new Pose3d(
          kRedStickPoses[9][0],
          kRedStickPoses[9][1],
          kL2H,
          new Rotation3d(Degrees.zero(), Degrees.of(-35), Degrees.of(60))),
      new Pose3d(
          kRedStickPoses[10][0],
          kRedStickPoses[10][1],
          kL2H,
          new Rotation3d(Degrees.zero(), Degrees.of(-35), Degrees.zero())),
      new Pose3d(
          kRedStickPoses[11][0],
          kRedStickPoses[11][1],
          kL2H,
          new Rotation3d(Degrees.zero(), Degrees.of(-35), Degrees.zero())),

      // Level 3 (L3 - Middle Branches)
      new Pose3d(
          kRedStickPoses[0][0],
          kRedStickPoses[0][1],
          kL3H,
          new Rotation3d(Degrees.zero(), Degrees.of(35), Degrees.zero())),
      new Pose3d(
          kRedStickPoses[1][0],
          kRedStickPoses[1][1],
          kL3H,
          new Rotation3d(Degrees.zero(), Degrees.of(35), Degrees.zero())),
      new Pose3d(
          kRedStickPoses[2][0],
          kRedStickPoses[2][1],
          kL3H,
          new Rotation3d(Degrees.zero(), Degrees.of(35), Degrees.of(60))),
      new Pose3d(
          kRedStickPoses[3][0],
          kRedStickPoses[3][1],
          kL3H,
          new Rotation3d(Degrees.zero(), Degrees.of(35), Degrees.of(-60))),
      new Pose3d(
          kRedStickPoses[4][0],
          kRedStickPoses[4][1],
          kL3H,
          new Rotation3d(Degrees.zero(), Degrees.of(35), Degrees.of(60))),
      new Pose3d(
          kRedStickPoses[5][0],
          kRedStickPoses[5][1],
          kL3H,
          new Rotation3d(Degrees.zero(), Degrees.of(35), Degrees.of(-60))),
      new Pose3d(
          kRedStickPoses[6][0],
          kRedStickPoses[6][1],
          kL3H,
          new Rotation3d(Degrees.zero(), Degrees.of(-35), Degrees.of(-60))),
      new Pose3d(
          kRedStickPoses[7][0],
          kRedStickPoses[7][1],
          kL3H,
          new Rotation3d(Degrees.zero(), Degrees.of(-35), Degrees.of(60))),
      new Pose3d(
          kRedStickPoses[8][0],
          kRedStickPoses[8][1],
          kL3H,
          new Rotation3d(Degrees.zero(), Degrees.of(-35), Degrees.of(-60))),
      new Pose3d(
          kRedStickPoses[9][0],
          kRedStickPoses[9][1],
          kL3H,
          new Rotation3d(Degrees.zero(), Degrees.of(-35), Degrees.of(60))),
      new Pose3d(
          kRedStickPoses[10][0],
          kRedStickPoses[10][1],
          kL3H,
          new Rotation3d(Degrees.zero(), Degrees.of(-35), Degrees.zero())),
      new Pose3d(
          kRedStickPoses[11][0],
          kRedStickPoses[11][1],
          kL3H,
          new Rotation3d(Degrees.zero(), Degrees.of(-35), Degrees.zero())),

      // Level 4 (L4 - Highest Branches)
      new Pose3d(
          kRedStickPoses[0][0],
          kRedStickPoses[0][1],
          kL4H,
          new Rotation3d(Degrees.zero(), Degrees.of(90), Degrees.zero())),
      new Pose3d(
          kRedStickPoses[1][0],
          kRedStickPoses[1][1],
          kL4H,
          new Rotation3d(Degrees.zero(), Degrees.of(90), Degrees.zero())),
      new Pose3d(
          kRedStickPoses[2][0],
          kRedStickPoses[2][1],
          kL4H,
          new Rotation3d(Degrees.zero(), Degrees.of(90), Degrees.zero())),
      new Pose3d(
          kRedStickPoses[3][0],
          kRedStickPoses[3][1],
          kL4H,
          new Rotation3d(Degrees.zero(), Degrees.of(90), Degrees.zero())),
      new Pose3d(
          kRedStickPoses[4][0],
          kRedStickPoses[4][1],
          kL4H,
          new Rotation3d(Degrees.zero(), Degrees.of(90), Degrees.zero())),
      new Pose3d(
          kRedStickPoses[5][0],
          kRedStickPoses[5][1],
          kL4H,
          new Rotation3d(Degrees.zero(), Degrees.of(90), Degrees.zero())),
      new Pose3d(
          kRedStickPoses[6][0],
          kRedStickPoses[6][1],
          kL4H,
          new Rotation3d(Degrees.zero(), Degrees.of(90), Degrees.zero())),
      new Pose3d(
          kRedStickPoses[7][0],
          kRedStickPoses[7][1],
          kL4H,
          new Rotation3d(Degrees.zero(), Degrees.of(90), Degrees.zero())),
      new Pose3d(
          kRedStickPoses[8][0],
          kRedStickPoses[8][1],
          kL4H,
          new Rotation3d(Degrees.zero(), Degrees.of(90), Degrees.zero())),
      new Pose3d(
          kRedStickPoses[9][0],
          kRedStickPoses[9][1],
          kL4H,
          new Rotation3d(Degrees.zero(), Degrees.of(90), Degrees.zero())),
      new Pose3d(
          kRedStickPoses[10][0],
          kRedStickPoses[10][1],
          kL4H,
          new Rotation3d(Degrees.zero(), Degrees.of(90), Degrees.zero())),
      new Pose3d(
          kRedStickPoses[11][0],
          kRedStickPoses[11][1],
          kL4H,
          new Rotation3d(Degrees.zero(), Degrees.of(90), Degrees.zero())),
    };
  }
}
