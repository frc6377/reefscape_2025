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

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.signals.GravityTypeValue;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.AngularAccelerationUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.subsystems.Elevator;

public final class Constants {
  /**
   * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when
   * running on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and
   * "replay" (log replay from a file).
   */
  public static final Mode simMode = Mode.SIM;

  public static final Mode currentMode = Robot.isReal() ? Mode.REAL : simMode;

  public static enum Mode {
    REAL,
    SIM,
    /** Replaying from a log file. */
    REPLAY
  }

  public static final String CANivoreName = "Default Name";
  public static final String RIOName = "rio";

  public static final int kStreamDeckTotalButtonCount = 64;
  public static final int kMaxControllerButtonCount = 32;
  public static final String[] kPoleLetters =
      new String[] {"A", "B", "C", "D", "E", "F", "G", "H", "I", "J", "K", "L"};

  public static class CANIDs {
    // Rev Can Bus
    // 1-8 Motor ID is reserved by the drivebase
    public static final int kScorerMotor = 15;
    public static final int kElevatorMotor1 = 10;
    public static final int kElevatorMotor2 = 11;
    public static final int kIntakeMotor = 13;
    public static final int kPivotMotor = 12;
    public static final int kConveyorMotor = 14;
    public static final int kConveyorSensor = 18; // FIXME: Change to correct ID -> 1
  }

  public static class DIOConstants {
    public static final int kthroughBoreEncoderID = 1;
    public static final int gear11ID = 13;
    public static final int gear3ID = 14;
  }

  // Scorer Constants
  public static class CoralScorerConstants {
    public static final double kSpeed = 0.5;
  }

  // Intake Constants
  public static class IntakeConstants {
    public static final double kIntakeSpeed = -1;
    public static final double kConveyorSpeed = 0.8;
    public static final double kPivotSpeed = 1;
    public static final Angle kPivotRetractAngle = Degrees.of(129.28); // FIXME: Put actual value
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

    // Simulation
    public static final Distance kLength = Feet.of(1);
    public static final Mass kMass = Pounds.of(8);
    public static final MomentOfInertia kMOI =
        KilogramSquareMeters.of(
            SingleJointedArmSim.estimateMOI(kLength.in(Meters), kMass.in(Kilograms)));

    // Motion Magic
    public static final AngularVelocity kMotionMagicCruiseVelocity = DegreesPerSecond.of(325);
    public static final AngularAcceleration kMotionMagicAcceleration =
        kMotionMagicCruiseVelocity.times(Hertz.of(5));
    public static final double kMotionMagicJerk = 10;

    public static final double armZero = 0.35; // TODO: Get units for this!

    // For maplesim Intake
    public static final Distance kIntakeWidth = Meters.of(0.470);
    public static final Distance kIntakeExtension = Meters.of(0.191);
    public static final int kIntakeCapacity = 1;
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
    public static final AngularVelocity MMVel = Elevator.heightToRotations(InchesPerSecond.of(60));
    public static final AngularAcceleration MMAcc = MMVel.times(Hertz.of(5));
    public static final Velocity<AngularAccelerationUnit> MMJerk =
        RotationsPerSecondPerSecond.per(Second).of(MMAcc.in(RotationsPerSecondPerSecond)).times(10);
  }

  public final class DrivetrainConstants {
    // PathPlanner config constants
    public static final double ROBOT_MASS_KG = 34.2462254;
    public static final double ROBOT_MOI = 4.682;
    public static final double WHEEL_COF = 1.2;

    // TODO: Get Correct Values
    public static final PathConstraints PATH_CONSTRAINTS =
        new PathConstraints(1, 1, Degrees.of(540).in(Radians), Degrees.of(720).in(Radians));

    // Scoring Poses for PathFinder
    public static final Pose2d[] SCORE_POSES =
        new Pose2d[] {
          new Pose2d(Meters.of(3.173), Meters.of(4.193), new Rotation2d(Degrees.of(0))),
          new Pose2d(Meters.of(3.173), Meters.of(3.854), new Rotation2d(Degrees.of(0))),
          new Pose2d(Meters.of(3.686), Meters.of(2.973), new Rotation2d(Degrees.of(60))),
          new Pose2d(Meters.of(3.974), Meters.of(2.801), new Rotation2d(Degrees.of(60))),
          new Pose2d(Meters.of(5.000), Meters.of(2.799), new Rotation2d(Degrees.of(120))),
          new Pose2d(Meters.of(5.286), Meters.of(2.966), new Rotation2d(Degrees.of(120))),
          new Pose2d(Meters.of(5.803), Meters.of(3.854), new Rotation2d(Degrees.of(180))),
          new Pose2d(Meters.of(5.803), Meters.of(4.194), new Rotation2d(Degrees.of(180))),
          new Pose2d(Meters.of(5.288), Meters.of(5.082), new Rotation2d(Degrees.of(-120))),
          new Pose2d(Meters.of(5.001), Meters.of(5.245), new Rotation2d(Degrees.of(-120))),
          new Pose2d(Meters.of(3.972), Meters.of(5.244), new Rotation2d(Degrees.of(-60))),
          new Pose2d(Meters.of(3.684), Meters.of(5.078), new Rotation2d(Degrees.of(-60))),
        };

    public static final Pose2d[] SOURSE_POSES =
        new Pose2d[] {
          new Pose2d(Meters.of(1.227), Meters.of(7.045), new Rotation2d(Degrees.of(-54))),
          new Pose2d(Meters.of(1.256), Meters.of(0.955), new Rotation2d(Degrees.of(54))),
        };
  }

  public final class SimulatedMechPoses {
    public static final Pose3d kIntakeStartPose =
        new Pose3d(
            Meters.of(0.191591),
            Meters.of(0.091696),
            Meters.of(0.242354),
            new Rotation3d(Degrees.of(0), Degrees.of(260), Degrees.of(0)));

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

  public final class SimulationFeildConstants {
    public static final Distance kScoreDistance = Inch.of(8.5);

    public static final Distance kFieldWidth = Inches.of(317);
    public static final Distance kFieldLength = Inches.of(690 + (7 / 8));

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

    // Feild Dimentions
    public static final Pose3d kBReefCent = new Pose3d(4.84505, 4.0259, 2, new Rotation3d());
    public static final Distance kFeildWidth = Inches.of(317);
    public static final Distance kFeildLength = Inches.of(690 + (7 / 8));

    // Heights
    private static final Distance kL1H = Meters.of(0.4572);
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
      // Level 1 (L1 - Trough Positions)
      new Pose3d(Meters.zero(), Meters.zero(), kL1H, new Rotation3d()),
      new Pose3d(Meters.zero(), Meters.zero(), kL1H, new Rotation3d()),
      new Pose3d(Meters.zero(), Meters.zero(), kL1H, new Rotation3d()),
      new Pose3d(Meters.zero(), Meters.zero(), kL1H, new Rotation3d()),
      new Pose3d(Meters.zero(), Meters.zero(), kL1H, new Rotation3d()),
      new Pose3d(Meters.zero(), Meters.zero(), kL1H, new Rotation3d()),

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
