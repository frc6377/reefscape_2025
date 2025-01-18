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

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
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

  public static class MotorIDConstants {
    // Rev Can Bus
    public static final int kElevatorMotor1 = 1;
    public static final int kIntakeMotor = 12;

    // CANavor Can Bus
  }

  public class IntakeConstants {
    public static final double kSpeed = 0.5;

    // Simulation Values
    public static final Distance kIntakeWidth = Inches.of(26);
    public static final Distance kIntakeExtension = Inches.of(12);
    public static final int kIntakeCapacity = 1;
  }

  public static class ElevatorConstants {
    public static final Distance kL1Height = Inches.of(18);
    public static final Distance kL2Height = Inches.of(31.875);
    public static final Distance kL3Height = Inches.of(47.625);
    public static final Distance kL4Height = Inches.of(72);

    public static final double P = 1;
    public static final double I = 0.0;
    public static final double D = 0.0;
    public static final double FF = 0.0;
    public static final double kElevatorConversion = 1.0;

    // The carriage on the elv effectivly adds a gearing multiplier of 2
    public static final double kCarageFactor = 2;

    // Simulation Constants
    public static final DCMotor kElevatorGearbox = DCMotor.getNEO(1);
    public static final double kElevatorGearing = 75.0;
    public static final Mass kCarriageMass = Pounds.of(10);
    public static final Distance kElevatorDrumRadius = Inches.of(1.729 / 2);
    public static final Distance kMinElevatorHeight = Inches.zero();
    public static final Distance kMaxElevatorHeight = Inches.of(76);
    public static final Distance kElevatorDrumCircumference =
        kElevatorDrumRadius.times(2 * Math.PI);
    public static final double kMechenismOffset = 0.1;
  }

  public class SimulationFeildConstants {
    public static final Distance kScoreDistance = Inch.of(17.25);

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
      // Level 1 (L1 - Trough Positions)
      new Pose3d(Meters.zero(), Meters.zero(), kL1H, new Rotation3d()),
      new Pose3d(Meters.zero(), Meters.zero(), kL1H, new Rotation3d()),
      new Pose3d(Meters.zero(), Meters.zero(), kL1H, new Rotation3d()),
      new Pose3d(Meters.zero(), Meters.zero(), kL1H, new Rotation3d()),
      new Pose3d(Meters.zero(), Meters.zero(), kL1H, new Rotation3d()),
      new Pose3d(Meters.zero(), Meters.zero(), kL1H, new Rotation3d()),

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
