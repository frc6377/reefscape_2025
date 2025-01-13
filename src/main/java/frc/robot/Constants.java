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
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
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

  public class IntakeConstants {
    // Simulation Values
    public static final Distance kIntakeWidth = Inches.of(26);
    public static final Distance kIntakeExtension = Inches.of(12);
    public static final int kIntakeCapacity = 1;
  }

  public class SimulationFeildConstants {
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

    // Blue Allience coral Poses
    public static final Pose3d kBReefCent = new Pose3d(4.84505, 4.0259, 2, new Rotation3d());

    // Heights
    private static final Distance kL1H = Meters.of(0.4572);
    private static final Distance kL2H = Meters.of(0.792082);
    private static final Distance kL3H = Meters.of(1.209675);
    private static final Distance kL4H = Meters.of(1.8288);

    // Angles
    private static final Angle kL2L3Angle = Degrees.of(35);
    private static final Angle kL4Angle = Degrees.of(90);

    private static final Angle kYaw60 = Degrees.of(60);
    private static final Angle kYaw180 = Degrees.of(180);

    // X, Y Cordiantes
    private static final Pose2d[] kStickPoses =
        new Pose2d[] {
          new Pose2d(3.71345, 3.85297, new Rotation2d()),
          new Pose2d(3.71345, 4.18158, new Rotation2d()),
          new Pose2d(3.96073, 3.42664, new Rotation2d()),
          new Pose2d(3.96073, 4.6089, new Rotation2d()),
          new Pose2d(4.24475, 3.26134, new Rotation2d()),
          new Pose2d(4.24475, 4.77222, new Rotation2d()),
          new Pose2d(4.73760, 3.26233, new Rotation2d()),
          new Pose2d(4.73760, 4.77321, new Rotation2d()),
          new Pose2d(5.02276, 3.42565, new Rotation2d()),
          new Pose2d(5.02276, 4.60791, new Rotation2d()),
          new Pose2d(5.26833, 3.85297, new Rotation2d()),
          new Pose2d(5.26833, 4.18158, new Rotation2d()),
        };

    public static final Pose3d[] kBlueCoralScorePoses = {
      // Level 1 (L1 - Trough Positions)
      new Pose3d(0, 0, kL1H.in(Meters), new Rotation3d()),
      new Pose3d(0, 0, kL1H.in(Meters), new Rotation3d()),
      new Pose3d(0, 0, kL1H.in(Meters), new Rotation3d()),
      new Pose3d(0, 0, kL1H.in(Meters), new Rotation3d()),
      new Pose3d(0, 0, kL1H.in(Meters), new Rotation3d()),
      new Pose3d(0, 0, kL1H.in(Meters), new Rotation3d()),

      // Level 2 (L2 - Lower Branches)
      new Pose3d(
          kStickPoses[0].getX(),
          kStickPoses[0].getY(),
          kL2H.in(Meters),
          new Rotation3d(0, kL2L3Angle.in(Radians), 0)),
      new Pose3d(
          kStickPoses[1].getX(),
          kStickPoses[1].getY(),
          kL2H.in(Meters),
          new Rotation3d(0, kL2L3Angle.in(Radians), 0)),
      new Pose3d(
          kStickPoses[2].getX(),
          kStickPoses[2].getY(),
          kL2H.in(Meters),
          new Rotation3d(0, kL2L3Angle.in(Radians), kYaw60.in(Radians))),
      new Pose3d(
          kStickPoses[3].getX(),
          kStickPoses[3].getY(),
          kL2H.in(Meters),
          new Rotation3d(0, kL2L3Angle.in(Radians), -kYaw60.in(Radians))),
      new Pose3d(
          kStickPoses[4].getX(),
          kStickPoses[4].getY(),
          kL2H.in(Meters),
          new Rotation3d(0, kL2L3Angle.in(Radians), kYaw60.in(Radians))),
      new Pose3d(
          kStickPoses[5].getX(),
          kStickPoses[5].getY(),
          kL2H.in(Meters),
          new Rotation3d(0, kL2L3Angle.in(Radians), -kYaw60.in(Radians))),
      new Pose3d(
          kStickPoses[6].getX(),
          kStickPoses[6].getY(),
          kL2H.in(Meters),
          new Rotation3d(0, -kL2L3Angle.in(Radians), -kYaw60.in(Radians))),
      new Pose3d(
          kStickPoses[7].getX(),
          kStickPoses[7].getY(),
          kL2H.in(Meters),
          new Rotation3d(0, -kL2L3Angle.in(Radians), kYaw60.in(Radians))),
      new Pose3d(
          kStickPoses[8].getX(),
          kStickPoses[8].getY(),
          kL2H.in(Meters),
          new Rotation3d(0, -kL2L3Angle.in(Radians), -kYaw60.in(Radians))),
      new Pose3d(
          kStickPoses[9].getX(),
          kStickPoses[9].getY(),
          kL2H.in(Meters),
          new Rotation3d(0, -kL2L3Angle.in(Radians), kYaw60.in(Radians))),
      new Pose3d(
          kStickPoses[10].getX(),
          kStickPoses[10].getY(),
          kL2H.in(Meters),
          new Rotation3d(0, kL2L3Angle.in(Radians), kYaw180.in(Radians))),
      new Pose3d(
          kStickPoses[11].getX(),
          kStickPoses[11].getY(),
          kL2H.in(Meters),
          new Rotation3d(0, kL2L3Angle.in(Radians), kYaw180.in(Radians))),

      // Level 3 (L3 - Middle Branches)
      new Pose3d(
          kStickPoses[0].getX(),
          kStickPoses[0].getY(),
          kL3H.in(Meters),
          new Rotation3d(0, kL2L3Angle.in(Radians), 0)),
      new Pose3d(
          kStickPoses[1].getX(),
          kStickPoses[1].getY(),
          kL3H.in(Meters),
          new Rotation3d(0, kL2L3Angle.in(Radians), 0)),
      new Pose3d(
          kStickPoses[2].getX(),
          kStickPoses[2].getY(),
          kL3H.in(Meters),
          new Rotation3d(0, kL2L3Angle.in(Radians), kYaw60.in(Radians))),
      new Pose3d(
          kStickPoses[3].getX(),
          kStickPoses[3].getY(),
          kL3H.in(Meters),
          new Rotation3d(0, kL2L3Angle.in(Radians), -kYaw60.in(Radians))),
      new Pose3d(
          kStickPoses[4].getX(),
          kStickPoses[4].getY(),
          kL3H.in(Meters),
          new Rotation3d(0, kL2L3Angle.in(Radians), kYaw60.in(Radians))),
      new Pose3d(
          kStickPoses[5].getX(),
          kStickPoses[5].getY(),
          kL3H.in(Meters),
          new Rotation3d(0, kL2L3Angle.in(Radians), -kYaw60.in(Radians))),
      new Pose3d(
          kStickPoses[6].getX(),
          kStickPoses[6].getY(),
          kL3H.in(Meters),
          new Rotation3d(0, -kL2L3Angle.in(Radians), -kYaw60.in(Radians))),
      new Pose3d(
          kStickPoses[7].getX(),
          kStickPoses[7].getY(),
          kL3H.in(Meters),
          new Rotation3d(0, -kL2L3Angle.in(Radians), kYaw60.in(Radians))),
      new Pose3d(
          kStickPoses[8].getX(),
          kStickPoses[8].getY(),
          kL3H.in(Meters),
          new Rotation3d(0, -kL2L3Angle.in(Radians), -kYaw60.in(Radians))),
      new Pose3d(
          kStickPoses[9].getX(),
          kStickPoses[9].getY(),
          kL3H.in(Meters),
          new Rotation3d(0, -kL2L3Angle.in(Radians), kYaw60.in(Radians))),
      new Pose3d(
          kStickPoses[10].getX(),
          kStickPoses[10].getY(),
          kL3H.in(Meters),
          new Rotation3d(0, kL2L3Angle.in(Radians), kYaw180.in(Radians))),
      new Pose3d(
          kStickPoses[11].getX(),
          kStickPoses[11].getY(),
          kL3H.in(Meters),
          new Rotation3d(0, kL2L3Angle.in(Radians), kYaw180.in(Radians))),

      // Level 4 (L4 - Highest Branches)
      new Pose3d(
          kStickPoses[0].getX(),
          kStickPoses[0].getY(),
          kL4H.in(Meters),
          new Rotation3d(0, kL4Angle.in(Radians), 0)),
      new Pose3d(
          kStickPoses[1].getX(),
          kStickPoses[1].getY(),
          kL4H.in(Meters),
          new Rotation3d(0, kL4Angle.in(Radians), 0)),
      new Pose3d(
          kStickPoses[2].getX(),
          kStickPoses[2].getY(),
          kL4H.in(Meters),
          new Rotation3d(0, kL4Angle.in(Radians), 0)),
      new Pose3d(
          kStickPoses[3].getX(),
          kStickPoses[3].getY(),
          kL4H.in(Meters),
          new Rotation3d(0, kL4Angle.in(Radians), 0)),
      new Pose3d(
          kStickPoses[4].getX(),
          kStickPoses[4].getY(),
          kL4H.in(Meters),
          new Rotation3d(0, kL4Angle.in(Radians), 0)),
      new Pose3d(
          kStickPoses[5].getX(),
          kStickPoses[5].getY(),
          kL4H.in(Meters),
          new Rotation3d(0, kL4Angle.in(Radians), 0)),
      new Pose3d(
          kStickPoses[6].getX(),
          kStickPoses[6].getY(),
          kL4H.in(Meters),
          new Rotation3d(0, kL4Angle.in(Radians), 0)),
      new Pose3d(
          kStickPoses[7].getX(),
          kStickPoses[7].getY(),
          kL4H.in(Meters),
          new Rotation3d(0, kL4Angle.in(Radians), 0)),
      new Pose3d(
          kStickPoses[8].getX(),
          kStickPoses[8].getY(),
          kL4H.in(Meters),
          new Rotation3d(0, kL4Angle.in(Radians), 0)),
      new Pose3d(
          kStickPoses[9].getX(),
          kStickPoses[9].getY(),
          kL4H.in(Meters),
          new Rotation3d(0, kL4Angle.in(Radians), 0)),
      new Pose3d(
          kStickPoses[10].getX(),
          kStickPoses[10].getY(),
          kL4H.in(Meters),
          new Rotation3d(0, kL4Angle.in(Radians), 0)),
      new Pose3d(
          kStickPoses[11].getX(),
          kStickPoses[11].getY(),
          kL4H.in(Meters),
          new Rotation3d(0, kL4Angle.in(Radians), 0)),
    };
  }

  public class SubsystemToggles {
    public static final boolean kUseIntake = true;
    public static final boolean kUseDrive = true;
    public static final boolean kUseVision = true;
  }
}
