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

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;

import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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

  public static final Distance kFieldWidth = Inches.of(317);
  public static final Distance kFieldLength = Inches.of(690 + (7 / 8));

  public final class DrivetrainConstants {
    // PathPlanner config constants
    public static final double ROBOT_MASS_KG = 54.431;
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
}
