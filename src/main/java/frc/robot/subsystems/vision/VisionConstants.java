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

package frc.robot.subsystems.vision;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;

public class VisionConstants {
  public static final boolean kUsingQuestNav = true;
  // AprilTag layout
  public static final AprilTagFieldLayout kAprilTagLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark);

  public static final boolean kVisionUpdatesOdometry = true;

  // Camera names, must match names configured on coprocessor
  public static final String camera0Name = "limelight-lowcam";

  // Robot to camera transforms
  // (Not used by Limelight, configure in web UI instead)
  // New pose for 14 degrees angle
  public static final Transform3d robotToCamera0 =
      new Transform3d(
          Inches.of(8.500177),
          Inches.of(-10.623276),
          Inches.of(9.536276),
          new Rotation3d(Degrees.of(0), Degrees.of(-14), Degrees.of(-90)));

  // Basic filtering thresholds
  public static final int minTags = 1;
  public static final double maxAmbiguity = 0.1; // Lower Better
  public static final double maxZError = 0.5; // / Lower Better

  // Standard deviation baselines, for 1 meter distance and 1 tag
  // (Adjusted automatically based on distance and # of tags)
  public static Distance linearStdDevBaseline = Meters.of(0.02);
  public static Angle angularStdDevBaseline = Radians.of(0.06);

  // Standard deviation multipliers for each camera
  // (Adjust to trust some cameras more than others)
  public static double[] cameraStdDevFactors =
      new double[] {
        1.0, // Camera 0
      };

  // Multipliers to apply for MegaTag 2 observations
  public static double linearStdDevMegatag2Factor = 0.5; // More stable than full 3D solve
  public static double angularStdDevMegatag2Factor =
      Double.POSITIVE_INFINITY; // No rotation data available
}
