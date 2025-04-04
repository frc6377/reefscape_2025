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
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static frc.robot.subsystems.vision.VisionConstants.*;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.subsystems.vision.VisionIO.PoseObservationType;
import java.util.LinkedList;
import java.util.List;
import java.util.Optional;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import utilities.LimelightHelpers;

public class Vision extends SubsystemBase {
  private final VisionConsumer consumer;
  private final VisionIO[] io;
  private final VisionIOInputsAutoLogged[] inputs;
  private final Alert[] disconnectedAlerts;

  private int currentTagCount = 0;

  public Vision(VisionConsumer consumer, VisionIO... io) {
    this.consumer = consumer;
    this.io = io;

    // Initialize inputs
    this.inputs = new VisionIOInputsAutoLogged[io.length];
    for (int i = 0; i < inputs.length; i++) {
      inputs[i] = new VisionIOInputsAutoLogged();
    }

    // Initialize disconnected alerts
    this.disconnectedAlerts = new Alert[io.length];
    for (int i = 0; i < inputs.length; i++) {
      disconnectedAlerts[i] =
          new Alert(
              "Vision camera " + Integer.toString(i) + " is disconnected.", AlertType.kWarning);
    }
  }

  /**
   * Returns the X angle to the best target, which can be used for simple servoing with vision.
   *
   * @param cameraIndex The index of the camera to use.
   */
  public Rotation2d getTargetX(int cameraIndex) {
    return inputs[cameraIndex].latestTargetObservation.tx();
  }

  public Pose3d getVisionPose(int cameraIndex) {
    if (inputs[cameraIndex].poseObservations.length == 0 || inputs[cameraIndex].tagIds.length == 0)
      return new Pose3d();
    return inputs[cameraIndex].poseObservations[0].pose();
  }

  public int getClosestTagID(int cameraIndex) {
    if (inputs[cameraIndex].tagIds.length == 0) return -1;

    Pose3d cameraPose = getVisionPose(cameraIndex);
    int closestTagID = -1;
    double minDistance = Double.MAX_VALUE;

    for (int tagId : inputs[cameraIndex].tagIds) {
      Pose3d tagPose = getTagPose(tagId);
      double distance = cameraPose.getTranslation().getDistance(tagPose.getTranslation());

      if (distance < minDistance) {
        minDistance = distance;
        closestTagID = tagId;
      }
    }

    return closestTagID;
  }

  public Pose3d getTagPose(int tagID) {
    Optional<Pose3d> tagPoseOptional = VisionConstants.kAprilTagLayout.getTagPose(tagID);
    if (tagPoseOptional.isEmpty()) return new Pose3d();
    return tagPoseOptional.get();
  }

  public Pose3d convertLLPose(Pose3d pose) {
    return new Pose3d(
        pose.getMeasureZ(),
        pose.getMeasureX(),
        pose.getMeasureY().times(-1),
        new Rotation3d(Degrees.zero(), Degrees.zero(), pose.getRotation().getMeasureY()));
  }

  @Override
  public void periodic() {
    Logger.recordOutput("Auto Align/LL Has Target", LimelightHelpers.getTV(camera0Name));
    Logger.recordOutput("Auto Align/LL Target ID", LimelightHelpers.getFiducialID(camera0Name));
    for (int i = 0; i < io.length; i++) {
      io[i].updateInputs(inputs[i]);
      Logger.processInputs("Vision/Camera" + i, inputs[i]);
    }

    // Initialize logging values
    List<Pose3d> allTagPoses = new LinkedList<>();
    List<Pose3d> allRobotPoses = new LinkedList<>();
    List<Pose3d> allRobotPosesAccepted = new LinkedList<>();
    List<Pose3d> allRobotPosesRejected = new LinkedList<>();

    // Loop over cameras
    for (int cameraIndex = 0; cameraIndex < io.length; cameraIndex++) {
      // Update disconnected alert
      disconnectedAlerts[cameraIndex].set(!inputs[cameraIndex].connected);

      // Initialize logging values
      List<Pose3d> tagPoses = new LinkedList<>();
      List<Pose3d> robotPoses = new LinkedList<>();
      List<Pose3d> robotPosesAccepted = new LinkedList<>();
      List<Pose3d> robotPosesRejected = new LinkedList<>();

      // Add tag poses
      for (int tagId : inputs[cameraIndex].tagIds) {
        var tagPose = kAprilTagLayout.getTagPose(tagId);
        if (tagPose.isPresent()) {
          tagPoses.add(tagPose.get());
        }
      }

      // Loop over pose observations
      for (var observation : inputs[cameraIndex].poseObservations) {
        // Check whether to reject pose
        currentTagCount = observation.tagCount();

        boolean rejectPose =
            currentTagCount < minTags // Must have enough tags
                || observation.ambiguity() > maxAmbiguity // Cannot be high ambiguity
                || Math.abs(observation.pose().getZ()) > maxZError // Must have realistic Z cord

                // Must be within the field boundaries
                || observation.pose().getX() < 0.0
                || observation.pose().getX() > kAprilTagLayout.getFieldLength()
                || observation.pose().getY() < 0.0
                || observation.pose().getY() > kAprilTagLayout.getFieldWidth();

        // Add pose to log
        robotPoses.add(observation.pose());
        if (rejectPose) {
          robotPosesRejected.add(observation.pose());
        } else {
          robotPosesAccepted.add(observation.pose());
        }

        // Skip if rejected
        if (rejectPose) continue;

        // Calculate standard deviations
        double stdDevFactor = Math.pow(observation.averageTagDistance(), 2.0) / currentTagCount;
        double linearStdDev = linearStdDevBaseline.in(Meters) * stdDevFactor;
        double angularStdDev = angularStdDevBaseline.in(Radians) * stdDevFactor;
        if (observation.type() == PoseObservationType.MEGATAG_2) {
          linearStdDev *= linearStdDevMegatag2Factor;
          angularStdDev *= angularStdDevMegatag2Factor;
        }
        if (cameraIndex < cameraStdDevFactors.length) {
          linearStdDev *= cameraStdDevFactors[cameraIndex];
          angularStdDev *= cameraStdDevFactors[cameraIndex];
        }

        // Send vision observation
        if (Robot.isUsingVision && VisionConstants.kVisionUpdatesOdometry) {
          consumer.accept(
              observation.pose().toPose2d(),
              observation.timestamp(),
              VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev));
        }
      }

      // Log camera datadata
      Logger.recordOutput(
          "Vision/Camera" + cameraIndex + "/TagPoses",
          tagPoses.toArray(new Pose3d[tagPoses.size()]));
      Logger.recordOutput(
          "Vision/Camera" + cameraIndex + "/RobotPoses",
          robotPoses.toArray(new Pose3d[robotPoses.size()]));
      Logger.recordOutput(
          "Vision/Camera" + cameraIndex + "/RobotPosesAccepted",
          robotPosesAccepted.toArray(new Pose3d[robotPosesAccepted.size()]));
      Logger.recordOutput(
          "Vision/Camera" + cameraIndex + "/RobotPosesRejected",
          robotPosesRejected.toArray(new Pose3d[robotPosesRejected.size()]));
      allTagPoses.addAll(tagPoses);
      allRobotPoses.addAll(robotPoses);
      allRobotPosesAccepted.addAll(robotPosesAccepted);
      allRobotPosesRejected.addAll(robotPosesRejected);
    }

    // Log summary data
    Logger.recordOutput(
        "Vision/Summary/TagPoses", allTagPoses.toArray(new Pose3d[allTagPoses.size()]));
    Logger.recordOutput(
        "Vision/Summary/RobotPoses", allRobotPoses.toArray(new Pose3d[allRobotPoses.size()]));
    Logger.recordOutput(
        "Vision/Summary/RobotPosesAccepted",
        allRobotPosesAccepted.toArray(new Pose3d[allRobotPosesAccepted.size()]));
    Logger.recordOutput(
        "Vision/Summary/RobotPosesRejected",
        allRobotPosesRejected.toArray(new Pose3d[allRobotPosesRejected.size()]));
  }

  @AutoLogOutput(key = "Vision/TagCount")
  public int getTagCount() {
    return currentTagCount;
  }

  @FunctionalInterface
  public interface VisionConsumer {
    void accept(
        Pose2d visionRobotPoseMeters,
        double timestampSeconds,
        Matrix<N3, N1> visionMeasurementStdDevs);
  }
}
