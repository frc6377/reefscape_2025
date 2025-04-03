// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;

/** Add your docs here. */
public class MechVisualizer {
  public static enum Axis {
    X,
    Y,
    Z
  };

  public static Pose3d[] originalPoseList;
  public static Pose3d[] mechPoses;

  public MechVisualizer(Pose3d[] startingList) {
    originalPoseList = startingList;
    mechPoses = startingList;
  }

  public void updateIndexPose(int index, Pose3d newPose) {
    mechPoses[index] = newPose;
  }

  public void updateIndexRotation(int index, Rotation3d newRotation) {
    mechPoses[index] = new Pose3d(mechPoses[index].getTranslation(), newRotation);
  }

  public void updateIndexRotation(int index, Axis axis, Angle value) {
    Rotation3d oldRotation = mechPoses[index].getRotation();
    Rotation3d newRotation;

    switch (axis) {
      case X:
        newRotation = new Rotation3d(value, oldRotation.getMeasureY(), oldRotation.getMeasureZ());
        break;
      case Y:
        newRotation = new Rotation3d(oldRotation.getMeasureX(), value, oldRotation.getMeasureZ());
        break;
      case Z:
        newRotation = new Rotation3d(oldRotation.getMeasureX(), oldRotation.getMeasureY(), value);
        break;
      default:
        newRotation = oldRotation;
        break;
    }

    mechPoses[index] = new Pose3d(mechPoses[index].getTranslation(), newRotation);
  }

  public void updateIndexTranslation(int index, Translation3d newTranslation) {
    mechPoses[index] = new Pose3d(newTranslation, mechPoses[index].getRotation());
  }

  public void updateIndexTranslation(int index, Axis axis, Distance value) {
    Pose3d oldPose = mechPoses[index];
    Translation3d newTranslation;

    switch (axis) {
      case X:
        newTranslation = new Translation3d(value, oldPose.getMeasureY(), oldPose.getMeasureZ());
        break;
      case Y:
        newTranslation = new Translation3d(oldPose.getMeasureX(), value, oldPose.getMeasureZ());
        break;
      case Z:
        newTranslation = new Translation3d(oldPose.getMeasureX(), oldPose.getMeasureY(), value);
        break;
      default:
        newTranslation = oldPose.getTranslation();
        break;
    }

    mechPoses[index] = new Pose3d(newTranslation, oldPose.getRotation());
  }

  public Pose3d getIndexPose(int index) {
    return mechPoses[index];
  }

  public Pose3d[] getMechPoses() {
    return mechPoses;
  }
}
