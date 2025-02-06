// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;
import static frc.robot.Constants.SimulatedMechPoses.kCoralScorerPose;
import static frc.robot.Constants.SimulationFeildConstants.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.SimulatedMechPoses;
import java.util.ArrayList;
import java.util.List;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoralOnField;
import org.littletonrobotics.junction.Logger;

public class MapleSimArenaSubsystem extends SubsystemBase {
  private final SwerveDriveSimulation swerveDriveSimulation;

  private List<Pose3d> scoredCoralPoses = new ArrayList<Pose3d>();

  private Pose3d robotCoralPose = SimulatedMechPoses.kCoralScorerPose;
  private Pose3d closestScorePose = null;

  private boolean robotHasCoral = false;

  /** Creates a new MapleSimArenaSubsystem. */
  public MapleSimArenaSubsystem(SwerveDriveSimulation swerveDriveSimulation) {
    this.swerveDriveSimulation = swerveDriveSimulation;

    logFeildArea(kSourceAreas);
  }

  public void logFeildArea(Pose2d[][] area) {
    Pose2d[][] newPoses =
        new Pose2d[][] {
          new Pose2d[] {
            area[0][0],
            new Pose2d(area[0][0].getX(), area[0][1].getY(), new Rotation2d()),
            area[0][1],
            new Pose2d(area[0][1].getX(), area[0][0].getY(), new Rotation2d()),
            area[0][0]
          },
          new Pose2d[] {
            area[1][0],
            new Pose2d(area[1][0].getX(), area[1][1].getY(), new Rotation2d()),
            area[1][1],
            new Pose2d(area[1][1].getX(), area[1][0].getY(), new Rotation2d()),
            area[1][0]
          },
          new Pose2d[] {
            area[2][0],
            new Pose2d(area[2][0].getX(), area[2][1].getY(), new Rotation2d()),
            area[2][1],
            new Pose2d(area[2][1].getX(), area[2][0].getY(), new Rotation2d()),
            area[2][0]
          },
          new Pose2d[] {
            area[3][0],
            new Pose2d(area[3][0].getX(), area[3][1].getY(), new Rotation2d()),
            area[3][1],
            new Pose2d(area[3][1].getX(), area[3][0].getY(), new Rotation2d()),
            area[3][0]
          },
        };
    Logger.recordOutput("FieldSimulation/Source Area", newPoses);
  }

  public void setRobotHasCoral(boolean newRobotHasCoral) {
    robotHasCoral = newRobotHasCoral;
  }

  public void updateRobotCoralPose(Distance elevatorHeight) {
    if (robotHasCoral) {
      Pose2d drivePose = swerveDriveSimulation.getSimulatedDriveTrainPose();

      Translation2d newCoralTranslation =
          new Translation2d(
                  kCoralScorerPose.getMeasureX().plus(drivePose.getMeasureX()),
                  kCoralScorerPose.getMeasureY().plus(drivePose.getMeasureY()))
              .rotateAround(
                  new Translation2d(drivePose.getMeasureX(), drivePose.getMeasureY()),
                  new Rotation2d(drivePose.getRotation().getMeasure()));
      robotCoralPose =
          new Pose3d(
              new Translation3d(
                  newCoralTranslation.getMeasureX(),
                  newCoralTranslation.getMeasureY(),
                  kCoralScorerPose.getMeasureZ().plus(elevatorHeight)),
              kCoralScorerPose.getRotation().plus(new Rotation3d(drivePose.getRotation())));
    } else {
      robotCoralPose = new Pose3d();
    }
  }

  public Boolean getRobotHasCoral() {
    return robotHasCoral;
  }

  public Boolean isInArea(Pose2d[] area, Pose2d robotPose) {
    double x = robotPose.getX();
    double y = robotPose.getY();
    double minX = Math.min(area[0].getX(), area[1].getX());
    double maxX = Math.max(area[0].getX(), area[1].getX());
    double minY = Math.min(area[0].getY(), area[1].getY());
    double maxY = Math.max(area[0].getY(), area[1].getY());
    return minX <= x && x <= maxX && minY <= y && y <= maxY;
  }

  public Pose3d getClosestScorePose() {
    Pose3d[] scorePoseList =
        DriverStation.getAlliance().orElse(Alliance.Blue).equals(Alliance.Blue)
            ? kBlueCoralScorePoses
            : kRedCoralScorePoses;

    Pose3d closestScorePose = null;
    double closestDistance = kScoreDistance.in(Meters);
    for (Pose3d scorePose : scorePoseList) {
      double currentDistance =
          robotCoralPose.getTranslation().getDistance(scorePose.getTranslation());
      if (currentDistance < kScoreDistance.in(Meters)
          && currentDistance < closestDistance
          && !scoredCoralPoses.contains(scorePose)) {
        closestDistance = currentDistance;
        closestScorePose = scorePose;
      }
    }

    return closestScorePose;
  }

  public Trigger canScore() {
    return new Trigger(() -> closestScorePose != null);
  }

  public Command scoreCoral() {
    return Commands.runOnce(
        () -> {
          if (robotHasCoral) {
            if (closestScorePose != null) {
              scoredCoralPoses.add(closestScorePose);
              robotHasCoral = false;
            }
          }
        });
  }

  public Command resetSimFeild() {
    return Commands.runOnce(
        () -> {
          SimulatedArena.getInstance().resetFieldForAuto();
          scoredCoralPoses = new ArrayList<Pose3d>() {};
        });
  }

  @Override
  public void periodic() {
    closestScorePose = getClosestScorePose();

    // Add game pieces at sources for robot to score
    for (int i = 0; i < kSourceAreas.length; i++) {
      Pose2d[] currentSourse = kSourceAreas[i];

      // Check if there is a coral at the source area
      if (isInArea(currentSourse, swerveDriveSimulation.getSimulatedDriveTrainPose())) {
        boolean isCoralAtSource = false;
        for (Pose3d coralPose : SimulatedArena.getInstance().getGamePiecesArrayByType("Coral")) {
          if (isInArea(currentSourse, coralPose.toPose2d())) {
            isCoralAtSource = true;
            break;
          }
        }

        // Add coral to the source area if there is none
        if (!isCoralAtSource) {
          SimulatedArena.getInstance()
              .addGamePiece(
                  new ReefscapeCoralOnField(
                      new Pose2d(
                          currentSourse[0]
                              .getTranslation()
                              .interpolate(currentSourse[1].getTranslation(), 0.5),
                          new Rotation2d())));
        }
      }
    }

    Pose3d[] tempArray = new Pose3d[] {};
    Logger.recordOutput("FieldSimulation/Scored Coral Poses", scoredCoralPoses.toArray(tempArray));
    Logger.recordOutput(
        "FieldSimulation/RobotPosition", swerveDriveSimulation.getSimulatedDriveTrainPose());
    Logger.recordOutput(
        "FieldSimulation/Algae", SimulatedArena.getInstance().getGamePiecesArrayByType("Algae"));
    Logger.recordOutput(
        "FieldSimulation/Coral", SimulatedArena.getInstance().getGamePiecesArrayByType("Coral"));
    Logger.recordOutput("FieldSimulation/Robot Game Piece Pose", robotCoralPose);
    if (closestScorePose != null) {
      Logger.recordOutput(
          "FieldSimulation/Closest Score Pose", new Pose3d[] {robotCoralPose, closestScorePose});
    } else {
      Logger.recordOutput("FieldSimulation/Closest Score Pose", new Pose3d[] {new Pose3d()});
    }
  }
}
