// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;
import static frc.robot.Constants.SimulationFeildConstants.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoralOnField;
import org.littletonrobotics.junction.Logger;

public class MapleSimArenaSubsystem extends SubsystemBase {
  private final SwerveDriveSimulation swerveDriveSimulation;

  private Boolean[] isAtSource = new Boolean[] {false, false, false, false};
  private List<Pose3d> scoredCoralPoses = new ArrayList<Pose3d>();

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

  public boolean getRobotHasCoral() {
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

  public Pose3d getClosestScorePose(Pose3d robotCoralPose) {
    Pose3d[] scorePoseList;
    if (DriverStation.getAlliance().orElse(Alliance.Blue).equals(Alliance.Blue)) {
      scorePoseList = kBlueCoralScorePoses;
    } else {
      scorePoseList = kRedCoralScorePoses;
    }

    Pose3d closestPose = null;
    double closestDistance = kScoreDistance.in(Meters);
    for (int i = 0; i < scorePoseList.length; i++) {
      double currentDistance =
          robotCoralPose.getTranslation().getDistance(scorePoseList[i].getTranslation());
      if (currentDistance < closestDistance
          && currentDistance < kScoreDistance.in(Meters)
          && !scoredCoralPoses.contains(scorePoseList[i])) {
        closestDistance = currentDistance;
        closestPose = scorePoseList[i];
      }
    }

    return closestPose;
  }

  public Command scoreCoral(Supplier<Pose3d> robotCoralPose) {
    return Commands.runOnce(
        () -> {
          Pose3d scorePose = getClosestScorePose(robotCoralPose.get());
          if (scorePose != null) {
            scoredCoralPoses.add(scorePose);
            robotHasCoral = false;
          }
        });
  }

  public Command clearSimFeild() {
    return Commands.runOnce(() -> SimulatedArena.getInstance().clearGamePieces());
  }

  public Command resetSimFeild() {
    return Commands.runOnce(
        () -> {
          SimulatedArena.getInstance().resetFieldForAuto();
          scoredCoralPoses = new ArrayList<Pose3d>();
        });
  }

  public Command resetSimFeildAuto() {
    return Commands.runOnce(() -> SimulatedArena.getInstance().resetFieldForAuto());
  }

  @Override
  public void periodic() {
    Pose3d[] tempArray = new Pose3d[] {};
    Logger.recordOutput("FieldSimulation/Scored Coral Poses", scoredCoralPoses.toArray(tempArray));

    for (int i = 0; i < kSourceAreas.length; i++) {
      Pose2d[] currentSourse = kSourceAreas[i];

      // Check if the robot is at the sorce area
      isAtSource[i] = isInArea(currentSourse, swerveDriveSimulation.getSimulatedDriveTrainPose());
      Logger.recordOutput("FieldSimulation/Area Bools/" + i, isAtSource[i]);

      // Check if there is a coral at the source area
      if (isAtSource[i]) {
        Pose3d[] coralPoses = SimulatedArena.getInstance().getGamePiecesArrayByType("Coral");
        boolean isCoralAtSource = false;
        for (int j = 0; j < coralPoses.length; j++) {
          if (isInArea(currentSourse, coralPoses[j].toPose2d())) {
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
                          (currentSourse[0].getX() + currentSourse[1].getX()) / 2,
                          (currentSourse[0].getY() + currentSourse[1].getY()) / 2,
                          new Rotation2d())));
        }
      }
    }
  }
}
