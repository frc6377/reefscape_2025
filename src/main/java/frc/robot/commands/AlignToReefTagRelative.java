// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ReefAlignConstants;
import frc.robot.Robot;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.Vision;
import frc.robot.util.LimelightHelpers;
import org.littletonrobotics.junction.Logger;

public class AlignToReefTagRelative extends Command {
  private PIDController xController, yController, rotController;
  private boolean isRightScore;
  private String cameraName;
  private Drive drivebase;
  private Vision vision;
  private int tagID = -1;
  private Pose3d tagPose;
  private Pose2d targetPose;

  private Trigger canSeeTagTrigger;
  private Trigger atPoseTrigger;

  private static final String NTFolder = "Swerve/AlignToReefTagRelative/";

  public AlignToReefTagRelative(
      boolean isRightScore, String cameraName, Drive drivebase, Vision vision) {
    xController = new PIDController(0.2, 0, 0); // Vertical movement
    yController = new PIDController(0.2, 0, 0); // Horitontal movement
    rotController = new PIDController(1, 0, 0); // Rotation
    this.isRightScore = isRightScore;
    this.cameraName = cameraName;
    this.drivebase = drivebase;
    this.vision = vision;
    addRequirements(drivebase);

    canSeeTagTrigger =
        new Trigger(
            () -> Robot.isReal() ? LimelightHelpers.getTV(cameraName) : vision.cameraHasTag(0));
    atPoseTrigger =
        new Trigger(
            () ->
                rotController.atSetpoint() && yController.atSetpoint() && xController.atSetpoint());
  }

  @Override
  public void initialize() {
    tagID =
        (int)
            (Robot.isReal()
                ? LimelightHelpers.getFiducialID(cameraName)
                : vision.getClosestTagID(0));
    tagPose = vision.getTagPose(tagID);

    targetPose =
        isRightScore ? ReefAlignConstants.kRightReefPose : ReefAlignConstants.kLeftReefPose;

    xController.setSetpoint(targetPose.getMeasureX().in(Meters));
    xController.setTolerance(ReefAlignConstants.kSetpointTolerance.in(Meters));

    yController.setSetpoint(targetPose.getMeasureY().in(Meters));
    yController.setTolerance(ReefAlignConstants.kSetpointTolerance.in(Meters));

    rotController.setSetpoint(targetPose.getRotation().getMeasure().in(Radians));
    rotController.setTolerance(ReefAlignConstants.kSetpointRotTolerance.in(Radians));

    Logger.recordOutput(NTFolder + "Tag Relative/Robot Target Pose", targetPose);
    Logger.recordOutput(
        NTFolder + "Field Relative/Robot Target Pose",
        vision.tagToFeildRelative(tagID, targetPose));
    Logger.recordOutput(NTFolder + "Tag Info/Target Side", isRightScore ? "Right" : "Left");
    Logger.recordOutput(NTFolder + "Tag Info/Camera Name", cameraName);
    Logger.recordOutput(NTFolder + "Tag Info/Tag ID", tagID);
    Logger.recordOutput(NTFolder + "Tag Info/Tag Pose", tagPose);
  }

  @Override
  public void execute() {
    Pose2d pose;
    double xSpeed, ySpeed, rotValue;
    ChassisSpeeds outputChassisSpeeds;

    Logger.recordOutput(
        NTFolder + "Tag Info/Fiducial Tag ID",
        Robot.isReal()
            ? (int) LimelightHelpers.getFiducialID(cameraName)
            : vision.getClosestTagID(0));
    Logger.recordOutput(NTFolder + "Tag Info/LL Has Target", canSeeTagTrigger.getAsBoolean());

    if (canSeeTagTrigger.getAsBoolean()) {
      // get pose from LL
      if (Robot.isReal()) {
        double[] postions = LimelightHelpers.getBotPose_TargetSpace(cameraName);
        pose = new Pose2d(-postions[2], postions[0], new Rotation2d(Degrees.of(postions[4])));
      } else {
        Pose3d robotFeildPose = vision.getVisionPose(0);
        pose = vision.feildToTagRelative(robotFeildPose, tagPose);
      }

      xSpeed = xController.calculate(pose.getMeasureX().in(Meters));
      ySpeed = -yController.calculate(pose.getMeasureY().in(Meters));
      rotValue = rotController.calculate(pose.getRotation().getMeasure().in(Radians));
    } else {
      pose = new Pose2d();
      xSpeed = 0.0;
      ySpeed = 0.0;
      rotValue = 0.0;
    }

    // outputChassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, rotValue);
    outputChassisSpeeds = new ChassisSpeeds(ySpeed, xSpeed, rotValue);

    Logger.recordOutput(NTFolder + "Tag Relative/Robot Pose", pose);
    Logger.recordOutput(
        NTFolder + "Feild Relative/Robot Pose", vision.tagToFeildRelative(tagID, pose));

    Logger.recordOutput(NTFolder + "PID/X Output", xSpeed);
    Logger.recordOutput(NTFolder + "PID/X Setpoint", xController.getSetpoint());

    Logger.recordOutput(NTFolder + "PID/Y Output", ySpeed);
    Logger.recordOutput(NTFolder + "PID/Y Setpoint", yController.getSetpoint());

    Logger.recordOutput(NTFolder + "PID/Rot Value (Rads)", rotValue);
    Logger.recordOutput(NTFolder + "PID/Rot Setpoint (Rads)", rotController.getSetpoint());

    Logger.recordOutput(
        NTFolder + "LimeLight/Can See Tag Trigger", canSeeTagTrigger.getAsBoolean());
    Logger.recordOutput(NTFolder + "PID/At Pose Trigger", atPoseTrigger.getAsBoolean());

    drivebase.runVelocity(outputChassisSpeeds);
  }

  @Override
  public void end(boolean interrupted) {
    Logger.recordOutput(
        NTFolder + "LimeLight/Can See Tag Trigger", canSeeTagTrigger.getAsBoolean());
    Logger.recordOutput(NTFolder + "PID/At Pose Trigger", atPoseTrigger.getAsBoolean());

    ChassisSpeeds outputChassisSpeeds = new ChassisSpeeds(0, 0, 0);
    Logger.recordOutput(NTFolder + "Robot Info/ChassisSpeeds Output", outputChassisSpeeds);
    drivebase.runVelocity(outputChassisSpeeds);
  }

  @Override
  public boolean isFinished() {
    return atPoseTrigger.getAsBoolean();
    // || (int) LimelightHelpers.getFiducialID(cameraName) != tagID;
  }
}
