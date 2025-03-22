// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
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
    rotController = new PIDController(5, 0, 0); // Rotation
    this.isRightScore = isRightScore;
    this.cameraName = cameraName;
    this.drivebase = drivebase;
    this.vision = vision;
    addRequirements(drivebase);

    canSeeTagTrigger = new Trigger(() -> vision.getTagCount() > 0);
    atPoseTrigger =
        new Trigger(
            () ->
                rotController.atSetpoint() && yController.atSetpoint() && xController.atSetpoint());
  }

  @Override
  public void initialize() {
    targetPose =
        isRightScore ? ReefAlignConstants.kRightReefPose : ReefAlignConstants.kLeftReefPose;

    rotController.setSetpoint(targetPose.getRotation().getRadians());
    rotController.setTolerance(ReefAlignConstants.kSetpointRotTolerance.in(Radians));
    rotController.enableContinuousInput(-Math.PI, Math.PI);

    xController.setSetpoint(targetPose.getMeasureX().in(Meters));
    xController.setTolerance(ReefAlignConstants.kSetpointTolerance.in(Meters));

    yController.setSetpoint(targetPose.getMeasureY().in(Meters));
    yController.setTolerance(ReefAlignConstants.kSetpointTolerance.in(Meters));

    tagID =
        (int)
            (Robot.isReal()
                ? LimelightHelpers.getFiducialID(cameraName)
                : vision.getClosestTagID(0));
    tagPose = vision.getTagPose(tagID);
  }

  @Override
  public void execute() {
    Logger.recordOutput(
        NTFolder + "Robot Info/Target Pose", vision.tagToFeildRelative(0, targetPose));
    Logger.recordOutput(NTFolder + "Robot Info/Is Right Side", isRightScore);
    Logger.recordOutput(NTFolder + "Tag Info/Camera Name", cameraName);
    Logger.recordOutput(NTFolder + "Tag Info/Tag ID", tagID);
    Logger.recordOutput(NTFolder + "Tag Info/Tag Pose", tagPose);

    double[] pose;
    double xSpeed, ySpeed, rotValue;
    ChassisSpeeds outputChassisSpeeds;

    if (canSeeTagTrigger.getAsBoolean()) {
      if (Robot.isReal()) {
        pose = LimelightHelpers.getBotPose_TargetSpace(cameraName);
        xSpeed = xController.calculate(pose[2]);
        ySpeed = -yController.calculate(pose[0]);
        rotValue = -rotController.calculate(pose[4]);
      } else {
        Pose3d robotFeildPose = vision.getVisionPose(0);
        Pose2d robotPoseTagRelative = vision.feildToTagRelative(robotFeildPose, tagPose);
        Logger.recordOutput(NTFolder + "Robot Info/Feild Relative Pose", robotFeildPose);
        Logger.recordOutput(NTFolder + "Robot Info/Tag Relative Pose", robotPoseTagRelative);
        pose = new double[0];
        xSpeed = xController.calculate(robotPoseTagRelative.getMeasureX().in(Meters));
        ySpeed = yController.calculate(robotPoseTagRelative.getMeasureY().in(Meters));
        Logger.recordOutput(
            NTFolder + "Rot Input", robotPoseTagRelative.getRotation().getRadians());
        rotValue = rotController.calculate(robotPoseTagRelative.getRotation().getRadians());
      }
    } else {
      pose = new double[0];
      xSpeed = 0.0;
      ySpeed = 0.0;
      rotValue = 0.0;
    }

    // outputChassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, rotValue);
    outputChassisSpeeds = new ChassisSpeeds(-ySpeed, xSpeed, rotValue);

    Logger.recordOutput(NTFolder + "Robot Info/ChassisSpeeds Output", outputChassisSpeeds);

    Logger.recordOutput(NTFolder + "LimeLight/positions", pose);

    Logger.recordOutput(NTFolder + "PID/X Speed", xSpeed);
    Logger.recordOutput(NTFolder + "PID/X Setpoint", xController.getSetpoint());

    Logger.recordOutput(NTFolder + "PID/Y Speed", ySpeed);
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
    ChassisSpeeds outputChassisSpeeds = new ChassisSpeeds(0, 0, 0);

    Logger.recordOutput(NTFolder + "Robot Info/ChassisSpeeds Output", outputChassisSpeeds);
    Logger.recordOutput(NTFolder + "LimeLight/positions", new double[0]);

    Logger.recordOutput(NTFolder + "PID/X Speed", 0.0);
    Logger.recordOutput(NTFolder + "PID/X Setpoint", 0.0);

    Logger.recordOutput(NTFolder + "PID/Y Speed", 0.0);
    Logger.recordOutput(NTFolder + "PID/Y Setpoint", 0.0);

    Logger.recordOutput(NTFolder + "PID/Rot Value", 0.0);
    Logger.recordOutput(NTFolder + "PID/Rot Setpoint", 0.0);

    Logger.recordOutput(
        NTFolder + "LimeLight/Can See Tag Trigger", canSeeTagTrigger.getAsBoolean());
    Logger.recordOutput(NTFolder + "PID/At Pose Trigger", atPoseTrigger.getAsBoolean());

    drivebase.runVelocity(outputChassisSpeeds);
  }

  @Override
  public boolean isFinished() {
    return atPoseTrigger.getAsBoolean();
  }
}
