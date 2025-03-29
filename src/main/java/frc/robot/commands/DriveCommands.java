// Copyright 2021-2025 FRC 6328
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

package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Seconds;
import static frc.robot.Constants.DrivetrainConstants.kPathConstraints;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.ReefAlignConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.Vision;
import frc.robot.util.LimelightHelpers;
import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.Set;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class DriveCommands {
  private DriveCommands() {}

  private static Translation2d getLinearVelocityFromJoysticks(double x, double y) {
    // Apply deadband
    double linearMagnitude = Math.hypot(x, y);
    Rotation2d linearDirection = new Rotation2d(Math.atan2(y, x));

    // Return new linear velocity
    return new Pose2d(new Translation2d(), linearDirection)
        .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
        .getTranslation();
  }

  public static Command GoToPose(Pose2d targetPose, Set<Subsystem> drive) {
    return new DeferredCommand(
            () -> AutoBuilder.pathfindToPose(targetPose, kPathConstraints), drive)
        .withName("Go To Pose");
  }

  public static Command GoToPosePID(Pose2d targetPose, Supplier<Pose2d> robotPose, Drive drive) {
    PIDController xController = ReefAlignConstants.kTranslationXController;
    PIDController yController = ReefAlignConstants.kTranslationYController;
    PIDController rotController = ReefAlignConstants.kRotationController;
    Trigger endTrigger =
        new Trigger(
                () ->
                    xController.atSetpoint()
                        && yController.atSetpoint()
                        && rotController.atSetpoint())
            .debounce(ReefAlignConstants.kAtPoseDebounce.in(Seconds));

    return Commands.sequence(
            Commands.runOnce(
                () -> {
                  xController.setSetpoint(targetPose.getMeasureX().in(Meters));
                  xController.setTolerance(ReefAlignConstants.kSetpointTolerance.in(Meters));

                  yController.setSetpoint(targetPose.getMeasureY().in(Meters));
                  yController.setTolerance(ReefAlignConstants.kSetpointTolerance.in(Meters));

                  rotController.setSetpoint(targetPose.getRotation().getDegrees());
                  rotController.setTolerance(ReefAlignConstants.kSetpointRotTolerance.in(Degrees));
                  rotController.enableContinuousInput(-180, 180);
                }),
            Commands.run(
                    () -> {
                      double xSpeed =
                          xController.calculate(robotPose.get().getMeasureX().in(Meters));
                      double ySpeed =
                          yController.calculate(robotPose.get().getMeasureY().in(Meters));
                      double rotValue =
                          rotController.calculate(robotPose.get().getRotation().getDegrees());

                      Logger.recordOutput("Auto Align/PID/X Output", xSpeed);
                      Logger.recordOutput("Auto Align/PID/X Setpoint", xController.getSetpoint());
                      Logger.recordOutput("Auto Align/PID/X At Setpoint", xController.atSetpoint());

                      Logger.recordOutput("Auto Align/PID/Y Output", ySpeed);
                      Logger.recordOutput("Auto Align/PID/Y Setpoint", yController.getSetpoint());
                      Logger.recordOutput("Auto Align/PID/Y At Setpoint", yController.atSetpoint());

                      Logger.recordOutput(
                          "Auto Align/PID/Rot Value (Rads)", Degrees.of(rotValue).in(Radians));
                      Logger.recordOutput(
                          "Auto Align/PID/Rot Setpoint (Rads)",
                          Degrees.of(rotController.getSetpoint()).in(Radians));
                      Logger.recordOutput(
                          "Auto Align/PID/Rot At Setpoint", rotController.atSetpoint());

                      ChassisSpeeds speeds = new ChassisSpeeds(xSpeed, ySpeed, rotValue);

                      // Convert to field Relative Speeds
                      drive.runVelocity(
                          ChassisSpeeds.fromFieldRelativeSpeeds(speeds, drive.getRotation()));

                      //   drive.runVelocity(speeds);
                    })
                .until(endTrigger))
        .withName("Go To Pose PID");
  }

  public static Command GoToPath(PathPlannerPath targetPath, Set<Subsystem> drive) {
    return new DeferredCommand(
            () -> AutoBuilder.pathfindThenFollowPath(targetPath, kPathConstraints), drive)
        .withName("Go To Path");
  }

  public static Command AlignToReef(
      boolean isRightScore, String cameraName, Drive drive, Vision vision) {
    return new DeferredCommand(
            () -> {
              Pose2d TagPose =
                  vision
                      .convertLLPose(LimelightHelpers.getTargetPose3d_RobotSpace(cameraName))
                      .toPose2d();
              Pose2d tagPoseRotated =
                  TagPose.rotateAround(
                      new Translation2d(), drive.getPose().getRotation().unaryMinus());
              Pose2d tagPoseFieldRelative =
                  new Pose2d(
                      tagPoseRotated.getMeasureX().plus(drive.getPose().getMeasureX()),
                      tagPoseRotated.getMeasureY().times(-1).plus(drive.getPose().getMeasureY()),
                      new Rotation2d(
                          vision
                              .getTagPose((int) LimelightHelpers.getFiducialID(cameraName))
                              .getRotation()
                              .getMeasureZ()));
              Logger.recordOutput("Auto Align/Tag Pose (RR)", TagPose);
              Logger.recordOutput("Auto Align/Tag Pose Rotated (RR)", tagPoseRotated);
              Logger.recordOutput("Auto Align/Tag Pose (FF)", tagPoseFieldRelative);

              Pose2d TargetOffset =
                  isRightScore
                      ? ReefAlignConstants.kRightReefPose
                      : ReefAlignConstants.kLeftReefPose;
              Pose2d TargetOffsetRotated =
                  TargetOffset.rotateAround(
                      new Translation2d(), tagPoseFieldRelative.getRotation());
              Pose2d targetPose =
                  new Pose2d(
                      tagPoseFieldRelative.getMeasureX().plus(TargetOffsetRotated.getMeasureX()),
                      tagPoseFieldRelative.getMeasureY().plus(TargetOffsetRotated.getMeasureY()),
                      new Rotation2d(
                          tagPoseFieldRelative
                              .getRotation()
                              .getMeasure()
                              .plus(TargetOffset.getRotation().getMeasure())));
              Logger.recordOutput("Auto Align/Target Pose (FF)", targetPose);
              return GoToPosePID(targetPose, () -> drive.getPose(), drive);
            },
            Set.of(drive))
        .withName("Align To Reef");
  }

  public static Command RunVelocity(Drive drive, LinearVelocity velocity, double timeSec) {
    return Commands.deadline(
            Commands.waitSeconds(timeSec),
            Commands.run(
                () -> {
                  drive.runVelocity(
                      new ChassisSpeeds(velocity, MetersPerSecond.zero(), DegreesPerSecond.zero()));
                },
                drive))
        .withName("Run Velocity");
  }

  /**
   * Field relative drive command using two joysticks (controlling linear and angular velocities).
   */
  public static Command joystickDrive(
      Drive drive,
      Supplier<Double> xSupplier,
      Supplier<Double> ySupplier,
      Supplier<Double> omegaSupplier,
      Trigger interuptButton) {
    return Commands.run(
            () -> {
              // Get linear velocity
              Translation2d linearVelocity =
                  getLinearVelocityFromJoysticks(xSupplier.get(), ySupplier.get());

              // Apply rotation deadband
              double omega = omegaSupplier.get();

              // Convert to field relative speeds & send command
              ChassisSpeeds speeds =
                  new ChassisSpeeds(
                      linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                      linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                      omega * drive.getMaxAngularSpeedRadPerSec());
              boolean isFlipped =
                  DriverStation.getAlliance().isPresent()
                      && DriverStation.getAlliance().get() == Alliance.Red;
              drive.runVelocity(
                  ChassisSpeeds.fromFieldRelativeSpeeds(
                      speeds,
                      isFlipped
                          ? drive.getRotation().plus(new Rotation2d(Math.PI))
                          : drive.getRotation()));
            },
            drive)
        .onlyWhile(interuptButton.negate())
        .withName("Joystick Drive");
  }

  public static Command POVDrive(
      Drive drive,
      Supplier<Double> xSupplier,
      Supplier<Double> ySupplier,
      Supplier<Double> omegaSupplier) {
    return Commands.run(
            () -> {
              // Get linear velocity
              Translation2d linearVelocity =
                  getLinearVelocityFromJoysticks(xSupplier.get(), ySupplier.get());

              ChassisSpeeds speeds =
                  new ChassisSpeeds(
                      linearVelocity.getX()
                          * DrivetrainConstants.kPOVDriveSpeed.in(MetersPerSecond),
                      linearVelocity.getY()
                          * DrivetrainConstants.kPOVDriveSpeed.in(MetersPerSecond),
                      omegaSupplier.get() * drive.getMaxAngularSpeedRadPerSec());
              drive.runVelocity(speeds);
            },
            drive)
        .withName("POV Drive");
  }

  /**
   * Field relative drive command using joystick for linear control and PID for angular control.
   * Possible use cases include snapping to an angle, aiming at a vision target, or controlling
   * absolute rotation with a joystick.
   */
  public static Command joystickDriveAtAngle(
      Drive drive,
      Supplier<Double> xSupplier,
      Supplier<Double> ySupplier,
      Supplier<Rotation2d> rotationSupplier) {

    // Create PID controller
    ProfiledPIDController angleController = DrivetrainConstants.kRotationController;
    angleController.enableContinuousInput(-Math.PI, Math.PI);

    // Construct command
    return Commands.run(
            () -> {
              // Get linear velocity
              Translation2d linearVelocity =
                  getLinearVelocityFromJoysticks(xSupplier.get(), ySupplier.get());

              // Calculate angular speed
              double omega =
                  angleController.calculate(
                      drive.getRotation().getRadians(), rotationSupplier.get().getRadians());

              // Convert to field relative speeds & send command
              ChassisSpeeds speeds =
                  new ChassisSpeeds(
                      linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                      linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                      omega);
              boolean isFlipped =
                  DriverStation.getAlliance().isPresent()
                      && DriverStation.getAlliance().get() == Alliance.Red;
              drive.runVelocity(
                  ChassisSpeeds.fromFieldRelativeSpeeds(
                      speeds,
                      isFlipped
                          ? drive.getRotation().plus(new Rotation2d(Math.PI))
                          : drive.getRotation()));
            },
            drive)
        .withName("Joystick Drive At Angle")

        // Reset PID controller when command starts
        .beforeStarting(() -> angleController.reset(drive.getRotation().getRadians()));
  }

  /** Measures the robot's wheel radius by spinning in a circle. */
  public static Command wheelRadiusCharacterization(Drive drive) {
    SlewRateLimiter limiter = new SlewRateLimiter(DrivetrainConstants.kWheelRadiusRampRate);
    WheelRadiusCharacterizationState state = new WheelRadiusCharacterizationState();

    return Commands.parallel(
            // Drive control sequence
            Commands.sequence(
                // Reset acceleration limiter
                Commands.runOnce(
                    () -> {
                      limiter.reset(0.0);
                    }),

                // Turn in place, accelerating up to full speed
                Commands.run(
                    () -> {
                      double speed = limiter.calculate(DrivetrainConstants.kWheelRadiusMaxVelocity);
                      drive.runVelocity(new ChassisSpeeds(0.0, 0.0, speed));
                    },
                    drive)),

            // Measurement sequence
            Commands.sequence(
                // Wait for modules to fully orient before starting measurement
                Commands.waitSeconds(1.0),

                // Record starting measurement
                Commands.runOnce(
                    () -> {
                      state.positions = drive.getWheelRadiusCharacterizationPositions();
                      state.lastAngle = drive.getRotation();
                      state.gyroDelta = 0.0;
                    }),

                // Update gyro delta
                Commands.run(
                        () -> {
                          var rotation = drive.getRotation();
                          state.gyroDelta += Math.abs(rotation.minus(state.lastAngle).getRadians());
                          state.lastAngle = rotation;
                        })

                    // When cancelled, calculate and print results
                    .finallyDo(
                        () -> {
                          double[] positions = drive.getWheelRadiusCharacterizationPositions();
                          double wheelDelta = 0.0;
                          for (int i = 0; i < 4; i++) {
                            wheelDelta += Math.abs(positions[i] - state.positions[i]) / 4.0;
                          }
                          double wheelRadius =
                              (state.gyroDelta * Drive.DRIVE_BASE_RADIUS) / wheelDelta;

                          NumberFormat formatter = new DecimalFormat("#0.000000");
                          System.out.println(
                              "********** Wheel Radius Characterization Results **********");
                          System.out.println(
                              "\tWheel Delta: " + formatter.format(wheelDelta) + " radians");
                          System.out.println(
                              "\tGyro Delta: " + formatter.format(state.gyroDelta) + " radians");
                          System.out.println(
                              "\tWheel Radius: "
                                  + formatter.format(wheelRadius)
                                  + " meters, "
                                  + formatter.format(Units.metersToInches(wheelRadius))
                                  + " inches");
                        })))
        .withName("wheelRadiusCharacterization");
  }

  private static class WheelRadiusCharacterizationState {
    double[] positions = new double[4];
    Rotation2d lastAngle = new Rotation2d();
    double gyroDelta = 0.0;
  }
}
