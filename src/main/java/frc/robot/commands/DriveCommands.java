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

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static frc.robot.Constants.DrivetrainConstants.kPathConstraints;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
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
import frc.robot.subsystems.vision.VisionConstants;
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

  public static Command GoToPath(PathPlannerPath targetPath, Set<Subsystem> drive) {
    return new DeferredCommand(
            () -> AutoBuilder.pathfindThenFollowPath(targetPath, kPathConstraints), drive)
        .withName("Go To Path");
  }

  public static Command AlignToReef(boolean isRightScore, Drive drive, Vision vision) {
    return new DeferredCommand(
            () -> {
              Pose2d TagPose =
                  vision
                      .convertLLPose(
                          LimelightHelpers.getTargetPose3d_RobotSpace(VisionConstants.camera0Name))
                      .toPose2d();
              Pose2d TagRelativeTargetPose =
                  isRightScore
                      ? ReefAlignConstants.kRightReefPose
                      : ReefAlignConstants.kLeftReefPose;
              Pose2d targetPose =
                  TagPose.transformBy(
                      new Transform2d(
                          TagRelativeTargetPose.getTranslation(),
                          new Rotation2d(
                              TagRelativeTargetPose.getRotation().getMeasure().times(-1))));
              Pose2d fieldRelativePose = vision.robotToFeildRelative(drive.getPose(), targetPose);

              Logger.recordOutput("Auto Align/Target Pose", targetPose);
              Logger.recordOutput("Auto Align/fieldRelativePose", fieldRelativePose);
              Logger.recordOutput("Auto Align/Testing", true);
              return AutoBuilder.pathfindToPose(fieldRelativePose, kPathConstraints, 0.0);
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
