// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.IntakeSubsystem;

public class RobotContainer {
  private double MaxSpeed =
      TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
  private double MaxAngularRate =
      RotationsPerSecond.of(0.75)
          .in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final SwerveRequest.FieldCentric drive =
      new SwerveRequest.FieldCentric()
          .withDeadband(MaxSpeed * 0.1)
          .withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
          .withDriveRequestType(
              DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

  private final Telemetry logger = new Telemetry(MaxSpeed);

  private final CommandXboxController m_driverController = new CommandXboxController(0);

  public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
  private final Elevator elevator = new Elevator();

  private final IntakeSubsystem intake = new IntakeSubsystem();

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    m_driverController.rightBumper().and(m_driverController.leftBumper()).onTrue(elevator.L0());
    m_driverController.rightBumper().and(m_driverController.a()).onTrue(elevator.L1());
    m_driverController.rightBumper().and(m_driverController.b()).onTrue(elevator.L2());
    m_driverController.rightBumper().and(m_driverController.x()).onTrue(elevator.L3());
    m_driverController.rightBumper().and(m_driverController.y()).onTrue(elevator.L4());
    m_driverController
        .b()
        .and(m_driverController.rightBumper().negate())
        .whileTrue(elevator.goUp());
    m_driverController
        .a()
        .and(m_driverController.rightBumper().negate())
        .whileTrue(elevator.goDown());
    m_driverController.start().onTrue(elevator.zeroMotorEncoder());
    // Note that X is defined as forward according to WPILib convention,
    // and Y is defined as to the left according to WPILib convention.
    drivetrain.setDefaultCommand(
        // Drivetrain will execute this command periodically
        drivetrain.applyRequest(
            () ->
                drive
                    .withVelocityX(
                        -m_driverController.getLeftY()
                            * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(
                        -m_driverController.getLeftX()
                            * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(
                        -m_driverController.getRightX()
                            * MaxAngularRate) // Drive counterclockwise with negative X (left)
            ));

    m_driverController.a().whileTrue(drivetrain.applyRequest(() -> brake));
    m_driverController
        .b()
        .whileTrue(
            drivetrain.applyRequest(
                () ->
                    point.withModuleDirection(
                        new Rotation2d(
                            -m_driverController.getLeftY(), -m_driverController.getLeftX()))));

    // Run SysId routines when holding back/start and X/Y.
    // Note that each routine should be run exactly once in a single log.
    m_driverController
        .back()
        .and(m_driverController.y())
        .whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
    m_driverController
        .back()
        .and(m_driverController.x())
        .whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
    m_driverController
        .start()
        .and(m_driverController.y())
        .whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
    m_driverController
        .start()
        .and(m_driverController.x())
        .whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

    // reset the field-centric heading on left bumper press
    m_driverController.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

    drivetrain.registerTelemetry(logger::telemeterize);
    // Set the intake rollers to the left and right triggers
    m_driverController.leftTrigger().whileTrue(intake.spinClockWise());
    m_driverController.rightTrigger().whileTrue(intake.spinCounterClockWise());
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
