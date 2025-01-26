// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator;

// import frc.robot.subsystems.IntakeSubsystem;

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

  public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
  private final Elevator elevator = new Elevator();

  // private final IntakeSubsystem intake = new IntakeSubsystem();

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    if (Robot.isSimulation()) {
      OI.getButton(OI.Keyboard.Z).onTrue(elevator.L0());
      OI.getButton(OI.Keyboard.X).onTrue(elevator.L1());
      OI.getButton(OI.Keyboard.M).whileTrue(elevator.goUp());
      OI.getButton(OI.Keyboard.Comma).whileTrue(elevator.goDown());
    } else {
      OI.getButton(OI.Driver.RBumper).and(OI.getButton(OI.Driver.LBumper)).onTrue(elevator.L0());
      OI.getButton(OI.Driver.RBumper).and(OI.getButton(OI.Driver.X)).onTrue(elevator.L1());
      OI.getButton(OI.Driver.RBumper).and(OI.getButton(OI.Driver.A)).onTrue(elevator.L2());
      OI.getButton(OI.Driver.RBumper).and(OI.getButton(OI.Driver.B)).onTrue(elevator.L3());
      OI.getButton(OI.Driver.RBumper).and(OI.getButton(OI.Driver.Y)).onTrue(elevator.L4());
      OI.getButton(OI.Driver.RBumper)
          .and(OI.getPOVButton(OI.Driver.DPAD_UP))
          .whileTrue(elevator.goUp());
      OI.getButton(OI.Driver.RBumper)
          .and(OI.getPOVButton(OI.Driver.DPAD_DOWN))
          .whileTrue(elevator.goDown());
      OI.getButton(OI.Driver.Start).onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

      // Note that X is defined as forward according to WPILib convention,
      // and Y is defined as to the left according to WPILib convention.
      drivetrain.setDefaultCommand(
          // Drivetrain will execute this command periodically
          drivetrain.applyRequest(
              () ->
                  drive
                      .withVelocityX(
                          -OI.getAxisSupplier(OI.Driver.LeftY).get()
                              * MaxSpeed) // Drive forward with negative Y (forward)
                      .withVelocityY(
                          OI.getAxisSupplier(OI.Driver.LeftX).get()
                              * MaxSpeed) // Drive left with negative X (left)
                      .withRotationalRate(
                          OI.getAxisSupplier(OI.Driver.RightX).get()
                              * MaxAngularRate) // Drive counterclockwise with negative X (left)
              ));

      // Run SysId routines when holding back/start and X/Y.
      // Note that each routine should be run exactly once in a single log.
      OI.getButton(OI.Driver.Back)
          .and(OI.getButton(OI.Driver.Y))
          .whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
      OI.getButton(OI.Driver.Back)
          .and(OI.getButton(OI.Driver.X))
          .whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
      OI.getButton(OI.Driver.Start)
          .and(OI.getButton(OI.Driver.Y))
          .whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
      OI.getButton(OI.Driver.Start)
          .and(OI.getButton(OI.Driver.X))
          .whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

      // reset the field-centric heading on left bumper press
      OI.getButton(OI.Driver.LBumper)
          .onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
    }

    drivetrain.registerTelemetry(logger::telemeterize);

    // Set the intake rollers to the left and right triggers
    /*OI.getPOVButton(OI.Driver.POV180)
        .and(OI.getButton(OI.Driver.RBumper).negate())
        .whileTrue(intake.IntakeCommand());
    OI.getPOVButton(OI.Driver.POV0)
        .and(OI.getButton(OI.Driver.RBumper).negate())
        .whileTrue(intake.OuttakeCommand());*/

    SmartDashboard.putData(elevator.limitHit());
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
