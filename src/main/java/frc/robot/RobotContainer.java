// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.CoralScorer;
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
          .withDriveRequestType(
              DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

  private final Telemetry logger = new Telemetry(MaxSpeed);

  public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
  private final Elevator elevator = new Elevator();

  private final IntakeSubsystem intake = new IntakeSubsystem();
  private final CoralScorer coralScorer = new CoralScorer();

  private boolean precisionMode = false;

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    OI.getButton(OI.Driver.X).onTrue(elevator.L0());
    OI.getButton(OI.Driver.Back).onTrue(elevator.L1());
    OI.getButton(OI.Driver.A).onTrue(elevator.L2());
    OI.getButton(OI.Driver.B).onTrue(elevator.L3());
    OI.getButton(OI.Driver.Y).onTrue(elevator.L4());
    OI.getPOVButton(OI.Driver.POV90).whileTrue(elevator.goUp());
    OI.getPOVButton(OI.Driver.POV270).whileTrue(elevator.goDown());
    
    OI.getButton(OI.Driver.Start).onTrue(elevator.zeroMotorEncoder());
    OI.getButton(OI.Driver.RSB).onTrue(new InstantCommand(() -> precisionMode = !precisionMode));

    // Note that X is defined as forward according to WPILib convention,
    // and Y is defined as to the left according to WPILib convention.
    drivetrain.setDefaultCommand(
        // Drivetrain will execute this command periodically
        new ConditionalCommand(
            drivetrain.applyRequest(
                () ->
                    drive
                        .withVelocityX(
                            -OI.getAxisSupplier(OI.Driver.LeftPrecisionY).get()
                                * MaxSpeed) // Drive forward with negative Y (forward)
                        .withVelocityY(
                            OI.getAxisSupplier(OI.Driver.LeftPrecisionX).get()
                                * MaxSpeed) // Drive left with negative X (left)
                        .withRotationalRate(
                            OI.getAxisSupplier(OI.Driver.RightPrecisionX).get()
                                * MaxAngularRate) // Drive counterclockwise with negative X (left)
                ),
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
                ),
            () -> precisionMode));

    // Run SysId routines when holding back/start and X/Y.
    // Note that each routine should be run exactly once in a single log.
    OI.getButton(OI.Operator.Back)
        .and(OI.getButton(OI.Operator.Y))
        .whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
    OI.getButton(OI.Operator.Back)
        .and(OI.getButton(OI.Operator.X))
        .whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
    OI.getButton(OI.Operator.Start)
        .and(OI.getButton(OI.Operator.Y))
        .whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
    OI.getButton(OI.Operator.Start)
        .and(OI.getButton(OI.Operator.X))
        .whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

    // reset the field-centric heading on left bumper press
    OI.getButton(OI.Driver.Start).onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

    drivetrain.registerTelemetry(logger::telemeterize);

    // Set the intake rollers to the left and right triggers
    OI.getTrigger(OI.Driver.RTrigger).whileTrue(intake.IntakeCommand());
    OI.getButton(OI.Driver.RBumper).whileTrue(intake.OuttakeCommand());
    OI.getTrigger(OI.Driver.LTrigger).whileTrue(coralScorer.scoreClockWise());
    OI.getButton(OI.Driver.LBumper).whileTrue(coralScorer.scoreCounterClockWise());

    /**
     * TODO Controls: OI.getButton(OI.Driver.LeftStick).whileTrue(autoAlign.AutoAlign());
     * OI.getButton(OI.Driver.RightStick).whileTrue(gearAlign.GearAlign());
     * OI.getButton(OI.Driver.POV180).whileTrue(drive.DriveMode());
     * OI.getButton(OI.Operator.LTrigger).whileTrue(climb.Climb());
     * OI.getButton(OI.Operator.RTrigger).whileTrue(climb.UnClimb());
     * OI.getButton(OI.Operator.RBumper).whileTrue(climb.EnableClimber());
     */
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
