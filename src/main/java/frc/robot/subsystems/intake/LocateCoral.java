// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import static frc.robot.Constants.IntakeConstants.kConveyorSpeed;
import static frc.robot.Constants.IntakeConstants.kPivotRetractAngle;
import static frc.robot.Constants.IntakeConstants.kcoralStation;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.IntakeConstants.CoralEnum;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class LocateCoral extends Command {

  private Supplier<CoralEnum> state;
  private IntakeSubsystem intakeSubsystem;
  private BooleanSupplier override_button; // TODO: Repurpose

  /** Creates a new LocateCoral. */
  public LocateCoral(
      Supplier<CoralEnum> state, IntakeSubsystem subsystem, BooleanSupplier override_button) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
    this.state = state;
    this.intakeSubsystem = subsystem;
    this.override_button = override_button;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intakeSubsystem.goToPivotPosition(kcoralStation);
    Logger.recordOutput("Locate Coral Running", true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch (state.get()) {
      case CORAL_TOO_CLOSE:
        intakeSubsystem.setIntakeMotor(IntakeConstants.kIntakeSpeed / 5);
        intakeSubsystem.setConveyerMotor(kConveyorSpeed);
        break;
      case CORAL_TOO_FAR:
        intakeSubsystem.setIntakeMotor(IntakeConstants.kIntakeSpeed / 5);
        intakeSubsystem.setConveyerMotor(-kConveyorSpeed);
        // intakeSubsystem.goToPivotPosition(kPivotRetractAngle);
        break;
      case IN_ELEVATOR:
      case NO_CORAL:
      case CORAL_ALIGNED:
        intakeSubsystem.setIntakeMotor(0);
        intakeSubsystem.setConveyerMotor(0);
        intakeSubsystem.goToPivotPosition(kPivotRetractAngle);
        break;
      case OTHER:
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intakeSubsystem.setIntakeMotor(0);
    intakeSubsystem.setConveyerMotor(0);
    intakeSubsystem.goToPivotPosition(kPivotRetractAngle);
    Logger.recordOutput("Locate Coral Running", false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return ((state.get() == CoralEnum.NO_CORAL || state.get() == CoralEnum.CORAL_ALIGNED)
            && intakeSubsystem.atSetpoint(kPivotRetractAngle))
        || override_button.getAsBoolean();
  }

  @Override
  public InterruptionBehavior getInterruptionBehavior() {
    if (state.get() == CoralEnum.CORAL_TOO_CLOSE || state.get() == CoralEnum.CORAL_TOO_FAR) {
      return InterruptionBehavior.kCancelIncoming;
    } else {
      return InterruptionBehavior.kCancelSelf;
    }
  }
}
