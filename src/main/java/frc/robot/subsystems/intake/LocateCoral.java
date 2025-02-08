// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import static frc.robot.Constants.IntakeConstants.kConveyorSpeed;
import static frc.robot.Constants.IntakeConstants.kIntakeSpeed;
import static frc.robot.Constants.IntakeConstants.kPivotRetractAngle;
import static frc.robot.Constants.IntakeConstants.kcoralStation;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants.CoralEnum;
import frc.robot.subsystems.CoralScorer;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class LocateCoral extends Command {

  private Supplier<CoralEnum> state;
  private IntakeSubsystem intakeSubsystem;
  private BooleanSupplier elevatorNotL1;
  private CoralScorer coralScorer;

  /** Creates a new LocateCoral. */
  public LocateCoral(
      Supplier<CoralEnum> state,
      IntakeSubsystem subsystem,
      BooleanSupplier elevatorNotL1,
      CoralScorer coralScorer) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
    this.state = state;
    this.intakeSubsystem = subsystem;
    this.elevatorNotL1 = elevatorNotL1;
    this.coralScorer = coralScorer;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intakeSubsystem.goToPivotPosition(kcoralStation);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch (state.get()) {
      case DONE:
        intakeSubsystem.setIntakeMotor(0);
        intakeSubsystem.setConveyerMotor(0);
        intakeSubsystem.goToPivotPosition(kPivotRetractAngle);
        break;
      case CORAL_TOO_CLOSE:
        intakeSubsystem.setIntakeMotor(kIntakeSpeed / 5);
        intakeSubsystem.setConveyerMotor(-kConveyorSpeed);
        break;
      case CORAL_TOO_FAR:
        intakeSubsystem.setIntakeMotor(kIntakeSpeed / 5);
        intakeSubsystem.setConveyerMotor(kConveyorSpeed);
        break;
      case IN_ELEVATOR:
      case NO_CORAL:
        intakeSubsystem.setIntakeMotor(0);
        intakeSubsystem.setConveyerMotor(0);
        intakeSubsystem.goToPivotPosition(kPivotRetractAngle);
        break;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intakeSubsystem.setIntakeMotor(0);
    intakeSubsystem.setConveyerMotor(0);
    intakeSubsystem.goToPivotPosition(kPivotRetractAngle);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return state.get() == CoralEnum.DONE && intakeSubsystem.atSetpoint(kPivotRetractAngle);
  }
}
