// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import static frc.robot.Constants.IntakeConstants.kConveyorSpeed;
import static frc.robot.Constants.IntakeConstants.kIntakeHandoffSpeed;
import static frc.robot.Constants.IntakeConstants.kPivotRetractAngle;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants.AlignMode;
import frc.robot.subsystems.CoralScorer;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class PassToScorer extends Command {

  private IntakeSubsystem intakeSubsystem;
  private BooleanSupplier elevatorNotL1;
  private CoralScorer coralScorer;

  /** Creates a new PassToScorer. */
  public PassToScorer(
      IntakeSubsystem subsystem, BooleanSupplier elevatorNotL1, CoralScorer coralScorer) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
    addRequirements(coralScorer);
    this.intakeSubsystem = subsystem;
    this.elevatorNotL1 = elevatorNotL1;
    this.coralScorer = coralScorer;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intakeSubsystem.goToPivotPosition(kPivotRetractAngle);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (elevatorNotL1.getAsBoolean() && intakeSubsystem.atSetpoint(kPivotRetractAngle)) {
      intakeSubsystem.setIntakeMotor(kIntakeHandoffSpeed);
      intakeSubsystem.setConveyerMotor(-kConveyorSpeed);
      coralScorer.setAlignMode(
          AlignMode.INWARD_ALIGN); // FIXME: CHange to outward if it doesn't work
      coralScorer.scoreCommand().initialize();
    } else {
      intakeSubsystem.goToPivotPosition(kPivotRetractAngle);
      intakeSubsystem.setIntakeMotor(0);
      intakeSubsystem.setConveyerMotor(0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intakeSubsystem.goToPivotPosition(kPivotRetractAngle);
    intakeSubsystem.setIntakeMotor(0);
    intakeSubsystem.setConveyerMotor(0);
    coralScorer.scoreCommand().end(interrupted);
    Logger.recordOutput("Pass To Scorer Ended", true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (elevatorNotL1.getAsBoolean()) {
      boolean intakeSensorBool =
          intakeSubsystem.getSensors().getSensorTrigger(2).negate().debounce(0.04).getAsBoolean();
      Logger.recordOutput("PassToScorer Intake State", intakeSensorBool);
      Logger.recordOutput("PassToScorer Coral State", coralScorer.hasCoral().getAsBoolean());
      return coralScorer.hasCoral().getAsBoolean() || intakeSensorBool;
    } else {
      return intakeSubsystem.atSetpoint(kPivotRetractAngle);
    }
  }
}
