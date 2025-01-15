// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import static frc.robot.Constants.IntakeConstants.*;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.MotorIDConstants;
import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */
  private SparkMax intakeMotor;

  private IntakeSimulation intakeSim;

  @SuppressWarnings("unused")
  private SwerveDriveSimulation driveSimulation;

  public IntakeSubsystem() {
    intakeMotor = new SparkMax(MotorIDConstants.kIntakeMotor, MotorType.kBrushed);
  }

  public IntakeSubsystem(SwerveDriveSimulation driveSimulation) {
    intakeMotor = new SparkMax(MotorIDConstants.kIntakeMotor, MotorType.kBrushed);

    this.driveSimulation = driveSimulation;
    intakeSim =
        IntakeSimulation.OverTheBumperIntake(
            "Coral",
            driveSimulation,
            IntakeConstants.kIntakeWidth,
            IntakeConstants.kIntakeExtension,
            IntakeSimulation.IntakeSide.FRONT,
            IntakeConstants.kIntakeCapacity);
  }

  public boolean GetPieceFromIntake() {
    return intakeSim.obtainGamePieceFromIntake();
  }

  // Made a command to spin clockwise
  public Command IntakeCommand() {
    return startEnd(
        () -> {
          intakeMotor.set(kSpeed);
          if (intakeSim != null) {
            intakeSim.startIntake();
          }
        },
        () -> {
          intakeMotor.set(0);
          if (intakeSim != null) {
            intakeSim.stopIntake();
          }
        });
  }

  // Made a command to spin counter clockwise
  public Command OuttakeCommand() {
    return startEnd(
        () -> {
          intakeMotor.set(-kSpeed);
        },
        () -> {
          intakeMotor.set(0);
        });
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Intake/Motor Ouput", intakeMotor.get());
  }
}
