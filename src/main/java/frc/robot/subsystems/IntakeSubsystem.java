// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Rotations;
import static frc.robot.Constants.IntakeConstants.*;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CtreCanID;
import frc.robot.Constants.RevCanID;
import utilities.HowdyPID;
import utilities.TOFSensorSimple;

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */
  private SparkMax intakeMotor;

  private TalonFX pivotMotor;
  private SparkMax conveyorMotor;
  private HowdyPID pivotPID;
  private Angle pivotSetpoint = kPivotRetractAngle;

  private TOFSensorSimple sensor;

  public IntakeSubsystem() {
    intakeMotor = new SparkMax(RevCanID.kIntakeMotor, MotorType.kBrushless);
    pivotMotor = new TalonFX(CtreCanID.kPivotMotor);
    conveyorMotor = new SparkMax(RevCanID.kConveyorMotor, MotorType.kBrushless);
    pivotPID = new HowdyPID(kPivotP, kPivotI, kPivotD);
    sensor = new TOFSensorSimple(RevCanID.kConveyorSensor, Inches.of(1));
    pivotPID.getPIDController().setTolerance(kPivotTolerance.in(Rotations));
  }

  private void setPivotMotor(double speed) {
    pivotMotor.set(speed);
  }

  private void setConveyerMotor(double speed) {
    conveyorMotor.set(speed);
  }

  private void setIntakeMotor(double speed) {
    intakeMotor.set(speed);
  }

  public Angle getPivotPosition() {
    return pivotMotor.getPosition().getValue();
  }

  /** Pivot down */
  public Command extendPivotCommand() {
    return runOnce(() -> pivotSetpoint = kPivotExtendAngle);
  }

  /** Pivot up */
  public Command retractPivotCommand() {
    return runOnce(() -> pivotSetpoint = kPivotRetractAngle);
  }

  // Made a command to spin clockwise
  public Command intakeCommand() {
    return startEnd(() -> setIntakeMotor(kIntakeSpeed), () -> setIntakeMotor(0));
  }

  // Made a command to spin counter clockwise
  public Command outtakeCommand() {
    return startEnd(() -> setIntakeMotor(-kIntakeSpeed), () -> setIntakeMotor(0));
  }

  /** Pivots down when user presses button */
  public Command pivotDownCommand() {
    return startEnd(() -> setPivotMotor(-kPivotSpeed), () -> setPivotMotor(0));
  }

  /** Pivots up when user presses button */
  public Command pivotUpCommand() {
    return startEnd(() -> setPivotMotor(kPivotSpeed), () -> setPivotMotor(0));
  }

  /** Pushes game piece from conveyor into birdhouse */
  public Command conveyorFeed() {
    return startEnd(() -> setConveyerMotor(-kConveyorSpeed), () -> setConveyerMotor(0));
  }

  public Command conveyorEject() {
    return startEnd(() -> setConveyerMotor(kConveyorSpeed), () -> setConveyerMotor(0));
  }

  public Command intakeToBirdhousePhase1() {
    return startEnd(
            () -> {
              extendPivotCommand().initialize();
              intakeCommand().initialize();
              conveyorFeed().initialize();
            },
            () -> {
              extendPivotCommand().end(false);
              intakeCommand().end(false);
              conveyorFeed().end(false);
            })
        .until(sensor.beamBroken());
  }

  public Command intakeToBirdhousePhase2() {
    return retractPivotCommand()
        .andThen(
            Commands.waitUntil(
                pivotPID.getPIDController()::atSetpoint)) // FIXME: Fix debouncing if neccesary
        .andThen(conveyorFeed().until(sensor.beamBroken().negate()));
  }

  public Command intakeToBirdhouse() {
    return intakeToBirdhousePhase1()
        .andThen(intakeToBirdhousePhase2())
        .withName("Intake Phase 1 and 2");
  }

  public Command ejectFromBirdhouse() {
    return conveyorEject()
        .until(sensor.beamBroken().negate())
        .andThen(extendPivotCommand())
        .andThen(Commands.waitSeconds(0.5))
        .andThen(outtakeCommand());
  }

  public double calculatePivotPID() {
    return pivotPID
        .getPIDController()
        .calculate(getPivotPosition().in(Rotations), pivotSetpoint.in(Rotations));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    setPivotMotor(calculatePivotPID());
    SmartDashboard.putNumber("Intake Motor Output", intakeMotor.get());
    SmartDashboard.putNumber("Pivot Motor Output", pivotMotor.get());
    SmartDashboard.putNumber("Conveyor Motor Output", conveyorMotor.get());
    SmartDashboard.putNumber("Pivot Setpoint", pivotSetpoint.in(Rotations));
  }

  public void simulationPeriodic() {}
}
