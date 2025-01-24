// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Rotations;
import static frc.robot.Constants.IntakeConstants.*;

import com.ctre.phoenix6.hardware.TalonFX;
import com.google.flatbuffers.Constants;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
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

  public Angle getPivotPositionCommand() {
    return pivotMotor.getPosition().getValue();
  }

  public Command extendPivotCommand() {
    return runOnce(() -> pivotSetpoint = kPivotExtendAngle);
  }

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

  public Command pivotDownCommand() {
    return startEnd(() -> setPivotMotor(-kPivotSpeed), () -> setPivotMotor(0));
  }
  
  public Command pivotUpCommand() {
    return startEnd(() -> setPivotMotor(kPivotSpeed), () -> setPivotMotor(0));
  }

  /**
   * Pushes game piece from conveyor into birdhouse
   */
  public Command conveyorFeed() {
    return startEnd(() -> setConveyerMotor(-kConveyorSpeed), null);
  }

  public Command conveyorEject() {
    return startEnd(() -> setConveyerMotor(kConveyorSpeed), () -> setConveyerMotor(0));
  }


  public Command intakeToBirdhouse() {
    return pivotDownCommand()
            .alongWith(intakeCommand())
            .alongWith(conveyorFeed())
            .until(sensor.beamBroken())
            .andThen(pivotUpCommand())
            .andThen(conveyorFeed())
            .until(sensor.beamBroken().negate());
  }
  
  public Command ejectFromBirdhouse(){
    return conveyorEject()
            .alongWith(outtakeCommand());
  }
  
  public double calculatePivotPID() {
    return pivotPID
        .getPIDController()
        .calculate(pivotMotor.getPosition().getValueAsDouble(), pivotSetpoint.in(Rotations));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    setPivotMotor(calculatePivotPID());
    SmartDashboard.putNumber("Intake/Motor Ouput", intakeMotor.get());
  }
}
