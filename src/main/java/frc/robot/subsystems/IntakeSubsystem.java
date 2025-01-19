// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Degrees;
import static frc.robot.Constants.IntakeConstants.*;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorIDConstants;
import utilities.HowdyPID;

import com.ctre.phoenix6.hardware.TalonFX;

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */
  private SparkMax intakeMotor;
  private TalonFX pivotMotor;
  private SparkMax conveyorMotor;
  private HowdyPID pivotPID;
  private Angle pivotSetpoint = kPivotRetractAngle;
  public IntakeSubsystem() {
    intakeMotor = new SparkMax(MotorIDConstants.kIntakeMotor, MotorType.kBrushless);
    pivotMotor = new TalonFX(MotorIDConstants.kPivotMotor);
    conveyorMotor = new SparkMax(MotorIDConstants.kConveyorMotor, MotorType.kBrushless);
    pivotPID = new HowdyPID(kPivotP, kPivotI, kPivotD);
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
  public Command IntakeCommand() {
    return startEnd(() -> setIntakeMotor(kIntakeSpeed), () -> setIntakeMotor(0));
  }

  // Made a command to spin counter clockwise
  public Command OuttakeCommand() {
    return startEnd(() -> setIntakeMotor(-kIntakeSpeed), () -> setIntakeMotor(0));
  }
  
  public double calculatePivotPID(double setpoint) {
    return pivotPID.getPIDController().calculate(pivotMotor.getPosition().getValueAsDouble(), setpoint);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    setPivotMotor(calculatePivotPID(pivotSetpoint.in(Degrees)));
    SmartDashboard.putNumber("Intake/Motor Ouput", intakeMotor.get());

  }
}