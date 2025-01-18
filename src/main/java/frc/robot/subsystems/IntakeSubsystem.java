// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.IntakeConstants.*;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorIDConstants;
import com.ctre.phoenix6.hardware.TalonFX;
// import 

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */
  private SparkMax intakeMotor;
  private TalonFX pivotMotor;
  private SparkMax conveyorMotor;
  private Encoder pivotEncoder;
  public IntakeSubsystem() {
    intakeMotor = new SparkMax(MotorIDConstants.kIntakeMotor, MotorType.kBrushless);
    pivotMotor = new TalonFX(MotorIDConstants.kPivotMotor);
    conveyorMotor = new SparkMax(MotorIDConstants.kConveyorMotor, MotorType.kBrushless);
    pivotEncoder = new Encoder(0, 1);
  }
  public Command setPivotMotor(double speed) {
    return run(() -> pivotMotor.set(speed));
  }
  public Command setConveyerCommand(double speed) {
    return run(() -> conveyorMotor.set(speed));
  }
  
  public Command setIntakeCommand(double speed) {
    return run(() -> intakeMotor.set(speed));
  }
  public double getPivotPositionCommand() {
    return pivotEncoder.get();
  }
  public Command extendPivotCommand() {
    return startEnd(() -> setPivotMotor(kPivotSpeed), () -> setPivotMotor(0));
  }
  
  public Command retractPivotCommand() {
    return startEnd(() -> setPivotMotor(-kPivotSpeed), () -> setPivotMotor(0));
  }
  
  // Made a command to spin clockwise
  public Command IntakeCommand() {
    return startEnd(() -> setIntakeCommand(kIntakeSpeed), () -> setIntakeCommand(0));
  }

  // Made a command to spin counter clockwise
  public Command OuttakeCommand() {
    return startEnd(() -> setIntakeCommand(-kIntakeSpeed), () -> setIntakeCommand(0));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Intake/Motor Ouput", intakeMotor.get());
  }
}
