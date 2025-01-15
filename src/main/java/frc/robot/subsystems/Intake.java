// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.IntakeConstants.*;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorIDConstants;

public class Intake extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */
  private SparkMax intakeMotor;

  public Intake() {
    intakeMotor = new SparkMax(MotorIDConstants.kIntakeMotor, MotorType.kBrushless);
  }

  // Made a command to spin clockwise
  public Command spinClockWise() {
    return startEnd(() -> intakeMotor.set(kSpeed), () -> intakeMotor.set(0));
  }

  // Made a command to spin counter clockwise
  public Command spinCounterClockWise() {
    return startEnd(() -> intakeMotor.set(-kSpeed), () -> intakeMotor.set(0));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
