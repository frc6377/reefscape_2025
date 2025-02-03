// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.CoralScorerConstants.*;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.CANIDs;

public class CoralScorer extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */
  private TalonFX scorerMotor;

  public CoralScorer() {
    scorerMotor = new TalonFX(CANIDs.kScorerMotor, Constants.RIOName);
  }

  // Made a command to spin clockwise
  public Command scoreClockWise() {
    return startEnd(() -> scorerMotor.set(-kSpeed), () -> scorerMotor.set(0));
  }

  // Made a command to spin counter clockwise
  public Command scoreCounterClockWise() {
    return startEnd(() -> scorerMotor.set(kSpeed), () -> scorerMotor.set(0));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("CoralScorer/Motor Output", scorerMotor.get());
  }
}
