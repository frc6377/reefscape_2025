// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.MotorIDConstants;

public class Climber extends SubsystemBase {
  /** Creates a new Climber. */
  private TalonFX climberMotorLeader;

  private TalonFX climberMotorFollower;
  private PIDController climberPID;

  public Climber() {
    climberMotorLeader = new TalonFX(MotorIDConstants.kCLimberMotorLeader);
    climberMotorFollower = new TalonFX(MotorIDConstants.kCLimberMotorFollower);
    climberMotorFollower.setControl(new Follower(MotorIDConstants.kCLimberMotorLeader, true));
    climberPID =
        new PIDController(
            ClimberConstants.kClimberP, ClimberConstants.kClimberI, ClimberConstants.kClimberD);
  }

  private double calcPID(Angle target) {
    return climberPID.calculate(
        climberMotorLeader.getPosition().getValue().in(Rotations), target.in(Rotations));
  }

  public Command climb() {
    return run(() -> climberMotorLeader.set(calcPID(ClimberConstants.kClimberExtended)));
  }

  public Command retract() {
    return run(() -> climberMotorLeader.set(calcPID(ClimberConstants.kClimberRetracted)));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
