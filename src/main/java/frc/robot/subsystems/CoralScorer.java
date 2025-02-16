// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Inches;
import static frc.robot.Constants.CoralScorerConstants.*;
import static frc.robot.Constants.SensorIDs.kScorerSensorID;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.Constants.CANIDs;
import utilities.TOFSensorSimple;
import utilities.TOFSensorSimple.TOFType;

public class CoralScorer extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */
  private TalonFX scorerMotor;

  private TalonFXConfiguration scoreMotorConfig;

  private TOFSensorSimple scorerSensor;

  boolean mode = false;

  public CoralScorer() {
    scorerMotor = new TalonFX(CANIDs.kScorerMotor, Constants.RIOName);
    scoreMotorConfig = new TalonFXConfiguration();
    scoreMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    scorerMotor.getConfigurator().apply(scoreMotorConfig);

    scorerSensor = new TOFSensorSimple(kScorerSensorID, Inches.of(2.5), TOFType.LASER_CAN);
  }

  public Trigger hasCoral() {
    return scorerSensor.beamBroken();
  }

  // Made a command to spin clockwise
  public Command scoreCommand() {
    return startEnd(() -> scorerMotor.set(-kSpeed), () -> scorerMotor.set(0));
  }

  public Command intakeCommand() {
    return startEnd(() -> scorerMotor.set(-kIntakeSpeed), () -> scorerMotor.set(0));
  }

  // Made a command to spin counter clockwise
  public Command reverseCommand() {
    return startEnd(() -> scorerMotor.set(kSpeed), () -> scorerMotor.set(0));
  }

  public Command alignScorerModeA() {
    return runOnce(() -> scorerMotor.set(kAlignSpeed))
        .until(scorerSensor.beamBroken())
        .andThen(() -> scorerMotor.set(-kAlignSpeed))
        .until(scorerSensor.beamBroken().negate());
  }

  public Command alignScorerModeB() {
    return runOnce(() -> scorerMotor.set(-kAlignSpeed))
        .until(scorerSensor.beamBroken())
        .andThen(() -> scorerMotor.set(kAlignSpeed))
        .until(scorerSensor.beamBroken().negate());
  }

  public Command toggleMode() {
    return runOnce(() -> mode = !mode);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("CoralScorer/Motor Output", scorerMotor.get());
  }
}
