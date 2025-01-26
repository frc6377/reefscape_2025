// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.MotorIDConstants;
import frc.robot.Robot;

public class Climber extends SubsystemBase {
  /** Creates a new Climber. */
  private TalonFX climberMotorLeader;

  private TalonFX climberMotorFollower;
  private PIDController climberPID;
  private DCMotor simClimberGearbox;
  private Mechanism2d simClimberArm;
  private SingleJointedArmSim climberSim;
  private Mechanism2d climbMech;
  private MechanismRoot2d climbMechRoot;
  private MechanismLigament2d climbMechLigament;
  private Slot0Configs climberConfigsToClimber;
  private Slot1Configs climberConfigsAtClimber;

  public Climber() {
    climberMotorLeader = new TalonFX(MotorIDConstants.kCLimberMotorLeader);
    climberMotorFollower = new TalonFX(MotorIDConstants.kCLimberMotorFollower);
    climberMotorFollower.setControl(new Follower(MotorIDConstants.kCLimberMotorLeader, true));
    climberConfigsToClimber =
        new Slot0Configs()
            .withKP(ClimberConstants.kClimberP0)
            .withKI(ClimberConstants.kClimberI0)
            .withKD(ClimberConstants.kClimberD0)
            .withKG(ClimberConstants.kClimberkG0)
            .withKV(ClimberConstants.kClimberkV0);
    climberConfigsAtClimber =
        new Slot1Configs()
            .withKP(ClimberConstants.kClimberP1)
            .withKI(ClimberConstants.kClimberI1)
            .withKD(ClimberConstants.kClimberD1)
            .withKG(ClimberConstants.kClimberkG1)
            .withKV(ClimberConstants.kClimberkV1);
    climberMotorLeader.getConfigurator().apply(climberConfigsToClimber);
    climberMotorFollower.getConfigurator().apply(climberConfigsToClimber);

    if (Robot.isSimulation()) {
      simClimberGearbox = DCMotor.getKrakenX60(1);
      climberSim =
          new SingleJointedArmSim(simClimberGearbox, 126, 0.077, 0.1, 0, Math.PI, false, 1);
      climbMech = new Mechanism2d(2, 2);
      climbMechRoot = climbMech.getRoot("Climb Mech root", 1, 1);
      climbMechLigament =
          climbMechRoot.append(
              new MechanismLigament2d(
                  "Climb Mech Ligament", 1, Radians.of(climberSim.getAngleRads()).in(Degrees)));
      SmartDashboard.putData("Climb Mech", climbMech);
    }
  }

  public Command climb() {
    return run(
        () -> {
          climberMotorLeader.setControl(
              new PositionDutyCycle(
                  ClimberConstants.kClimberExtended.in(Rotations) * ClimberConstants.KGearRatio));
          SmartDashboard.putNumber("Target Angle", ClimberConstants.kClimberExtended.in(Degrees));
        });
  }

  public Command retract() {
    return run(
        () -> {
          climberMotorLeader.setControl(
              new PositionDutyCycle(
                  ClimberConstants.kClimberRetracted.in(Rotations) * ClimberConstants.KGearRatio));
          SmartDashboard.putNumber("Target Angle", ClimberConstants.kClimberRetracted.in(Degrees));
        });
  }

  @Override
  public void simulationPeriodic() {
    if (Robot.isSimulation()) {
      climberSim.setInput(climberMotorLeader.get());
      climberSim.update(0.02);
      climbMechLigament.setAngle(Radians.of(climberSim.getAngleRads()).in(Degrees));
      SmartDashboard.putData("Climb Mech", climbMech);
      SmartDashboard.putNumber("Climber Angle", climberSim.getAngleRads());
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
