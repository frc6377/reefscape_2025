// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.IntakeConstants.*;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.MotorIDConstants;
import frc.robot.Constants.SimulatedMechPoses;
import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.Logger;

@SuppressWarnings("unused")
public class TempIntake extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */
  private SparkMax intakeMotor;

  private IntakeSimulation intakeSim;

  private SwerveDriveSimulation driveSimulation;

  private Pose3d intakeSimPose;

  public TempIntake() {
    intakeMotor = new SparkMax(MotorIDConstants.kIntakeMotor, MotorType.kBrushless);
  }

  public TempIntake(SwerveDriveSimulation driveSimulation) {
    intakeMotor = new SparkMax(MotorIDConstants.kIntakeMotor, MotorType.kBrushless);

    this.driveSimulation = driveSimulation;

    intakeSim =
        IntakeSimulation.OverTheBumperIntake(
            "Coral",
            driveSimulation,
            IntakeConstants.kIntakeWidth,
            IntakeConstants.kIntakeExtension,
            IntakeSimulation.IntakeSide.FRONT,
            IntakeConstants.kIntakeCapacity);

    intakeSimPose = SimulatedMechPoses.kIntakeStartPose;

    Logger.recordOutput("Odometry/Mech Poses/Intake Pose", SimulatedMechPoses.kIntakeStartPose);
    // Temp until we have real climb code
    Logger.recordOutput("Odometry/Mech Poses/Climber 1 Pose", SimulatedMechPoses.kClimber1Pose);
    Logger.recordOutput("Odometry/Mech Poses/Climber 2 Pose", SimulatedMechPoses.kClimber2Pose);
  }

  public boolean GetPieceFromIntake() {
    return intakeSim.obtainGamePieceFromIntake();
  }

  // Made a command to spin clockwise
  public Command IntakeCommand() {
    return startEnd(
        () -> {
          intakeMotor.set(kSpeed);
          if (RobotBase.isSimulation()) {
            intakeSim.startIntake();
            intakeSimPose = SimulatedMechPoses.kIntakeEndPose;
          }
        },
        () -> {
          intakeMotor.set(0);
          if (RobotBase.isSimulation()) {
            intakeSim.stopIntake();
            intakeSimPose = SimulatedMechPoses.kIntakeStartPose;
          }
        });
  }

  // Made a command to spin counter clockwise
  public Command OuttakeCommand() {
    return startEnd(() -> intakeMotor.set(-kSpeed), () -> intakeMotor.set(0));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Intake/Motor Ouput", intakeMotor.get());
  }

  @Override
  public void simulationPeriodic() {
    Logger.recordOutput("Odometry/Mech Poses/Intake Pose", intakeSimPose);
  }
}
