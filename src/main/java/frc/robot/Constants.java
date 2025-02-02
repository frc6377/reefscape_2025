package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Hertz;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RevolutionsPerSecond;

import com.ctre.phoenix6.signals.GravityTypeValue;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

public final class Constants {
  public static class CtreCanID {
    // Rev Can Bus
    public static final int kElevatorMotor1 = 1;
    public static final int kElevatorMotor2 = 2;
    public static final int kPivotMotor = 10; // FIXME: Change to correct ID
    public static final int kFeedBackSensorID = 0;
  }

  public static class RevCanID {
    // CANivore Can Bus
    public static final int kIntakeMotor = 9;
    public static final int kConveyorMotor = 0; // FIXME: Change to correct ID
    public static final int kConveyorSensor = 4; // FIXME: Change to correct ID
  }

  // Intake Constants
  public static class IntakeConstants {
    public static final double kIntakeSpeed = 0.5;
    public static final double kConveyorSpeed = 0.5;
    public static final double kPivotSpeed = 0.5;
    public static final Angle kPivotRetractAngle = Degrees.of(90); // FIXME: Put actual value
    public static final Angle kPivotExtendAngle =
        Degrees.of(0); // FIXME: Might have to be negative to reach over bumper
    public static final Angle kPivotTolerance = Degrees.of(3);
    public static final double kPivotP = 100;
    public static final double kPivotI = 0.0;
    public static final double kPivotD = 0.0;
    public static final double kPivotG = 0.21;
    public static final double kPivotV = 0; // 7.20;
    public static final double kPivotA = 0; // 0.03;
    public static final GravityTypeValue kPivotGravityType = GravityTypeValue.Arm_Cosine;

    public static final double kGearing = 60;
    public static final double kSensorToMechanism = 1;
    public static final Distance kLength = Feet.of(1);
    public static final Mass kMass = Pounds.of(8);
    public static final MomentOfInertia kMOI =
        KilogramSquareMeters.of(
            SingleJointedArmSim.estimateMOI(kLength.in(Meters), kMass.in(Kilograms)));
    public static final AngularVelocity kMotionMagicCruiseVelocity =
        RevolutionsPerSecond.of(Double.MAX_VALUE);
    public static final AngularAcceleration kMotionMagicAcceleration =
        kMotionMagicCruiseVelocity.times(Hertz.of(Double.MAX_VALUE));
    public static final double kMotionMagicJerk = Double.MAX_VALUE * Double.MAX_VALUE;
  }

  // Elevator Constants
  public static class ElevatorConstants {
    public static final Distance kL0Height = Meters.of(0.252);
    public static final Distance kL1Height = Inches.of(18);
    public static final Distance kL2Height = Inches.of(31.875);
    public static final Distance kL3Height = Inches.of(47.625);
    // public static final Distance kL4Height = Inches.of(72);
    public static final Distance kL4Height = Inches.of(68.6);

    public static final int elvLimitID = 1;

    public static final double P = 0.5;
    public static final double I = 0.0;
    public static final double D = 0.0;
    public static final double FF = 0.0;
    public static final Distance kBottomLimit = Inches.of(9);
    public static final Distance kTopLimit = Inches.of(75);
    public static final double kElevatorConversion = 1.0;

    // The carriage on the elv effectivly adds a gearing multiplier of 2
    public static final double kCarageFactor = 2;

    // Simulation Constants
    public static final DCMotor kElevatorGearbox = DCMotor.getKrakenX60Foc(2);
    public static final double kElevatorGearing = 1.0;
    public static final Mass kCarriageMass = Pounds.of(4);
    public static final Distance kElevatorDrumRadius = Inches.of(.75 / 2);
    public static final Distance kMinElevatorHeight = Inches.zero();
    public static final Distance kMaxElevatorHeight = Inches.of(63);
    public static final Distance kElevatorDrumCircumference =
        kElevatorDrumRadius.times(2 * Math.PI);
  }

  /**
   * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when
   * running on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and
   * "replay" (log replay from a file).
   */
  public static final Mode simMode = Mode.SIM;

  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static final Distance kFieldWidth = Inches.of(317);
  public static final Distance kFieldLength = Inches.of(690 + (7 / 8));

  public final class DrivetrainConstants {
    // PathPlanner config constants
    public static final double ROBOT_MASS_KG = 74.088;
    public static final double ROBOT_MOI = 6.883;
    public static final double WHEEL_COF = 1.2;

    // Scoring Poses for PathFinder
    public static final Pose2d[] SCORE_POSES =
        new Pose2d[] {
          new Pose2d(Meters.of(3.173), Meters.of(4.193), new Rotation2d(0)),
          new Pose2d(Meters.of(3.173), Meters.of(3.854), new Rotation2d(0)),
          new Pose2d(Meters.of(3.686), Meters.of(2.973), new Rotation2d(60)),
          new Pose2d(Meters.of(3.974), Meters.of(2.801), new Rotation2d(60)),
          new Pose2d(Meters.of(5.000), Meters.of(2.799), new Rotation2d(120)),
          new Pose2d(Meters.of(5.286), Meters.of(2.966), new Rotation2d(120)),
          new Pose2d(Meters.of(5.803), Meters.of(3.854), new Rotation2d(180)),
          new Pose2d(Meters.of(5.803), Meters.of(4.194), new Rotation2d(180)),
          new Pose2d(Meters.of(5.288), Meters.of(5.082), new Rotation2d(-120)),
          new Pose2d(Meters.of(5.001), Meters.of(5.245), new Rotation2d(-120)),
          new Pose2d(Meters.of(3.972), Meters.of(5.244), new Rotation2d(-60)),
          new Pose2d(Meters.of(3.684), Meters.of(5.078), new Rotation2d(-60)),
        };
  }
}
