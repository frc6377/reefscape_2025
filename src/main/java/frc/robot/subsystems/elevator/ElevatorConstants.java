package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;

public final class ElevatorConstants {
  public static final Distance kL0Height = Inches.of(0.5);
  public static final Distance kL2Height = Inches.of(18);
  public static final Distance kL3Height = Inches.of(30.9);
  public static final Distance kL4Height = Inches.of(54);

  public static final Slot0Configs kElevatorSlot0Configs =
      new Slot0Configs()
          .withKP(2)
          .withKI(0.08)
          .withKD(0.02)
          .withKS(0.5)
          .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);

  public static final MotionMagicConfigs kElevatorMotionMagicConfigs =
      new MotionMagicConfigs()
          .withMotionMagicCruiseVelocity(RotationsPerSecond.of(200))
          .withMotionMagicAcceleration(RotationsPerSecondPerSecond.of(250));

  public static final Distance kSetpointTolerance = Inches.of(1.25);

  // Mech/Sim Constants
  public static final Distance kElevatorDrumRadius = Inches.of(0.375);
  public static final Distance kElevatorDrumCircumference = kElevatorDrumRadius.times(2 * Math.PI);
  public static final double kElvRawOutput = .10;
  public static final double kElevatorGearing = 3;
  public static final int kGearToothing1 = 3;
  public static final int kGearToothing2 = 11;

  public static final Distance kBottomLimit = Inches.of(0);
  public static final Distance kTopLimit = Inches.of(75);

  public static final DCMotor kElevatorGearbox = DCMotor.getKrakenX60(2);
  public static final Mass kCarriageMass = Pounds.of(4.75);
  public static final Distance kMinElevatorHeight = Inches.zero();
  public static final Distance kMaxElevatorHeight = Inches.of(72);
}
