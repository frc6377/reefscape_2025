package frc.robot;

import static edu.wpi.first.units.Units.Hertz;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.InchesPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Second;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.AngularAccelerationUnit;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.Velocity;
import frc.robot.subsystems.Elevator;

public final class Constants {

  public static final String CANivoreName = "Default Name";
  public static final String RIOName = "rio";

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class MotorIDConstants {
    // Rev Can Bus
    public static final int kElevatorMotor1 = 1;
    public static final int kElevatorMotor2 = 2;
    public static final int kIntakeMotor = 9;
    // CANivore Can Bus
  }

  // Intake Constants
  public static class IntakeConstants {
    public static final double kSpeed = 0.5;
  }

  public static class ElevatorConstants {
    public static final Distance kL0Height = Meters.of(0.252);
    // L1 needs to be adjusted once it actually is worth it
    public static final Distance kL1Height = Inches.of(15);
    public static final Distance kL2Height = Inches.of(16.62);
    public static final Distance kL3Height = Inches.of(30.9);
    public static final Distance kL4Height = Inches.of(55);

    public static final int elvLimitID = 0;

    public static final double P = 1.5;
    public static final double I = 0.04;
    public static final double D = 0.02;
    public static final double FF = 0.0;
    public static final Distance kBottomLimit = Inches.of(9);
    public static final Distance kTopLimit = Inches.of(75);
    public static final double kElevatorConversion = 1.0;

    // The carriage on the elv effectivly adds a gearing multiplier of 1
    public static final double kCarageFactor = 1;

    // Simulation Constants
    public static final DCMotor kElevatorGearbox = DCMotor.getKrakenX60(2);
    public static final double elevatorOutput = .30;
    public static final double kElevatorGearing = 1.0;
    public static final Mass kCarriageMass = Pounds.of(5.15);
    public static final Distance kElevatorDrumRadius = Inches.of(.75 / 2);
    public static final Distance kMinElevatorHeight = Inches.zero();
    public static final Distance kMaxElevatorHeight = Inches.of(72);
    public static final Distance kElevatorDrumCircumference =
        kElevatorDrumRadius.times(2 * Math.PI);
    public static final AngularVelocity MMVel = Elevator.heightToRotations(InchesPerSecond.of(60));
    public static final AngularAcceleration MMAcc = MMVel.times(Hertz.of(5));
    public static final Velocity<AngularAccelerationUnit> MMJerk =
        RotationsPerSecondPerSecond.per(Second).of(MMAcc.in(RotationsPerSecondPerSecond)).times(10);
  }
}
