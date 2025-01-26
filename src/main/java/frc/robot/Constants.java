package frc.robot;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Pounds;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;

public final class Constants {

  public static final String CANivoreName = "CANivore";

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
    public static final Distance kL1Height = Inches.of(18);
    public static final Distance kL2Height = Inches.of(31.875);
    public static final Distance kL3Height = Inches.of(47.625);
    // public static final Distance kL4Height = Inches.of(72);
    public static final Distance kL4Height = Inches.of(68.6);

    public static final int elvLimitID = 0;

    public static final double P = 2.0;
    public static final double I = 0.05;
    public static final double D = 0.0;
    public static final double FF = 0.0;
    public static final double MMVel = 30;
    public static final Distance kBottomLimit = Inches.of(9);
    public static final Distance kTopLimit = Inches.of(75);
    public static final double kElevatorConversion = 1.0;

    // The carriage on the elv effectivly adds a gearing multiplier of 2
    public static final double kCarageFactor = 1;

    // Simulation Constants
    public static final DCMotor kElevatorGearbox = DCMotor.getKrakenX60(2);
    public static final double elevatorOutput = .15;
    public static final double kElevatorGearing = 1.0;
    public static final Mass kCarriageMass = Pounds.of(5.15);
    public static final Distance kElevatorDrumRadius = Inches.of(.75 / 2);
    public static final Distance kMinElevatorHeight = Inches.zero();
    public static final Distance kMaxElevatorHeight = Inches.of(72);
    public static final Distance kElevatorDrumCircumference =
        kElevatorDrumRadius.times(2 * Math.PI);
  }
}
