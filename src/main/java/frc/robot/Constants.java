package frc.robot;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pounds;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;

public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class MotorIDConstants {
    // Rev Can Bus
    public static final int kElevatorMotor1 = 1;
    public static final int kIntakeMotor = 2;
    // CANavor Can Bus
  }

  // Intake Constants
  public static class IntakeConstants {
    public static final double kSpeed = 0.5;
  }

  public static class ElevatorConstants {
    public static final Distance kL1Height = Inches.of(18);
    public static final Distance kL2Height = Inches.of(31.875);
    public static final Distance kL3Height = Inches.of(47.625);
    public static final Distance kL4Height = Inches.of(72);
    public static final double P = 0.10;
    public static final double I = 0.0;
    public static final double D = 0.0;
    public static final double FF = 0.0;
    public static final double kElevatorConversion = 1.0;

    // Simulation Constants
    public static final DCMotor kElevatorGearbox = DCMotor.getNEO(1);
    public static final double kElevatorGearing = 75.0;
    public static final Mass kCarriageMass = Pounds.of(10);
    public static final Distance kElevatorDrumRadius = Inches.of(1.729 / 2);
    public static final Distance kMinElevatorHeight = Inches.zero();
    public static final Distance kMaxElevatorHeight = Inches.of(76);
    public static final Distance kElevatorDrumCircumference =
        kElevatorDrumRadius.times(2 * Math.PI);
  }
}
