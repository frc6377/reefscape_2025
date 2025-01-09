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

  public static class ElevatorConstants {
    // TODO: get actual rotation values
    public static final Distance L1Height = Inches.of(18);
    public static final Distance L2Height = Inches.of(31.875);
    public static final Distance L3Height = Inches.of(47.625);
    public static final Distance L4Height = Inches.of(72);
    // TODO: update with proper PID values
    public static final double elevatorP = 200000.0;
    public static final double elevatorI = 0.0;
    public static final double elevatorD = 0.0;
    // TODO: Update to conversion between rotations and height for elevator
    public static final double elevatorConversion = 1.0;
  }

  public static class ElevatorSimConstants {
    public static final DCMotor m_elevatorGearbox = DCMotor.getNEO(1);
    public static final double kElevatorGearing = 75.0;
    public static final Mass kCarriageMass = Pounds.of(1);
    public static final Distance kElevatorDrumRadius = Inches.of(1);
    public static final double kMinElevatorHeightMeters = 0.0;
    public static final double kMaxElevatorHeightMeters = 1.8796;
  }
}
