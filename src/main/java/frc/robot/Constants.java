package frc.robot;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.Rotation;
import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;

public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class MotorIDConstants {
    public static final int kCLimberMotorLeader = 0;
    public static final int kCLimberMotorFollower = 0;
  }

  public static class ClimberConstants {
    public static final double kClimberP = 1;
    public static final double kClimberI = 0;
    public static final double kClimberD = 0;
    public static final Angle kClimberExtended = Angle.ofBaseUnits(63, Rotations);
    public static final Angle kClimberRetracted = Angle.ofBaseUnits(0, Rotations);
  }
}
