package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static frc.robot.subsystems.elevator.ElevatorConstants.*;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
  @AutoLog
  class ElevatorIOInputs {
    public boolean motor1Connected = false;
    public Angle motor1Position = Degrees.zero();
    public AngularVelocity motor1VelocityMetersPerSec = DegreesPerSecond.zero();
    public double motor1AppliedVolts = 0.0;
    public double motor1CurrentAmps = 0.0;

    public boolean motor2Connected = false;
    public Angle motor2Position = Degrees.zero();
    public AngularVelocity motor2VelocityMetersPerSec = DegreesPerSecond.zero();
    public double motor2AppliedVolts = 0.0;
    public double motor2CurrentAmps = 0.0;
  }

  // height = C * rot * 2/75 -> rot = height * 75/2C
  public static Angle heightToRotations(Distance height) {
    return height.times(kElevatorGearing).div(kElevatorDrumCircumference).times(Rotations.one());
  }

  // rot/s = (G)/(C*2/vel)
  public static AngularVelocity heightToRotations(LinearVelocity vel) {
    return Rotations.one().times(kElevatorGearing).div((kElevatorDrumCircumference).div(vel));
  }

  default void updateInputs(ElevatorIOInputs inputs) {}

  default void setOpenLoop(double output) {}

  default void setVelocity(double velocityMetersPerSec) {}

  default void goToHeight(Distance heightMeters) {}
}
