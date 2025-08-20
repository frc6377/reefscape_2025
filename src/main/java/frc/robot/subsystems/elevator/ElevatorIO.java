package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Meters;
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
    public double motor1TemperatureF = 1;

    public boolean motor2Connected = false;
    public Angle motor2Position = Degrees.zero();
    public AngularVelocity motor2VelocityMetersPerSec = DegreesPerSecond.zero();
    public double motor2AppliedVolts = 0.0;
    public double motor2CurrentAmps = 0.0;
    public double motor2TemperatureF = 1;

    public Distance elevatorHeight = Meters.zero();
    public Distance elevatorHeightSetpoint = Meters.zero();
  }

  // height = C * rot * 2/75 -> rot = height * 75/2C
  default Angle heightToRotations(Distance height) {
    return height.times(kElevatorGearing).div(kElevatorDrumCircumference).times(Rotations.one());
  }

  // rot/s = (G)/(C*2/vel)
  default AngularVelocity heightToRotVel(LinearVelocity vel) {
    return Rotations.one().times(kElevatorGearing).div((kElevatorDrumCircumference).div(vel));
  }

  default Distance rotationsToHeight(Angle rotations) {
    return kElevatorDrumCircumference.times(rotations.in(Rotations)).div(kElevatorGearing);
  }

  default Distance getElvHeight() {
    return Meters.of(0);
  }

  default Distance getElvHeightSetpoint() {
    return Meters.of(0);
  }

  default void updateInputs(ElevatorIOInputs inputs) {}

  default void setOpenLoop(double output) {}

  default void goToHeight(Distance heightMeters) {}

  default void zeroMotorEncoder() {}

  default void disableSoftLimits() {}

  default void enableSoftLimits() {}
}
