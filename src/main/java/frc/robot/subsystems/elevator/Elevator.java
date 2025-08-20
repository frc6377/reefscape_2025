package frc.robot.subsystems.elevator;

import static frc.robot.subsystems.elevator.ElevatorConstants.*;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.DIOConstants;
import frc.robot.Robot;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {
  private final ElevatorIO motors;

  private DigitalInput elvLimitSwitch;

  public Elevator(ElevatorIO motors) {
    this.motors = motors;

    elvLimitSwitch = new DigitalInput(DIOConstants.elvLimitID);
  }

  public Distance getElvHeight() {
    return motors.getElvHeight();
  }

  private void motorStateLog(String state) {
    Logger.recordOutput("state", state);
    new WaitCommand(0.04)
        .andThen(new InstantCommand(() -> Logger.recordOutput("state", "blank"), new Subsystem[0]))
        .schedule();
  }

  public Trigger elevatorAtSetpointTrigger(Distance setpoint) {
    return new Trigger(() -> motors.getElvHeight().isNear(setpoint, kSetpointTolerance))
        .debounce(0.25);
  }

  public Trigger elevatorAtCurrentSetpointTrigger() {
    return elevatorAtSetpointTrigger(motors.getElvHeightSetpoint());
  }

  public Command setElvPercent(double percentPower) {
    return runEnd(
        () -> {
          motorStateLog("Percent was set");
          motors.setOpenLoop(percentPower);
        },
        () -> {
          motorStateLog("Percent was set");
          motors.setOpenLoop(0);
        });
  }

  public Command limitHit() {
    if (Robot.isSimulation()) {
      return Commands.none();
    }
    return runOnce(() -> motors.disableSoftLimits())
        .andThen(setElvPercent(-0.2).until(elvLimitSwitch::get))
        .andThen(zeroMotorEncoder())
        .andThen(runOnce(() -> motors.enableSoftLimits()))
        .withName("Elevator Zero");
  }

  public Command zeroMotorEncoder() {
    return runOnce(
        () -> {
          motorStateLog("Position Set");
          motors.zeroMotorEncoder();
        });
  }

  public Command changeElevation(Distance heightLevel) {
    return runOnce(
        () -> {
          motors.goToHeight(heightLevel);
        });
  }

  public Command L0() {
    return changeElevation(ElevatorConstants.kL0Height);
  }

  public Command L2() {
    return changeElevation(ElevatorConstants.kL2Height);
  }

  public Command L3() {
    return changeElevation(ElevatorConstants.kL3Height);
  }

  public Command L4() {
    return changeElevation(ElevatorConstants.kL4Height);
  }

  @Override
  public void periodic() {
    Logger.recordOutput("Elevator/limit switch state", elvLimitSwitch.get());

    Logger.recordOutput(
        "Elevator/Current Command",
        this.getCurrentCommand() != null ? this.getCurrentCommand().getName() : "None");
  }

  @Override
  public void simulationPeriodic() {}
}
