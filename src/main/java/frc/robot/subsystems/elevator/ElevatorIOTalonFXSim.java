package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static frc.robot.subsystems.elevator.ElevatorConstants.*;

import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.Robot;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;

public class ElevatorIOTalonFXSim extends ElevatorIOTalonFX {
  private final TalonFXSimState motor1SimState;

  private final LoggedMechanism2d mech = new LoggedMechanism2d(2, 2);
  private final LoggedMechanismLigament2d elevatorMech;
  private final ElevatorSim m_elevatorSim;

  public ElevatorIOTalonFXSim(int motor1ID, int motor2ID, String CANBusName) {
    super(motor1ID, motor2ID, CANBusName);

    motor1SimState = motor1.getSimState();
    motor1SimState.Orientation = ChassisReference.CounterClockwise_Positive;

    m_elevatorSim =
        new ElevatorSim(
            kElevatorGearbox,
            kElevatorGearing,
            kCarriageMass.in(Kilograms),
            kElevatorDrumRadius.in(Meters),
            kMinElevatorHeight.in(Meters),
            kMaxElevatorHeight.in(Meters),
            false,
            0);
    elevatorMech =
        mech.getRoot("root", 1, 0)
            .append(
                new LoggedMechanismLigament2d(
                    "Elevator Mech [0]", 1, 90, 10, new Color8Bit(Color.kPurple)));
    SmartDashboard.putData("Mech2Ds/Elevator Mech", mech);
  }

  @Override
  public Distance getElvHeight() {
    return Meters.of(m_elevatorSim.getPositionMeters());
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    super.updateInputs(inputs);

    // Update Elevator Simulation
    m_elevatorSim.setInputVoltage(motor1SimState.getMotorVoltage());
    m_elevatorSim.update(Robot.defaultPeriodSecs);
    final Distance simHeight = getElvHeight();
    final LinearVelocity simVel = MetersPerSecond.of(m_elevatorSim.getVelocityMetersPerSecond());

    // Update Simulated Motor
    var motorPosition1 = heightToRotations(simHeight);
    var motorVelocity1 = heightToRotVel(simVel);
    motor1SimState.setRawRotorPosition(motorPosition1);
    motor1SimState.setRotorVelocity(motorVelocity1);
    motor1SimState.setSupplyVoltage(RobotController.getBatteryVoltage());

    // Update Visualization
    elevatorMech.setLength(simHeight.in(Meters));

    // Update Inputs
    inputs.motor1Connected = true;
    inputs.motor1Position = motorPosition1;
    inputs.motor1VelocityMetersPerSec = motorVelocity1;
    inputs.motor1AppliedVolts = motor1SimState.getMotorVoltage();
    inputs.motor1CurrentAmps = motor1SimState.getSupplyCurrent();

    inputs.motor2Connected = true;
    inputs.motor2Position = motorPosition1;
    inputs.motor2VelocityMetersPerSec = motorVelocity1;
    inputs.motor2AppliedVolts = motor1SimState.getMotorVoltage();
    inputs.motor2CurrentAmps = motor1SimState.getSupplyCurrent();

    inputs.elevatorHeight = simHeight;
  }
}
