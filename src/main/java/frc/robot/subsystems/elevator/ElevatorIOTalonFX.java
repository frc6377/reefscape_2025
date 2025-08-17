package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants.CANIDs;

public abstract class ElevatorIOTalonFX implements ElevatorIO {
  protected final TalonFX motor1;
  protected final TalonFX motor2;

  protected final VoltageOut voltageRequest = new VoltageOut(0);
  protected final PositionVoltage positionVoltageRequest = new PositionVoltage(0.0);

  // Torque-current control requests
  protected final TorqueCurrentFOC torqueCurrentRequest = new TorqueCurrentFOC(0);
  protected final PositionTorqueCurrentFOC positionTorqueCurrentRequest =
      new PositionTorqueCurrentFOC(0.0);
  protected final VelocityTorqueCurrentFOC velocityTorqueCurrentRequest =
      new VelocityTorqueCurrentFOC(0.0);

  // Inputs from motor 1
  protected final StatusSignal<Angle> motorPosition1;
  protected final StatusSignal<AngularVelocity> motorVelocity1;
  protected final StatusSignal<Voltage> motorAppliedVolts1;
  protected final StatusSignal<Current> motorCurren1;

  // Inputs from motor 2
  protected final StatusSignal<Angle> motorPosition2;
  protected final StatusSignal<AngularVelocity> motorVelocity2;
  protected final StatusSignal<Voltage> motorAppliedVolts2;
  protected final StatusSignal<Current> motorCurren2;

  // Connection debouncers
  private final Debouncer motorConnectedDebounce1 = new Debouncer(0.5);
  private final Debouncer motorConnectedDebounce2 = new Debouncer(0.5);

  protected ElevatorIOTalonFX(int motor1ID, int motor2ID, String CANBusName) {
    motor1 = new TalonFX(motor1ID, CANBusName);
    motor2 = new TalonFX(motor2ID, CANBusName);

    voltageRequest.EnableFOC = true;
    positionVoltageRequest.EnableFOC = true;

    var currentLimit = new CurrentLimitsConfigs();
    currentLimit.StatorCurrentLimit = 90;
    currentLimit.SupplyCurrentLimit = 70;
    currentLimit.SupplyCurrentLowerLimit = 40;
    currentLimit.SupplyCurrentLowerTime = 1;
    currentLimit.StatorCurrentLimitEnable = true;
    currentLimit.SupplyCurrentLimitEnable = true;

    var motorConfig = new TalonFXConfiguration();
    motorConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.02;
    motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    motorConfig.Slot0 = ElevatorConstants.kElevatorSlot0Configs;
    motorConfig.SoftwareLimitSwitch = ElevatorConstants.elvSoftLimit;
    motorConfig.CurrentLimits = currentLimit;
    motorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    motorConfig.MotionMagic = ElevatorConstants.kElevatorMotionMagicConfigs;
    tryUntilOk(5, () -> motor1.getConfigurator().apply(motorConfig, 0.25));
    tryUntilOk(5, () -> motor2.getConfigurator().apply(motorConfig, 0.25));
    motor2.setControl(new Follower(CANIDs.kElevatorMotor1, true));

    // Create status signals
    motorPosition1 = motor1.getPosition();
    motorVelocity1 = motor1.getVelocity();
    motorAppliedVolts1 = motor1.getMotorVoltage();
    motorCurren1 = motor1.getStatorCurrent();

    motorPosition2 = motor2.getPosition();
    motorVelocity2 = motor2.getVelocity();
    motorAppliedVolts2 = motor2.getMotorVoltage();
    motorCurren2 = motor2.getStatorCurrent();

    // Configure periodic frames
    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        motorPosition1,
        motorPosition2,
        motorVelocity1,
        motorVelocity2,
        motorAppliedVolts1,
        motorAppliedVolts2,
        motorCurren1,
        motorCurren2);
    ParentDevice.optimizeBusUtilizationForAll(motor1, motor2);
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    // Refresh all signals
    var motorStatus1 =
        BaseStatusSignal.refreshAll(
            motorPosition1, motorVelocity1, motorAppliedVolts1, motorCurren1);
    var motorStatus2 =
        BaseStatusSignal.refreshAll(
            motorPosition2, motorVelocity2, motorAppliedVolts2, motorCurren2);

    // Update motor 1 inputs
    inputs.motor1Connected = motorConnectedDebounce1.calculate(motorStatus1.isOK());
    inputs.motor1Position = motorPosition1.getValue();
    inputs.motor1VelocityMetersPerSec = motorVelocity1.getValue();
    inputs.motor1AppliedVolts = motorAppliedVolts1.getValue().in(Volts);
    inputs.motor1CurrentAmps = motorCurren1.getValue().in(Amps);

    // Update motor 2 inputs
    inputs.motor2Connected = motorConnectedDebounce2.calculate(motorStatus2.isOK());
    inputs.motor2Position = motorPosition2.getValue();
    inputs.motor2VelocityMetersPerSec = motorVelocity2.getValue();
    inputs.motor2AppliedVolts = motorAppliedVolts2.getValue().in(Volts);
    inputs.motor2CurrentAmps = motorCurren2.getValue().in(Amps);
  }

  @Override
  public void setOpenLoop(double output) {
    motor1.setControl(
        voltageRequest.withOutput(output)); // Set voltage output for open-loop control
  }

  @Override
  public void goToHeight(Distance heightMeters) {
    // Implementation for moving to a specific height using the Talon FX motor controller
  }
}
