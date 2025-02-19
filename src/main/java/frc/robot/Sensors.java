package frc.robot;

import static edu.wpi.first.units.Units.Inches;
import static frc.robot.Constants.SensorIDs.kSensor2ID;
import static frc.robot.Constants.SensorIDs.kSensor3ID;
import static frc.robot.Constants.SensorIDs.kSensor4ID;
import static frc.robot.Constants.SensorIDs.kSensor5ID;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.IntakeConstants.CoralEnum;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import utilities.TOFSensorSimple;
import utilities.TOFSensorSimple.TOFType;

public class Sensors {
  private TOFSensorSimple sensor2 =
      new TOFSensorSimple(kSensor2ID, Inches.of(1.5), TOFType.LASER_CAN);
  private TOFSensorSimple sensor3 =
      new TOFSensorSimple(kSensor3ID, Inches.of(2.5), TOFType.LASER_CAN);
  private TOFSensorSimple sensor4 =
      new TOFSensorSimple(kSensor4ID, Inches.of(2.5), TOFType.LASER_CAN);
  private TOFSensorSimple sensor5 =
      new TOFSensorSimple(
          kSensor5ID, Inches.of(2.5), TOFType.LASER_CAN); // Parallel sensor for ball presence

  public CoralEnum simState = CoralEnum.NO_CORAL;

  public Sensors() {}

  public TOFSensorSimple getSensor(int sensorID) {
    switch (sensorID) {
      case kSensor2ID:
        return sensor2;
      case kSensor3ID:
        return sensor3;
      case kSensor4ID:
        return sensor4;
      case kSensor5ID:
        return sensor5;
      default:
        return null;
    }
  }

  public Distance getSensorDist(int sensorID) {
    switch (sensorID) {
      case kSensor2ID:
        return sensor2.getDistance();
      case kSensor3ID:
        return sensor3.getDistance();
      case kSensor4ID:
        return sensor4.getDistance();
      case kSensor5ID:
        return sensor5.getDistance();
      default:
        return Inches.zero();
    }
  }

  public boolean getSensorBool(int sensorID) {
    switch (sensorID) {
      case kSensor2ID:
        return sensor2.getBeamBroke();
      case kSensor3ID:
        return sensor3.getBeamBroke();
      case kSensor4ID:
        return sensor4.getBeamBroke();
      case kSensor5ID:
        return sensor5.getBeamBroke();
      default:
        return false;
    }
  }

  public Trigger getSensorTrigger(int sensorID) {
    switch (sensorID) {
      case kSensor2ID:
        return sensor2.getBeamBrokenTrigger();
      case kSensor3ID:
        return sensor3.getBeamBrokenTrigger();
      case kSensor4ID:
        return sensor4.getBeamBrokenTrigger();
      case kSensor5ID:
        return sensor5.getBeamBrokenTrigger();
      default:
        return new Trigger(null);
    }
  }

  @AutoLogOutput(key = "Intake/States/Sensor State Enum")
  public CoralEnum getSensorState() {
    if (Robot.isSimulation()) {
      return simState;
    }

    // If parallel sensor (sensor5) doesn't detect anything, there's no coral
    if (!sensor5.getBeamBroke()) {
      return CoralEnum.NO_CORAL;
    }

    int state = sensor2.getBeamBroke() ? 1 : 0;
    state += sensor3.getBeamBroke() ? 10 : 0;
    state += sensor4.getBeamBroke() ? 100 : 0;
    Logger.recordOutput("Intake/Laser Can State", state);
    switch (state) {
      case 0:
        return CoralEnum.NO_CORAL;
      case 1:
        return CoralEnum.CORAL_TOO_CLOSE;
      case 11:
        return CoralEnum.CORAL_ALIGNED;
      case 10:
        return CoralEnum.CORAL_TOO_FAR;
      case 110:
        return CoralEnum.CORAL_TOO_FAR;
      case 100:
        return CoralEnum.CORAL_TOO_FAR;
      case 101:
        return CoralEnum.NO_CORAL;
      case 111:
        return CoralEnum.NO_CORAL;
      default:
        return CoralEnum.OTHER;
    }
  }

  public void setSimState(CoralEnum state) {
    simState = state;
  }
}
