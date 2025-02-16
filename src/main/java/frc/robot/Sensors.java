package frc.robot;

import static edu.wpi.first.units.Units.Inches;
import static frc.robot.Constants.SensorIDs.*;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.IntakeConstants.CoralEnum;
import org.littletonrobotics.junction.AutoLogOutput;
import utilities.TOFSensorSimple;
import utilities.TOFSensorSimple.TOFType;

public class Sensors {
  private TOFSensorSimple sensor2 =
      new TOFSensorSimple(kSensor2ID, Inches.of(1.5), TOFType.LASER_CAN);
  private TOFSensorSimple sensor3 =
      new TOFSensorSimple(kSensor3ID, Inches.of(2.5), TOFType.LASER_CAN);
  private TOFSensorSimple sensor4 =
      new TOFSensorSimple(kSensor4ID, Inches.of(2.5), TOFType.LASER_CAN);

  public CoralEnum simState = CoralEnum.NO_CORAL;

  public Sensors() {}

  public Distance getSensorDist(int sensorID) {
    switch (sensorID) {
      case 2:
        return sensor2.getDistance();
      case 3:
        return sensor3.getDistance();
      case 4:
        return sensor4.getDistance();
      default:
        return Inches.zero();
    }
  }

  public boolean getSensorBool(int sensorID) {
    switch (sensorID) {
      case 2:
        return sensor2.isBeamBroke();
      case 3:
        return sensor3.isBeamBroke();
      case 4:
        return sensor4.isBeamBroke();
      default:
        return false;
    }
  }

  public Trigger getSensorTrigger(int sensorID) {
    switch (sensorID) {
      case 2:
        return sensor2.beamBroken();
      case 3:
        return sensor3.beamBroken();
      case 4:
        return sensor4.beamBroken();
      default:
        return new Trigger(null);
    }
  }

  @AutoLogOutput(key = "Intake/Sensor State Enum")
  public CoralEnum getSensorState() {
    if (Robot.isSimulation()) {
      return simState;
    }

    int state = sensor2.isBeamBroke() ? 1 : 0;
    state += sensor3.isBeamBroke() ? 10 : 0;
    state += sensor4.isBeamBroke() ? 100 : 0;
    SmartDashboard.putNumber("Laser Can State", state);
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
