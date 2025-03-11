package frc.robot.subsystems.signaling;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.FireAnimation;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.CANIDs;
import frc.robot.Constants.SignalingConstants;
import frc.robot.OI;
import frc.robot.subsystems.signaling.patterns.AlliancePattern;
import frc.robot.subsystems.signaling.patterns.PatternNode;
import frc.robot.subsystems.signaling.patterns.RainbowPattern;
import org.littletonrobotics.junction.Logger;

public class Signaling extends SubsystemBase {

  private final CANdle candle = new CANdle(CANIDs.kCANdle);
  private int tick;
  private int patternTick;
  private long tagCount;
  private DisablePattern disablePattern = DisablePattern.FIRE;
  private PowerDistribution pdp = new PowerDistribution();

  public Signaling() {
    tick = 0;
    patternTick = 0;
  }

  @Override
  public void periodic() {
    // Update Light Pattern

    if (DriverStation.isDisabled()) {
      tagCount =
          NetworkTableInstance.getDefault().getTable("Vision").getEntry("TagCount").getInteger(0);
      switch ((int) tagCount) {
        case 0:
          setCandle(RGB.RED);
          Logger.recordOutput("Signaling/CANdle Color", "Red");
          break;
        case 1:
          setCandle(RGB.YELLOW);
          Logger.recordOutput("Signaling/CANdle Color", "Yellow");
          break;
        default:
          setCandle(RGB.GREEN);
          Logger.recordOutput("Signaling/CANdle Color", "Green");
          break;
      }
      updatePattern();
    } else {
      if (getProblem()) {
        setCandle(RGB.RED);
      } else {
        if (DriverStation.isAutonomous()) {
          setCandle(RGB.PURPLE);
        } else if (DriverStation.isTest()) {
          setCandle(RGB.BLUE);
        } else {
          setCandle(RGB.GREEN);
        }
      }
    }
    Logger.recordOutput("Signaling/Pattern", disablePattern.toString());
  }

  private boolean getProblem() {
    return (pdp.getVoltage() < 6) || (pdp.getTemperature() > 60);
  }

  public Command setColor(RGB rgb) {
    return runOnce(
        () -> {
          setFullStrip(rgb);
          Logger.recordOutput("Signaling/LED Color", rgb.toHex());
        });
  }

  private void setCandle(RGB rgb) {
    setSection(rgb, 0, 8, true);
  }

  public Command setToAlliance() {
    return startEnd(
        () -> {
          setFullStrip(getColorFromAlliance(Constants.kAllianceColor));
        },
        this::resetLEDs);
  }

  public RGB getColorFromAlliance(Alliance alliance) {
    if (alliance == Alliance.Red) {
      return RGB.RED;
    } else if (alliance == Alliance.Blue) {
      return RGB.BLUE;
    }
    return RGB.BLUE;
  }

  private void setRumble(double intensity) {
    OI.Driver.setRumble(intensity);
  }

  private void resetLEDs() {
    setFullStrip(RGB.BLACK);
  }

  private void setFullStrip(final RGB rgb) {
    setSection(rgb, 0, SignalingConstants.kNumLEDs);
  }

  private void setSection(
      final RGB rgb, final int startID, final int count, final Boolean candleBool) {
    candle.setLEDs(rgb.red, rgb.green, rgb.blue, 0, startID + (8 * (candleBool ? 0 : 1)), count);
  }

  private void setSection(final RGB rgb, final int startID, final int count) {
    setSection(rgb, startID, count, false);
  }

  private void updatePattern() {
    PatternNode[] pattern;
    int patternLength;

    tick++;
    if (tick > SignalingConstants.kPatternSpeed * 50) {
      tick = 0;
      patternTick++;
    } else {
      return;
    }
    switch (disablePattern) {
      case FIRE:
        candle.animate(
            new FireAnimation(
                1.0,
                SignalingConstants.kPatternSpeed,
                SignalingConstants.kNumLEDs,
                0.5,
                0.5,
                false,
                8));
      case RAINBOW:
        pattern = RainbowPattern.getPattern();
        patternLength = RainbowPattern.getPatternLength();
        getLEDIndex(pattern, patternLength);
        break;
      case SWEEP:
        pattern = RainbowPattern.getPattern();
        patternLength = RainbowPattern.getPatternLength();
        getLEDIndex(pattern, patternLength);
        break;
      case ALLIANCE:
        pattern = AlliancePattern.getPattern();
        patternLength = AlliancePattern.getPatternLength();
        getLEDIndex(pattern, patternLength);
        break;
      default:
        pattern = AlliancePattern.getPattern();
        patternLength = AlliancePattern.getPatternLength();
        getLEDIndex(pattern, patternLength);
        break;
    }
  }

  private void getLEDIndex(PatternNode[] pattern, int patternLength) {
    int patternIndex = 0;
    patternTick %= patternLength;
    int LEDIndex = -patternTick - 1;
    while (LEDIndex < SignalingConstants.kNumLEDs) {
      patternIndex %= pattern.length;
      PatternNode node = pattern[patternIndex];
      RGB color = node.color;
      Logger.recordOutput("Signaling/LED Color", color.toHex());
      setSection(color, LEDIndex, node.repeat);
      LEDIndex += node.repeat;
      patternIndex++;
    }
  }

  public void randomizePattern() {
    disablePattern = DisablePattern.getRandom();
  }

  private enum DisablePattern {
    RAINBOW,
    ALLIANCE,
    FIRE,
    SWEEP;

    public static DisablePattern getRandom() {
      DisablePattern[] allPatterns = DisablePattern.values();
      return allPatterns[(int) (Math.random() * allPatterns.length)];
    }
  }

  public void clearLEDs() {
    setFullStrip(RGB.BLACK);
  }
}
