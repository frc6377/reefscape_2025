package frc.robot.subsystems.signaling;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.PWMIDs;
import frc.robot.Constants.SignalingConstants;
import frc.robot.OI;
import frc.robot.subsystems.signaling.patterns.AlliancePattern;
import frc.robot.subsystems.signaling.patterns.PatternNode;
import frc.robot.subsystems.signaling.patterns.RainbowPattern;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class Signaling extends SubsystemBase {

  private final AddressableLED ledStrip;
  private final AddressableLEDBuffer ledBuffer;

  private int tick;
  private int patternTick;

  private DisablePattern disablePattern = DisablePattern.getRandom();

  public Signaling() {
    tick = 0;
    patternTick = 0;

    // Initialize LED Strip
    ledStrip = new AddressableLED(PWMIDs.kLED_PWM_PORT);
    ledBuffer = new AddressableLEDBuffer(SignalingConstants.NUMBER_OF_LEDS);
    ledStrip.setLength(SignalingConstants.NUMBER_OF_LEDS);
    ledStrip.start();
    ledStrip.setData(ledBuffer);
  }

  @Override
  public void periodic() {
    // Update Light Pattern
    if (DriverStation.isDisabled()) {
      updatePattern();
    } else {
      getReefSignalingCommand().schedule();
    }
    Logger.recordOutput("LED green", ledBuffer.getGreen(1));
  }

  public Supplier<RGB> getRGB() {
    if (true) {
      if (true) {
        return () -> RGB.GREEN;
      } else {
        return () -> RGB.YELLOW;
      }
    } else {
      return () -> RGB.BLUE;
    }
  }

  public Command getReefSignalingCommand() {
    return setColor(getRGB().get());
  }

  public Command setColor(RGB rgb) {
    return runOnce(
        () -> {
          setFullStrip(rgb);
          ledStrip.setData(ledBuffer);
          Logger.recordOutput("Signaling/LED Color", rgb.toHex());
        });
  }

  public Command setToAlliance() {
    return startEnd(
        () -> {
          setFullStrip(getColorFromAlliance(Constants.kAllianceColor));
          ledStrip.setData(ledBuffer);
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
    ledStrip.setData(ledBuffer);
  }

  private void setFullStrip(final RGB rgb) {
    setSection(rgb, 0, SignalingConstants.NUMBER_OF_LEDS);
  }

  private void setSection(final RGB rgb, final int startID, final int count) {
    for (var i = startID; i < startID + count; i++) {
      if (i >= 0 && i < SignalingConstants.NUMBER_OF_LEDS) {
        ledBuffer.setRGB(
            i,
            (int) (rgb.red * SignalingConstants.LED_BRIGHTNESS),
            (int) (rgb.green * SignalingConstants.LED_BRIGHTNESS),
            (int) (rgb.blue * SignalingConstants.LED_BRIGHTNESS));
      }
    }
  }

  private void updatePattern() {
    PatternNode[] pattern;
    int patternLength;

    tick++;
    if (tick > SignalingConstants.PATTERN_SPEED * 50) {
      tick = 0;
      patternTick++;
    } else {
      return;
    }

    switch (disablePattern) {
      case RAINBOW:
        pattern = RainbowPattern.getPattern();
        patternLength = RainbowPattern.getPatternLength();
        break;
      case SWEEP:
        pattern = RainbowPattern.getPattern();
        patternLength = RainbowPattern.getPatternLength();
        break;
      case ALLIANCE:
        pattern = AlliancePattern.getPattern();
        patternLength = AlliancePattern.getPatternLength();
      default:
        pattern = AlliancePattern.getPattern();
        patternLength = AlliancePattern.getPatternLength();
        break;
    }
    getLEDIndex(pattern, patternLength);
  }

  private void getLEDIndex(PatternNode[] pattern, int patternLength) {
    int patternIndex = 0;
    patternTick %= patternLength;
    int LEDIndex = -patternTick - 1;
    while (LEDIndex < SignalingConstants.NUMBER_OF_LEDS) {
      patternIndex %= pattern.length;
      PatternNode node = pattern[patternIndex];
      RGB color = node.color;
      Logger.recordOutput("Signaling/LED Color", color.toHex());
      setSection(color, LEDIndex, node.repeat);
      LEDIndex += node.repeat;
      patternIndex++;
    }
    ledStrip.setData(ledBuffer);
  }

  public void randomizePattern() {
    disablePattern = DisablePattern.getRandom();
  }

  private enum DisablePattern {
    RAINBOW,
    ALLIANCE,
    SWEEP;

    public static DisablePattern getRandom() {
      DisablePattern[] allPatterns = DisablePattern.values();
      return allPatterns[(int) (Math.random() * allPatterns.length)];
    }
  }

  public void clearLEDs() {
    setFullStrip(RGB.BLACK);
    ledStrip.setData(ledBuffer);
  }
}
