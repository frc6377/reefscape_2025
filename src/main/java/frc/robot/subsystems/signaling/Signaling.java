package frc.robot.subsystems.signaling;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.PWMIDs;
import frc.robot.Constants.SignalingConstants;
import frc.robot.OI;
import frc.robot.subsystems.signaling.patterns.PatternNode;
import frc.robot.subsystems.signaling.patterns.RainbowPattern;
import org.littletonrobotics.junction.Logger;

public class Signaling extends SubsystemBase {

  private final AddressableLED ledStrip;
  private final AddressableLEDBuffer ledBuffer;

  private int tick;
  private int patternTick;
  private final Timer rumbleTimer;
  private double rumbleEndTime = 10;

  private DisablePattern disablePattern = DisablePattern.getRandom();

  public Signaling() {

    tick = 0;
    patternTick = 0;
    rumbleTimer = new Timer();

    // Initialize LED Strip
    ledStrip = new AddressableLED(PWMIDs.kLED_PWM_PORT);
    ledBuffer = new AddressableLEDBuffer(SignalingConstants.NUMBER_OF_LEDS);
    ledStrip.setLength(SignalingConstants.NUMBER_OF_LEDS);
    ledStrip.setData(ledBuffer);
    ledStrip.start();
  }

  @Override
  public void periodic() {
    // Update Light Pattern
    if (DriverStation.isDisabled()) updatePattern();
    else {                              
    }

    // End Signaling
    if (rumbleTimer.get() > rumbleEndTime) {
      endSignal();
    }
  }

  public Command setColor(RGB rgb) {
    return startEnd(
        () -> {
          setFullStrip(rgb);
          ledStrip.setData(ledBuffer);
          Logger.recordOutput("Signaling/LED Color", rgb.toHex());
        },
        () -> {
          resetLEDs();
        });
  }

  public Command setToAlliance() {
    return startEnd(
        () -> {
          setFullStrip(getColorFromAlliance(Constants.kAllianceColor));
          ledStrip.setData(ledBuffer);
        },
        () -> {
          resetLEDs();
        });
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

  private void startSignal(final double time, final double intensity) {
    setRumble(intensity);
    rumbleEndTime = time;
    rumbleTimer.reset();
    rumbleTimer.start();
  }

  private void startSignal(final double time, final RGB rgb) {
    rumbleEndTime = time;
    setFullStrip(rgb);
    ledStrip.setData(ledBuffer);
    rumbleTimer.reset();
    rumbleTimer.start();
  }

  private void startSignal(final double time, final double intensity, final RGB rgb) {
    setRumble(intensity);

    rumbleEndTime = time;
    setFullStrip(rgb);
    ledStrip.setData(ledBuffer);
    rumbleTimer.reset();
    rumbleTimer.start();
  }

  private void resetLEDs() {
    setFullStrip(RGB.BLACK);
    ledStrip.setData(ledBuffer);
  }

  public void endSignal() {
    rumbleTimer.reset();
    rumbleTimer.stop();
    setRumble(0);
    resetLEDs();
  }

  private void setFullStrip(final RGB rgb) {
    setSection(rgb, 0, SignalingConstants.NUMBER_OF_LEDS);
  }

  private void setHalfStrip(final RGB rgb) {
    setFullStrip(RGB.BLACK);
    for (var i = 0; i < SignalingConstants.NUMBER_OF_LEDS; i += 2) {
      ledBuffer.setRGB(
          i,
          (int) (rgb.red * SignalingConstants.LED_BRIGHTNESS),
          (int) (rgb.green * SignalingConstants.LED_BRIGHTNESS),
          (int) (rgb.blue * SignalingConstants.LED_BRIGHTNESS));
    }
    ledStrip.setData(ledBuffer);
  }

  private void setSection(final RGB rgb, final int startID, final int count) {
    for (var i = startID; i < startID + count; i++) {
      if (i > -1 && i < SignalingConstants.NUMBER_OF_LEDS) {
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

    // DeltaBoard.putString("Disable Pattern", disablePattern.name());

    switch (disablePattern) {
        // case BI_FLAG:
        //   pattern = BIFlag.getColors(patternTick);
        //   break;
      case RAINBOW:
        pattern = RainbowPattern.getPattern();
        patternLength = RainbowPattern.getPatternLength();
        break;
      default:
        pattern = RainbowPattern.getPattern();
        patternLength = RainbowPattern.getPatternLength();
        break;
    }
    int patternIndex = 0;
    patternTick %= patternLength;
    int LEDIndex = -patternTick - 1;
    while (LEDIndex < SignalingConstants.NUMBER_OF_LEDS) {
      patternIndex %= pattern.length;

      PatternNode node = pattern[patternIndex];
      RGB c = pattern[patternIndex].color;
      Logger.recordOutput("Signaling/LED Color", c.toHex());
      setSection(c, LEDIndex, node.repeat);
      LEDIndex += node.repeat;
      patternIndex += 1;
    }
    ledStrip.setData(ledBuffer);
  }

  public void randomizePattern() {
    disablePattern = DisablePattern.getRandom();
  }

  private enum DisablePattern {
    RAINBOW;

    public static DisablePattern getRandom() {
      DisablePattern[] allPatterns = DisablePattern.values();
      return allPatterns[(int) Math.floor(Math.random() * (allPatterns.length))];
    }
  }

  public void clearLEDs() {
    setFullStrip(RGB.BLACK);
    ledStrip.setData(ledBuffer);
  }
}
