package frc.robot.subsystems.signaling;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.FireAnimation;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.CANIDs;
import frc.robot.Constants.SignalingConstants;
import frc.robot.OI;
import frc.robot.subsystems.signaling.patterns.AlliancePattern;
import frc.robot.subsystems.signaling.patterns.PatternNode;
import frc.robot.subsystems.signaling.patterns.RainbowPattern;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.util.LimelightHelpers;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class Signaling extends SubsystemBase {

  private final CANdle candle = new CANdle(CANIDs.kCANdle);
  private int tick;
  private int patternTick;
  private DisablePattern disablePattern = DisablePattern.getRandom();
  private PowerDistribution pdp;

  public enum LightState {
    IDLE,
    HAS_CORAL,
    HANDOFF_DONE,
    LL_HAS_TAG,
    AUTO_ALIGNING,
    ALGAE_MODE
  };

  public LightState currentState = LightState.IDLE;

  public Supplier<Boolean> hasCoral;
  public Supplier<Boolean> handoffComplete;
  public Supplier<Boolean> LLHasTag;
  public Supplier<Boolean> autoAligning;
  public Supplier<Boolean> AlgaeMode;

  public Signaling(PowerDistribution power) {
    tick = 0;
    patternTick = 0;
    pdp = power;
    candle.configBrightnessScalar(SignalingConstants.kLEDBrightness);
  }

  private void setState(LightState newState) {
    switch (newState) {
      case IDLE:
        setColor(RGB.RED);
        break;
      case HAS_CORAL:
        setColor(RGB.ORANGE);
        break;
      case HANDOFF_DONE:
        setColor(RGB.YELLOW);
        break;
      case LL_HAS_TAG:
        setColor(RGB.BLUE);
        break;
      case AUTO_ALIGNING:
        setColor(RGB.GREEN);
        break;
      case ALGAE_MODE:
        setColor(RGB.PURPLE);
        break;
      default:
        break;
    }
    return;
  }

  @Override
  public void periodic() {
    // Update Light Pattern

    if (DriverStation.isDisabled()) {
      if (LimelightHelpers.getTV(VisionConstants.camera0Name)) {
        setCandle(RGB.RED);
        Logger.recordOutput("Signaling/CANdle Color", "Red");
      } else {
        setCandle(RGB.GREEN);
        Logger.recordOutput("Signaling/CANdle Color", "Green");
      }
    } else {
      switch (currentState) {
        case IDLE:
          if (AlgaeMode.get()) {
            setState(LightState.ALGAE_MODE);
          } else if (hasCoral.get()) {
            setState(LightState.HAS_CORAL);
          }
          break;
        case ALGAE_MODE:
          if (!AlgaeMode.get()) {
            setState(LightState.IDLE);
          }
          break;
        case HAS_CORAL:
          if (!hasCoral.get()) {
            setState(LightState.IDLE);
          }
          if (handoffComplete.get()) {
            setState(LightState.HANDOFF_DONE);
          }
          break;
        case HANDOFF_DONE:
          if (!handoffComplete.get()) {
            setState(LightState.IDLE);
          }
          if (LLHasTag.get()) {
            setState(LightState.LL_HAS_TAG);
          }
          break;
        case LL_HAS_TAG:
          if (!LLHasTag.get()) {
            setState(LightState.HANDOFF_DONE);
          }
          if (autoAligning.get()) {
            setState(LightState.AUTO_ALIGNING);
          }
          if (!handoffComplete.get()) {
            setState(LightState.IDLE);
          }
          break;
        case AUTO_ALIGNING:
          if (!autoAligning.get()) {
            setState(LightState.LL_HAS_TAG);
          }
          break;
        default:
          break;
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
    SmartDashboard.putString("Signaling/CANdle Color", rgb.toHex());
    setSection(rgb, 0, 100);
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
    OI.Operator.setRumble(intensity);
  }

  public Command startSignal(RGB color) {
    return startEnd(
        () -> {
          setFullStrip(color);
          setRumble(1);
        },
        () -> {
          resetLEDs();
          setRumble(0);
        });
  }

  private void resetLEDs() {
    setFullStrip(RGB.BLACK);
  }

  private void setFullStrip(final RGB rgb) {
    Logger.recordOutput("Signaling/LED Color", rgb.toHex());
    setSection(rgb, 8, SignalingConstants.kNumLEDs);
  }

  private void setSection(final RGB rgb, final int startID, final int count) {
    candle.setLEDs(rgb.red, rgb.green, rgb.blue, 0, startID, count);
  }

  private void setSectionStrip(final RGB rgb, final int startID, final int count) {
    if (startID == 10) {
      Logger.recordOutput("Signaling/LED Color", rgb.toHex());
    }
    setSection(rgb, startID + 8, count);
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
      setSectionStrip(color, LEDIndex + 9, node.repeat);
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
