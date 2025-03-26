// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController;
import utilities.OI_Utils;
import utilities.OI_Utils.Control.ControlType;

/** Add your docs here. */
public class OI extends OI_Utils {

  public static final class Driver {
    public static final XboxController controller = new XboxController(driverJoystickPort);
    // A B Y X Buttons
    public static final Control A =
        new Control(XboxController.Button.kA, "The elevator moves L2.", controller);
    public static final Control B =
        new Control(XboxController.Button.kB, "The elevator moves to L3.", controller);
    public static final Control Y =
        new Control(XboxController.Button.kY, "The elevator moves to L4.", controller);
    public static final Control X =
        new Control(XboxController.Button.kX, "The elevator moves to the ground.", controller);

    // Bumpers & Triggers
    public static final Control LTrigger =
        new Control(
            XboxController.Axis.kLeftTrigger, "The robot scores the coral.", controller, 0.5);
    public static final Control LScoreTrigger =
        new Control(
            XboxController.Axis.kLeftTrigger, "Uses Trigger axis to run scorer", controller, 0.1);

    public static final Control RTrigger =
        new Control(XboxController.Axis.kRightTrigger, "The robot intakes.", controller, 0.5);
    public static final Control LBumper =
        new Control(XboxController.Button.kLeftBumper, "The robot unscores the coral.", controller);
    public static final Control RBumper =
        new Control(XboxController.Button.kRightBumper, "The robot outakes.", controller);

    // Start, End & Left/Right stick buttons
    public static final Control Start =
        new Control(XboxController.Button.kStart, "The robot gets zeroed.", controller);
    public static final Control Back =
        new Control(XboxController.Button.kBack, "The elevator moves to L1.", controller);
    public static final Control LSB =
        new Control(
            XboxController.Button.kLeftStick,
            "The robot auto aligns to the reef's wall.",
            controller);
    public static final Control RSB =
        new Control(
            XboxController.Button.kRightStick,
            "The gear gets switched to slow gear as you hold it.",
            controller);

    // POV Buttons
    public static final Control POV0 = new Control(0, "", controller);
    public static final Control POV90 = new Control(90, "The elevator goes up.", controller);
    public static final Control POV180 =
        new Control(180, "The robot switches to drive mode.", controller);
    public static final Control POV270 = new Control(270, "The elevator goes down.", controller);

    // POV Buttons
    public static final Control DPAD_UP = new Control(0, "", controller);
    public static final Control DPAD_RIGHT = new Control(90, "", controller);
    public static final Control DPAD_DOWN = new Control(180, "", controller);
    public static final Control DPAD_LEFT = new Control(270, "", controller);

    // Control Curves
    private static final ControlCurve xTranslationCurve = new ControlCurve(1, 0, 0.8, 0.1, true);
    private static final ControlCurve yTranslationCurve = new ControlCurve(1, 0, 0.8, 0.1, true);
    public static final ControlCurve translationMagnitudeCurve = new ControlCurve(1, 0, 1, 0.1);
    public static final ControlCurve rotationCurve = new ControlCurve(1, 0, 0.8, 0.15, true);
    public static final ControlCurve elevatorCurve = new ControlCurve(1, 0, 0, 0.1, true);
    public static final ControlCurve scorerCurve = new ControlCurve(1, 0, 0, 0.1, true);

    // Joystick Axes
    public static final Control LeftX =
        new Control(XboxController.Axis.kLeftX, null, controller, xTranslationCurve);
    public static final Control LeftY =
        new Control(XboxController.Axis.kLeftY, null, controller, yTranslationCurve);
    public static final Control RightX =
        new Control(XboxController.Axis.kRightX, null, controller, rotationCurve);
    public static final Control RightY =
        new Control(XboxController.Axis.kRightY, null, controller, elevatorCurve);
    public static final Control LeftTriggerAxis =
        new Control(
            XboxController.Axis.kLeftTrigger,
            "The robot scores the coral.",
            controller,
            scorerCurve);

    public static void setRumble(double rumbleIntensity) {
      controller.setRumble(RumbleType.kBothRumble, rumbleIntensity);
    }
  }

  public static final class Operator {
    public static final XboxController controller = new XboxController(operatorJoystickPort);

    // A B Y X Buttons
    public static final Control A = new Control(XboxController.Button.kA, null, controller);
    public static final Control B = new Control(XboxController.Button.kB, null, controller);
    public static final Control X = new Control(XboxController.Button.kX, null, controller);
    public static final Control Y = new Control(XboxController.Button.kY, null, controller);

    // Trigger Cruve
    public static final ControlCurve TriggerCurve = new ControlCurve(1, 0, 0, 0.1);

    // Bumpers & Triggers
    public static final Control LTrigger =
        new Control(XboxController.Axis.kLeftTrigger, "The robot unclimbs.", controller, 0.5);
    public static final Control LTriggerAxis =
        new Control(XboxController.Axis.kLeftTrigger, null, controller, TriggerCurve);
    public static final Control RTrigger =
        new Control(XboxController.Axis.kRightTrigger, "The robot climbs.", controller, 0.5);
    public static final Control RTriggerAxis =
        new Control(XboxController.Axis.kRightTrigger, null, controller, TriggerCurve);
    public static final Control LBumper =
        new Control(XboxController.Button.kLeftBumper, null, controller);
    public static final Control RBumper =
        new Control(
            XboxController.Button.kRightBumper,
            "The robot enables the climber when you hold the button.",
            controller);

    // Start, End & Left/Right stick buttons
    public static final Control Start = new Control(XboxController.Button.kStart, null, controller);
    public static final Control LeftStick =
        new Control(XboxController.Button.kLeftStick, null, controller);
    public static final Control RightStick =
        new Control(XboxController.Button.kRightStick, null, controller);
    public static final Control Back = new Control(XboxController.Button.kBack, null, controller);

    // POV Buttons
    public static final Control DPAD_UP = new Control(0, "", controller);
    public static final Control DPAD_RIGHT = new Control(90, "", controller);
    public static final Control DPAD_DOWN = new Control(180, "", controller);
    public static final Control DPAD_LEFT = new Control(270, "", controller);

    // Control Curves
    private static final ControlCurve xTranslationCurve = new ControlCurve(0.75, 0, 0.5, 0.0);
    private static final ControlCurve yTranslationCurve = new ControlCurve(0.75, 0, 0.5, 0.0);
    public static final ControlCurve translationMagnitudeCurve = new ControlCurve(1, 0, 1, 0.0);
    public static final ControlCurve rotationCurve = new ControlCurve(0.8, 0, 0.5, 0.0, true);

    // Joystick Axes
    public static final Control LeftX =
        new Control(XboxController.Axis.kLeftX, null, controller, xTranslationCurve);
    public static final Control LeftY =
        new Control(XboxController.Axis.kLeftY, null, controller, yTranslationCurve);
    public static final Control RightX =
        new Control(XboxController.Axis.kRightX, null, controller, rotationCurve);
    public static final Control RightY =
        new Control(XboxController.Axis.kRightY, null, controller, rotationCurve);

    public static void setRumble(double rumbleIntensity) {
      controller.setRumble(RumbleType.kBothRumble, rumbleIntensity);
    }
  }

  public static final class Keyboard {
    private static final XboxController keyboard1 = new XboxController(0);
    private static final XboxController keyboard2 = new XboxController(1);
    private static final XboxController keyboard3 = new XboxController(2);

    private static final ControlCurve defualtCurve = new ControlCurve(-1, 0, 0, 0);
    private static final ControlCurve TranslationCurveX = new ControlCurve(-1, 0, 0, 0);
    private static final ControlCurve TranslationCurveY = new ControlCurve(-1, 0, 0, 0);

    public static final Control Z = new Control(1, null, null, keyboard1, ControlType.BUTTON);
    public static final Control X = new Control(2, null, null, keyboard1, ControlType.BUTTON);
    public static final Control C = new Control(3, null, null, keyboard1, ControlType.BUTTON);
    public static final Control V = new Control(4, null, null, keyboard1, ControlType.BUTTON);
    public static final Control WS =
        new Control(XboxController.Axis.kLeftX, null, keyboard1, TranslationCurveX);
    public static final Control AD =
        new Control(XboxController.Axis.kLeftY, null, keyboard1, TranslationCurveY);

    public static final Control M = new Control(1, null, null, keyboard2, ControlType.BUTTON);
    public static final Control Comma = new Control(2, null, null, keyboard2, ControlType.BUTTON);
    public static final Control Period = new Control(3, null, null, keyboard2, ControlType.BUTTON);
    public static final Control ForwardSlash =
        new Control(4, null, null, keyboard2, ControlType.BUTTON);
    public static final Control IK =
        new Control(XboxController.Axis.kLeftX, null, keyboard2, defualtCurve);
    public static final Control JL =
        new Control(XboxController.Axis.kLeftY, null, keyboard2, defualtCurve);

    public static final Control ArrowUpDown =
        new Control(XboxController.Axis.kLeftY, null, keyboard3, defualtCurve);
    public static final Control ArrowLR =
        new Control(XboxController.Axis.kLeftX, null, keyboard3, defualtCurve);
  }

  public static final class StreamDeck {
    public static final XboxController streamDeck1 = new XboxController(2);
    public static final XboxController streamDeck2 = new XboxController(3);

    public static final ControlCurve nobCurve = new ControlCurve(1, 0, 0, 0);

    public static final Control Nob1 =
        new Control(XboxController.Axis.kLeftX, null, streamDeck1, nobCurve);
    public static final Control Nob2 =
        new Control(XboxController.Axis.kLeftY, null, streamDeck1, nobCurve);
    public static final Control Nob3 =
        new Control(XboxController.Axis.kLeftTrigger, null, streamDeck1, nobCurve);
    public static final Control Nob4 =
        new Control(XboxController.Axis.kRightTrigger, null, streamDeck1, nobCurve);

    public static Control[][] streamDeckButtons =
        new Control[2][Constants.kStreamDeckTotalButtonCount];

    static {
      for (int i = 0; i < Constants.kMaxControllerButtonCount; i++) {
        streamDeckButtons[0][i] = new Control(i + 1, null, null, streamDeck1, ControlType.BUTTON);
        streamDeckButtons[1][i] = new Control(i + 1, null, null, streamDeck2, ControlType.BUTTON);
      }
    }
  }
}
