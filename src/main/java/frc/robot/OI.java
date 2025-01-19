// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController;
import utilities.OI_Utils;

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

    private static final ControlCurve xTranslationCurve = new ControlCurve(1, 0, 1, 0.01, true);
    private static final ControlCurve yTranslationCurve = new ControlCurve(1, 0, 1, 0.01);
    private static final ControlCurve xPrecisionCurve = new ControlCurve(0.5, 0, 1, 0.01, true);
    private static final ControlCurve yPrecisionCurve = new ControlCurve(0.5, 0, 1, 0.01);
    public static final ControlCurve translationMagnitudeCurve = new ControlCurve(1, 0, 1, 0.01);
    public static final ControlCurve rotationCurve = new ControlCurve(1, 0, 1, 0.01, true);
    private static final ControlCurve rotationPrecisionCurve =
        new ControlCurve(0.5, 0, 1, 0.01, true);
    // Joystick Axes
    public static final Control LeftX =
        new Control(XboxController.Axis.kLeftX, "", controller, xTranslationCurve);
    public static final Control LeftY =
        new Control(XboxController.Axis.kLeftY, "", controller, yTranslationCurve);
    public static final Control RightX =
        new Control(XboxController.Axis.kRightX, "", controller, rotationCurve);
    public static final Control RightY =
        new Control(XboxController.Axis.kRightY, null, controller, null);

    // Precision Joystick Axes
    public static final Control LeftPrecisionX =
        new Control(XboxController.Axis.kLeftX, "", controller, xPrecisionCurve);
    public static final Control LeftPrecisionY =
        new Control(XboxController.Axis.kLeftY, "", controller, yPrecisionCurve);
    public static final Control RightPrecisionX =
        new Control(XboxController.Axis.kRightX, "", controller, rotationPrecisionCurve);

    public static void setRumble(double rumbleIntensity) {
      controller.setRumble(RumbleType.kBothRumble, rumbleIntensity);
    }
  }

  public static final class Operator {
    public static final XboxController controller = new XboxController(operatorJoystickPort);

    // A B Y X Buttons
    public static final Control A = new Control(XboxController.Button.kA, "", controller);
    public static final Control B = new Control(XboxController.Button.kB, "", controller);
    public static final Control X = new Control(XboxController.Button.kX, "", controller);
    public static final Control Y = new Control(XboxController.Button.kY, "", controller);

    // Bumpers & Triggers
    public static final Control LTrigger =
        new Control(XboxController.Axis.kLeftTrigger, "The robot unclimbs.", controller, 0.5);
    public static final Control RTrigger =
        new Control(XboxController.Axis.kRightTrigger, "The robot climbs.", controller, 0.5);
    public static final Control LBumper =
        new Control(XboxController.Button.kLeftBumper, "", controller);
    public static final Control RBumper =
        new Control(
            XboxController.Button.kRightBumper,
            "The robot enables the climber when you hold the button.",
            controller);

    // Start, End & Left/Right stick buttons
    public static final Control Start = new Control(XboxController.Button.kStart, "", controller);
    public static final Control LeftStick =
        new Control(XboxController.Button.kLeftStick, null, controller);
    public static final Control RightStick =
        new Control(XboxController.Button.kRightStick, null, controller);
    public static final Control Back = new Control(XboxController.Button.kBack, null, controller);

    // Control Curves
    private static final ControlCurve xTranslationCurve = new ControlCurve(1, 0, 1, 0.0);
    private static final ControlCurve yTranslationCurve = new ControlCurve(1, 0, 1, 0.0);
    public static final ControlCurve translationMagnitudeCurve = new ControlCurve(1, 0, 1, 0.0);
    public static final ControlCurve rotationCurve = new ControlCurve(0.8, 0, 1, 0.0, true);

    // Joystick Axes
    public static final Control LeftX =
        new Control(XboxController.Axis.kLeftX, "", controller, xTranslationCurve);
    public static final Control LeftY =
        new Control(XboxController.Axis.kLeftY, "", controller, yTranslationCurve);
    public static final Control RightX =
        new Control(XboxController.Axis.kRightX, "", controller, rotationCurve);
    public static final Control RightY =
        new Control(XboxController.Axis.kRightY, null, controller, null);

    public static void setRumble(double rumbleIntensity) {
      controller.setRumble(RumbleType.kBothRumble, rumbleIntensity);
    }
  }
}
