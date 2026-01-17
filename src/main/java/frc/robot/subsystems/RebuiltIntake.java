package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;

public class RebuiltIntake {
    private TalonFX intakeMotor;

    public RebuiltIntake() {
        intakeMotor = new TalonFX(Constants.RebuiltIntakeConstants.motorID);
    }

    public Command setSpeed(double speed) {
        return Commands.runOnce(() ->intakeMotor.set(speed));
    }

    public Command stop() {
        return Commands.runOnce(() ->intakeMotor.set(0));
    }

    public Command intakeCommand() {
        return Commands.runEnd(() -> setSpeed(1), () -> stop());
    }
}