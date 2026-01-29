package frc.robot.subsystems.IntakeSubsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.HopperSubsystem;

public class throughBumperIntake extends SubsystemBase {

    private TalonFX intakeMotor1 = new TalonFX(10);
    private TalonFX intakeMotor2 = new TalonFX(10);

    HopperSubsystem hopperSubsystem = new HopperSubsystem();

    public void MoveintakeMotor1(double targetSpeed) {
        intakeMotor1.set(targetSpeed);
    }

    public void MoveintakeMotor2(double targetSpeed) {
        intakeMotor2.set(targetSpeed);
    }

    public Command IntakeToHopperCommand() {
        return runOnce(() -> {
            MoveintakeMotor2(-0.5);
            MoveintakeMotor1(.5);
            hopperSubsystem.MoveHopperMotor(.5);
        });
    }

    public Command IntakeToTurretCommand() {
        return runOnce(() -> {
            MoveintakeMotor2(-0.5);
            MoveintakeMotor1(.5);
            hopperSubsystem.MoveHopperMotor(.5);

        });
    }

    public Command HopperToTurretCommand() {
        return runOnce(() -> {
            MoveintakeMotor2(-0.5);
            MoveintakeMotor1(.5);
            hopperSubsystem.MoveHopperMotor(.5);

        });
    }

    public Command HopperToIntakeCommand() {
        return runOnce(() -> {
            MoveintakeMotor2(-0.5);
            MoveintakeMotor1(.5);
            hopperSubsystem.MoveHopperMotor(.5);

        });
    }

    public Command IntakeStop() {
        return runOnce(() -> {
            MoveintakeMotor2(0);
            MoveintakeMotor1(0);
            hopperSubsystem.MoveHopperMotor(0);

        });
    }

}