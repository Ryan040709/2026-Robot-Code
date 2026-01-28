package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class intake extends SubsystemBase{

    private TalonFX intakeMotor1 = new TalonFX(10);
    private TalonFX intakeMotor2 = new TalonFX(10);

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
        });
    }

    public Command IntakeToTurretCommand() {
        return runOnce(() -> {
            MoveintakeMotor2(-0.5);
            MoveintakeMotor1(.5);    
        });
    }
    public Command HopperToTurretCommand() {
        return runOnce(() -> {
            MoveintakeMotor2(-0.5);
            MoveintakeMotor1(.5);
        });
    }

    public Command HopperToIntakeCommand() {
        return runOnce(() -> {
            MoveintakeMotor2(-0.5);
            MoveintakeMotor1(.5);
        });
    }

}