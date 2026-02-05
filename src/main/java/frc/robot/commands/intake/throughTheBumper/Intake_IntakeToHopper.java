package frc.robot.commands.intake.throughTheBumper;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.IntakeSubsystems.throughBumperIntake;
import frc.robot.subsystems.HopperSubsystem;

import frc.robot.Constants;

public class Intake_IntakeToHopper extends Command {

    HopperSubsystem hopperSubsystem = new HopperSubsystem();
    throughBumperIntake intakeSubsystem = new throughBumperIntake();

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {

    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        intakeSubsystem
                .IntakeToHopperCommand(Constants.ThroughBumperIntakeSubsystem.ThroughBumperIntake_IntakeVelocity);
        hopperSubsystem.MoveHopperMotor(.5);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {

    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
