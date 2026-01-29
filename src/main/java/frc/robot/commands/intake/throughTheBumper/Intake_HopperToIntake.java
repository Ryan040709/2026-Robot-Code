package frc.robot.commands.intake.throughTheBumper;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.IntakeSubsystems.throughBumperIntake;
import frc.robot.subsystems.HopperSubsystem;

public class Intake_HopperToIntake extends Command {

    private TalonFX intakeMotor1 = new TalonFX(10);
    private TalonFX intakeMotor2 = new TalonFX(10);

    HopperSubsystem hopperSubsystem = new HopperSubsystem();
    throughBumperIntake intakeSubsystem = new throughBumperIntake();

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {

    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        intakeSubsystem.MoveintakeMotor2(-0.5);
        intakeSubsystem.MoveintakeMotor1(.5);//TODO update these
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
