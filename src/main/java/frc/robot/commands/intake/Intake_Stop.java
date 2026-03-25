package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.outOfBumperIntake;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.Constants;

public class Intake_Stop extends Command {

    IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
    HopperSubsystem hopperSubsystem = new HopperSubsystem();

    public Intake_Stop(IntakeSubsystem intakeSubsystem, HopperSubsystem hopperSubsystem) {

        this.intakeSubsystem = intakeSubsystem;
        this.hopperSubsystem = hopperSubsystem;
        addRequirements(intakeSubsystem, hopperSubsystem);

    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {

    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        hopperSubsystem.pivotIntake(0); // TODO: tune the target positions!
        intakeSubsystem.setIntakeVoltage(0);
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
