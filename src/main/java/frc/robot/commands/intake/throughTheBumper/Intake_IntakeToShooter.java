package frc.robot.commands.intake.throughTheBumper;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.throughBumperIntake;
import frc.robot.Constants;

public class Intake_IntakeToShooter extends Command {

    throughBumperIntake intakeSubsystem = new throughBumperIntake();

    public Intake_IntakeToShooter(throughBumperIntake intakeSubsystem){
 
        this.intakeSubsystem = intakeSubsystem;
        addRequirements(intakeSubsystem);

    }


    // Called when the command is initially scheduled.
    @Override
    public void initialize() {

    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        intakeSubsystem.SetIntakeFront(-.5);
        intakeSubsystem.SetIntakeBack(-.5);
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
