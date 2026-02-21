package frc.robot.commands.intake.throughTheBumper;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.throughBumperIntake;

public class Intake_Stop extends Command {

    HopperSubsystem hopperSubsystem = new HopperSubsystem();
    throughBumperIntake intakeSubsystem = new throughBumperIntake();

    public Intake_Stop(HopperSubsystem hopperSubsystem,
     throughBumperIntake intakeSubsystem){
 
        this.hopperSubsystem = hopperSubsystem;
        this.intakeSubsystem = intakeSubsystem;
        addRequirements(hopperSubsystem, intakeSubsystem);

    }


    // Called when the command is initially scheduled.
    @Override
    public void initialize() {

    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        intakeSubsystem.IntakeStop(0);
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
