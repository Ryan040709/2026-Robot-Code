package frc.robot.commands.intake.throughTheBumper;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.outOfBumperIntake;
import frc.robot.subsystems.throughBumperIntake;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.hoodSubsystem;

public class Auto_Intake_IntakeToShooter extends Command {

    throughBumperIntake intakeSubsystem;
    outOfBumperIntake outtaBumperIntakeSubsystem;


    public Auto_Intake_IntakeToShooter(throughBumperIntake intakeSubsystem, outOfBumperIntake outttBumperIntake) {

        this.intakeSubsystem = intakeSubsystem;
        this.outtaBumperIntakeSubsystem = outttBumperIntake;
        addRequirements(intakeSubsystem, outttBumperIntake);

    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {

    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        outtaBumperIntakeSubsystem.PivotIntake(0);
        if (outtaBumperIntakeSubsystem.AtPosition(0)) {
                intakeSubsystem.SetIntakeFront(-7);
                intakeSubsystem.SetIntakeBack(-7);
          
        }

        // outta bumper intake
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
