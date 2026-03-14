package frc.robot.commands.intake.throughTheBumper;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.outOfBumperIntake;
import frc.robot.subsystems.throughBumperIntake;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.hoodSubsystem;

public class Intake_IntakeToShooter extends Command {

    throughBumperIntake intakeSubsystem;
    outOfBumperIntake outtaBumperIntakeSubsystem;
    TurretSubsystem turretSubsystem;
    hoodSubsystem hoodSubsystem;

    CommandSwerveDrivetrain drivetrain;

    public Intake_IntakeToShooter(throughBumperIntake intakeSubsystem, outOfBumperIntake outttBumperIntake,
            TurretSubsystem turretSubsystem, CommandSwerveDrivetrain drivetrain,
            hoodSubsystem hoodSubsystem) {

        this.intakeSubsystem = intakeSubsystem;
        this.outtaBumperIntakeSubsystem = outttBumperIntake;
        this.turretSubsystem = turretSubsystem;
        this.drivetrain = drivetrain;
        this.hoodSubsystem = hoodSubsystem;
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
            if (turretSubsystem.turretAtTarget(drivetrain.getTurretTarget(), drivetrain.getTurretOffset())&&
                !hoodSubsystem.robotIsNeartrench(drivetrain.getPose(), drivetrain.getFieldRelativeSpeeds())) {
                intakeSubsystem.SetIntakeFront(-7);
                intakeSubsystem.SetIntakeBack(-7);
            } else {
                intakeSubsystem.SetIntakeFront(-7);
                intakeSubsystem.SetIntakeBack(7);
            }
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
