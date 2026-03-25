package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.newRobotSubsystems.HopperSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.hoodSubsystem;

public class Intake_Run extends Command {

    IntakeSubsystem intakeSubsystem;
    HopperSubsystem hopperSubsystem;
    TurretSubsystem turretSubsystem;
    CommandSwerveDrivetrain drivetrain;

    public Intake_Run(
            IntakeSubsystem intakeSubsystem,
            HopperSubsystem hopperSubsystem,
            TurretSubsystem turretSubsystem,
            CommandSwerveDrivetrain drivetrain) {

        this.intakeSubsystem = intakeSubsystem;
        this.hopperSubsystem = hopperSubsystem;
        this.turretSubsystem = turretSubsystem;
        this.drivetrain = drivetrain;

        addRequirements(intakeSubsystem, hopperSubsystem);

    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {

    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (intakeSubsystem.robotIsNeartrench(drivetrain.getTurretOffset(), drivetrain.getRobotFieldRelativeSpeeds())) {
            if (turretSubsystem.turretAtTarget(
                    drivetrain.getTurretTarget(), drivetrain.getTurretOffset())) {

                intakeSubsystem.setIntakeVoltage(-7);
            } else {
                intakeSubsystem.setIntakeVoltage(-7);
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
