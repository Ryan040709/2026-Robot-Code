package frc.robot.commands.intake.throughTheBumper;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.newRobotSubsystems.HopperSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.hoodSubsystem;

public class Auto_Intake_IntakeToShooter extends Command {

    IntakeSubsystem intakeSubsystem;
    HopperSubsystem HopperSubsystem;


    public Auto_Intake_IntakeToShooter(IntakeSubsystem intakeSubsystem, HopperSubsystem HopperSubsystem) {

        this.intakeSubsystem = intakeSubsystem;
        this.HopperSubsystem = HopperSubsystem;
        addRequirements(intakeSubsystem, HopperSubsystem);

    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {

    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        HopperSubsystem.pivotIntake(0);
        if (HopperSubsystem.atPosition(0)) {
                intakeSubsystem.setIntakeVoltage(-7);
          
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
