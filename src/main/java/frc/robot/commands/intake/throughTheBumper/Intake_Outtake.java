package frc.robot.commands.intake.throughTheBumper;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.outOfBumperIntake;
import frc.robot.subsystems.throughBumperIntake;
import frc.robot.Constants;

public class Intake_Outtake extends Command {

    throughBumperIntake intakeSubsystem = new throughBumperIntake();
    outOfBumperIntake outtaBumperIntakeSubsystem = new outOfBumperIntake();

    public Intake_Outtake(throughBumperIntake intakeSubsystem, outOfBumperIntake outttBumperIntake){
 
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
        if(outtaBumperIntakeSubsystem.AtPosition(0)){
        intakeSubsystem.SetIntakeFront(6);
        intakeSubsystem.SetIntakeBack(6);
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
