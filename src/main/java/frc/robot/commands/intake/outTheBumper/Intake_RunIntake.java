package frc.robot.commands.intake.outTheBumper;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.IntakeSubsystems.outOfBumperIntake;

import frc.robot.Constants;

public class Intake_RunIntake extends Command {
  outOfBumperIntake s_outOfBumperIntake;

  /** Creates a new Hood_RunToPosition. */
  public Intake_RunIntake(outOfBumperIntake s_outOfBumperIntake) {
    this.s_outOfBumperIntake = s_outOfBumperIntake;
    addRequirements(s_outOfBumperIntake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    s_outOfBumperIntake.IntakeSpeed(Constants.OutOfBumperIntakeSubsystem.OutBumperIntake_IntakeVelocity);
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
