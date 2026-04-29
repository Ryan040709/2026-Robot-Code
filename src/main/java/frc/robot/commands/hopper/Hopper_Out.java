package frc.robot.commands.hopper;
 
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.HopperSubsystem;

public class Hopper_Out extends Command {
  HopperSubsystem hopperSubsystem;
  IntakeSubsystem intakeSubsystem;

  /** Creates a new Hood_RunToPosition. */
  public Hopper_Out(HopperSubsystem hopperSubsystem, IntakeSubsystem intakeSubsystem) {
    this.hopperSubsystem = hopperSubsystem;
    this.intakeSubsystem = intakeSubsystem;
    addRequirements(hopperSubsystem);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    hopperSubsystem.setHopperPosition(31); 

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return hopperSubsystem.atPosition(31.5);
  }
}
