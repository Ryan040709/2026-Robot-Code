package frc.robot.commands.autoPaths;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class drive_Path extends Command {
  CommandSwerveDrivetrain drivetrain;
  Command delegate;

  public drive_Path(CommandSwerveDrivetrain drivetrain){
    this.drivetrain = drivetrain;
    addRequirements(drivetrain);
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(true){
      try{
        delegate = AutoBuilder.followPath(PathPlannerPath.fromPathFile("splitPath2"));
      }catch(Exception e){
        try{
          NamedCommands.registerCommand("right sweep", AutoBuilder.followPath(
            PathPlannerPath.fromPathFile("splitPath3")));
          }catch(Exception e2){}
      }
    }else{
      try{
        delegate = AutoBuilder.followPath(PathPlannerPath.fromPathFile("splitPath3"));
      }catch(Exception e){
        try{
          NamedCommands.registerCommand("right sweep", AutoBuilder.followPath(
            PathPlannerPath.fromPathFile("splitPath2")));
          }catch(Exception e2){}
      }
    }
    delegate.initialize();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    delegate.execute();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    delegate.end(interrupted);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return delegate.isFinished();
  }
}
