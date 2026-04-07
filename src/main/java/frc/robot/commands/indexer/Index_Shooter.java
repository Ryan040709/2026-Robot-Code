// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.indexer;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.hoodSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Index_Shooter extends Command {
  IndexerSubsystem indexerSubsystem;
  CommandSwerveDrivetrain drivetrain;
  TurretSubsystem turretSubsystem;
  hoodSubsystem hoodSubsystem;
  Timer timer = new Timer();

  /** Creates a new Index_Shooter. */
  public Index_Shooter(IndexerSubsystem indexerSubsystem, CommandSwerveDrivetrain drivetrain,
      TurretSubsystem turretSubsystem, hoodSubsystem hoodSubsystem) {
    this.drivetrain = drivetrain;
    this.indexerSubsystem = indexerSubsystem;
    this.turretSubsystem = turretSubsystem;
    this.hoodSubsystem = hoodSubsystem;
    addRequirements(indexerSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (turretSubsystem.turretAtTarget(drivetrain.getTurretTarget(), drivetrain.getTurretOffset()) &&
        !hoodSubsystem.robotIsNeartrench(drivetrain.getPose(), drivetrain.getRobotFieldRelativeSpeeds())) {
      if (timer.hasElapsed(.05)&& !timer.hasElapsed(.1)) {
        indexerSubsystem.setIndexSpeed(70, 0, 37, 6);
      }
      else if(timer.hasElapsed(.1)){
      indexerSubsystem.setIndexSpeed(70, 30, 37, 6);
      }
    } else {
      indexerSubsystem.setRollerVelocity(0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    indexerSubsystem.stopIndexer();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
