package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.IntakeSubsystems.outOfBumperIntake;
import frc.robot.subsystems.IntakeSubsystems.throughBumperIntake;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.commands.intake.throughTheBumper.Intake_HopperToIntake;
import frc.robot.commands.intake.throughTheBumper.Intake_HopperToShooter;
import frc.robot.commands.intake.throughTheBumper.Intake_IntakeToHopper;
import frc.robot.commands.intake.throughTheBumper.Intake_IntakeToShooter;
import frc.robot.commands.shooter.Shooter_RunToRPM;
import frc.robot.commands.shooter.Hood_SetToPosition;
//out of bumper intake commands
import frc.robot.commands.intake.outTheBumper.Intake_LowerIntake;
import frc.robot.commands.intake.outTheBumper.Intake_RaiseIntake;
import frc.robot.commands.intake.outTheBumper.Intake_RunOuttake;
import frc.robot.commands.intake.outTheBumper.Intake_RunIntake;
import frc.robot.commands.intake.outTheBumper.Intake_StopIntake;
//turret commands
import frc.robot.commands.turret.Turret_TargetLocking;

public class RobotContainer {

        private double MaxSpeed = 0.5 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired
                                                                                            // top
        // speed
        private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per
                                                                                          // second
                                                                                          // max angular velocity

        private final SendableChooser<Command> autoChooser;

        /* Setting up bindings for necessary control of the swerve drive platform */
        private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
                        .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
                        .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive
                                                                                 // motors
        private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
        private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

        private final Telemetry logger = new Telemetry(MaxSpeed);

        private final CommandXboxController driverController = new CommandXboxController(0);

        // manipulator controller
        private final CommandXboxController manipulatorController = new CommandXboxController(1);

        public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
        // turret subsystem
        TurretSubsystem turretTest = new TurretSubsystem(drivetrain::getPose);
        // shooter subsystem
        ShooterSubsystem shooter = new ShooterSubsystem(drivetrain::getPose);
        // in the bumper intake subsystem
        throughBumperIntake intake = new throughBumperIntake();
        // out of bumper intake subsystem
        outOfBumperIntake OutOfBumperIntake = new outOfBumperIntake();

        // shooter commands
        Shooter_RunToRPM shooter_RunToRPM = new Shooter_RunToRPM(shooter);
        Hood_SetToPosition shooter_setToPosition = new Hood_SetToPosition(shooter);
        // turret commands
        Turret_TargetLocking turret_Locking = new Turret_TargetLocking(turretTest);
        // out of bumper intake commands
        Intake_LowerIntake intake_LowerIntake = new Intake_LowerIntake(OutOfBumperIntake);
        Intake_RaiseIntake intake_RaiseIntake = new Intake_RaiseIntake(OutOfBumperIntake);
        Intake_RunIntake intake_RunIntake = new Intake_RunIntake(OutOfBumperIntake);
        Intake_StopIntake intake_StopIntake = new Intake_StopIntake(OutOfBumperIntake);
        Intake_RunOuttake intake_RunOuttake = new Intake_RunOuttake(OutOfBumperIntake);
        // in the bumper intake commands
        Intake_HopperToIntake hopperToIntake = new Intake_HopperToIntake();
        Intake_HopperToShooter hopperToShooter = new Intake_HopperToShooter();
        Intake_IntakeToHopper IntakeToHopper = new Intake_IntakeToHopper();
        Intake_IntakeToShooter IntakeToShooter = new Intake_IntakeToShooter();

        public RobotContainer() {
                // turret commands
                NamedCommands.registerCommand("turret-locking", turret_Locking);
                // out of bumper intake commands
                NamedCommands.registerCommand("intake-lower", intake_LowerIntake);
                NamedCommands.registerCommand("intake-raise", intake_RaiseIntake);
                NamedCommands.registerCommand("intake-intake", intake_RunIntake);
                NamedCommands.registerCommand("intake-stop", intake_StopIntake);
                NamedCommands.registerCommand("intake-stop", intake_RunOuttake);
                // in the bumper intake commands
                NamedCommands.registerCommand("intake-hopperToIntake", hopperToIntake);
                NamedCommands.registerCommand("intake-hopperToShooter", hopperToShooter);
                NamedCommands.registerCommand("intake-intakeToHopper", IntakeToHopper);
                NamedCommands.registerCommand("intake-intakeToShooter", IntakeToShooter);
                // shooter commands
                NamedCommands.registerCommand("intake-intakeToHopper", IntakeToHopper);
                NamedCommands.registerCommand("intake-intakeToShooter", IntakeToShooter);
                // climber commands

                configureBindings();
                autoChooser = AutoBuilder.buildAutoChooser();

                SmartDashboard.putData("autoChoose", autoChooser);

        }

        private void configureBindings() {
                // Note that X is defined as forward according to WPILib convention,
                // and Y is defined as to the left according to WPILib convention.
                drivetrain.setDefaultCommand(
                                // Drivetrain will execute this command periodically
                                drivetrain.applyRequest(() -> drive
                                                .withVelocityX(-driverController.getLeftY() * MaxSpeed) // Drive
                                                                                                        // forward
                                                                                                        // with
                                                                                                        // negative Y
                                                                                                        // (forward)
                                                .withVelocityY(-driverController.getLeftX() * MaxSpeed) // Drive left
                                                                                                        // with negative
                                                                                                        // X (left)
                                                .withRotationalRate(-driverController.getRightX() * MaxAngularRate) // Drive
                                                                                                                    // counterclockwise
                                                                                                                    // with
                                                                                                                    // negative
                                                                                                                    // X
                                                                                                                    // (left)
                                ));

                // Idle while the robot is disabled. This ensures the configured
                // neutral mode is applied to the drive motors while disabled.
                final var idle = new SwerveRequest.Idle();
                RobotModeTriggers.disabled().whileTrue(
                                drivetrain.applyRequest(() -> idle).ignoringDisable(true));

                driverController.a().whileTrue(drivetrain.applyRequest(() -> brake));
                driverController.b().whileTrue(drivetrain.applyRequest(() -> point
                                .withModuleDirection(new Rotation2d(-driverController.getLeftY(),
                                                -driverController.getLeftX()))));

                // Run SysId routines when holding back/start and X/Y.
                // Note that each routine should be run exactly once in a single log.
                driverController.back().and(driverController.y())
                                .whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
                driverController.back().and(driverController.x())
                                .whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
                driverController.start().and(driverController.y())
                                .whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
                driverController.start().and(driverController.x())
                                .whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

                // Reset the field-centric heading on left bumper press.
                driverController.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

                drivetrain.registerTelemetry(logger::telemeterize);

                manipulatorController.leftTrigger(0.05)
                                .whileTrue(Commands.run(() -> turretTest.MoveMotor(manipulatorController.getLeftX()),
                                                turretTest));
                // manual zeroing
                manipulatorController.x().whileTrue(Commands.run(() -> turretTest.zeroPosition(), turretTest));
                // set to run to x position
                driverController.pov(0).toggleOnFalse(turret_Locking);

                driverController.pov(90)
                                .whileTrue(Commands.run(() -> drivetrain.resetPose(new Pose2d(8, 4, new Rotation2d(0))),
                                                drivetrain));

                manipulatorController.a().whileTrue(Commands.run(() -> turretTest.setToZero(), turretTest));

                manipulatorController.pov(0).whileTrue(hopperToShooter);

                manipulatorController.pov(90).whileTrue(hopperToIntake);

                manipulatorController.pov(180).whileTrue(IntakeToHopper);

                manipulatorController.pov(270).whileTrue(IntakeToShooter);

                manipulatorController.leftTrigger(0.05).whileTrue(shooter_RunToRPM);

                // out of bumper intake commands
                manipulatorController.leftBumper().whileTrue(intake_LowerIntake);
                manipulatorController.rightBumper().whileTrue(intake_RaiseIntake);

        }

        public Command getAutonomousCommand() {

                return autoChooser.getSelected();
        }

}
