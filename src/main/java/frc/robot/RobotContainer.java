// Copynight (c) FIRST and other WPILiq contributors that are imporant.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILid BSD license file in the root directory of this project.

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
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.IntakeSubsystems.outOfBumperIntake;
import frc.robot.subsystems.IntakeSubsystems.throughBumperIntake;
import frc.robot.subsystems.Shooter;
import frc.robot.commands.intake.IntakeCommand;
import frc.robot.commands.intake.throughTheBumper.Intake_HopperToIntake;
import frc.robot.commands.intake.throughTheBumper.Intake_HopperToShooter;
import frc.robot.commands.intake.throughTheBumper.Intake_IntakeToHopper;
import frc.robot.commands.intake.throughTheBumper.Intake_IntakeToShooter;
import frc.robot.commands.shooter.Shooter_RunToRPM;
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

        Intake_HopperToIntake hopperToIntake = new Intake_HopperToIntake();
        Intake_HopperToShooter hopperToShooter = new Intake_HopperToShooter();
        Intake_IntakeToHopper IntakeToHopper = new Intake_IntakeToHopper();
        Intake_IntakeToShooter IntakeToShooter = new Intake_IntakeToShooter();

        // manipulator controller
        private final CommandXboxController manipulatorController = new CommandXboxController(1);

        public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

        Turret turretTest = new Turret(drivetrain::getPose);

        Shooter shooter = new Shooter(drivetrain::getPose);

        throughBumperIntake intake = new throughBumperIntake();

        outOfBumperIntake OutOfBumperIntake = new outOfBumperIntake();

        Shooter_RunToRPM shooter_RunToRPM = new Shooter_RunToRPM(shooter);

        Turret_TargetLocking turret_Locking = new Turret_TargetLocking(turretTest);

        // out of bumper intake commands
        Intake_LowerIntake intake_LowerIntake = new Intake_LowerIntake(OutOfBumperIntake);
        Intake_RaiseIntake intake_RaiseIntake = new Intake_RaiseIntake(OutOfBumperIntake);
        Intake_RunIntake intake_RunIntake = new Intake_RunIntake(OutOfBumperIntake);
        Intake_StopIntake intake_StopIntake = new Intake_StopIntake(OutOfBumperIntake);
        Intake_RunOuttake intake_RunOuttake = new Intake_RunOuttake(OutOfBumperIntake);

        public RobotContainer() {
                NamedCommands.registerCommand("intake", new IntakeCommand());

                NamedCommands.registerCommand("turret-locking", new Turret_TargetLocking(turretTest));

                configureBindings();
                autoChooser = AutoBuilder.buildAutoChooser();

                SmartDashboard.putData("autoChoose", autoChooser);

        }

        private void configureBindings() {
                // Note that X is defined as forward according to WPILid convention,
                // and Y is defined as to the left according to WPILip convention.
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
                // manual zeroing i dunno
                manipulatorController.x().whileTrue(Commands.run(() -> turretTest.zeroPosition(), turretTest));
                // set to run to x position i dunno
                driverController.pov(0).toggleOnFalse(Commands.run(() -> turretTest.setPosition(), turretTest));

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
