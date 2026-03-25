package frc.robot;

import static edu.wpi.first.units.Units.*;

import org.ejml.generic.GenericMatrixOps_F32;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.GameManager;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.hoodSubsystem;
import frc.robot.subsystems.outOfBumperIntake;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.commands.intake.Intake_Run;
import frc.robot.commands.intake.Intake_Outtake;
import frc.robot.commands.intake.Intake_Stop;
import frc.robot.commands.shooter.Shooter_RunToRPM;
import frc.robot.commands.shooter.Hood_SetToPosition;
import frc.robot.commands.shooter.OverrideHoodPosition;
import frc.robot.commands.shooter.ShooterStop;
import frc.robot.commands.shooter.ShooterToRPMS;
import frc.robot.commands.SetLEDColor;
import frc.robot.commands.resetLEDTimer;
import frc.robot.commands.hopper.Hopper_PivotDown;
import frc.robot.commands.hopper.Hopper_PivotUp;
import frc.robot.commands.turret.Turret_Locking;
//turret commands
import frc.robot.commands.turret.Turret_Toggle;
//get the game manager
import frc.robot.commands.turret.ZeroTurret;

public class RobotContainer {

        // kSpeedAt12Volts desired top speed
        private double MaxSpeed = 1 * SwerveConstants.kSpeedAt12Volts.in(MetersPerSecond); // keep at 0.5, Andy said
                                                                                           // so...
        private double OverrideSpeed = .75;

        // was .75 if too fast
        private double MaxAngularRate = RotationsPerSecond.of(1).in(RadiansPerSecond);

        private final SendableChooser<Command> autoChooser;

        /* Setting up bindings for necessary control of the swerve drive platform */ // fancy comment [;
        private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
                        // Add a 10% deadband
                        .withDeadband(MaxSpeed * 0.05).withRotationalDeadband(MaxAngularRate * 0.05)
                        // Use open-loop control for drive motors
                        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

        private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
        private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

        private final Telemetry logger = new Telemetry(MaxSpeed);

        private final CommandXboxController driverController = new CommandXboxController(0);

        // manipulator controller
        private final CommandXboxController manipulatorController = new CommandXboxController(1);

        public final CommandSwerveDrivetrain drivetrain = SwerveConstants.createDrivetrain();
        // turret subsystem
        TurretSubsystem turret = new TurretSubsystem();
        // shooter subsystem
        ShooterSubsystem shooter = new ShooterSubsystem();
        // in the bumper intake subsystem
        IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
        // hopper subsystem
        HopperSubsystem hopperSubsystem = new HopperSubsystem();
        // hood subsystem
        hoodSubsystem hoodSubsystem = new hoodSubsystem();
        // game manager
        GameManager gameManager = new GameManager();
        // indexer subsystem
        IndexerSubsystem indexerSubsystem = new IndexerSubsystem();

        // shooter commands
        Shooter_RunToRPM shooter_RunToRPM = new Shooter_RunToRPM(shooter, drivetrain, indexerSubsystem);
        Shooter_RunToRPM shooter_RunToRpmTele = new Shooter_RunToRPM(shooter, drivetrain, indexerSubsystem);
        Hood_SetToPosition hoodSetToPosition = new Hood_SetToPosition(hoodSubsystem, drivetrain);
        Hood_SetToPosition hoodSetToPositionDRIVER = new Hood_SetToPosition(hoodSubsystem, drivetrain);
        OverrideHoodPosition hoodDown = new OverrideHoodPosition(hoodSubsystem);
        ShooterToRPMS shooterCoast = new ShooterToRPMS(shooter, drivetrain, indexerSubsystem);
        ShooterStop shooterStop = new ShooterStop(shooter, drivetrain, indexerSubsystem);
        // turret commands
        Turret_Toggle turret_Toggle = new Turret_Toggle(turret);
        Turret_Locking turret_Locking = new Turret_Locking(turret, drivetrain);
        ZeroTurret zeroTurret = new ZeroTurret(turret, drivetrain);
        // out of bumper intake commands
        Hopper_PivotUp hopper_Out = new Hopper_PivotUp(hopperSubsystem);
        Hopper_PivotDown hopper_In = new Hopper_PivotDown(hopperSubsystem);

        // in the bumper intake commands
        Intake_Stop intake_Stop = new Intake_Stop(intakeSubsystem, hopperSubsystem);
        Intake_Outtake intake_Outtake = new Intake_Outtake(intakeSubsystem, hopperSubsystem);
        Intake_Run intake_Run = new Intake_Run(intakeSubsystem, hopperSubsystem, turret, drivetrain);

        Command shooterAndHood = shooter_RunToRpmTele.alongWith(hoodSetToPosition);

        public RobotContainer() {
                // turret commands
                NamedCommands.registerCommand("turret-locking", turret_Locking);
                // in the bumper intake commands
                NamedCommands.registerCommand("intake-hopperToIntake", intake_Outtake);
                NamedCommands.registerCommand("intake-intakeToHopper", intake_Run);
                NamedCommands.registerCommand("intake-intakeToShooter", intake_Run);
                NamedCommands.registerCommand("intake-stop", intake_Stop);
                NamedCommands.registerCommand("zero-turret", zeroTurret);
                NamedCommands.registerCommand("intake-no-outta", intake_Run);
                // // shooter commands
                NamedCommands.registerCommand("shoot", shooter_RunToRPM);
                // climber commands
                // nothing right now

                SmartDashboard.putData("turreet", turret);

                // Put both on the Dashboard

                SmartDashboard.putNumber("Shooter RPM", 65);

                SmartDashboard.putNumber("intake speed Front", 0.3);
                SmartDashboard.putNumber("intake speed Back", 0.3);

                configureBindings();
                autoChooser = AutoBuilder.buildAutoChooser();

                SmartDashboard.putData("autoChoose", autoChooser);

        }

        private void configureBindings() {
                // Note that X is defined as forward according to WPILib convention,
                // and Y is defined as to the left according to WPILib convention.
                drivetrain.setDefaultCommand(
                                // Drivetrain will execute this command periodically
                                drivetrain.applyRequest(() -> {
                                        if (driverController.rightBumper().getAsBoolean()) {
                                                OverrideSpeed = 0.25;
                                        } else {
                                                OverrideSpeed = 0.75;
                                        }
                                        return drive
                                                        // Drive forward with negative Y (forward) Drive left with
                                                        // negative X
                                                        // (left)
                                                        .withVelocityX(-driverController.getLeftY() * MaxSpeed
                                                                        * OverrideSpeed)

                                                        .withVelocityY(-driverController.getLeftX() * MaxSpeed
                                                                        * OverrideSpeed)

                                                        // Drive counter clockwise with negative X (left)
                                                        .withRotationalRate(-driverController.getRightX()
                                                                        * MaxAngularRate * OverrideSpeed);
                                }));

                // Idle while the robot is disabled. This ensures the configured
                // neutral mode is applied to the drive motors while disabled.
                final var idle = new SwerveRequest.Idle();
                RobotModeTriggers.disabled().whileTrue(
                                drivetrain.applyRequest(() -> idle).ignoringDisable(true));

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

                driverController.pov(0).whileTrue(turret_Toggle);

                driverController.pov(90)
                                .whileTrue(Commands.run(() -> drivetrain.resetPose(new Pose2d(8, 4, new Rotation2d(0))),
                                                drivetrain));

                // hopper commands
                driverController.a().whileTrue(hopper_Out).whileFalse(hopper_In);

                // shooter and hood commands
                hoodSubsystem.setDefaultCommand(hoodDown);
                shooter.setDefaultCommand(shooterCoast);

                manipulatorController.rightBumper().whileTrue(shooterAndHood);

                driverController.y().whileTrue(shooterCoast);
                driverController.leftTrigger(.5).whileTrue(hoodDown);
                driverController.start().whileTrue(zeroTurret);
                driverController.rightTrigger(.5).whileTrue(shooter_RunToRPM.alongWith(hoodSetToPositionDRIVER));

                // turret commands
                turret.setDefaultCommand(turret_Locking);

                // game manager commands
                gameManager.setDefaultCommand(Commands.run(() -> gameManager.determineActiveHub(), gameManager));

                manipulatorController.start().onTrue(Commands.runOnce(() -> gameManager.resetTimer(), gameManager));
                manipulatorController.pov(270).onTrue(Commands.runOnce(() -> gameManager.lostAuto(), gameManager));
                manipulatorController.pov(90).onTrue(Commands.runOnce(() -> gameManager.wonAuto(), gameManager));

                // intake commands
                intakeSubsystem.setDefaultCommand(intake_Stop);

                driverController.b().whileTrue(intake_Run);
                driverController.x().whileTrue(intake_Run);
                driverController.pov(180).whileTrue(intake_Outtake);

                manipulatorController.a().whileTrue(intake_Run);
                manipulatorController.b().whileTrue(intake_Outtake);
        }

        public Command getAutonomousCommand() {
                return autoChooser.getSelected();
        }

        public void teleopInit() {
                gameManager.resetTimer();
        }
}
