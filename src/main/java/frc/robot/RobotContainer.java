package frc.robot;

import static edu.wpi.first.units.Units.*;

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
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.outOfBumperIntake;
import frc.robot.subsystems.throughBumperIntake;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.commands.intake.throughTheBumper.Intake_Outtake;
import frc.robot.commands.intake.throughTheBumper.Intake_Stop;
import frc.robot.commands.intake.throughTheBumper.Intake_IntakeToHopper;
import frc.robot.commands.intake.throughTheBumper.Intake_IntakeToShooter;
import frc.robot.commands.shooter.Shooter_RunToRPM;
import frc.robot.commands.shooter.Hood_SetToPosition;
import frc.robot.commands.shooter.ShooterStop;
import frc.robot.commands.shooter.ShooterToRPMS;
//out of bumper intake commands
import frc.robot.commands.intake.outTheBumper.Intake_LowerIntake;
import frc.robot.commands.intake.outTheBumper.Intake_RaiseIntake;
import frc.robot.commands.intake.outTheBumper.Intake_RunOuttake;
import frc.robot.commands.intake.outTheBumper.Intake_RunIntake;
import frc.robot.commands.intake.outTheBumper.Intake_StopIntake;
//turret commands
import frc.robot.commands.turret.Turret_Toggle;
//get the game manager

public class RobotContainer {

        // kSpeedAt12Volts desired top speed
        private double MaxSpeed = 1 * SwerveConstants.kSpeedAt12Volts.in(MetersPerSecond); // keep at 0.5, Andy said
                                                                                           // so...

        // 3/4 of a rotation per second max angular velocity
        private double MaxAngularRate = RotationsPerSecond.of(.75).in(RadiansPerSecond); // TODO change to 1
        // we want to do one rotation per sec, I think.
        // faster than last year's if beyond 0.75

        private final SendableChooser<Command> autoChooser;

        // let's the driver pick the actual auton they want.
        private final SendableChooser<String> routineChooser = new SendableChooser<>();
        // let's the driver pick an auton "variation"
        private final SendableChooser<String> variationChooser = new SendableChooser<>();

        /* Setting up bindings for necessary control of the swerve drive platform */
        private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
                        // Add a 10% deadband
                        .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1)
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
        TurretSubsystem turret = new TurretSubsystem(drivetrain::getPose);
        // shooter subsystem
        ShooterSubsystem shooter = new ShooterSubsystem();
        // in the bumper intake subsystem
        throughBumperIntake throughBumperIntake = new throughBumperIntake();
        // out of bumper intake subsystem
        outOfBumperIntake OutOfBumperIntake = new outOfBumperIntake();

        HopperSubsystem hopperSubsystem = new HopperSubsystem();
        // game manager
        GameManager gameManager = new GameManager();

        // shooter commands
        Shooter_RunToRPM shooter_RunToRPM = new Shooter_RunToRPM(shooter, drivetrain);
        Hood_SetToPosition shooter_setToPosition = new Hood_SetToPosition(shooter, drivetrain);
        ShooterToRPMS shooterToRPMS = new ShooterToRPMS(shooter, drivetrain);
        ShooterStop shooterStop = new ShooterStop(shooter, drivetrain);
        // turret commands
        Turret_Toggle turret_Locking = new Turret_Toggle(turret);
        // out of bumper intake commands
        Intake_LowerIntake intake_LowerIntake = new Intake_LowerIntake(OutOfBumperIntake);
        Intake_RaiseIntake intake_RaiseIntake = new Intake_RaiseIntake(OutOfBumperIntake);
        Intake_RunIntake intake_RunIntake = new Intake_RunIntake(OutOfBumperIntake);
        Intake_StopIntake intake_StopIntake = new Intake_StopIntake(OutOfBumperIntake);
        Intake_RunOuttake intake_RunOuttake = new Intake_RunOuttake(OutOfBumperIntake);

        // in the bumper intake commands
        // test
        Intake_Stop intake_Stop = new Intake_Stop(hopperSubsystem, throughBumperIntake);
        Intake_Outtake outtake = new Intake_Outtake(hopperSubsystem, throughBumperIntake);
        Intake_IntakeToHopper IntakeToHopper = new Intake_IntakeToHopper(hopperSubsystem, throughBumperIntake);
        Intake_IntakeToShooter IntakeToShooter = new Intake_IntakeToShooter(hopperSubsystem, throughBumperIntake);

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
                NamedCommands.registerCommand("intake-hopperToIntake", outtake);
                NamedCommands.registerCommand("intake-intakeToHopper", IntakeToHopper);
                NamedCommands.registerCommand("intake-intakeToShooter", IntakeToShooter);
                // // shooter commands
                NamedCommands.registerCommand("intake-intakeToHopper", IntakeToHopper);
                NamedCommands.registerCommand("intake-intakeToShooter", IntakeToShooter);
                // climber commands

                // Setup Routines
                routineChooser.setDefaultOption("Left Trench Auto", "left_trench");
                routineChooser.setDefaultOption("Right Trench Auto", "right_trench");
                routineChooser.setDefaultOption("Hub Auto", "Hub");

                // Setup Variations
                variationChooser.setDefaultOption("Depot", "to_depot");
                variationChooser.addOption("Sweep", "to_sweep");
                variationChooser.addOption("Double Sweep", "to_double_sweep");
                variationChooser.addOption("Human Player", "to_human_player_zone");

                // Put both on the Dashboard
                SmartDashboard.putData("Auto Routine", routineChooser);
                SmartDashboard.putData("Auto Variation", variationChooser);

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
                                                // Drive forward with negative Y (forward) Drive left with negative X
                                                // (left)
                                                .withVelocityX(-driverController.getLeftY() * MaxSpeed)

                                                .withVelocityY(-driverController.getLeftX() * MaxSpeed)

                                                // Drive counter clockwise with negative X (left)
                                                .withRotationalRate(-driverController.getRightX() * MaxAngularRate)));

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

                // manipulatorController.leftTrigger(0.05)
                // .whileTrue(Commands.run(() ->
                // turret.MoveMotor(manipulatorController.getLeftX()),
                // turret));
                // manual zeroing
                manipulatorController.x().whileTrue(Commands.run(() -> turret.zeroPosition(), turret));
                // set to run to x position
                driverController.pov(0).toggleOnTrue(turret_Locking);

                driverController.pov(90)
                                .whileTrue(Commands.run(() -> drivetrain.resetPose(new Pose2d(8, 4, new Rotation2d(0))),
                                                drivetrain));
                driverController.x().whileTrue(IntakeToShooter).whileFalse(intake_Stop);
                driverController.y().whileTrue(shooterToRPMS).whileFalse(shooterStop);

                manipulatorController.a().whileTrue(Commands.run(() -> turret.setToZero(), turret));

                manipulatorController.pov(0).toggleOnTrue(turret_Locking);


                //thru bumper intake commands
                manipulatorController.pov(90).whileTrue(outtake).whileFalse(intake_Stop);

                manipulatorController.pov(180).whileTrue(IntakeToHopper).whileFalse(intake_Stop);

                manipulatorController.pov(270).whileTrue(IntakeToShooter).whileFalse(intake_Stop);


                manipulatorController.leftTrigger(0.05).whileTrue(shooter_RunToRPM);

                gameManager.setDefaultCommand(Commands.run(() -> {
                }, gameManager));

                // turret.setDefaultCommand(Commands.run(() ->
                // turret.MoveMotor(manipulatorController.getLeftX()), turret));
                turret.setDefaultCommand(Commands.run(() -> turret.setPosition(drivetrain.robotVelocityX,
                                drivetrain.robotVelocityY, manipulatorController.getLeftX()), turret));

                // out of bumper intake commands
                manipulatorController.leftBumper().whileTrue(intake_LowerIntake);
                manipulatorController.rightBumper().whileTrue(intake_RaiseIntake);

        }

        public Command getAutonomousCommand() {

                String routine = routineChooser.getSelected();
                String variation = variationChooser.getSelected();
                String autoName = routine + "_" + variation;

                try {
                        return AutoBuilder.buildAuto(autoName);
                } catch (Exception e) { // if for whatever reason the driver requests an inalid auto
                        DriverStation.reportError("Auto " + autoName + " not found!", e.getStackTrace());
                        return AutoBuilder.buildAuto("nothing_auto"); // the auton that does... nothing
                }
        }
}