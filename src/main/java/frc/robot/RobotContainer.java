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
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.hoodSubsystem;
import frc.robot.subsystems.outOfBumperIntake;
import frc.robot.subsystems.throughBumperIntake;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.commands.intake.throughTheBumper.Intake_Outtake;
import frc.robot.commands.intake.throughTheBumper.Intake_Stop;
import frc.robot.commands.intake.throughTheBumper.Intake_IntakeToHopper;
import frc.robot.commands.intake.throughTheBumper.Intake_IntakeToShooter;
import frc.robot.commands.intake.throughTheBumper.Auto_Intake_IntakeToShooter;
import frc.robot.commands.intake.throughTheBumper.IntakeToShooterNoOutta;
import frc.robot.commands.shooter.Shooter_RunToRPM;
import frc.robot.commands.shooter.Hood_SetToPosition;
import frc.robot.commands.shooter.OverrideHoodPosition;
import frc.robot.commands.shooter.ShooterStop;
import frc.robot.commands.shooter.ShooterToRPMS;
import frc.robot.commands.SetLEDColor;
import frc.robot.commands.resetLEDTimer;
//out of bumper intake commands
import frc.robot.commands.intake.outTheBumper.Intake_LowerIntake;
import frc.robot.commands.intake.outTheBumper.Intake_RaiseIntake;
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
        private double DeadZone = .1;


        // was .75 if too fast
        private double MaxAngularRate = RotationsPerSecond.of(1).in(RadiansPerSecond);

        private final SendableChooser<Command> autoChooser;

        // let's the driver pick the actual auton they want.
        private final SendableChooser<String> routineChooser = new SendableChooser<>();
        // let's the driver pick an auton "variation"
        private final SendableChooser<String> variationChooser = new SendableChooser<>();

        /* Setting up bindings for necessary control of the swerve drive platform */
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
        throughBumperIntake throughBumperIntake = new throughBumperIntake();
        // out of bumper intake subsystem
        outOfBumperIntake OutOfBumperIntake = new outOfBumperIntake();

        hoodSubsystem hoodSubsystem = new hoodSubsystem();

        // game manager
        GameManager gameManager = new GameManager();

        // shooter commands
        Shooter_RunToRPM shooter_RunToRPM = new Shooter_RunToRPM(shooter, drivetrain);
        Shooter_RunToRPM shooter_RunToRpmTele = new Shooter_RunToRPM(shooter, drivetrain);
        Hood_SetToPosition hoodSetToPosition = new Hood_SetToPosition(hoodSubsystem, drivetrain);
        Hood_SetToPosition hoodSetToPositionDRIVER = new Hood_SetToPosition(hoodSubsystem, drivetrain);
        OverrideHoodPosition hoodDown = new OverrideHoodPosition(hoodSubsystem);
        ShooterToRPMS shooterToRPMS = new ShooterToRPMS(shooter, drivetrain);
        ShooterStop shooterStop = new ShooterStop(shooter, drivetrain);
        // turret commands
        Turret_Toggle turret_Toggle = new Turret_Toggle(turret);
        Turret_Locking turret_Locking = new Turret_Locking(turret, drivetrain);
        ZeroTurret zeroTurret = new ZeroTurret(turret, drivetrain);
        // out of bumper intake commands
        Intake_LowerIntake intake_LowerIntake = new Intake_LowerIntake(OutOfBumperIntake);
        Intake_RaiseIntake intake_RaiseIntake = new Intake_RaiseIntake(OutOfBumperIntake);

        // in the bumper intake commands
        Intake_Stop intake_Stop = new Intake_Stop(throughBumperIntake, OutOfBumperIntake);
        Intake_Outtake outtake = new Intake_Outtake(throughBumperIntake, OutOfBumperIntake);
        Intake_IntakeToHopper IntakeToHopper = new Intake_IntakeToHopper(throughBumperIntake, OutOfBumperIntake);
        Intake_IntakeToShooter IntakeToShooter = new Intake_IntakeToShooter(throughBumperIntake, OutOfBumperIntake,
                        turret, drivetrain, hoodSubsystem);
        IntakeToShooterNoOutta intakeNoOutta = new IntakeToShooterNoOutta(throughBumperIntake, OutOfBumperIntake, turret, drivetrain);
        Auto_Intake_IntakeToShooter auto_Intake_IntakeToShooter = new Auto_Intake_IntakeToShooter(throughBumperIntake, OutOfBumperIntake);

        public RobotContainer() {
                // turret commands
                NamedCommands.registerCommand("turret-locking", turret_Locking);
                // in the bumper intake commands
                NamedCommands.registerCommand("intake-hopperToIntake", outtake);
                NamedCommands.registerCommand("intake-intakeToHopper", IntakeToHopper);
                NamedCommands.registerCommand("intake-intakeToShooter", auto_Intake_IntakeToShooter);
                NamedCommands.registerCommand("intake-stop", intake_Stop);
                NamedCommands.registerCommand("zero-turret", zeroTurret);
                NamedCommands.registerCommand("intake-no-outta", auto_Intake_IntakeToShooter);
                // // shooter commands
                NamedCommands.registerCommand("shoot", shooter_RunToRPM);
                // climber commands
                // nothing right now

                // Setup Routines
                routineChooser.setDefaultOption("Testing", "testing");

                routineChooser.setDefaultOption("Left Trench Auto", "left_trench");
                routineChooser.setDefaultOption("Right Trench Auto", "right_trench");
                routineChooser.setDefaultOption("Hub Auto", "Hub");

                // Setup Variations
                variationChooser.setDefaultOption("Testing", "to_testing");

                variationChooser.setDefaultOption("Depot", "to_depot");
                variationChooser.addOption("Sweep", "to_sweep");
                variationChooser.addOption("Double Sweep", "to_double_sweep");
                variationChooser.addOption("Human Player", "to_human_player_zone");

                SmartDashboard.putData("turreet", turret);

                // Put both on the Dashboard
                SmartDashboard.putData("Auto Routine", routineChooser);
                SmartDashboard.putData("Auto Variation", variationChooser);

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
                driverController.pov(180).whileTrue(outtake).whileFalse(intake_Stop);

                driverController.y().whileTrue(shooterToRPMS).whileFalse(shooterStop);

                driverController.b().whileTrue(IntakeToHopper).whileFalse(intake_Stop);
                driverController.x().whileTrue(IntakeToShooter).whileFalse(intake_Stop);
                driverController.rightTrigger(.5).whileTrue(shooter_RunToRPM.alongWith(hoodSetToPositionDRIVER)).whileFalse(shooterStop);
                driverController.start().whileTrue(zeroTurret);
                driverController.a().whileTrue(intake_LowerIntake).whileFalse(intake_RaiseIntake);
                driverController.leftTrigger(.5).whileTrue(hoodDown);

                turret.setDefaultCommand(turret_Locking); // manipulatorController.getLeftX()*0.0125

                gameManager.setDefaultCommand(Commands.run(() -> gameManager.determineActiveHub(), gameManager));
                hoodSubsystem.setDefaultCommand(hoodDown);
                //shooter.setDefaultCommand(shooterStop);

                //driverController.leftBumper().whileTrue(hoodSetToPosition);
               // driverController.a().whileTrue(IntakeToHopper).whileFalse(intake_Stop);

                // out of bumper intake commands
               // manipulatorController.a().whileTrue(IntakeToShooter).whileFalse(intake_Stop);
                manipulatorController.a().whileTrue(IntakeToShooter).whileFalse(intake_Stop);
                
                manipulatorController.y().whileTrue(IntakeToHopper).whileFalse(intake_Stop);
                manipulatorController.b().whileTrue(outtake).whileFalse(intake_Stop);

                manipulatorController.rightBumper().whileTrue(shooter_RunToRpmTele.alongWith(hoodSetToPosition));


                manipulatorController.start().onTrue(Commands.runOnce(()-> gameManager.resetTimer(), gameManager));
                manipulatorController.pov(270).onTrue(Commands.runOnce(()-> gameManager.lostAuto(), gameManager));
                manipulatorController.pov(90).onTrue(Commands.runOnce(()-> gameManager.wonAuto(), gameManager));
        }

        public Command getAutonomousCommand() {

                // String routine = routineChooser.getSelected();
                // String variation = variationChooser.getSelected();
                // String autoName = routine + "_" + variation;

                // try {
                // return AutoBuilder.buildAuto(autoName);
                // } catch (Exception e) { // if for whatever reason the driver requests an
                // inalid auto
                // DriverStation.reportError("Auto " + autoName + " not found!",
                // e.getStackTrace());
                // return AutoBuilder.buildAuto("nothing_auto"); // the auton that does...
                // nothing
                // }

                return autoChooser.getSelected();
        }
        public void teleopInit(){
                gameManager.resetTimer();
        }
}
