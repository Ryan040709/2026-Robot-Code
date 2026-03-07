package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import java.util.Optional;
import java.util.function.Supplier;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.SwerveConstants;
import frc.robot.SwerveConstants.TunerSwerveDrivetrain;
import frc.robot.subsystems.LimelightHelpers.PoseEstimate;

public class CommandSwerveDrivetrain extends TunerSwerveDrivetrain implements Subsystem {
    private static final double kSimLoopPeriod = 0.004; // 4 ms
    private Notifier simNotifier = null;
    private double lastSimTime;

    /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
    private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.kZero;
    /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
    private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.k180deg;
    /* Keep track if we've ever applied the operator perspective before or not */
    private boolean m_hasAppliedOperatorPerspective = false;

    private Translation2d frontLeftLocation = new Translation2d(.3429, .3429);
    private Translation2d frontRightLocation = new Translation2d(.3429, -0.3429);
    private Translation2d backLeftLocation = new Translation2d(-0.3429, .3429);
    private Translation2d backRightLocation = new Translation2d(-0.3429, -0.3429);

    private SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
            frontLeftLocation,
            frontRightLocation,
            backLeftLocation,
            backRightLocation);

    /* Swerve requests to apply during SysId characterization */
    private final SwerveRequest.SysIdSwerveTranslation translationCharacterization = new SwerveRequest.SysIdSwerveTranslation();
    private final SwerveRequest.SysIdSwerveSteerGains steerCharacterization = new SwerveRequest.SysIdSwerveSteerGains();
    private final SwerveRequest.SysIdSwerveRotation rotationCharacterization = new SwerveRequest.SysIdSwerveRotation();

    // gyro
    Pigeon2 gyro = new Pigeon2(SwerveConstants.kPigeonId, SwerveConstants.kCANBus.getName());

    // setting the postions of our swerve modules for kinematics
    private Translation2d m_frontLeftLocation = new Translation2d(0.282575, 0.282575);
    private Translation2d m_frontRightLocation = new Translation2d(0.282575, -0.282575);
    private Translation2d m_backLeftLocation = new Translation2d(-0.282575, 0.282575);
    private Translation2d m_backRightLocation = new Translation2d(-0.282575, -0.282575);

    // Creating my kinematics object using the module locations
    private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
            m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation,
            m_backRightLocation);
    // setting up odometery
    private SwerveDrivePoseEstimator poseEstimator;
    private Field2d m_Fields = new Field2d();
    public HolonomicDriveController controller;
    private SendableChooser<Boolean> visionSwitch;
    TalonFX frontRightDrive = new TalonFX(4);

    /*
     * SysId routine for characterizing translation. This is used to find PID gains
     * for the drive motors.
     */
    private final SysIdRoutine sysIdRoutineTranslation = new SysIdRoutine(
            new SysIdRoutine.Config(
                    null, // Use default ramp rate (1 V/s)
                    Volts.of(4), // Reduce dynamic step voltage to 4 V to prevent brownout
                    null, // Use default timeout (10 s)
                    // Log state with SignalLogger class
                    state -> SignalLogger.writeString("SysIdTranslation_State", state.toString())),
            new SysIdRoutine.Mechanism(
                    output -> setControl(translationCharacterization.withVolts(output)),
                    null,
                    this));

    /*
     * SysId routine for characterizing steer. This is used to find PID gains for
     * the steer motors.
     */
    private final SysIdRoutine sysIdRoutineSteer = new SysIdRoutine(
            new SysIdRoutine.Config(
                    null, // Use default ramp rate (1 V/s)
                    Volts.of(7), // Use dynamic voltage of 7 V
                    null, // Use default timeout (10 s)
                    // Log state with SignalLogger class
                    state -> SignalLogger.writeString("SysIdSteer_State", state.toString())),
            new SysIdRoutine.Mechanism(
                    volts -> setControl(steerCharacterization.withVolts(volts)),
                    null,
                    this));

    /*
     * SysId routine for characterizing rotation.
     * This is used to find PID gains for the FieldCentricFacingAngle
     * HeadingController.
     * See the documentation of SwerveRequest.SysIdSwerveRotation for info on
     * importing the log to SysId.
     */
    private final SysIdRoutine sysIdRoutineRotation = new SysIdRoutine(
            new SysIdRoutine.Config(
                    /* This is in radians per second², but SysId only supports "volts per second" */
                    Volts.of(Math.PI / 6).per(Second),
                    /* This is in radians per second, but SysId only supports "volts" */
                    Volts.of(Math.PI),
                    null, // Use default timeout (10 s)
                    // Log state with SignalLogger class
                    state -> SignalLogger.writeString("SysIdRotation_State", state.toString())),
            new SysIdRoutine.Mechanism(
                    output -> {
                        /* output is actually radians per second, but SysId only supports "volts" */
                        setControl(rotationCharacterization.withRotationalRate(output.in(Volts)));
                        /* also log the requested output for SysId */
                        SignalLogger.writeDouble("Rotational_Rate", output.in(Volts));
                    },
                    null,
                    this));

    /* The SysId routine to test */
    private SysIdRoutine sysIdRoutineToApply = sysIdRoutineTranslation;

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not
     * construct
     * the devices themselves. If they need the devices, they can access them
     * through
     * getters in the classes.
     *
     * @param drivetrainConstants Drivetrain-wide constants for the swerve drive
     * @param modules             Constants for each specific module
     */
    public CommandSwerveDrivetrain(
            SwerveDrivetrainConstants drivetrainConstants,
            SwerveModuleConstants<?, ?, ?>... modules) {
        super(drivetrainConstants, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }

        setUpPoseEstimater();
        PathPlannerSetup();

    }

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not
     * construct
     * the devices themselves. If they need the devices, they can access them
     * through
     * getters in the classes.
     *
     * @param drivetrainConstants     Drivetrain-wide constants for the swerve drive
     * @param odometryUpdateFrequency The frequency to run the odometry loop. If
     *                                unspecified or set to 0 Hz, this is 250 Hz on
     *                                CAN FD, and 100 Hz on CAN 2.0.
     * @param modules                 Constants for each specific module
     */
    public CommandSwerveDrivetrain(
            SwerveDrivetrainConstants drivetrainConstants,
            double odometryUpdateFrequency,
            SwerveModuleConstants<?, ?, ?>... modules) {
        super(drivetrainConstants, odometryUpdateFrequency, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }

        setUpPoseEstimater();
        PathPlannerSetup();

    }

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not
     * construct
     * the devices themselves. If they need the devices, they can access them
     * through
     * getters in the classes.
     *
     * @param drivetrainConstants       Drivetrain-wide constants for the swerve
     *                                  drive
     * @param odometryUpdateFrequency   The frequency to run the odometry loop. If
     *                                  unspecified or set to 0 Hz, this is 250 Hz
     *                                  on
     *                                  CAN FD, and 100 Hz on CAN 2.0.
     * @param odometryStandardDeviation The standard deviation for odometry
     *                                  calculation
     *                                  in the form [x, y, theta]ᵀ, with units in
     *                                  meters
     *                                  and radians
     * @param visionStandardDeviation   The standard deviation for vision
     *                                  calculation
     *                                  in the form [x, y, theta]ᵀ, with units in
     *                                  meters
     *                                  and radians
     * @param modules                   Constants for each specific module
     */
    public CommandSwerveDrivetrain(
            SwerveDrivetrainConstants drivetrainConstants,
            double odometryUpdateFrequency,
            Matrix<N3, N1> odometryStandardDeviation,
            Matrix<N3, N1> visionStandardDeviation,
            SwerveModuleConstants<?, ?, ?>... modules) {
        super(drivetrainConstants, odometryUpdateFrequency, odometryStandardDeviation, visionStandardDeviation,
                modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }

        setUpPoseEstimater();
        PathPlannerSetup();

    }

    private void setUpPoseEstimater() {
        poseEstimator = new SwerveDrivePoseEstimator(
                m_kinematics,
                gyro.getRotation2d(),
                getModulePositions(true),

                new Pose2d(3, 3, Rotation2d.fromDegrees(180)));

        SmartDashboard.putData("Field", m_Fields);
    }

    // creating array for swerve module postions
    private SwerveModulePosition[] getModulePositions(boolean refresh) {
        SwerveModulePosition[] positions = {
                getModules()[0].getPosition(refresh),
                getModules()[1].getPosition(refresh),
                getModules()[2].getPosition(refresh),
                getModules()[3].getPosition(refresh) };
        return positions;
    }

    public ChassisSpeeds getRobotRelativeSpeeds() {
        return m_kinematics.toChassisSpeeds(getModuleStates());
    }

    public void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds) {

        ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(robotRelativeSpeeds, 0.02);
        this.setControl(new SwerveRequest.ApplyRobotSpeeds().withSpeeds(targetSpeeds));

    }

    public void PathPlannerSetup() {
        RobotConfig config;
        try {
            config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            // Handle exception as needed
            throw new RuntimeException(e);
        }

        // Configure AutoBuilder last
        AutoBuilder.configure(
                this::getPose, // Robot pose supplier
                this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
                this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                (speeds, feedforwards) -> driveRobotRelative(speeds), // Method that will drive the robot given ROBOT
                                                                      // RELATIVE ChassisSpeeds. Also optionally outputs
                                                                      // individual module feedforwards
                new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for
                                                // holonomic drive trains
                        new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                        new PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
                ),
                config, // The robot configuration
                () -> {
                    // Boolean supplier that controls when the path will be mirrored for the red
                    // alliance
                    // This will flip the path being followed to the red side of the field.
                    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },
                this // Reference to this subsystem to set requirements
        );
    }

    /**
     * Returns a command that applies the specified control request to this swerve
     * drivetrain.
     *
     * @param request Function returning the request to apply
     * @return Command to run
     */
    public Command applyRequest(Supplier<SwerveRequest> request) {
        return run(() -> this.setControl(request.get()));
    }

    /**
     * Runs the SysId Quasistatic test in the given direction for the routine
     * specified by {@link #sysIdRoutineToApply}.
     *
     * @param direction Direction of the SysId Quasistatic test
     * @return Command to run
     */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return sysIdRoutineToApply.quasistatic(direction);
    }

    /**
     * Runs the SysId Dynamic test in the given direction for the routine
     * specified by {@link #sysIdRoutineToApply}.
     *
     * @param direction Direction of the SysId Dynamic test
     * @return Command to run
     */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return sysIdRoutineToApply.dynamic(direction);
    }

    @Override
    public void periodic() {

        // Translation2d turretOffsetFromRobot = new Translation2d(getPose().getX() -
        // 0.2101215 , getPose().getY() - .1412875 );
        // double distanceFromHub =
        // getPose()
        // .getTranslation()
        // .rotateAround(turretOffsetFromRobot, getPose().getRotation()).getDistance(new
        // Translation2d(4.62554, 4.03606));
        // SmartDashboard.putNumber("DistanceFromBlueHub", distanceFromHub);

        // SmartDashboard.putNumber("steer motor amps", );
        /*
         * Periodically try to apply the operator perspective.
         * If we haven't applied the operator perspective before, then we should apply
         * it regardless of DS state.
         * This allows us to correct the perspective in case the robot code restarts
         * mid-match.
         * Otherwise, only check and apply the operator perspective if the DS is
         * disabled.
         * This ensures driving behavior doesn't change until an explicit disable event
         * occurs during testing.
         */

        if (!m_hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
            DriverStation.getAlliance().ifPresent(allianceColor -> {
                setOperatorPerspectiveForward(
                        allianceColor == Alliance.Red
                                ? kRedAlliancePerspectiveRotation
                                : kBlueAlliancePerspectiveRotation);
                m_hasAppliedOperatorPerspective = true;
            });
        }

        var gyroAngle = gyro.getRotation2d();

        SwerveModulePosition[] currentPositions = getModulePositions(false);
        // Update the pose

        SmartDashboard.putNumber("GetXSpeeds", getFieldRelativeSpeeds().vxMetersPerSecond);
        SmartDashboard.putNumber("GetYSpeeds", getFieldRelativeSpeeds().vyMetersPerSecond);
        SmartDashboard.putNumber("GetRotationSpeeds", getFieldRelativeSpeeds().omegaRadiansPerSecond);
        

            limelightPoseUpdate("limelight-tags");
            limelightPoseUpdate("limelight-intake");

             poseEstimator.update(gyroAngle, currentPositions);
            SmartDashboard.putNumber("estimatedRotation",
                    poseEstimator.getEstimatedPosition().getRotation().getDegrees());
            SmartDashboard.putNumber("estimated Pose X", poseEstimator.getEstimatedPosition().getX());
            SmartDashboard.putNumber("estimated Pose Y", poseEstimator.getEstimatedPosition().getY());
            m_Fields.setRobotPose(poseEstimator.getEstimatedPosition());
    }

    // creating array to get the module states
    private SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = {
                getModules()[0].getCurrentState(),
                getModules()[1].getCurrentState(),
                getModules()[2].getCurrentState(),
                getModules()[3].getCurrentState() };
        return states;
    }

    // updating the robot pose
    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    // reseting the robot pose
    public void resetPose(Pose2d pose) {
        System.out.println(pose);
        poseEstimator.resetPosition(gyro.getRotation2d(), getModulePositions(true), pose);
        super.resetPose(pose);
    }

    // geting the speed of the swerves
    public ChassisSpeeds getFieldRelativeSpeeds() {
        var speeds = m_kinematics.toChassisSpeeds(getModuleStates());
        var fieldSpeeds = new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond)
                .rotateBy(getPose().getRotation());

        return new ChassisSpeeds(fieldSpeeds.getX(), fieldSpeeds.getY(), speeds.omegaRadiansPerSecond);
    }

    public double getVelocity() {
        return Math.sqrt(Math.pow(getFieldRelativeSpeeds().vxMetersPerSecond, 2)
                + Math.pow(getFieldRelativeSpeeds().vyMetersPerSecond, 2));
    }

    private void startSimThread() {
        lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - lastSimTime;
            lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILip */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        simNotifier.startPeriodic(kSimLoopPeriod);
    }

    /**
     * Adds a vision measurement to the Kalman Filter. This will correct the
     * odometry pose estimate
     * while still accounting for measurement noise.
     *
     * @param visionRobotPoseMeters The pose of the robot as measured by the vision
     *                              camera.
     * @param timestampSeconds      The timestamp of the vision measurement in
     *                              seconds.
     */
    @Override
    public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds) {
        super.addVisionMeasurement(visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds));
    }

    /**
     * Adds a vision measurement to the Kalman Filter. This will correct the
     * odometry pose estimate
     * while still accounting for measurement noise.
     * <p>
     * Note that the vision measurement standard deviations passed into this method
     * will continue to apply to future measurements until a subsequent call to
     * {@link #setVisionMeasurementStdDevs(Matrix)} or this method.
     *
     * @param visionRobotPoseMeters    The pose of the robot as measured by the
     *                                 vision camera.
     * @param timestampSeconds         The timestamp of the vision measurement in
     *                                 seconds.
     * @param visionMeasurementStdDevs Standard deviations of the vision pose
     *                                 measurement
     *                                 in the form [x, y, theta]ᵀ, with units in
     *                                 meters and radians.
     */
    @Override
    public void addVisionMeasurement(
            Pose2d visionRobotPoseMeters,
            double timestampSeconds,
            Matrix<N3, N1> visionMeasurementStdDevs) {
        super.addVisionMeasurement(visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds),
                visionMeasurementStdDevs);
    }

    /**
     * Return the pose at a given timestamp, if the buffer is not empty.
     *
     * @param timestampSeconds The timestamp of the pose in seconds.
     * @return The pose at the given timestamp (or Optional.empty() if the buffer is
     *         empty).
     */
    @Override
    public Optional<Pose2d> samplePoseAt(double timestampSeconds) {
        return super.samplePoseAt(Utils.fpgaToCurrentTime(timestampSeconds));
    }

    public void limelightPoseUpdate(String limelightname){
        LimelightHelpers.SetRobotOrientation( limelightname,
                poseEstimator.getEstimatedPosition().getRotation().getDegrees(), 0, 0, 0, 0, 0);
     PoseEstimate robotPoseEstimateTags = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightname);

        Pose3d targetPoseRobotSpace = LimelightHelpers.getTargetPose3d_RobotSpace(limelightname);
        double distance = Math.sqrt(
                Math.pow(targetPoseRobotSpace.getX(), 2) +
                        Math.pow(targetPoseRobotSpace.getY(), 2) +
                        Math.pow(targetPoseRobotSpace.getZ(), 2));
        if (robotPoseEstimateTags != null) {
            if (robotPoseEstimateTags.tagCount != 0 && gyro.getAngularVelocityZWorld().getValueAsDouble() < 60
                    && distance < 3.5) {
                poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(distance * 0.75, distance * 0.75, 9999999));
                poseEstimator.addVisionMeasurement(robotPoseEstimateTags.pose,
                        robotPoseEstimateTags.timestampSeconds);
                System.out.println("using limelight");
                // poseEstimator.addVisionMeasurement(robotPoseEstimateTurret.pose,
                // robotPoseEstimateTurret.timestampSeconds);

            }

        } else {
            System.out.println("limelight estimation null");
        }
    }



    public boolean turretDeadZone() {
        if (getPose().getY() > 2.8 && getPose().getY() < 5.3) {
            // if (getPose().getX() > 5.5 && getPose().getX() < 7.0) {
            return true;
            // } else {
            // return false;
            // }

        } else {
            return false;
        }
    }

    public boolean isFeeding() {
        double Hx = GameManager.isBlueAlliance ? 4.62554 : 11.98482;

        return GameManager.isBlueAlliance ? getPose().getX() > Hx : getPose().getX() < Hx;
    }

    public Pose2d feedingTargets() {

        double Hy = 4.03606;

        // uses new feeder shot positions
        double tX = GameManager.isBlueAlliance ? 0.5 : 7.5; // 4.62554 / 4 : 11.98482 + ((4.62554 / 2)+(4.62554 / 4));
        // double tYFar = getPose().getY() > 4.03606 && getPose().getY() < 6.5 ? 7.5 :
        // 0.5;
        double tY = getPose().getY() > Hy ? 7.5 : 0.5;

        if (turretDeadZone()) {
            // do the deadzone swap!
            return new Pose2d((tX - (getFieldRelativeSpeeds().vyMetersPerSecond * ShooterSubsystem.tof)),
                    (tY - (getFieldRelativeSpeeds().vyMetersPerSecond * ShooterSubsystem.tof)),
                    Rotation2d.fromDegrees(0));
        } else {
            return new Pose2d((tX - (getFieldRelativeSpeeds().vyMetersPerSecond * ShooterSubsystem.tof)),
                    (tY - (getFieldRelativeSpeeds().vyMetersPerSecond * ShooterSubsystem.tof)),
                    Rotation2d.fromDegrees(0));
        }
    }

    public Pose2d getHubTarget() {
        Translation2d BlueHubPosition = new Translation2d(
                4.62554 - (getFieldRelativeSpeeds().vxMetersPerSecond * ShooterSubsystem.tof),
                4.03606 - (getFieldRelativeSpeeds().vyMetersPerSecond * ShooterSubsystem.tof));
        Translation2d RedHubPosistion = new Translation2d(
                11.98482 - (getFieldRelativeSpeeds().vxMetersPerSecond * ShooterSubsystem.tof),
                4.03606 - (getFieldRelativeSpeeds().vyMetersPerSecond * ShooterSubsystem.tof));

        return new Pose2d(GameManager.isBlueAlliance ? BlueHubPosition : RedHubPosistion, Rotation2d.fromDegrees(0));
    }

    public Pose2d getTurretTarget() {
        SmartDashboard.putBoolean("is feeding", isFeeding());

        return new Pose2d(isFeeding() ? feedingTargets().getTranslation() : getHubTarget().getTranslation(),
                Rotation2d.fromDegrees(0));
    }

    public double getDistanceToTarget() { // used to be called "GetDistanceToHub"
        double DistanceToTarget = getTurretOffset().getTranslation().getDistance(getTurretTarget().getTranslation());

        // puts the target on Glass so we can visualize it
        m_Fields.getObject("scoring target")
                .setPose(new Pose2d(getTurretTarget().getTranslation(), Rotation2d.fromDegrees(0)));
        // turret postions
        // m_Fields.getObject("turret position").setPose(
        //         new Pose2d(getTurretOffset().getTranslation(), Rotation2d.fromDegrees(TurretSubsystem.turretPosition)));
        // m_Fields.getObject("feeding target").setPose(new
        // Pose2d(getTurretTarget().getTranslation(), Rotation2d.fromDegrees(0)));
        SmartDashboard.putNumber("distance to target", DistanceToTarget);
        return DistanceToTarget;
    }

    public Pose2d getTurretOffset() {
        // turret offset x: 5.6475in OR 0.1434465m
        // turret offset y: 2.9375in OR 0.0746125m
        Transform2d turretOffsetFromRobot = new Transform2d(5.6475 / 39.37, 2.9375 / 39.37, Rotation2d.fromDegrees(0));

        Pose2d turretOffset = getPose().transformBy(turretOffsetFromRobot);

        // // gives the robot pose
        // SmartDashboard.putString("robot pose", getPose().toString());

        // // gives the turret pose
        // SmartDashboard.putString("turret pose", turretOffset.toString());
        return turretOffset;
    }
}
