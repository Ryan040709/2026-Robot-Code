package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import java.util.Optional;
import java.util.function.Supplier;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.GyroTrimConfigs;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.configs.Pigeon2Configurator;
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
    private static final Translation2d BlueHubPosition = new Translation2d(4.62554, 4.03606);
    private static final Translation2d RedHubPosistion = new Translation2d(11.98482, 4.03606);
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
    Pigeon2Configuration gyroConfig;

    // setting up odometery
    // private SwerveDrivePoseEstimator poseEstimator;

    private Field2d m_Fields = new Field2d();
    public HolonomicDriveController controller;
    private SendableChooser<Boolean> visionSwitch;
    private Pose2d target = new Pose2d();

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
        gyroConfig = new Pigeon2Configuration();
        gyroConfig.GyroTrim.GyroScalarZ = -3.28;
        gyro.getConfigurator().apply(gyroConfig);

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
        // poseEstimator = new SwerveDrivePoseEstimator(
        // m_kinematics,
        // gyro.getRotation2d(),
        // getModulePositions(true),

        // new Pose2d(3, 3, Rotation2d.fromDegrees(180)));

        SmartDashboard.putData("Field", m_Fields);
    }

    // creating array for swerve module postions
    // private SwerveModulePosition[] getModulePositions(boolean refresh) {
    // SwerveModulePosition[] positions = {
    // getModules()[0].getPosition(refresh),
    // getModules()[1].getPosition(refresh),
    // getModules()[2].getPosition(refresh),
    // getModules()[3].getPosition(refresh) };
    // return positions;
    // }

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
        // Update the pose

        limelightPoseUpdate("limelight-tags");
        limelightPoseUpdate("limelight-back");
        limelightPoseUpdate("limelight-intake");

        // poseEstimator.update(gyroAngle, currentPositions);

        m_Fields.setRobotPose(getPose());
        computeTurretTaget();
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
        // return poseEstimator.getEstimatedPosition();
        return getState().Pose;
    }

    // reseting the robot pose
    public void resetPose(Pose2d pose) {
        System.out.println(pose);
        // poseEstimator.resetPosition(gyro.getRotation2d(), getModulePositions(true),
        // pose);

        super.resetPose(pose);
    }

    // geting the speed of the swerves
    public ChassisSpeeds getFieldRelativeSpeeds() {
        var speeds = m_kinematics.toChassisSpeeds(getModuleStates());
        var fieldSpeeds = new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond)
                .rotateBy(getPose().getRotation());

        return new ChassisSpeeds(fieldSpeeds.getX(), fieldSpeeds.getY(), speeds.omegaRadiansPerSecond);
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

    public void limelightPoseUpdate(String limelightname) {
        LimelightHelpers.SetRobotOrientation(limelightname,
                getPose().getRotation().getDegrees(), 0, 0, 0, 0, 0);
        PoseEstimate robotPoseEstimateTags = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightname);

        Pose3d targetPoseRobotSpace = LimelightHelpers.getTargetPose3d_RobotSpace(limelightname);
        double distance = Math.sqrt(
                Math.pow(targetPoseRobotSpace.getX(), 2) +
                        Math.pow(targetPoseRobotSpace.getY(), 2) +
                        Math.pow(targetPoseRobotSpace.getZ(), 2));
        if (robotPoseEstimateTags != null) {
            if (robotPoseEstimateTags.tagCount != 0 && gyro.getAngularVelocityZWorld().getValueAsDouble() < 60
                    && distance < 3.5) {
                // poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(distance * 0.75,
                // distance * 0.75, 9999999));
                // poseEstimator.addVisionMeasurement(robotPoseEstimateTags.pose,
                // robotPoseEstimateTags.timestampSeconds);
                addVisionMeasurement(robotPoseEstimateTags.pose, robotPoseEstimateTags.timestampSeconds,
                        VecBuilder.fill(Math.pow( distance,2),Math.pow( distance,2), 9999999));
                // poseEstimator.addVisionMeasurement(robotPoseEstimateTurret.pose,
                // robotPoseEstimateTurret.timestampSeconds);

            }
        }
        // } else {
        //     System.out.println("limelight estimation null");
        // }
    }

    public boolean isFeeding() {
        double Hx = GameManager.isBlueAlliance ? 4.62554 : 11.98482;

        return GameManager.isBlueAlliance ? getPose().getX() > Hx : getPose().getX() < Hx;
    }

    public Pose2d feedingTargets(Translation2d swimOffset) {
        double Huby = 4.03606;
        Translation2d feedPosition = new Translation2d(
                GameManager.isBlueAlliance ? 0.5 : 15,
                getPose().getY() > Huby ? 6.75 : 1.25);
        return new Pose2d(feedPosition.minus(swimOffset), Rotation2d.fromDegrees(0));
    }

    public Pose2d getHubTarget(Translation2d swimOffset) {
        Translation2d hubPosition = GameManager.isBlueAlliance ? BlueHubPosition : RedHubPosistion;
        return new Pose2d(hubPosition.minus(swimOffset), Rotation2d.fromDegrees(0));
    }

    public Pose2d getTurretTarget() {
        return target;
    }

    public void computeTurretTaget() {
        boolean feeding = isFeeding();
        ChassisSpeeds chassisSpeeds = getFieldRelativeSpeeds();
        Translation2d swimoffset = new Translation2d(
                chassisSpeeds.vxMetersPerSecond * ShooterSubsystem.tof,
                chassisSpeeds.vyMetersPerSecond * ShooterSubsystem.tof);
        SmartDashboard.putBoolean("is feeding", feeding);

        target = new Pose2d(
                feeding ? feedingTargets(swimoffset).getTranslation() : getHubTarget(swimoffset).getTranslation(),
                Rotation2d.fromDegrees(0));
        m_Fields.getObject("scoring target")
                .setPose(target);
    }

    public double getDistanceToTarget() { // used to be called "GetDistanceToHub"
        double DistanceToTarget = getTurretOffset().getTranslation().getDistance(getTurretTarget().getTranslation());
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
