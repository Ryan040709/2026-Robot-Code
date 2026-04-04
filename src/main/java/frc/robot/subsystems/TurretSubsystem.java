package frc.robot.subsystems;

import java.util.Optional;

import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;

import com.ctre.phoenix6.HootReplay;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
//our constants
import frc.robot.Constants;
import frc.robot.SwerveConstants;

public class TurretSubsystem extends SubsystemBase {
    private static final Logger log = LogManager.getLogger(TurretSubsystem.class);

    public DigitalInput zeroingSensor = new DigitalInput(0); // the magnetic sensor

    private TalonFXS turret = new TalonFXS(10);
    private TalonFXS turretHoot = new TalonFXS(10, SwerveConstants.kCANBus.getName());
    private MotionMagicVoltage m_request = new MotionMagicVoltage(0);

    Translation2d redHubPos = new Translation2d(11.98482, 4.03606); // red hub

    Translation2d blueHubPos = new Translation2d(4.62554, 4.03606); // blue hub

    Translation2d robotPos = new Translation2d(0, 0); // robot position

    public double redFx = 11.98482 + (4.62554 / 2); // blueFeedingX
    public double blueFx = 4.62554 / 2; // blueFeedingX
    public double rightFy = 2; // right feedingY
    public double rightFyFar = 0.5; // right feedingY
    public double leftFy = 6; // left feedingY
    public double leftFyFar = 7.5; // left feedingY

    public boolean isFeeding = true;
    public boolean turretZeroed = false;

    public double tagID;
    public Pose2d botPose = new Pose2d();

    public double elapsedTime;

    public double turretRotation;

    public double turretHubAngle = 0;

    public double waitTime = 0;

    public double theta = 0;
    public boolean isBlue = true;

    public boolean turretLocking = true;

    boolean limelightTurret = false;

    double Hy = isBlue ? blueHubPos.getY() : redHubPos.getY();

    PIDController turretPID = new PIDController(1, 1, 1);

    static double turretPosition;

    public TurretSubsystem() {

        // hoot testing stuff!

        // stop and return to start of log
        HootReplay.stop();
        var startPos = turretHoot.getPosition().getValue();
        // advance by 1 second and compare positions
        HootReplay.stepTiming(1.0);
        var endPos = turretHoot.getPosition().getValue();

        // pid
        TalonFXSConfiguration motorConfig = new TalonFXSConfiguration();
        motorConfig.MotorOutput.PeakForwardDutyCycle = Constants.TurretSubsystem.Turret_PeakForwardDutyCycle;
        motorConfig.MotorOutput.PeakReverseDutyCycle = Constants.TurretSubsystem.Turret_PeakReverseDutyCycle;
        // motor "friction" type?
        motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        // regulars
        motorConfig.Slot0.kP = Constants.TurretSubsystem.Turret_Slot0_kP;
        motorConfig.Slot0.kI = Constants.TurretSubsystem.Turret_Slot0_kI;
        motorConfig.Slot0.kD = Constants.TurretSubsystem.Turret_Slot0_kD;
        motorConfig.CurrentLimits.StatorCurrentLimitEnable = Constants.TurretSubsystem.Turret_StatorCurrentLimitEnable;
        motorConfig.CurrentLimits.StatorCurrentLimit = Constants.TurretSubsystem.Turret_StatorCurrentLimit;
        motorConfig.CurrentLimits.SupplyCurrentLimitEnable = Constants.TurretSubsystem.Turret_SupplyCurrentLimitEnable;
        motorConfig.CurrentLimits.SupplyCurrentLimit = Constants.TurretSubsystem.Turret_SupplyCurrentLimit;
        motorConfig.CurrentLimits.SupplyCurrentLowerLimit = Constants.TurretSubsystem.Turret_SupplyCurrentLowerLimit;
        motorConfig.CurrentLimits.SupplyCurrentLowerTime = Constants.TurretSubsystem.Turret_SupplyCurrentLowerTime;
        motorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = Constants.TurretSubsystem.Turret_FowardSoftLimitEnable;
        motorConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Constants.TurretSubsystem.Turret_FowardSoftLimit;
        motorConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = Constants.TurretSubsystem.Turret_ReverseSoftLimitEnable;
        motorConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = Constants.TurretSubsystem.Turret_ReverseSoftLimit;
        // Voltage
        motorConfig.Voltage.PeakForwardVoltage = Constants.TurretSubsystem.Turret_PeakForwardVoltage;
        motorConfig.Voltage.PeakReverseVoltage = Constants.TurretSubsystem.Turret_PeakReverseVoltage;
        // motion magic
        motorConfig.MotionMagic.MotionMagicAcceleration = 2000;
        motorConfig.MotionMagic.MotionMagicCruiseVelocity = 500;
        // gear ratio
        motorConfig.ExternalFeedback.SensorToMechanismRatio = Constants.TurretSubsystem.Turret_SensorToMechanismRatio;

        motorConfig.Commutation.MotorArrangement = MotorArrangementValue.Minion_JST;

        turret.getConfigurator().apply(motorConfig);

        Optional<Pose3d> poseTest = Constants.AprilTagPositions.aprilTags.getTagPose(1);

    }

    public void turretSensor() {
        if (!zeroingSensor.get()) {
            // zeroPosition();
        }
    }

    public void MoveMotor(double targetSpeed) {
        if (!turretLocking) {
            if (Math.abs(targetSpeed) > 0.1) {
                turret.set(-targetSpeed);
            } else {
                turret.set(0);
            }
        }

    }

    @Override
    public void periodic() {
        turretPosition = turret.getPosition().getValueAsDouble();

        Hy = isBlue ? blueHubPos.getY() : redHubPos.getY();
        isBlue = GameManager.isBlueAlliance;
        elapsedTime = Timer.getTimestamp();
        robotPos = new Translation2d(botPose.getX(), botPose.getY());
        theta = botPose.getRotation().getDegrees();

        turretSensor();

        SmartDashboard.putNumber("Turret Angle", turret.getPosition().getValueAsDouble());
        SmartDashboard.putBoolean("turretZeroSwitch", zeroingSensor.get());
    }

    public boolean turretToggle() {
        if (turretLocking == true) {
            turretLocking = false;
        } else if (!turretLocking) {
            turretLocking = true;
        }
        return turretLocking;
    }

    public void setPosition(Pose2d turretTarget, double targetSpeed, Pose2d turretPose) {
        MoveMotor(targetSpeed);

        if (turretLocking) {
            determineLockingMethod(turretTarget, turretPose);
        }

    }

    public void determineLockingMethod(Pose2d turretTarget, Pose2d turretPose) {
        turret.setControl(m_request.withPosition((calculateAngleToHub(turretTarget, turretPose))));
        limelightTurret = false;
    }

    public void setToZero() {
        turret.setControl(m_request.withPosition(-(0)));
    }

    double tagX = 0;

    double tagY = 0;

    double tagRotation; // certain values must change to accomadate apriltag orientation

    Optional<Pose3d> tagPose;

    double fX = 0;
    double fY = 0;
    // not like f(x), "f" just stands for furthest
    double sX = 0;
    double sY = 0;

    double offsetX = 0;
    double offsetY = 0;

    double rotatedX = 0;
    double rotatedY = 0;

    public double calculateAngleToHub(Pose2d turretTarget, Pose2d turretPose) {

        double diffY = (turretTarget.getY() - turretPose.getY());
        double diffX = (turretTarget.getX() - turretPose.getX());
        turretHubAngle = Math.toDegrees(Math.atan2(diffY, diffX));
        double goldenAngle = MathUtil.clamp(
                MathUtil.inputModulus((turretHubAngle - turretPose.getRotation().getDegrees()), -160, 200), -150, 190);
        SmartDashboard.putNumber("golden angle", goldenAngle);
        return goldenAngle;

    }

    public void setTurretPower(double setSpeed) {
        turret.set(setSpeed);
    }

    public void zeroTurretPosition(double zeroPosition) {
        turret.setPosition(zeroPosition);
    }

    public boolean turretAtTarget(Pose2d turretTarget, Pose2d turretPose) {
        boolean atTarget = MathUtil.isNear(calculateAngleToHub(turretTarget, turretPose), turretPosition, 10);
        SmartDashboard.putBoolean("Turret is at Target", atTarget);
        log.warn("turret is off {} degrees", calculateAngleToHub(turretTarget, turretPose) - turretPosition);
        return atTarget;
    }

}