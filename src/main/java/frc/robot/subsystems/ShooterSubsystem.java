package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {

    private TalonFX shooterMotorL = new TalonFX(15);
    private TalonFX shooterMotorR = new TalonFX(16);

    private VelocityVoltage m_velocity = new VelocityVoltage(0);

    public static double tof = 1.1; // used to be 1.1
     double targetRPM = 0;
     Timer timer = new Timer();

    // Basic targeting data
    // in degrees
    VelocityVoltage velocity = new VelocityVoltage(0);

    InvertedValue Invert = InvertedValue.Clockwise_Positive;
    NeutralModeValue Coast = NeutralModeValue.Coast;
    NeutralModeValue Break = NeutralModeValue.Brake;

    public ShooterSubsystem() {

        // pid
        TalonFXConfiguration shooterConfig = new TalonFXConfiguration();
        shooterConfig.MotorOutput.PeakForwardDutyCycle = Constants.ShooterSubsystem.Shooter_PeakForwardDutyCycle;
        shooterConfig.MotorOutput.PeakReverseDutyCycle = Constants.ShooterSubsystem.Shooter_PeakReverseDutyCycle;
        // motor "friction" type?
        shooterConfig.MotorOutput.NeutralMode = Coast;
        shooterConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        // regulars
        shooterConfig.CurrentLimits.StatorCurrentLimitEnable = Constants.ShooterSubsystem.Shooter_StatorCurrentLimitEnable;
        shooterConfig.CurrentLimits.StatorCurrentLimit = Constants.ShooterSubsystem.Shooter_CurrentLimit;

        shooterConfig.CurrentLimits.SupplyCurrentLimitEnable = Constants.ShooterSubsystem.Shooter_SupplyCurrentLimitEnable;
        shooterConfig.CurrentLimits.SupplyCurrentLimit = Constants.ShooterSubsystem.Shooter_SupplyCurrentLimit;

        // Voltage
        shooterConfig.Voltage.PeakForwardVoltage = Constants.ShooterSubsystem.Shooter_PeakForwardVoltage;
        shooterConfig.Voltage.PeakReverseVoltage = Constants.ShooterSubsystem.Shooter_PeakReverseVoltage;

        // Motion Magic
        shooterConfig.Slot0.kP = Constants.ShooterSubsystem.Shooter_Slot0_kP;
        shooterConfig.Slot0.kI = Constants.ShooterSubsystem.Shooter_Slot0_kI;
        shooterConfig.Slot0.kD = Constants.ShooterSubsystem.Shooter_Slot0_kD;
        shooterConfig.Slot0.kS = Constants.ShooterSubsystem.Shooter_Slot0_kS;
        shooterConfig.Slot0.kV = Constants.ShooterSubsystem.Shooter_Slot0_kV;
        shooterConfig.MotionMagic.MotionMagicCruiseVelocity = Constants.ShooterSubsystem.Shooter_MotionMagicCruiseVelocity;
        shooterConfig.MotionMagic.MotionMagicAcceleration = Constants.ShooterSubsystem.Shooter_MotionMagicAcceleration;

        shooterMotorL.getConfigurator().apply(shooterConfig);
        shooterMotorR.getConfigurator().apply(shooterConfig);

        shooterMotorR.setControl(new Follower(15, MotorAlignmentValue.Opposed));
        timer.start();

    }

    public void RuntoRPMs(double distanceToHub, boolean isFeeding) {
        shooterMotorL.setControl(m_velocity.withVelocity(CalculateRpms(distanceToHub, isFeeding)));
    }

    public void SetShooterRPMS(double setRPMS) {
        shooterMotorL.setControl(m_velocity.withVelocity(setRPMS));
    }
    

    @Override
    public void periodic() {
        // if(timer.advanceIfElapsed(1)){
        //     System.out.println(String.format("targetRpms: %f, actualRpms: %f", targetRPM , shooterMotorL.getVelocity().getValueAsDouble()));
        // }

        //SmartDashboard.putNumber("shooterRPMS", shooterMotorL.getVelocity().getValueAsDouble());
     //   SmartDashboard.putNumber("shooterAMPS", shooterMotorL.getSupplyCurrent().getValueAsDouble());

    }

    public double CalculateRpms(double distanceToHub, boolean isFeeding) {
       
        double closeHubM = 4.67989; // 6.99334;
        double closeHubB = 37; // 38.81499;
        double farHubM = 6.48875;
        double farHubB = 30;
        double feedM = 3.43042;
        double feedB = 40.27793;
        if (isFeeding) {
            targetRPM = feedM * (distanceToHub) + feedB;
        } else {
            if(distanceToHub <= 4.5){
                targetRPM = closeHubM * (distanceToHub) + closeHubB;
            } else {
                targetRPM = farHubM * (distanceToHub) + farHubB;
            }
        }

       

        // y=mx+b where "y" is the target RPM and "x" is the distance between the robot
        // and target. to find the slope, determine positions and rpms that we know work
        // on certain spots on the field, and create a line of best fit.

        return targetRPM;
    }

}