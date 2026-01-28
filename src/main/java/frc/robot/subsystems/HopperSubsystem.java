package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class HopperSubsystem extends SubsystemBase{

    private TalonFX HopperMotor = new TalonFX(999999999);

    public void MoveHopperMotor(double targetSpeed) {
        HopperMotor.set(targetSpeed);
    }

}
