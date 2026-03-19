package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Ramp extends SubsystemBase {
    
    
    private final SparkMax motor = new SparkMax(29, SparkLowLevel.MotorType.kBrushless);

    private final double rampStrength = 0.3;
    
    
    public Ramp() {}

    public void setMotors(double percent) {
        double power = percent * rampStrength;

        SmartDashboard.putNumber("Ramp power(%)", power);

        if (percent == 0){
            motor.stopMotor();
        } else {
            motor.set(power);
        }
        
    }
    
}
