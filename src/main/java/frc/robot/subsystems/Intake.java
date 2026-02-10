package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    
    
    private final SparkMax intakeMotor = new SparkMax(27, SparkLowLevel.MotorType.kBrushless);

    private final double intakeStrength = 0.5;
    
    
    public Intake() {}

    public void setMotors(double percent) {
        double power = percent * intakeStrength;

        SmartDashboard.putNumber("Intake power(%)", power);

        if (percent == 0){
            intakeMotor.stopMotor();
        } else {
            intakeMotor.set(power);
        }
        
    }
    
}
