package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakePivot extends SubsystemBase {
    
    
    private final SparkMax intakePivotMotor = new SparkMax(24, SparkLowLevel.MotorType.kBrushless);

    private final double intakeStrenght = 1.0;
    
    
    public IntakePivot() {}

    public void setMotors(double percent) {
        double power = percent * intakeStrenght;

        SmartDashboard.putNumber("Intake Pivot power(%)", power);

        if (percent == 0){
            intakePivotMotor.stopMotor();
        } else {
            intakePivotMotor.set(power);
        }
        
    }
    
}
