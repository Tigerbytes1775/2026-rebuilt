package frc.robot.subsystems.TurretSubsystems;


import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Launch extends SubsystemBase {

    private final SparkFlex motor = new SparkFlex(32, MotorType.kBrushless);
    private final RelativeEncoder encoder = motor.getEncoder();
    
    private boolean enabled = false;

    public final double incline = Math.toRadians(75);

    private final double launchWheelRadius = 2 * 0.0254;//  2 inches converted to meters
    private final double gearRatio = 1;

    //private final double launchKvRating = 505.44;// rpm/voltage 
    private final double strength = 1;

    private final double mpsToRpm = (60 * gearRatio) / (2*Math.PI*launchWheelRadius);
    private final double rpmToMps = (2*Math.PI*launchWheelRadius)/ (60 * gearRatio);


    private PIDController pidController;
    

    public Launch() {
        
        SmartDashboard.setPersistent("Launch PID");

        if (SmartDashboard.getNumberArray("Launch PID", new double[0]).length == 0) {
            SmartDashboard.putNumberArray("Launch PID", new double[]{0,0,0});
        }

        double[] PIDvalues = SmartDashboard.getNumberArray("Launch PID", new double[]{0,0,0});

        pidController =  new PIDController(
            PIDvalues[0],
            PIDvalues[1],
            PIDvalues[2]
        );
    }


    public void setMotors(double percent) {
        double power = percent * strength;
        motor.set(power);

        SmartDashboard.putNumber("Launch Power:(%)", power);
                
    }

    public void setTargetRPM(double rpm) {
        if (rpm == 0) {
            enabled = false;
        } else {
            enabled = true;
        }
        pidController.setSetpoint(rpm);
      
    }

    public void setLaunchSpeed(double mps) {
        double rpm = mps * mpsToRpm;
        setTargetRPM(rpm);
    }

    public double getRPM() {
        return encoder.getVelocity();
    }

    public double getLaunchSpeed() {
        return getRPM() * rpmToMps;
    }

    @Override
    public void periodic() {
        double power = 0;
        if (enabled) {
            power = pidController.calculate(encoder.getVelocity());
        }

        setMotors(power);
        SmartDashboard.putNumber("Launch RPM", getRPM());
        SmartDashboard.putNumber("Launch Speed", getRPM() * rpmToMps);
    }

    
}
