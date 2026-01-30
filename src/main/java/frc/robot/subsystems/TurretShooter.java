// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;

import edu.wpi.first.wpilibj2.command.SubsystemBase;



import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class TurretShooter extends SubsystemBase {
    
    private final SparkFlex launchMotor1 = new SparkFlex(0, SparkLowLevel.MotorType.kBrushless);
    private final SparkFlex launchMotor2 = new SparkFlex(1, SparkLowLevel.MotorType.kBrushless);

    private final double strength = 0.75;


    public TurretShooter() {}

    public void SetMotors(double percent) {

        double power = percent * strength;

        launchMotor1.set(power);
        launchMotor2.set(power);
        SmartDashboard.putNumber("Launch Power:(%)", power);
    }

}
