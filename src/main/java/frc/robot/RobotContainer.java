// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.IntakePivotCommand;
import frc.robot.commands.RampCommand;
import frc.robot.commands.TurretTeleopCommand;
import frc.robot.commands.VisionTeleopCommand;
import frc.robot.commands.ClimbCommand;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.IntakePivot;
import frc.robot.subsystems.Ramp;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.TurretSubsystems.Launch;
import frc.robot.subsystems.TurretSubsystems.LazySusan;
import frc.robot.subsystems.TurretSubsystems.Turret;
import swervelib.SwerveDrive;
import swervelib.SwerveInputStream;



/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here..
  
  private final SwerveSubsystem swerveSubsystem= new SwerveSubsystem();
  private final SwerveDrive swerveDrive = swerveSubsystem.getSwerveDrive();

  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  
  private final Intake intake = new Intake();
  private final IntakePivot intakePivot = new IntakePivot();
  private final Ramp ramp = new Ramp();
  private final Launch launch = new Launch();
  private final LazySusan lazySusan = new LazySusan();
  private final Vision vision = new Vision(swerveDrive);


  private final Climb climb = new Climb();

  private final Turret turret = new Turret(launch,lazySusan,ramp,swerveSubsystem);
  
  private final XboxController driverController = new XboxController(0);
  private final XboxController mechController = new XboxController(1);

  private final SendableChooser<Command> autoChooser;


  public RobotContainer() {
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
    // Configure the trigger bindings
    


    configureBindings();
    configuesCommands();
  }

  private void configuesCommands() {
    
    intake.setDefaultCommand(new IntakeCommand(intake, mechController));
    climb.setDefaultCommand(new ClimbCommand(climb, driverController) );
    intakePivot.setDefaultCommand(new IntakePivotCommand(intakePivot, mechController));
    ramp.setDefaultCommand(new RampCommand(ramp, mechController));
    turret.setDefaultCommand(new TurretTeleopCommand(turret, driverController));
    vision.setDefaultCommand(new VisionTeleopCommand(vision, () -> mechController.getPOV() == 270));
  }
  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    DoubleSupplier swerveScalar = () -> driverController.getLeftBumperButton() || driverController.getRightBumperButton() ? 0.5 : 1.0;
    SwerveInputStream driveAngularVelocity = SwerveInputStream.of(
      swerveSubsystem.getSwerveDrive(),
      () -> driverController.getLeftY() * -swerveScalar.getAsDouble(),
      () -> driverController.getLeftX() * -swerveScalar.getAsDouble()
    )
      .withControllerRotationAxis(() -> -driverController.getRightX() * swerveScalar.getAsDouble())
      .deadband(OperatorConstants.DEADBAND)
      .scaleTranslation(0.8)//####################################driver 1 deadband##################################3
      .allianceRelativeControl(true);


      
    //SwerveInputStream driveDirectAngle = driveAngularVelocity.copy()
    //  .withControllerHeadingAxis(driverController::getRightX,
    //      driverController::getRightY)
    //  .headingWhile(false);
    //// affects the things
    //Command driveFieldOrientedDirectAngle = swerveSubsystem.driveFieldOriented(driveDirectAngle);

    Command DriveFieldOrientedAngularVelocity = swerveSubsystem.driveFieldOriented(
      driveAngularVelocity, 
      () -> driverController.getPOV() != -1
    );
    swerveSubsystem.setDefaultCommand(DriveFieldOrientedAngularVelocity);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
