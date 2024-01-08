// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.autos.PathPlannerExample;
import frc.robot.commands.TeleopSwerve;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.smartdashboard.*;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.Limelight;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Constants.SwerveConstants;
import java.io.IOException;


public class RobotContainer {

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_XboxController = new CommandXboxController(0);

  //private final CommandJoystick m_JoystickL = new CommandJoystick(0);
  //private final CommandJoystick m_JoystickR = new CommandJoystick(1);

  //path planner sendable chooser
  public static final String m_testauto = "Default";
  public static final String m_auto1 = "Auto Option Example";

  //side sendable chooser
  public static final String m_blue = "Blue";
  public static final String m_red = "Red";

  public String m_autoSelected;
  public String m_sideChosen;

  public final SendableChooser<String> m_chooser = new SendableChooser<>();
  public final SendableChooser<String> m_side_chooser = new SendableChooser<>();

  /* Drive Controls */
  private final int translationAxis = XboxController.Axis.kLeftY.value;
  private final int strafeAxis = XboxController.Axis.kLeftX.value;
  private final int rotationAxis = XboxController.Axis.kRightX.value;
  
  private final Trigger robotCentric =
  new Trigger(m_XboxController.leftBumper());
  //path planner

  
  //Subsystems 
  private final SwerveSubsystem m_SwerveSubsystem = new SwerveSubsystem();
  private final Limelight m_Limelight = new Limelight();
  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    m_chooser.setDefaultOption("Test Path", m_testauto);
    m_chooser.addOption("Example Option", m_auto1);
    m_side_chooser.setDefaultOption("Blue", m_blue);
    m_side_chooser.addOption("Red", m_red);
    SmartDashboard.putData("Auto Choices", m_chooser);
    SmartDashboard.putData("Side", m_side_chooser);

    m_SwerveSubsystem.setDefaultCommand(
      new TeleopSwerve(
          m_SwerveSubsystem,
          () -> -m_XboxController.getRawAxis(translationAxis),
          () -> -m_XboxController.getRawAxis(strafeAxis),
          () -> -m_XboxController.getRawAxis(rotationAxis),
          () -> robotCentric.getAsBoolean()));


    
    // Configure the trigger bindings
    configureBindings();

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
    m_XboxController.button(Button.kY.value).onTrue(new InstantCommand(() -> m_SwerveSubsystem.zeroGyro()));
    m_XboxController.button(Button.kB.value).onTrue(new InstantCommand(() -> m_SwerveSubsystem.setWheelsToX()));
    m_XboxController.button(Button.kA.value).onTrue(new InstantCommand(() -> m_Limelight.LimeToDrive()));
   
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  /*private void configureAutoCommands() {

    AUTO_EVENT_MAP.put("event1", new PrintCommand("passed marker 1"));

    ArrayList<PathPlannerTrajectory> auto1Paths =
      PathPlanner.loadPathGroup(
        "testpath",
        automaxspeed,
        autoacceleration
      );
     
    )*/
    public String getSide() {
      m_sideChosen = m_side_chooser.getSelected();

      return m_sideChosen;
    }
    public Command getAutonomousCommand() {
      // An ExampleCommand will run in
      m_autoSelected = m_chooser.getSelected();


      switch (m_autoSelected)
      {
        //to add an auto, first declare as a string, then add as a smart dash option
        //make path and make an auto file copying the "PathPlannerExample.java" file
        //replace the trajectory with your own name of the path planner export
        //make a case in this string where you return a command beginning a new instance of your auto
        case m_auto1:
          return new PathPlannerExample(m_SwerveSubsystem);
        default:
          return new PathPlannerExample(m_SwerveSubsystem);
      }
    }

}
