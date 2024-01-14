// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import frc.robot.RobotContainer;
import java.lang.Math;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.SwerveSubsystem;

public class TeleopSwerve extends CommandBase {
  private SwerveSubsystem m_SwerveSubsystem;

  private DoubleSupplier m_translationSupplier;
  private DoubleSupplier m_strafeSupplier;
  private DoubleSupplier m_rotationSupplier;
  private BooleanSupplier m_robotCentricSupplier;
  private BooleanSupplier m_SnapPressed;
  private Limelight m_Limelight;
  private double m_tagHeightInches = 57.4166666;

  private SlewRateLimiter translationLimiter = new SlewRateLimiter(3.0); //can only change by 3 m/s in the span of 1 s
  private SlewRateLimiter strafeLimiter = new SlewRateLimiter(3.0);
  private SlewRateLimiter rotationLimiter = new SlewRateLimiter(3.0);
  private double rotationVal;
  private double translationVal;
  private double strafeVal;
  /** Creates a new TeleopSwerve. */
    public TeleopSwerve(SwerveSubsystem SwerveSubsystem,
        Limelight Limelight,     
        DoubleSupplier translationSupplier,
        DoubleSupplier strafeSupplier,
        DoubleSupplier rotationSupplier,
        BooleanSupplier robotCentricSupplier,
        BooleanSupplier rotationSnapPressed) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_SwerveSubsystem = SwerveSubsystem;
    this.m_Limelight = Limelight;
    this.m_SnapPressed = rotationSnapPressed;
    addRequirements(m_SwerveSubsystem);
    this.m_translationSupplier = translationSupplier;
    this.m_strafeSupplier = strafeSupplier;
    this.m_rotationSupplier = rotationSupplier;
    this.m_robotCentricSupplier = robotCentricSupplier;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
        /* Get Values, applies Deadband, (doesnt do anything if stick is less than a value)*/
        if (m_SnapPressed.getAsBoolean() == false) {
    strafeVal =
        strafeLimiter.calculate(
            MathUtil.applyDeadband(m_strafeSupplier.getAsDouble(), Constants.SwerveConstants.inputDeadband));
        }
        else {
          double m_y_angleToTagDegrees = Constants.LimelightConstants.m_limelightMountAngleDegree +  NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
          double m_y_angleToTagRadians = m_y_angleToTagDegrees * (3.14159 / 180.);

          double m_limelightToTagInches = Units.inchesToMeters(((m_tagHeightInches - Constants.LimelightConstants.m_limelightLensHeightInches) / Math.tan(m_y_angleToTagRadians)) - Constants.LimelightConstants.m_limelightToFrontOfRobot)*.07;

          strafeVal = m_limelightToTagInches*Math.tan(NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0));
          strafeVal = strafeLimiter.calculate(
            MathUtil.applyDeadband((strafeVal), Constants.SwerveConstants.inputDeadband));
        }
    double translationVal =
        translationLimiter.calculate(
            MathUtil.applyDeadband(m_translationSupplier.getAsDouble(), Constants.SwerveConstants.inputDeadband));
    if (m_SnapPressed.getAsBoolean() == false) 
    {
      SmartDashboard.putString("DB/String 5", Double.toString(m_rotationSupplier.getAsDouble()));
      rotationVal =
      rotationLimiter.calculate(
          MathUtil.applyDeadband(m_rotationSupplier.getAsDouble(), 0.7));
    }
    else 
    {
      double x_offset = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
      rotationVal = rotationLimiter.calculate(
           MathUtil.applyDeadband((x_offset/-27), Constants.SwerveConstants.inputDeadband));
      }
      
    
    /* Drive */
    m_SwerveSubsystem.drive(
        //the joystick values (-1 to 1) multiplied by the max speed of the drivetrain
        new Translation2d(translationVal, strafeVal).times(Constants.SwerveConstants.maxSpeed),
        //rotation value times max spin speed
        rotationVal * Constants.SwerveConstants.maxAngularVelocity,
        //whether or not in field centric mode
        !m_robotCentricSupplier.getAsBoolean(),
        //open loop control
        true);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
