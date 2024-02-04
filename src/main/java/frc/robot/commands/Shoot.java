package frc.robot.commands;

import java.util.function.BooleanSupplier;

import javax.lang.model.util.ElementScanner14;

import frc.robot.subsystems.Climber;
import frc.robot.subsystems.GroundIntake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.Limelight;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;


public class Shoot extends Command
{
    Climber climber;
    Shooter shooter;
    GroundIntake intake;
    SwerveSubsystem swerve;
    Limelight limelight;

    private double elevator_point;

    private double podium_degrees = 33.333;
    private double subwoofer_degrees = 65.662787;
    private double x_distance;
    private double degrees;

    public Shoot(Climber m_Climber, Shooter m_Shooter, GroundIntake m_Intake, SwerveSubsystem m_SwerveSubsystem, Limelight m_Limelight) 
    {
        this.climber = m_Climber;
        this.shooter = m_Shooter;
        this.intake = m_Intake;
        this.limelight = m_Limelight;
        this.swerve = m_SwerveSubsystem;

        addRequirements(m_Climber);
        addRequirements(m_Shooter);
        addRequirements(m_Intake);
        addRequirements(m_Limelight);
        addRequirements(m_SwerveSubsystem);
    }
    
    public void initialize()
    {            

        this.elevator_point = climber.getInches();
        intake.toSetPoint(0);



        if (elevator_point <= 1)
        {
            x_distance = Limelight.find_Tag_Y_Distance();
            degrees = degreesFromDistance(x_distance);
            shooter.runFlyWheels(80);
            intake.setRollers(-50);
            shooter.runFeederWheels(95);
            shooter.toCustomSetpoint(degrees);

            
            //CALL AUTOMATIC LIMELIGHTSHOOTING HERE!!!!
        } 
        else if (elevator_point <= 7)
        {
            intake.setRollers(0);
            shooter.runFlyWheels(80);
            shooter.runFeederWheels(95);
            
        }
        else 
        {
            intake.setRollers(0);
            shooter.runFlyWheels(-50);
            shooter.runFeederWheels(-50);
        }
        isFinished();
    }
    
    public boolean isFinished()
    {
        return true;
    }

    public double degreesFromDistance(double distance)
    {
        return (Math.atan(2.0447/distance));
    }
}