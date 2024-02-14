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
import edu.wpi.first.wpilibj.Timer;
 

public class Shoot extends Command
{
   
    Timer timer = new Timer();

    Climber climber;
    Shooter shooter;
    GroundIntake intake;
    SwerveSubsystem swerve;
    Limelight limelight;
    int MODE;

    private double elevator_point;
    //commented out bc no reasonable way to determine if we are there without doing math we would be "skipping"
    //  private double podium_degrees = 33.333;
    //  private double subwoofer_degrees = 65.662787;

    private double distance;
    private double degrees;

    public Shoot(Climber m_Climber, Shooter m_Shooter, GroundIntake m_Intake, SwerveSubsystem m_SwerveSubsystem, Limelight m_Limelight, int mode) 
    {
        this.climber = m_Climber;
        this.shooter = m_Shooter;
        this.intake = m_Intake;
        this.limelight = m_Limelight;
        this.swerve = m_SwerveSubsystem;
        this.MODE = mode;

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

        if (MODE == 0)
        {
            if (elevator_point <= 1)
            {
                int cur_id = limelight.getID();

                distance = limelight.find_Tag_Y_Distance(limelight.findTagHeightFromID(limelight.check_eligible_id(cur_id)));

                if (distance != -1) 
                {
                    degrees = (Math.atan(2.0447/distance));

                    shooter.toCustomSetpoint(degrees);
                    shooter.runFlyWheels(80);
                    intake.setRollers(-50);

                    isFinished(true);
                }
                
                //CALL AUTOMATIC LIMELIGHTSHOOTING HERE!!!!
            } 
            else if (elevator_point <= 7)
            {
                intake.setRollers(0);
                shooter.runFlyWheels(80);
                shooter.runFeederWheels(95);
                isFinished(false);
            }
            else 
            {
                intake.setRollers(0);
                shooter.runFlyWheels(-50);
                shooter.runFeederWheels(-50);
                isFinished(false);
            }
        }
        else if (MODE == 1)
        {
            //shooter.toSetPoint(0);
            shooter.toCustomSetpoint(20);
            shooter.runFeederWheels(0);
            shooter.runFlyWheels(0);
            
            intake.setRollers(0);
        }
        else
        {
            shooter.toSetPoint(0);
            shooter.runFeederWheels(0);
            shooter.runFlyWheels(0);
            intake.setRollers(0);
            isFinished(false);

        }
        timer.restart();
    }

    public void execute() 
    {
        if (MODE != 2)
        {
            isFinished(true);
        }
        else
        {
            isFinished(false);
        }
    }

    public boolean isFinished(boolean keepChecking)
    {
        SmartDashboard.putNumber("shooter velocity", shooter.getV());

        if (keepChecking == false)
        {
            SmartDashboard.putString("Command finished","Finished");

            return true;
        } 

        else if (timer.get() > 1)
        {
            if (Math.abs(shooter.getV()) > 0)
            {
                SmartDashboard.putString("Command finished","Not");

                return false;
            } 
            else 
            {
                SmartDashboard.putString("Command finished","Finished");

            //  shooter.toSetPoint(0);
                
                shooter.runFeederWheels(80);
                shooter.runFlyWheels(95);
                intake.setRollers(-50);

                return true;
            }
        }
     //   }
        return false;
    }
}