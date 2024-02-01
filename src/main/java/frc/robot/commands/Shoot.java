package frc.robot.commands;

import java.util.function.BooleanSupplier;

import javax.lang.model.util.ElementScanner14;

import frc.robot.subsystems.Climber;
import frc.robot.subsystems.GroundIntake;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj2.command.Command;

public class Shoot extends Command
{
    Climber climber;
    Shooter shooter;
    GroundIntake intake;

    private int elevator_point;

    public Shoot(Climber m_Climber, Shooter m_Shooter, GroundIntake m_Intake) 
    {
        this.climber = m_Climber;
        this.shooter = m_Shooter;
        this.intake = m_Intake;

        this.elevator_point = climber.getInches();

        addRequirements(m_Climber);
        addRequirements(m_Shooter);
        addRequirements(m_Intake);

        shooter.toSetPoint(0);
        intake.toSetPoint(0);

        if (elevator_point <= 1)
        {
            intake.setRollers(-50);
            //CALL AUTOMATIC LIMELIGHTSHOOTING HERE!!!!
        } 
        else if (elevator_point <= 7)
        {
            shooter.runFlyWheels(80);
            shooter.runFeederWheels(95);
            
        }
        else 
        {
            intake.setRollers(0);
            shooter.runFlyWheels(-50);
            shooter.runFeederWheels(-50);
        }
    }
}