package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Shooter extends SubsystemBase{
    private TalonFX shooterMotor = new TalonFX(Constants.ShooterConstants.Shooter_ID);

    private TalonFX flyWheelOne = new TalonFX(Constants.ShooterConstants.FlyWheelOne_ID);
    private TalonFX flyWheelTwo = new TalonFX(Constants.ShooterConstants.FlyWheelTwo_ID);
    private TalonFX feederWheel = new TalonFX(Constants.ShooterConstants.FeederWheel_ID);

    private double intakeHandoff = 18;
    private double climbingPosition = 38.56;
    private double ampPosition = 49;

    private double[] m_setPoints = {0, intakeHandoff, climbingPosition, ampPosition};

  /** Creates a new ExampleSubsystem. */
    public Shooter() {
      shooterMotor.setPosition(0);
      flyWheelOne.setInverted(true);
      flyWheelTwo.setControl(new Follower(13, false));

      var talonFXConfigs = new TalonFXConfiguration();

      var slot0Configs = talonFXConfigs.Slot0;

      slot0Configs.kV = Constants.ShooterConstants.kV;
      slot0Configs.kP = Constants.ShooterConstants.kP;
      slot0Configs.kI = Constants.ShooterConstants.kI;
      slot0Configs.kD = Constants.ShooterConstants.kD;

      shooterMotor.getConfigurator().apply(slot0Configs, 0.020);
    }

    public void toSetPoint(int setPoint) 
    {
      var motorPosSignal = shooterMotor.getRotorPosition();
      var motorPos = motorPosSignal.getValue();

      final MotionMagicExpoVoltage m_PIDRequest = new MotionMagicExpoVoltage(0);
      shooterMotor.setControl(m_PIDRequest.withPosition(m_setPoints[setPoint]));
      SmartDashboard.putNumber("supposed setpoint", setPoint);

      
    }

    public void toCustomSetpoint(double degrees)
    {
      double toTicks = degreesToRotations(degrees);

      final MotionMagicExpoVoltage m_PIDRequest = new MotionMagicExpoVoltage(0);
      shooterMotor.setControl(m_PIDRequest.withPosition(toTicks));
    }

    public double degreesToRotations(double degrees)
    {
      return (degrees / 360. * 218.75);
    }

    public void runFeederAtSet(int OutputPercent)
    {

      //OutputPercent /= 100;
      //feederWheel.set(OutputPercent);
    }
    
    public double getEncoder()
    {
      return shooterMotor.getPosition().getValueAsDouble();
    }
    public void runFlyWheels(double OutputPercent)
    {
      OutputPercent /= 100.;
      SmartDashboard.putString("DB/String 9", Double.toString(OutputPercent));
      flyWheelOne.set(OutputPercent);
    }

    public void runFeederWheels(double OutputPercent)
    {
      OutputPercent /= 100.;
      SmartDashboard.putString("DB/String 0", Double.toString(OutputPercent));
      feederWheel.set(OutputPercent);
    }  
    public double getV()
    {
      return shooterMotor.getVelocity().getValueAsDouble();
    }
    /**
     * An example method querying a boolean state of the subsystem (for example, a digital sensor).
     *
     * @return value of some boolean subsystem state, such as a digital sensor.
     */
    @Override
    public void periodic() 
    {
    // This method will be called once per scheduler run
    }

    @Override
    public void simulationPeriodic() 
    {
      // This method will be called once per scheduler run during simulation
    }

    public void print(String message)
    {
      SmartDashboard.putString("DB/String 3", message);
    }
}

