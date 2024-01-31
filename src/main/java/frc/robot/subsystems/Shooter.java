package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Shooter extends SubsystemBase{
    private TalonFX shooterMotor = new TalonFX(Constants.ShooterConstants.Shooter_ID);

    private Timer feeder_timer = new Timer();

    private TalonFX flyWheelOne = new TalonFX(Constants.ShooterConstants.FlyWheelOne_ID);
    private TalonFX flyWheelTwo = new TalonFX(Constants.ShooterConstants.FlyWheelTwo_ID);
    private TalonFX feederWheel = new TalonFX(Constants.ShooterConstants.FeederWheel_ID);

    private int intakeHandoff = 5;
    private int shootingPosition = 10;

    private int[] m_setPoints = {0, intakeHandoff, shootingPosition};

  /** Creates a new ExampleSubsystem. */
    public Shooter() {
        shooterMotor.setPosition(0);
        //flyWheelTwo.f(flyWheelOne);

        var talonFXConfigs = new TalonFXConfiguration();

        var slot0Configs = talonFXConfigs.Slot0;


        slot0Configs.kV = Constants.ShooterConstants.kV;
        slot0Configs.kP = Constants.ShooterConstants.kP;
        slot0Configs.kI = Constants.ShooterConstants.kI;
        slot0Configs.kD = Constants.ShooterConstants.kD;

        /* var motionMagicConfigs = talonFXConfigs.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = 0;
        motionMagicConfigs.MotionMagicExpo_kV = 0.12;
        motionMagicConfigs.MotionMagicExpo_kA = 0.1; */

        shooterMotor.getConfigurator().apply(slot0Configs, 0.020);
    }

    public void toSetPoint(int setPoint) 
    {
        var motorPosSignal = shooterMotor.getRotorPosition();
        var motorPos = motorPosSignal.getValue();

        final MotionMagicExpoVoltage m_PIDRequest = new MotionMagicExpoVoltage(0);
        shooterMotor.setControl(m_PIDRequest.withPosition(m_setPoints[setPoint]));
    }

    public void runFlyWheels(int OutputPercent)
    {
        OutputPercent /= 100;
        flyWheelOne.set(OutputPercent);
    }

    public void runFeederWheels(int OutputPercent)
    {
        OutputPercent /= 100;
        feederWheel.set(OutputPercent);

    }  
    /**
     * An example method querying a boolean state of the subsystem (for example, a digital sensor).
     *
     * @return value of some boolean subsystem state, such as a digital sensor.
     */
    @Override
    public void periodic() {
    // This method will be called once per scheduler run
    }

    @Override
    public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    }
    public void print(String message)
  {
    SmartDashboard.putString("DB/String 3", message);
  }
}

