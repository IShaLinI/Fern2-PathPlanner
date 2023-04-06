package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXSimCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PivotConstants;
import frc.robot.Constants.PivotConstants.State;
import frc.robot.Constants.RobotConstants.CAN;

public class Pivot extends SubsystemBase {
    
    private final WPI_TalonFX mMaster;
    private final WPI_TalonFX mSlave;

    private final DutyCycleEncoder mEncoder;
    private final PIDController mPID;

    private State mCurrentState = State.STARTING;
    public double falconOffset;

    private final SingleJointedArmSim mArmSim = new SingleJointedArmSim(
        LinearSystemId.createSingleJointedArmSystem(
            DCMotor.getFalcon500(2), SingleJointedArmSim.estimateMOI(0.57, Units.lbsToKilograms(7.169)), (20d * 60d/16d)
        ),
        DCMotor.getFalcon500(2),
        (20 * 60d/16d),
        0.57,
        Units.degreesToRadians(-90),
        Units.degreesToRadians(140),
        true
    );

    private final TalonFXSimCollection mArmMotorSim;

    private final Mechanism2d mMech2d = new Mechanism2d(60, 60);
    private final MechanismRoot2d mArmPivot = mMech2d.getRoot("Pivot", 30, 30);
    private final MechanismLigament2d mArmTower = mArmPivot.append(new MechanismLigament2d("ArmTower", 30, -90));
    private final MechanismLigament2d mArm = mArmPivot.append(
        new MechanismLigament2d("Arm", 20, Units.degreesToRadians(mArmSim.getAngleRads()),
        6,
        new Color8Bit(Color.kYellow)
    ));

    public Pivot() {

        mMaster = new WPI_TalonFX(CAN.kFrontPivot);
        mSlave = new WPI_TalonFX(CAN.kBackPivot);

        mEncoder = new DutyCycleEncoder(0);
        mPID = new PIDController(2.5 / 20d, 0, 0);
       
        Timer.delay(2);
        
        mMaster.setSelectedSensorPosition(0);

        if(RobotBase.isReal()){
            mEncoder.setPositionOffset(PivotConstants.kThroughboreOffset);
            falconOffset = degreeToFalcon(getThroughBoreAngle());
        }
        mArmMotorSim = mMaster.getSimCollection();


        SmartDashboard.putData("Arm Sim", mMech2d);

        configureMotor();
        
    }

    public void configureMotor() {

        mMaster.configFactoryDefault();
        mMaster.setNeutralMode(NeutralMode.Brake);

        mMaster.configVoltageCompSaturation(10);
        mMaster.enableVoltageCompensation(true);
        mMaster.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 25, 25, 0));
    
        mSlave.configFactoryDefault();
        mSlave.setNeutralMode(NeutralMode.Brake);

        mSlave.configVoltageCompSaturation(10);
        mSlave.enableVoltageCompensation(true);
        mSlave.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 25, 25, 0));
        
        mSlave.follow(mMaster);
        mSlave.setInverted(InvertType.OpposeMaster);

    }

    public void runPivot() {

        double output = MathUtil.clamp(mPID.calculate(getAngle(), mCurrentState.angle)/12, -0.8, 0.8);

        double kS = 0.009;

        output += (getAngle() < 0) ? kS : -kS; 
        
        mMaster.set(output);
        
    }

    public boolean atTarget() {

        double tolerance = 5; 

        return (Math.abs(getAngle() - mCurrentState.angle) < tolerance);

    }
  

    public double falconToDegrees(double val){
        return val * 1/2048d * PivotConstants.kGearing * 360;
    }

    public void zeroEncoder() {
        mMaster.setSelectedSensorPosition(0);
        falconOffset = degreeToFalcon(getThroughBoreAngle());
    }

    public double getAngle() {
        return falconToDegrees(mMaster.getSelectedSensorPosition() + falconOffset);
    }

    public double getThroughBoreAngle () {
        return ((mEncoder.getAbsolutePosition()) - mEncoder.getPositionOffset()) * 360;
    }

    public void resetFalcon() {
        mMaster.setSelectedSensorPosition(0);
    }

    public double degreeToFalcon(double deg) {
        return (deg * 2048d * 1/PivotConstants.kGearing * 1/360);
    }

    public Command changeState(State state){
      return new InstantCommand(() -> mCurrentState = state);
    }

    public void set(double percent){
      mMaster.set(percent);
    }

    @Override
    public void periodic() {

        runPivot();

        SmartDashboard.putNumber("TBE Raw", mEncoder.getAbsolutePosition());
        SmartDashboard.putNumber("TBE Degrees", getThroughBoreAngle());
        SmartDashboard.putNumber("Falcon Degrees", getAngle());
        SmartDashboard.putNumber("Falcon Offset", falconOffset);
        SmartDashboard.putBoolean("At Setpoint", atTarget());
        SmartDashboard.putNumber("Motor Voltage", mMaster.get() * 12);
        SmartDashboard.putNumber("Set Point", mCurrentState.angle);
        SmartDashboard.putNumber("Error", mPID.getPositionError());
        SmartDashboard.putNumber("front raw encoder", mMaster.getSelectedSensorPosition());
        SmartDashboard.putNumber("back raw encoder", mSlave.getSelectedSensorPosition());

    }

    @Override
    public void simulationPeriodic() {
        mArmSim.setInput(mMaster.get() * RobotController.getBatteryVoltage());
        mArmSim.update(0.02);
        double poseDelta = (Units.radiansPerSecondToRotationsPerMinute(mArmSim.getVelocityRadPerSec()) / 600) * 60d/16d * 20 * 2048;
        mArmMotorSim.addIntegratedSensorPosition((int)poseDelta);
        mArm.setAngle(getAngle() + 90);

    }

}