package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PivotConstants;
import frc.robot.Constants.PivotConstants.State;
import frc.robot.Constants.RobotConstants.CAN;
import frc.robot.subsystems.sim.FernPivotSim;

public class Pivot extends SubsystemBase {
    
    private final WPI_TalonFX mMaster;
    private final WPI_TalonFX mSlave;

    private final DutyCycleEncoder mEncoder;
    private final PIDController mPID;

    private State mCurrentState = State.STARTING;
    public double falconOffset;

    public FernPivotSim mPivotSim;

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

        configureMotor();

        mPivotSim = new FernPivotSim(
            mMaster.getSimCollection(), 
            this::getMotorSet, 
            this::getAngle
        );
        
    }

    public void configureMotor() {

        mMaster.configFactoryDefault();
        mSlave.configFactoryDefault();

        mMaster.setNeutralMode(NeutralMode.Brake);
        mSlave.setNeutralMode(NeutralMode.Brake);


        SupplyCurrentLimitConfiguration mPivotCurrentLimit = new SupplyCurrentLimitConfiguration(true, 25, 25, 0);
        mMaster.configSupplyCurrentLimit(mPivotCurrentLimit);
        mSlave.configSupplyCurrentLimit(mPivotCurrentLimit);
        
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

    public double getMotorSet(){
        return mMaster.get();
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

        SmartDashboard.putNumber("Pivot/TBE Raw", mEncoder.getAbsolutePosition());
        SmartDashboard.putNumber("Pivot/TBE Degrees", getThroughBoreAngle());
        SmartDashboard.putNumber("Pivot/Falcon Degrees", getAngle());
        SmartDashboard.putNumber("Pivot/Falcon Offset", falconOffset);
        SmartDashboard.putBoolean("Pivot/At Setpoint", atTarget());
        SmartDashboard.putNumber("Pivot/Motor Voltage", mMaster.get() * 10);
        SmartDashboard.putNumber("Pivot/Set Point", mCurrentState.angle);
        SmartDashboard.putNumber("Pivot/Error", mPID.getPositionError());

        SmartDashboard.putString("States/Pivot", mCurrentState.toString());

    }

    @Override
    public void simulationPeriodic() {
        mPivotSim.update();
    }

}