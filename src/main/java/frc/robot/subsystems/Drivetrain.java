package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.TalonFXSimCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.BasePigeonSimCollection;
import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod;
import com.ctre.phoenix.sensors.WPI_Pigeon2;
import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.controller.LinearPlantInversionFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotGearing;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotWheelSize;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.DriveConstants.FrontState;
import frc.robot.Constants.RobotConstants.CAN;

public class Drivetrain extends SubsystemBase {

  private final WPI_TalonFX mFrontLeft = new WPI_TalonFX(CAN.kFrontLeft);
  private final WPI_TalonFX mFrontRight = new WPI_TalonFX(CAN.kFrontRight);
  private final WPI_TalonFX mBackLeft = new WPI_TalonFX(CAN.kBackLeft);
  private final WPI_TalonFX mBackRight = new WPI_TalonFX(CAN.kBackRight);

  private final TalonFXSimCollection mLeftSim = mFrontLeft.getSimCollection();
  private final TalonFXSimCollection mRightSim = mFrontRight.getSimCollection();

  private final WPI_Pigeon2 mPigeon = new WPI_Pigeon2(CAN.kPigeon);

  private final BasePigeonSimCollection mPigeonSim = mPigeon.getSimCollection();

  private final DifferentialDriveOdometry mOdometry = new DifferentialDriveOdometry(mPigeon.getRotation2d(), 0, 0);

  private PIDController mPID = new PIDController(0.5, 0, 0);

  private FrontState mCurrentState;

  public static final LinearSystem<N2,N2,N2> mDrivetrainPlant = LinearSystemId.identifyDrivetrainSystem(
    2.2195, //Linear kV
    0.32787, //Linear kA
    2.53, //Angular kV
    0.081887 //Angular kA
  );

  private LinearPlantInversionFeedforward<N2,N2,N2> mFeedForward = new LinearPlantInversionFeedforward<N2,N2,N2>(mDrivetrainPlant,0.02);

  // Default Sim
  public DifferentialDrivetrainSim mDrivetrainSim = new DifferentialDrivetrainSim( // Simulation
      mDrivetrainPlant,
      DCMotor.getFalcon500(2),
      KitbotGearing.k10p71.value,
      Constants.DriveConstants.kTrackwidth,
      KitbotWheelSize.kSixInch.value,
      null
  );


  public void configureMotors(){

    mFrontLeft.configFactoryDefault();
    mFrontRight.configFactoryDefault();
    mBackLeft.configFactoryDefault();
    mBackRight.configFactoryDefault();
 
     // Set the back motors to follow the commands of the front
     mBackLeft.follow(mFrontLeft);
     mBackRight.follow(mFrontRight);
    
     // Set the neutral modes
     mFrontLeft.setNeutralMode(NeutralMode.Brake);
     mFrontRight.setNeutralMode(NeutralMode.Brake);
     mBackLeft.setNeutralMode(NeutralMode.Brake);
     mBackRight.setNeutralMode(NeutralMode.Brake);
 
     // Makes Green go Forward. Sim is weird so thats what the if statement is for
     if (RobotBase.isReal()) {
       mFrontLeft.setInverted(TalonFXInvertType.CounterClockwise);
       mBackLeft.setInverted(TalonFXInvertType.FollowMaster);
       mFrontRight.setInverted(TalonFXInvertType.Clockwise);
       mBackRight.setInverted(TalonFXInvertType.FollowMaster);
     } else {
       mFrontLeft.setInverted(TalonFXInvertType.CounterClockwise);
       mBackLeft.setInverted(TalonFXInvertType.FollowMaster);
       mFrontRight.setInverted(TalonFXInvertType.CounterClockwise);
       mBackRight.setInverted(TalonFXInvertType.FollowMaster);
     }
 
     mFrontLeft.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
     mFrontRight.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);

     mFrontLeft.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 40, 40, 0));
     mFrontRight.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 40, 40, 0));
     mBackLeft.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 40, 40, 0));
     mBackRight.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 40, 40, 0));

     mFrontLeft.configVelocityMeasurementPeriod(SensorVelocityMeasPeriod.Period_1Ms);
     mFrontLeft.configVelocityMeasurementWindow(1);
 
     mFrontLeft.setStatusFramePeriod(StatusFrameEnhanced.Status_10_Targets, 227);
     mFrontLeft.setStatusFramePeriod(StatusFrameEnhanced.Status_11_UartGadgeteer, 229);
     mFrontLeft.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 233);
     mFrontLeft.setStatusFramePeriod(StatusFrameEnhanced.Status_14_Turn_PIDF1, 239);
     mFrontLeft.setStatusFramePeriod(StatusFrameEnhanced.Status_9_MotProfBuffer, 241);
 
     mFrontRight.configVelocityMeasurementPeriod(SensorVelocityMeasPeriod.Period_1Ms);
     mFrontRight.configVelocityMeasurementWindow(1);
 
     mFrontRight.setStatusFramePeriod(StatusFrameEnhanced.Status_10_Targets, 227);
     mFrontRight.setStatusFramePeriod(StatusFrameEnhanced.Status_11_UartGadgeteer, 229);
     mFrontRight.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 233);
     mFrontRight.setStatusFramePeriod(StatusFrameEnhanced.Status_14_Turn_PIDF1, 239);
     mFrontRight.setStatusFramePeriod(StatusFrameEnhanced.Status_9_MotProfBuffer, 241);
 
     mBackLeft.configVelocityMeasurementPeriod(SensorVelocityMeasPeriod.Period_100Ms);
     mBackLeft.configVelocityMeasurementWindow(32);
 
     mBackLeft.setStatusFramePeriod(StatusFrameEnhanced.Status_10_Targets, 227);
     mBackLeft.setStatusFramePeriod(StatusFrameEnhanced.Status_11_UartGadgeteer, 229);
     mBackLeft.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 233);
     mBackLeft.setStatusFramePeriod(StatusFrameEnhanced.Status_14_Turn_PIDF1, 239);
     mBackLeft.setStatusFramePeriod(StatusFrameEnhanced.Status_9_MotProfBuffer, 241);
 
     mBackRight.configVelocityMeasurementPeriod(SensorVelocityMeasPeriod.Period_100Ms);
     mBackRight.configVelocityMeasurementWindow(32);
 
     mBackRight.setStatusFramePeriod(StatusFrameEnhanced.Status_10_Targets, 227);
     mBackRight.setStatusFramePeriod(StatusFrameEnhanced.Status_11_UartGadgeteer, 229);
     mBackRight.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 233);
     mBackRight.setStatusFramePeriod(StatusFrameEnhanced.Status_14_Turn_PIDF1, 239);
     mBackRight.setStatusFramePeriod(StatusFrameEnhanced.Status_9_MotProfBuffer, 241);

     mCurrentState = FrontState.FORWARD;

  }

  public DifferentialDriveWheelSpeeds getWheelVelocities(){
    return new DifferentialDriveWheelSpeeds(
      mFrontLeft.getSelectedSensorVelocity() * 10 * Constants.DriveConstants.kDistancePerPulse,
      mFrontRight.getSelectedSensorVelocity() * 10 * Constants.DriveConstants.kDistancePerPulse
    );
  }

  public double[] getWheelDistances(){
    double[] distances = {
      mFrontLeft.getSelectedSensorPosition() * Constants.DriveConstants.kDistancePerPulse,
      mFrontRight.getSelectedSensorPosition() * Constants.DriveConstants.kDistancePerPulse
    };
    return distances;
  }

  public Pose2d getRobotPosition(){
    return mOdometry.getPoseMeters();
  }

  public double getHeading(){
    return -mPigeon.getYaw();
  }

  public void stop(){
    mFrontLeft.set(0);
    mFrontRight.set(0);
  }

  public void setSpeeds(DifferentialDriveWheelSpeeds speeds) {

    var feedForward = mFeedForward.calculate(
      new MatBuilder<>(
        Nat.N2(), 
        Nat.N1()
      )
      .fill(
        speeds.leftMetersPerSecond, 
        speeds.rightMetersPerSecond
      )
    );

    final double leftOutput = mPID.calculate(getWheelVelocities().leftMetersPerSecond, speeds.leftMetersPerSecond);
    final double rightOutput = mPID.calculate(getWheelVelocities().rightMetersPerSecond, speeds.rightMetersPerSecond);
    
    mFrontLeft.setVoltage(leftOutput + feedForward.get(0,0));
    mFrontRight.setVoltage(rightOutput + feedForward.get(1,0));

  }

  public void drive(double xSpeed, double zRot, boolean turn) {
    DifferentialDrive.curvatureDriveIK(xSpeed * mCurrentState.direction, zRot * mCurrentState.direction, turn);
  }

  public Command changeState(FrontState frontState){
    return new InstantCommand(() -> mCurrentState = frontState);
  }

  public void resetEncoders(){
    mFrontLeft.setSelectedSensorPosition(0);
    mFrontRight.setSelectedSensorPosition(0);
  }

  public void resetOdometry(){
    resetOdometry(new Pose2d());
  }
  public void resetOdometry(Pose2d newPose){
    resetEncoders();
    mOdometry.update(mPigeon.getRotation2d(), getWheelDistances()[0], getWheelDistances()[1]);
  }

  public void resetSimulation(){
    mDrivetrainSim = new DifferentialDrivetrainSim( // Simulation
      mDrivetrainPlant,
      DCMotor.getFalcon500(2),
      KitbotGearing.k10p71.value,
      DriveConstants.kTrackwidth,
      KitbotWheelSize.kSixInch.value,
      null
    );
  }

  @Override
  public void simulationPeriodic() {

    mDrivetrainSim.setInputs(mFrontLeft.get() * RobotController.getInputVoltage(),
        mFrontRight.get() * RobotController.getInputVoltage());

    mDrivetrainSim.update(0.02);

    mLeftSim.setIntegratedSensorRawPosition(
        (int) (mDrivetrainSim.getLeftPositionMeters() / DriveConstants.kDistancePerPulse));
    mRightSim.setIntegratedSensorRawPosition(
        (int) (mDrivetrainSim.getRightPositionMeters() / DriveConstants.kDistancePerPulse));
    mLeftSim.setIntegratedSensorVelocity(
        (int) (mDrivetrainSim.getLeftVelocityMetersPerSecond() / (10 * DriveConstants.kDistancePerPulse)));
    mRightSim.setIntegratedSensorVelocity(
        (int) (mDrivetrainSim.getRightVelocityMetersPerSecond() / (10 * DriveConstants.kDistancePerPulse)));

    mPigeonSim.setRawHeading(mDrivetrainSim.getHeading().getDegrees());

  }

  @Override
  public void periodic() {
    resetOdometry();
  }

}