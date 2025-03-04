package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.TalonFXSimCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.BasePigeonSimCollection;
import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod;
import com.ctre.phoenix.sensors.WPI_Pigeon2;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotGearing;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotWheelSize;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.DriveConstants.SpeedState;
import frc.robot.Constants.DriveConstants.DirState;
import frc.robot.Constants.RobotConstants.CAN;

import frc.util.controls.AngleUtil;

public class Drivetrain extends SubsystemBase {

  private final WPI_TalonFX mFrontLeft = new WPI_TalonFX(CAN.kFrontLeft);
  private final WPI_TalonFX mFrontRight = new WPI_TalonFX(CAN.kFrontRight);
  private final WPI_TalonFX mBackLeft = new WPI_TalonFX(CAN.kBackLeft);
  private final WPI_TalonFX mBackRight = new WPI_TalonFX(CAN.kBackRight);

  private final WPI_Pigeon2 mPigeon = new WPI_Pigeon2(CAN.kPigeon);

  private final TalonFXSimCollection mLeftSim = mFrontLeft.getSimCollection();
  private final TalonFXSimCollection mRightSim = mFrontRight.getSimCollection();

  private final BasePigeonSimCollection mPigeonSim = mPigeon.getSimCollection();

  private final DifferentialDriveKinematics mKinematics = new DifferentialDriveKinematics(DriveConstants.kTrackwidth);
  private final DifferentialDriveOdometry mOdometry;

  private final SimpleMotorFeedforward mFeedForward = new SimpleMotorFeedforward(0.13305, 2.2876, 0.31596);
  private final PIDController mPIDController = new PIDController(0.5, 0, 0);

  private Field2d mField = new Field2d();

  private FieldObject2d mIntakeVisual = mField.getObject("Intake");

  private DirState mCurrentDirState = DirState.FORWARD;
  private SpeedState mCurrentSpeedState = SpeedState.NORMAL;

  public static final LinearSystem<N2, N2, N2> mDrivetrainPlant = LinearSystemId.identifyDrivetrainSystem(
      2.2195, // Linear kV
      0.32787, // Linear kA
      2.53, // Angular kV
      0.081887 // Angular kA
  );

  public DifferentialDrivetrainSim mDrivetrainSim = new DifferentialDrivetrainSim( // Simulation
    mDrivetrainPlant,
    DCMotor.getFalcon500(2),
    KitbotGearing.k10p71.value,
    DriveConstants.kTrackwidth,
    KitbotWheelSize.kSixInch.value,
    null
  );

  public Drivetrain() {

    //TODO Tune Drive and Turn kS

    mPigeon.reset();
    mOdometry = new DifferentialDriveOdometry(mPigeon.getRotation2d(), 0, 0);
    SmartDashboard.putData("Field", mField);
  }

  public void configureMotors() {

    mFrontLeft.configFactoryDefault();
    mFrontRight.configFactoryDefault();
    mBackLeft.configFactoryDefault();
    mBackRight.configFactoryDefault();

    mFrontRight.setInverted(true);
    mBackRight.setInverted(true);

    mFrontLeft.setNeutralMode(NeutralMode.Brake);
    mFrontRight.setNeutralMode(NeutralMode.Brake);
    mBackLeft.setNeutralMode(NeutralMode.Brake);
    mBackRight.setNeutralMode(NeutralMode.Brake);

    mFrontLeft.setInverted(TalonFXInvertType.CounterClockwise);
    mBackLeft.setInverted(TalonFXInvertType.FollowMaster);
    mFrontRight.setInverted(TalonFXInvertType.CounterClockwise);
    mBackRight.setInverted(TalonFXInvertType.FollowMaster);

    mFrontLeft.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    mFrontRight.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);

    mFrontLeft.configSupplyCurrentLimit(DriveConstants.kDriveCurrentLimit);
    mFrontRight.configSupplyCurrentLimit(DriveConstants.kDriveCurrentLimit);
    mBackLeft.configSupplyCurrentLimit(DriveConstants.kDriveCurrentLimit);
    mBackRight.configSupplyCurrentLimit(DriveConstants.kDriveCurrentLimit);

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

  }

  public DifferentialDriveWheelSpeeds getWheelVelocities() {
    return new DifferentialDriveWheelSpeeds(
      mFrontLeft.getSelectedSensorVelocity() * 10 * DriveConstants.kDistancePerPulse,
      mFrontRight.getSelectedSensorVelocity() * 10 * DriveConstants.kDistancePerPulse
    );
  }

  public double[] getWheelDistances() {
    double[] distances = {
      mFrontLeft.getSelectedSensorPosition() * DriveConstants.kDistancePerPulse,
      mFrontRight.getSelectedSensorPosition() * DriveConstants.kDistancePerPulse
    };
    return distances;
  }

  public Pose2d getRobotPosition() {
    return mOdometry.getPoseMeters();
  }
  
  public void setSpeeds(DifferentialDriveWheelSpeeds speeds) {

    final double leftFeedforward = mFeedForward.calculate(speeds.leftMetersPerSecond);
    final double rightFeedforward = mFeedForward.calculate(speeds.rightMetersPerSecond);

    final double leftOutput = mPIDController.calculate(getWheelVelocities().leftMetersPerSecond, speeds.leftMetersPerSecond);
    final double rightOutput = mPIDController.calculate(getWheelVelocities().rightMetersPerSecond, speeds.rightMetersPerSecond);

    setVoltages(leftOutput + leftFeedforward, rightOutput + rightFeedforward);
  }

  public void setVoltages(double leftVolts, double rightVolts) {
    mFrontLeft.setVoltage(leftVolts);
    mFrontRight.setVoltage(rightVolts);
    mBackLeft.setVoltage(leftVolts);
    mBackRight.setVoltage(rightVolts);
  }

  public void drive(double xSpeed, double rot) {

    var wheelSpeeds = new ChassisSpeeds(
      xSpeed * mCurrentSpeedState.xMod * mCurrentDirState.direction,
      0,
      rot * mCurrentSpeedState.rotMod * mCurrentDirState.direction
    );

    setSpeeds(mKinematics.toWheelSpeeds(wheelSpeeds));

  }

  public void driveWGridMode(double xSpeed, double trim){

    if(xSpeed == 0){
      trim = 0;
    }

    if(xSpeed < 0){
      trim *= -1;
    }

    double gridAngle = (DriverStation.getAlliance() == Alliance.Blue ? -90 : 90) + trim;

    PIDController mSnapPID = DriveConstants.kSnapPID;
    
    double effort = mSnapPID.calculate(AngleUtil.normalizeAngle(getAngle().getDegrees()), gridAngle);

    var wheelSpeeds = new ChassisSpeeds(
      xSpeed * mCurrentSpeedState.xMod * mCurrentDirState.direction,
      0,
       effort * DriveConstants.kMaxTurnSpeed
    );

    setSpeeds(mKinematics.toWheelSpeeds(wheelSpeeds));

  }

  public Rotation2d getAngle() {
    return mPigeon.getRotation2d();
  }
  
  public double getPitch() {
    return mPigeon.getRoll(); // Pigeon is mounted wrong
  }
  
  public class RotateRelative extends CommandBase {

    private DoubleSupplier mInput;
    private double mSetpoint;

    private double kS = (RobotBase.isReal()) ? DriveConstants.kTurnKS : 0;

    private PIDController mPID = DriveConstants.kTurnPID;

    public RotateRelative(double angle) {
      mInput = () -> angle;
    }

    public RotateRelative(DoubleSupplier angleSupplier) {
      mInput = angleSupplier;
    }

    @Override
    public void initialize() {

      double inputDeg = mInput.getAsDouble();
      double normalizedCurrent = AngleUtil.normalizeAngle(getAngle().getDegrees());

      double error = inputDeg - normalizedCurrent;

      mSetpoint = getAngle().getDegrees() + error;

    }

    @Override
    public void execute() {

      double output = mPID.calculate(getAngle().getDegrees(), mSetpoint);
      output += (output > 0) ? kS : -kS;

      output = MathUtil.clamp(output, -3, 3); //Limit to 3v

      setVoltages(-output, output);

      SmartDashboard.putNumber("RotateRelative/Setpoint", mPID.getSetpoint());
      SmartDashboard.putBoolean("RotateRelative/At Setpoint", mPID.atSetpoint());
      SmartDashboard.putNumber("RotateRelative/Current Angle", getAngle().getDegrees());
      SmartDashboard.putNumber("RotateRelative/Position Error", mPID.getPositionError());
      SmartDashboard.putNumber("RotateRelative/Velocity Error", mPID.getVelocityError());
      SmartDashboard.putNumber("RotateRelative/Output", output);

    }

    @Override
    public boolean isFinished() {

      return mPID.atSetpoint();

    }
  }

  public class RotateAbsolute extends CommandBase {

    private DoubleSupplier mInput;
    private double mSetpoint;

    private double kS = (RobotBase.isReal()) ? 0.05 : 0;

    private PIDController mPID = DriveConstants.kTurnPID;

    public RotateAbsolute(Rotation2d angle) {
      mInput = () -> angle.getDegrees();
    }

    public RotateAbsolute(double angle) {
      mInput = () -> angle;
    }

    public RotateAbsolute(DoubleSupplier angleSupplier) {
      mInput = angleSupplier;
    }

    @Override
    public void initialize() {

      double normalizedCurrent = AngleUtil.normalizeAngle(getAngle().getDegrees());
      double inputDeg = mInput.getAsDouble();

      double error = inputDeg - normalizedCurrent;

      mSetpoint = getAngle().getDegrees() + error;

    }

    @Override
    public void execute() {

      double output = mPID.calculate(getAngle().getDegrees(), mSetpoint);
      output += (output > 0) ? kS : -kS;

      output = MathUtil.clamp(output, -3, 3); //Limit to 3v

      setVoltages(-output, output);

      SmartDashboard.putNumber("RotateAbsolute/Setpoint", mPID.getSetpoint());
      SmartDashboard.putBoolean("RotateAbsolute/At Setpoint", mPID.atSetpoint());
      SmartDashboard.putNumber("RotateAbsolute/Current Angle", getAngle().getDegrees());
      SmartDashboard.putNumber("RotateAbsolute/Position Error", mPID.getPositionError());
      SmartDashboard.putNumber("RotateAbsolute/Velocity Error", mPID.getVelocityError());
      SmartDashboard.putNumber("RotateAbsolute/Output", output);

    }

    @Override
    public boolean isFinished() {

      return mPID.atSetpoint();

    }
  }

  public class DriveMeters extends CommandBase {

    private DoubleSupplier mDistanceSupplier;

    private double kS = (RobotBase.isReal()) ? DriveConstants.kDriveKS : 0;

    private PIDController mPID = DriveConstants.kAutoDrivePID;

    private double speedCap = DriveConstants.kMaxSpeed;

    public DriveMeters(double distance) {
      mDistanceSupplier = () -> distance;
    }

    public DriveMeters(DoubleSupplier distanceSupplier) {
      mDistanceSupplier = distanceSupplier;
    }

    public DriveMeters(double distance, double maxSpeed) {
      mDistanceSupplier = () -> distance;
      speedCap = maxSpeed;
    }

    public DriveMeters(DoubleSupplier distanceSupplier, double maxSpeed) {
      mDistanceSupplier = distanceSupplier;
      speedCap = maxSpeed;
    }

    @Override
    public void initialize() {
      mPID.setSetpoint(getWheelDistances()[0] + mDistanceSupplier.getAsDouble());
    }

    @Override
    public void execute() {

      double output = mPID.calculate(getWheelDistances()[0], mPID.getSetpoint());

      output += (output > 0) ? kS : -kS;

      output = MathUtil.clamp(output, -speedCap, speedCap);

      setSpeeds(new DifferentialDriveWheelSpeeds(output, output));

      SmartDashboard.putNumber("DriveMeters/Input", mDistanceSupplier.getAsDouble());
      SmartDashboard.putNumber("DriveMeters/Setpoint", mPID.getSetpoint());
      SmartDashboard.putBoolean("DriveMeters/At Setpoint", mPID.atSetpoint());
      SmartDashboard.putNumber("DriveMeters/Current Distance", getWheelDistances()[0]);
      SmartDashboard.putNumber("DriveMeters/Position Error", mPID.getPositionError());
      SmartDashboard.putNumber("DriveMeters/Velocity Error", mPID.getVelocityError());
      SmartDashboard.putNumber("DriveMeters/Output", output);

    }

    @Override
    public boolean isFinished() {
      return mPID.atSetpoint();
    }
  }

  public class Charge extends CommandBase {

    private PIDController mPID = DriveConstants.kChargePID;

    @Override
    public void execute() {
        
      double output = mPID.calculate(getPitch(), 0);

      setVoltages(output, output);

      SmartDashboard.putNumber("Charge/Output", output);
      SmartDashboard.putNumber("Charge/Current Pitch", getPitch());
      SmartDashboard.putNumber("Charge/Error", mPID.getPositionError());

    }

    @Override
    public boolean isFinished() {
      return mPID.atSetpoint();
    }

  }

  public Command changeState(DirState frontState) {
    return new InstantCommand(
        () -> mCurrentDirState = frontState);
  }

  public Command changeState(SpeedState modState) {
    return new InstantCommand(
        () -> mCurrentSpeedState = modState);
  }

  public void updateOdometry() {
    mOdometry.update(mPigeon.getRotation2d(), getWheelDistances()[0], getWheelDistances()[1]);
    mField.setRobotPose(mOdometry.getPoseMeters());

    // Intake Visualizer
    Pose2d robotPose = mOdometry.getPoseMeters();

    Pose2d intakePose = new Pose2d(
      robotPose.getX() - 0.4 * Math.sin(Units.degreesToRadians(-robotPose.getRotation().getDegrees())),
      robotPose.getY() - 0.4 * Math.cos(Units.degreesToRadians(-robotPose.getRotation().getDegrees())),
      robotPose.getRotation()
    );
    mIntakeVisual.setPose(intakePose);

  }

  public void resetEncoders() {
    mFrontRight.setSelectedSensorPosition(0, 0, 0);
    mFrontLeft.setSelectedSensorPosition(0, 0, 0);
    mBackRight.setSelectedSensorPosition(0, 0, 0);
    mBackLeft.setSelectedSensorPosition(0, 0, 0);
  }

  public void resetPoseAndGyro(Pose2d pose) {
    mPigeon.setYaw(pose.getRotation().getDegrees());
    mOdometry.resetPosition(pose.getRotation(), getWheelDistances()[0], getWheelDistances()[1], pose);
  }

  public void stop() {
    setVoltages(0, 0);
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

    SmartDashboard.putNumber("Drivetrain/Left Distance Meters", getWheelDistances()[0]);
    SmartDashboard.putNumber("Drivetrain/Right Distance Meters", getWheelDistances()[1]);
    SmartDashboard.putNumber("Drivetrain/Robot Angle", getAngle().getDegrees());
    SmartDashboard.putNumber("Drivetrain/Robot Pitch", getPitch());
    SmartDashboard.putNumber("Drivetrain/NormalizedAngle", AngleUtil.normalizeAngle(getAngle().getDegrees()));

    SmartDashboard.putString("States/DriveDirection", mCurrentDirState.toString());
    SmartDashboard.putString("States/DriveSpeed", mCurrentSpeedState.toString());

    updateOdometry();
  }

}
