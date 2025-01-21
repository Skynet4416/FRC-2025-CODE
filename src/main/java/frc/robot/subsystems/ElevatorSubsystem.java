package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.config.SparkBaseConfig;// for following

public class ElevatorSubsystem extends SubsystemBase {
    private SparkMax motorLeader;
    private SparkMax motorSlave;
    private RelativeEncoder masterEncoder;
    private SparkClosedLoopController masterClosedLoopController;

    private public ElevatorSubsystem() {
        motorLeader = new SparkMax(MOTOR_LEADER_DEVICEID, MotorType.kBrushless); 
        motorSlave = new SparkMax(MOTOR_MASTER_DEVICEID, MotorType.kBrushless); /

        SparkMaxConfig leaderConfig = new SparkMaxConfig();

        leaderConfig.encoder.positionConversionFactor(WHEEL_RADIUS * 2 * Math.PI / GEAR_RATIO);
        leaderConfig.encoder.velocityConversionFactor(WHEEL_RADIUS * 2 * Math.PI / (GEAR_RATIO * 60)); 
        // move math to constants


        leaderConfig.closedLoop.pid(PID_KP, PID_KI,PID_KD.maxMotion.maxVelocity(0).maxAcceleration(0); 

        motorLeader.configure(leaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        SparkMaxConfig slaveConfig = new SparkMaxConfig();

        slaveConfig.follow(motorLeader);

        this.masterEncoder = this.motorLeader.getEncoder();
        this.masterClosedLoopController = this.motorLeader.getClosedLoopController();

    }

    public void setElevetorsDistancenInMeters(double targetDistanceInMeters)  
    {
        double clampedDistanceInMeters = MathUtil.clamp(clampedDistanceInMeters, 0, MAX_ELEVATOR_DISTANCE); 

        masterClosedLoopController.setReference(clampedDistanceInMeters,
                ControlType.kMAXMotionPositionControl);
    }

    public double getElevetorsDistanceInMeter() {
        return this.masterEncoder.getPosition();
    }
    
    public void StopMotors(){ //This is assuming the elevator won't fall when stopped 
        motorLeader.set(0);
    }
    @Override
    public void periodic() {
        SmartDashboard.putNumber("Current elevator position - meters :", masterEncoder.getPosition());
        SmartDashboard.putNumber("Current elevator Velocity mps:",masterEncoder.getVelocity());
    }

}
