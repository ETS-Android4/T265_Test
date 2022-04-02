package org.firstinspires.ftc.teamcode.Localizers;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.Localizer;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Transform2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.RobotLog;
import com.spartronics4915.lib.T265Camera;

@Config
public class T265LocalizerTest implements Localizer {
    private T265Camera slamra = null;
    public Pose2d currentPose = new Pose2d(0,0,0);
    private T265Camera.CameraUpdate up;
    private T265Camera.PoseConfidence poseConfidence;

    public T265LocalizerTest(HardwareMap hardwareMap) {
        slamra = new T265Camera(new Transform2d(), 0.8, hardwareMap.appContext);
        stopRealsense();

        RobotLog.d("Created Realsense Object");
        setPoseEstimate(new Pose2d(0,0,0));
        int i = 0;
        while (!slamra.isStarted() && i < 1000) {
            try {
                startRealsense();
            } catch (Exception ignored) {
                RobotLog.v("Realsense already started");
                slamra.setPose(new com.arcrobotics.ftclib.geometry.Pose2d(0,0, new Rotation2d(0)));
            }
            if (slamra.getLastReceivedCameraUpdate().confidence == T265Camera.PoseConfidence.Failed) {
                RobotLog.e("Realsense Failed to get Position");
            }
            i++;
        }
    }

    @Override
    public void update() {
        if (slamra != null) {
            up = slamra.getLastReceivedCameraUpdate();
            poseConfidence = up.confidence;
        }
    }

    @NonNull
    @Override
    public Pose2d getPoseEstimate() {
        if (up != null) {
            Translation2d oldPose = up.pose.getTranslation();
            Rotation2d oldRot = up.pose.getRotation();

            //The T265's unit of measurement is meters.  dividing it by .0254 converts meters to inches.
            currentPose = new Pose2d(oldPose.getX() / .0254, oldPose.getY() / .0254, norm(oldRot.getRadians() + 0));
            return currentPose;
        } else {
            RobotLog.v("NULL Camera Update");
            return new Pose2d(0,123,0);
        }
    }

    @Override
    public void setPoseEstimate(Pose2d pose) {

    }

    @Override
    public Pose2d getPoseVelocity() {
        if (up != null) {
            ChassisSpeeds velocity = up.velocity;
            return new Pose2d(velocity.vxMetersPerSecond /.0254,velocity.vyMetersPerSecond /.0254,velocity.omegaRadiansPerSecond);
        } else {
            return new Pose2d(0,12,0);
        }
    }

    @Deprecated
    public void startRealsense() {
        RobotLog.v("Staring realsense");
        slamra.start();
    }

    @Deprecated
    public void stopRealsense() {
        RobotLog.v("Stopping Realsense");
        slamra.stop();
    }

    private double norm(double angle) {
        while (angle>Math.toRadians(360)) angle-=Math.toRadians(360);
        while (angle<=0) angle+=Math.toRadians(360);
        return angle;
    }

    private static double norma(double angle) {
        while (angle>Math.toRadians(360)) angle-=Math.toRadians(360);
        while (angle<=0) angle+=Math.toRadians(360);
        return angle;
    }
}
