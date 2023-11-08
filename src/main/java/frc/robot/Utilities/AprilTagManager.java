package frc.robot.Utilities;

import org.photonvision.PhotonCamera;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonPoseEstimator;

import edu.wpi.first.math.geometry.Pose2d;




public class AprilTagManager {
    PhotonCamera camera1;
    PhotonCamera camera2;
    PhotonCamera camera3;
    PhotonCamera camera4;
    PhotonPoseEstimator poseEstimator1;
    PhotonPoseEstimator poseEstimator2;
    PhotonPoseEstimator poseEstimator3;
    PhotonPoseEstimator poseEstimator4;
    EstimatedRobotPose robotPose;

    public void init(){
        camera1 = new PhotonCamera(null);
        poseEstimator1 = new PhotonPoseEstimator(null, null, camera1, null);
        poseEstimator2 = new PhotonPoseEstimator(null, null, camera1, null);
        poseEstimator3 = new PhotonPoseEstimator(null, null, camera1, null);
        poseEstimator4 = new PhotonPoseEstimator(null, null, camera1, null);
        
    }



}
