package frc.robot.vision;

import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

public class ColorMatchResult {
    ///range [0-1]
    public double width;
    ///range [0-1]
    public double height;
    ///range [-1,1]
    public double centerX;
    //range [-1,1]
    public double centerY;

    public ColorMatchResult() {

    }

    public ColorMatchResult(PhotonTrackedTarget target, double frameWidth, double frameHeight) {
        //find edges of box
        double boxLeft, boxRight, boxTop, boxBottom;
        boxLeft = frameWidth;
        boxRight = 0;
        boxTop = frameHeight;
        boxBottom = 0;
        for(TargetCorner corner : target.minAreaRectCorners) {
          if (corner.x < boxLeft) boxLeft = corner.x;
          if (corner.x > boxRight) boxRight = corner.x;
          if (corner.y < boxTop) boxTop = corner.y;
          if (corner.y > boxBottom) boxBottom = corner.y;
        }

        //prepare match result
        width = (boxRight - boxLeft) / frameWidth;
        height = (boxBottom - boxTop) / frameHeight;
        centerX = ((boxRight + boxLeft) / frameWidth) - 1.;
        centerY = ((boxBottom + boxTop) / frameHeight) - 1.;
    }
}