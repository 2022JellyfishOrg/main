package org.firstinspires.ftc.teamcode.vision;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class Detector extends OpenCvPipeline {
    Telemetry telemetry;
    Mat mat = new Mat();
    Mat bmat = new Mat();
    Mat gmat = new Mat();
    Mat rmat = new Mat();
    Boolean debug = true;
    public enum Location {
        LEFT,
        RIGHT,
        NOT_FOUND
    }
    private Location location;
    public int num = 0;

    public int getPosition() {
        return num;
    }

    static final Rect LEFT_ROI = new Rect(
            new Point(140, 110),
            new Point(180, 50));
    static double PERCENT_COLOR_THRESHOLD = 0.4;

    public Detector(Telemetry t) { telemetry = t; }

    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);
//      green
        Scalar highHSV = new Scalar(80, 255, 255);
        Scalar lowHSV = new Scalar(40, 60, 70);
//      blue hsvs
        Scalar bhighHSV = new Scalar(140,255,255);
        Scalar blowHSV = new Scalar(100,130,0);
        //      red
        Scalar rhighHSV = new Scalar(200,200,255);
        Scalar rlowHSV = new Scalar(150,0,0);
        try{
            Core.inRange(mat, lowHSV, highHSV, gmat);
            Core.inRange(mat, blowHSV, bhighHSV, bmat);
            Core.inRange(mat, rlowHSV, rhighHSV, rmat);



            Mat bleft = bmat.submat(LEFT_ROI);
            Mat gleft = gmat.submat(LEFT_ROI);
            Mat rleft = rmat.submat(LEFT_ROI);

            double gleftValue = Core.sumElems(gleft).val[0] / LEFT_ROI.area() / 255;
            double bleftValue = Core.sumElems(bleft).val[0] / LEFT_ROI.area() / 255;
            double rleftValue = Core.sumElems(rleft).val[0] / LEFT_ROI.area() / 255;


            bleft.release();
            gleft.release();
            rleft.release();

            telemetry.addData("Blue Left percentage", Math.round(bleftValue * 100) + "%");
            telemetry.addData("Green Left percentage", Math.round(gleftValue * 100) + "%");
            telemetry.addData("Red Left percentage", Math.round(rleftValue * 100) + "%");
            telemetry.update();
            int blu = (int) Math.round(bleftValue * 100);
            int green = (int) Math.round(gleftValue * 100);
            int red = (int) Math.round(rleftValue * 100);
            if(blu>green){
                if(blu>red){
                    //return blue
                    num = 2;

                }
                else{
                    //return red
                    num = 1;
                }
            }else{
                if (green > red) {

                    num = 3;
                }else{
                    num = 1;
                }

            }
            Scalar colorStone = new Scalar(255, 0, 0);
            Scalar colorSkystone = new Scalar(0, 255, 0);

            Imgproc.rectangle(bmat, LEFT_ROI, location == Location.LEFT? colorSkystone:colorStone);
            return bmat;

        } catch (Exception e) {
            e.printStackTrace();
            return rmat;
        }


    }


}