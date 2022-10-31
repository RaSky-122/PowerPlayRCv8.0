package org.firstinspires.ftc.teamcode.rasky.detection;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class TestPipeline extends OpenCvPipeline {
    Telemetry telemetry;
    public TestPipeline(Telemetry telemetry) {
        this.telemetry = telemetry;
    }
    List<MatOfPoint> redContours = new ArrayList<>();
    private final Mat ycrcb = new Mat();
    private final Mat thresh = new Mat();
    Scalar RED = new Scalar(255, 0, 0);
    Scalar GREEN = new Scalar(0, 255, 0);
    Scalar BLUE = new Scalar(0, 0, 255);

    Mat erode = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(15, 15));
    Mat dilate = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(15, 15));

    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, ycrcb, Imgproc.COLOR_RGB2YCrCb);
        Core.extractChannel(ycrcb, ycrcb, 1);
        Imgproc.threshold(ycrcb, thresh, 150, 255, Imgproc.THRESH_BINARY);

        redContours.clear();

        //Imgproc.erode(thresh, thresh, erode);
        //Imgproc.erode(thresh, thresh, erode);

        //Imgproc.dilate(thresh, thresh, dilate);
        //Imgproc.dilate(thresh, thresh, dilate);

        Imgproc.findContours(thresh, redContours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
        Imgproc.drawContours(input, redContours, -1, GREEN);

        Rect boundingRect = Imgproc.boundingRect(thresh);
        Point basePoint = new Point(boundingRect.x + boundingRect.width/2., boundingRect.y + boundingRect.height);
        Imgproc.drawMarker(input, basePoint, BLUE, Imgproc.MARKER_CROSS, 20);
        telemetry.addData("Point Coords: ", basePoint);
        telemetry.update();
        return input;

    }
}