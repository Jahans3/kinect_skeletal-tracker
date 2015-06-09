
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Data;
using System.Windows.Documents;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Navigation;
using System.Windows.Shapes;
using Microsoft.Kinect;
using Coding4Fun.Kinect.Wpf;

namespace Costume
{

    public partial class MainWindow : Window
    {

        public MainWindow()
        {
            InitializeComponent();
        }

        bool closing = false; //If our program is closing or not
        const int numberOfSkels = 6; //number of maximum skeletons (cannot be static)
        Skeleton[] skeletons = new Skeleton[numberOfSkels]; //array of skeletons
        
		
        private void Window_Loaded(object sender, RoutedEventArgs e)
        {
            chooser.KinectSensorChanged += new DependencyPropertyChangedEventHandler(chooser_KinectSensorChanged);
        }

        void chooser_KinectSensorChanged(object sender, DependencyPropertyChangedEventArgs e)
        {
            //If we have a sensor and it gets disconnected, discard it
            KinectSensor oldSensor = (KinectSensor)e.OldValue;
            StopKinect(oldSensor);

            //When new sensor is connected we load it back up
            KinectSensor sensor = (KinectSensor)e.NewValue;

            //If null break
            if (sensor == null)
            {
                return;
            }

            //Enable all the data streams we want
            sensor.SkeletonStream.Enable();

            //All frames ready event
            sensor.AllFramesReady += new EventHandler<AllFramesReadyEventArgs>(sensor_AllFramesReady);
            //Depth and colour streams
            sensor.DepthStream.Enable(DepthImageFormat.Resolution640x480Fps30);
            sensor.ColorStream.Enable(ColorImageFormat.RgbResolution640x480Fps30);

            //Start our sensor
            try
            {
                sensor.Start();
            }
            catch (System.IO.IOException)
            {
                //In case we have an application conflict
                chooser.AppConflictOccurred();
                
            }
        }

        //Event handler for all frames ready
        void sensor_AllFramesReady(object sender, AllFramesReadyEventArgs e)
        {
            //If closing is true break (return empty)
            if (closing)
            {
                return;
            }

            //Get our skeleton
            Skeleton skelly = GetSkelly(e);

            //If skelly is null break
            if (skelly == null)
            {
                return;
            }

            GetPoints(skelly, e);
        }

        //Function where we take skeleton data, pass it to depth image and then pass that onto a color image
        void GetPoints(Skeleton first, AllFramesReadyEventArgs e)
        {
            using (DepthImageFrame depth = e.OpenDepthImageFrame())
            {
                //If null then break, but this time checking for depth AND(or) skeleton data being null
                if (depth == null || chooser.Kinect == null)
                {
                    return;
                }

                //Map our joints to a location on the depth map
                DepthImagePoint headPoint = depth.MapFromSkeletonPoint(first.Joints[JointType.Head].Position); //Head
                DepthImagePoint leftHandPoint = depth.MapFromSkeletonPoint(first.Joints[JointType.HandLeft].Position); //Left hand
                DepthImagePoint rightHandPoint = depth.MapFromSkeletonPoint(first.Joints[JointType.HandRight].Position); //Right hand
                DepthImagePoint torsoPoint = depth.MapFromSkeletonPoint(first.Joints[JointType.Spine].Position); //torso/spine
                DepthImagePoint forearmLeftPoint = depth.MapFromSkeletonPoint(first.Joints[JointType.ElbowLeft].Position); //Forearm left
                DepthImagePoint forearmRightPoint = depth.MapFromSkeletonPoint(first.Joints[JointType.ElbowRight].Position); //Forearm right
                DepthImagePoint armLeftPoint = depth.MapFromSkeletonPoint(first.Joints[JointType.ShoulderLeft].Position); //Arm left
                DepthImagePoint armRightPoint = depth.MapFromSkeletonPoint(first.Joints[JointType.ShoulderRight].Position); //Arm right
                DepthImagePoint shinLeftPoint = depth.MapFromSkeletonPoint(first.Joints[JointType.KneeLeft].Position); //Shin left
                DepthImagePoint shinRightPoint = depth.MapFromSkeletonPoint(first.Joints[JointType.KneeRight].Position); //Shin Right
                DepthImagePoint thighLeftPoint = depth.MapFromSkeletonPoint(first.Joints[JointType.HipLeft].Position); //Thigh left
                DepthImagePoint thighRightPoint = depth.MapFromSkeletonPoint(first.Joints[JointType.HipRight].Position); //Thigh right

                //Map the above depth points to points on color image
                ColorImagePoint headCPoint = depth.MapToColorImagePoint(headPoint.X, headPoint.Y, ColorImageFormat.RgbResolution640x480Fps30); //Head
                ColorImagePoint leftHandCPoint = depth.MapToColorImagePoint(leftHandPoint.X, leftHandPoint.Y, ColorImageFormat.RgbResolution640x480Fps30); //Left hand
                ColorImagePoint rightHandCPoint = depth.MapToColorImagePoint(rightHandPoint.X, rightHandPoint.Y, ColorImageFormat.RgbResolution640x480Fps30); //Right hand
                ColorImagePoint torsoCPoint = depth.MapToColorImagePoint(torsoPoint.X, torsoPoint.Y, ColorImageFormat.RgbResolution640x480Fps30); //Torso
                ColorImagePoint forearmLCPoint = depth.MapToColorImagePoint(forearmLeftPoint.X, forearmLeftPoint.Y, ColorImageFormat.RgbResolution640x480Fps30); //Elbow left
                ColorImagePoint forearmRCPoint = depth.MapToColorImagePoint(forearmRightPoint.X, forearmRightPoint.Y, ColorImageFormat.RgbResolution640x480Fps30); //Elbow right
                ColorImagePoint armLCPoint = depth.MapToColorImagePoint(armLeftPoint.X, armLeftPoint.Y, ColorImageFormat.RgbResolution640x480Fps30); //Arm left
                ColorImagePoint armRCPoint = depth.MapToColorImagePoint(armRightPoint.X, armRightPoint.Y, ColorImageFormat.RgbResolution640x480Fps30); //Arm right
                ColorImagePoint shinLCPoint = depth.MapToColorImagePoint(shinLeftPoint.X, shinLeftPoint.Y, ColorImageFormat.RgbResolution640x480Fps30); //Shin left
                ColorImagePoint shinRCPoint = depth.MapToColorImagePoint(shinRightPoint.X, shinRightPoint.Y, ColorImageFormat.RgbResolution640x480Fps30); //Shin right
                ColorImagePoint thighLCPoint = depth.MapToColorImagePoint(thighLeftPoint.X, thighLeftPoint.Y, ColorImageFormat.RgbResolution640x480Fps30); //Thigh left
                ColorImagePoint thighRCPoint = depth.MapToColorImagePoint(thighRightPoint.X, thighRightPoint.Y, ColorImageFormat.RgbResolution640x480Fps30); //Thigh right

                //Get joints as vectors
                //Base vector (neck)
                Vector centreNeck = new Vector(first.Joints[JointType.ShoulderCenter].Position.X, first.Joints[JointType.ShoulderCenter].Position.Y);

                //Left arm vectors
                Vector leftElbow = new Vector(first.Joints[JointType.ElbowLeft].Position.X, first.Joints[JointType.ElbowLeft].Position.Y);
                Vector leftShoulder = new Vector(first.Joints[JointType.ShoulderLeft].Position.X, first.Joints[JointType.ShoulderLeft].Position.Y);
                Vector leftWrist = new Vector(first.Joints[JointType.WristLeft].Position.X, first.Joints[JointType.WristLeft].Position.Y);

                //Left elbow vectors
                Vector leftWristToEblow = leftWrist - leftElbow;
                Vector leftShoulderToElbow = leftShoulder - leftElbow;
                
                //Left shoulder vectors
                Vector leftElbowToShoulder = leftElbow - leftShoulder;
                Vector neckToLeftElbow = centreNeck - leftShoulder;

                //Right arm vectores
                Vector rightElbow = new Vector(first.Joints[JointType.ElbowRight].Position.X, first.Joints[JointType.ElbowRight].Position.Y);
                Vector rightShoulder = new Vector(first.Joints[JointType.ShoulderRight].Position.X, first.Joints[JointType.ShoulderRight].Position.Y);
                Vector rightWrist = new Vector(first.Joints[JointType.WristRight].Position.X, first.Joints[JointType.WristRight].Position.Y);

                //Right elbow vectors
                Vector rightWristToElbow = rightWrist - rightElbow;
                Vector rightShoulderToElbow = rightShoulder - rightElbow;

                //Right shoulder vectors
                Vector rightElbowToShoulder = rightElbow - rightShoulder;
                Vector neckToRightElbow = centreNeck - rightShoulder;

                //Based vector (hip)
                Vector centreHip = new Vector(first.Joints[JointType.HipCenter].Position.X, first.Joints[JointType.HipCenter].Position.Y);

                //Left leg vectors
                Vector leftThigh = new Vector(first.Joints[JointType.HipLeft].Position.X, first.Joints[JointType.HipLeft].Position.Y);
                Vector leftKnee = new Vector(first.Joints[JointType.KneeLeft].Position.X, first.Joints[JointType.KneeLeft].Position.Y);
                Vector leftAnkle = new Vector(first.Joints[JointType.AnkleLeft].Position.X, first.Joints[JointType.AnkleLeft].Position.Y);

                //Left knee vectors
                Vector leftAnkleToKnee = leftAnkle - leftKnee;
                Vector leftHipToKnee = leftThigh - leftKnee;

                //Left hip vectors
                Vector leftKneeToThigh = leftKnee - leftThigh;
                Vector hipToLeftThigh = centreHip - leftThigh;

                //Right leg vectors
                Vector rightThigh = new Vector(first.Joints[JointType.HipRight].Position.X, first.Joints[JointType.HipRight].Position.Y);
                Vector rightKnee = new Vector(first.Joints[JointType.KneeRight].Position.X, first.Joints[JointType.KneeRight].Position.Y);
                Vector rightAnkle = new Vector(first.Joints[JointType.AnkleRight].Position.X, first.Joints[JointType.AnkleRight].Position.Y);

                //Right knee vectors
                Vector rightAnkleToKnee = rightAnkle - rightKnee;
                Vector rightHipToKnee = rightThigh - rightKnee;

                //Right hip vectors
                Vector rightKneeToThigh = rightKnee - rightThigh;
                Vector hipToRightThigh = centreHip - leftThigh;

                //Final left arm angles
                double leftShoulderFinal = Vector.AngleBetween(leftElbowToShoulder, neckToLeftElbow);
                double ls = leftShoulderFinal - 135; //Shoulder
                double leftElbowFinal = Vector.AngleBetween(leftWristToEblow, leftShoulderToElbow) + leftShoulderFinal + 45; //Elbow

                //Final right arm angles
                double rightShoulderFinal = Vector.AngleBetween(rightElbowToShoulder, neckToRightElbow);
                double rs = rightShoulderFinal + 135; //Shoulder
                double rightElbowFinal = Vector.AngleBetween(rightWristToElbow, rightShoulderToElbow) + rightShoulderFinal - 45; //Elbow

                //Final left leg angles
                double leftHipFinal = Vector.AngleBetween(leftKneeToThigh, hipToLeftThigh);
                double lh = leftHipFinal - 135;
                double leftKneeFinal = Vector.AngleBetween(leftAnkleToKnee, leftHipToKnee) + leftHipFinal + 45;

                //Final right leg angles
                double rightHipFinal = Vector.AngleBetween(rightKneeToThigh, hipToRightThigh);
                double rh = rightHipFinal - 135;
                double rightKneeFinal = Vector.AngleBetween(rightAnkleToKnee, rightHipToKnee) + rightHipFinal + 45;

                //We set our final locations of joints to corresponding images
                SetPosition(headImg, headCPoint);
                SetPosition(leftHandImg, leftHandCPoint);
                SetPosition(rightHandImg, rightHandCPoint);
                SetPosition(torsoImg, torsoCPoint);
                
                //Left arm final position and angles
                SetPositionRotating(elbowLeftImg, forearmLCPoint);
                SetPositionRotating(armLeftImg, armLCPoint);
                SetBone(elbowLeftImg, leftElbowFinal);
                SetBone(armLeftImg, ls);

                //Right arm final position and angles
                SetPositionRotating(elbowRightImg, forearmRCPoint);
                SetPositionRotating(armRightImg, armRCPoint);
                SetBone(elbowRightImg, rightElbowFinal);
                SetBone(armRightImg, rs);

                //Left leg final position and angles
                SetPositionRotating(leftShinImg, shinLCPoint);
                SetPositionRotating(leftThighImg, thighLCPoint);
                SetBone(leftShinImg, leftKneeFinal);
                SetBone(leftThighImg, lh);

                //Right leg final position and angles
                SetPositionRotating(rightShinImg, shinRCPoint);
                SetPositionRotating(rightThighImg, thighRCPoint);
                SetBone(rightShinImg, rightKneeFinal);
                SetBone(rightThighImg, rh);
            }
        }

        //Function that grabs all of our skeleton data
        Skeleton GetSkelly(AllFramesReadyEventArgs e)
        {
            using (SkeletonFrame skellyData = e.OpenSkeletonFrame())
            {
                //If no data break
                if (skellyData == null)
                {
                    return null;
                }

                //Copy the skeleton data to our skeleton array 
                skellyData.CopySkeletonDataTo(skeletons);

                //Get the first skeleton that is tracked
                Skeleton first = (from s in skeletons where s.TrackingState == SkeletonTrackingState.Tracked select s).FirstOrDefault();

                //return the skeleton
                return first;
            }
        }

        //Stop function
        void StopKinect(KinectSensor sensor)
        {
            if (sensor != null)
            {
                if (sensor.IsRunning)
                {
                    //stop sensor 
                    sensor.Stop();

                    //stop audio if not null
                    if (sensor.AudioSource != null)
                    {
                        sensor.AudioSource.Stop();
                    }


                }
            }
        }

        //This function maps our non-rotating image objects/elements to a passed position and centre them
        private void SetPosition(FrameworkElement element, ColorImagePoint point)
        {
            //Divid width and height by 2 to centre
            Canvas.SetLeft(element, point.X - element.Width / 2); //X axis
            Canvas.SetTop(element, point.Y - element.Height / 2); //Y axis
            
        }

        //This function is used to map out our rotating image objects/elements to a passed position
        //This function maps our image objects/elements to a passed position
        private void SetPositionRotating(FrameworkElement element, ColorImagePoint point)
        {
            Canvas.SetLeft(element, point.X - element.Width / 2); //X axis
            Canvas.SetTop(element, point.Y); //Y axis

        }

		//This function sets the rotation of a "bone" that is between two joints
        private void SetBone(FrameworkElement element, double angle)
        {
            RotateTransform r = new RotateTransform(angle);
            element.RenderTransform = r;
        }
        
        

        //This function gets the angle between two vectors
        public int AngleBetweenTwoVectors(Vector vectorA, Vector vectorB)
        {
            double dotProduct = 0.0f;
            //dotProduct = Vector.Dot(vectorA, vectorB);
            dotProduct = Vector.AngleBetween(vectorA, vectorB);

            return (int)Math.Acos(dotProduct);
        }

		//This function executes when our window is closed
        private void Window_Closing(object sender, System.ComponentModel.CancelEventArgs e)
        {
            //Execute stop kinect function when window is closing
            closing = true;
            StopKinect(chooser.Kinect);
        }
    }
}
