#pragma once

#include <QObject>
#include <QDialog>
#include "ui_LaserScanner.h"
#include <opencv2/core.hpp>
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/calib3d.hpp"

#include <QApplication>
#include <QDesktopWidget>

#include <QtWidgets/QMainWindow>
#include <qmessagebox.h>
#include <qtimer.h>
#include "qdatetime.h"

#include "PointcloudPLY.h"
#include "BaslerCamera.h"
#include <pylon/PylonIncludes.h>

using namespace Pylon;

using namespace cv;

using namespace std;


#define MINLASER 15//20

enum CALIBRATION_TYPE { CAMERA = 1, LASER = 2 };
enum ACTION { ADDIMAGE = 1, LOADIMAGE = 2 };

class LaserScanner : public QMainWindow
{
    Q_OBJECT

public:
    LaserScanner(QWidget* parent = Q_NULLPTR);
    ~LaserScanner();
    bool findPeakBR4Estimate(Mat& img, vector<Point2f>& peaks, Rect roi);
    void Scan();

private:
    Ui::LaserScannerClass ui;

    QThread* thread;

    static inline bool INVALID(const cv::Vec3f& pt) { return _isnan(pt[0]) || _isnan(pt[1]) || _isnan(pt[2]); }

    BaslerCamera* camera;
    Pointcloud pointCloud;

    Size patternsize = Size(11, 8);

    Mat gray, gray1;
    int action = 0;

    Mat bgr1;
    QImage image;
    QPixmap pix;

    int scanCount = 0;
    bool scanTrigger = 0;

    QString elde;
    int value;

    int calibration_type;

    QString path = "C:/Users/ghvl1/Desktop/adsf";
    QTimer* camera_t;
    vector<Point2f>  corners;
    vector<Point3f>  object;

    vector<Mat> CCalibrationImages;
    vector<Mat> LCalibrationImages;

    vector<QString> CImageName;
    vector<QString> LImageName;

    double barB;
    Point3f normal, translation;
    Mat cameraMatrix, distCoeffs, R, T;

    vector<vector<Point3f>> scanResult;
   

//    QList<QListWidgetItem*> items;
    int CImageCount = 0;
    int LImageCount = 0;
    //    Mat tempImage;

    //    vector<Mat> imagesOnList;

private slots:
    void show_img(Mat frame);
    void get_path(void);
    void SerialList(void);
    void SerialConnect(void);
    void LoadParameters(void);
    void Revolution(void);
    void startScan(void);
    void Checking(void);
    //void on_pbTest_clicked();
};
