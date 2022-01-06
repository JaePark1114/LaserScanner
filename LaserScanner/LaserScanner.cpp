#include "LaserScanner.h"

#include <QApplication>
#include <QDesktopWidget>
#include <QPainter>

#include <QtSerialPort/QtSerialPort>
#include <QtSerialPort/QSerialPortInfo>

#include <QThread>
#include <QFileDialog>
#include <QDebug>

#include <stdio.h>
#include <iostream>
#include <assert.h>

#include <string>

#include <bitset>

using namespace cv;
QSerialPort* serial;

LaserScanner::LaserScanner(QWidget* parent)
	: QMainWindow(parent)
{
	ui.setupUi(this);

	//camera_t = new QTimer();
	camera = new BaslerCamera(SOFTWARE, "23472007");
	//vector<int> vect;

	SerialList();

	thread = new QThread();

	connect(ui.pbCheckSerial, SIGNAL(clicked()), this, SLOT(SerialList()));
	connect(ui.pbSerial, SIGNAL(clicked()), this, SLOT(SerialConnect()));
	connect(ui.pbLoadPara, SIGNAL(clicked()), this, SLOT(LoadParameters()));
	connect(ui.pbRevolution, SIGNAL(clicked()), this, SLOT(Revolution()));
	connect(ui.pbScan, SIGNAL(clicked()), this, SLOT(startScan()));
	connect(ui.pbPath, SIGNAL(clicked()), this, SLOT(get_path()));

	camera->moveToThread(thread);
	LoadParameters();
	typedef Mat mat;
	qRegisterMetaType<mat>("Mat");
	connect(camera, &BaslerCamera::NewImageCame, this, &LaserScanner::show_img, Qt::AutoConnection);
	thread->start();
	camera->initCam();
	camera->startGrab();
	camera->softwareTrigger();
	camera_t = new QTimer;
	connect(camera_t, SIGNAL(timeout()), this, SLOT(Checking()));
	camera_t->start(500);
}
LaserScanner::~LaserScanner()
{
	thread->quit();
	camera->destroyCam();
}
void LaserScanner::get_path() {
	path = QFileDialog::getExistingDirectory(this, "Search Folder", QDir::homePath(), QFileDialog::ShowDirsOnly);
	ui.lePath->setText(path);
}
void LaserScanner::SerialList() {
	QList<QSerialPortInfo> list;
	list = QSerialPortInfo::availablePorts();
	ui.comboSerial->clear();

	for (int i = 0; i < list.length(); i++)
	{
		ui.comboSerial->addItem(list[i].portName());
		ui.pbScan->setEnabled(true);
	}
}
void LaserScanner::SerialConnect() {
	serial = new QSerialPort(this);
	serial->setPortName(ui.comboSerial->currentText());
	serial->setBaudRate(QSerialPort::Baud115200); 
	serial->setDataBits(QSerialPort::Data8);
	serial->setParity(QSerialPort::NoParity);
	serial->setStopBits(QSerialPort::OneStop);
	serial->setFlowControl(QSerialPort::NoFlowControl);
	if (serial->open(QIODevice::ReadWrite))
	{
		ui.progressBar->setValue(100);
		ui.pbRevolution->setEnabled(true);
	}
	int value = 1;
	QString srtemp = QString::number(value);
	srtemp.append('\n');
	String temp = srtemp.toStdString();
	serial->write(temp.c_str(), temp.size());
	serial->waitForBytesWritten();
}
void LaserScanner::show_img(Mat frame) {
	
	if (scanTrigger == 1)
	{
		gray = frame.clone();
		cvtColor(frame, bgr1, COLOR_GRAY2BGR);
		image = QImage(bgr1.data, bgr1.cols, bgr1.rows, bgr1.step, QImage::Format_RGB888);
		pix = QPixmap::fromImage(image);
		ui.display->setPixmap(pix.scaled(300, 250, Qt::KeepAspectRatio, Qt::FastTransformation));
		cout << scanCount << endl;
		Scan();
		scanCount++;
	}
	else
	{
		cvtColor(frame, bgr1, COLOR_GRAY2BGR);
		image = QImage(bgr1.data, bgr1.cols, bgr1.rows, bgr1.step, QImage::Format_RGB888);
		pix = QPixmap::fromImage(image);
		ui.display->setPixmap(pix.scaled(300, 250, Qt::KeepAspectRatio, Qt::FastTransformation));
	}
}
void LaserScanner::LoadParameters()
{
	String filename = path.toStdString() + "/camera.xml";
	cv::FileStorage fs(filename, cv::FileStorage::READ);
	std::cout << "Reading camera" << std::endl;
	fs["cameraMatrix"] >> cameraMatrix;
	fs["distCoeffs"] >> distCoeffs;
	cameraMatrix.convertTo(cameraMatrix, CV_64F);
	qDebug() << " cameraMatrix";
	std::cout << cameraMatrix << std::endl;
	distCoeffs.convertTo(distCoeffs, CV_64F);
	qDebug() << " distCoeffs";
	std::cout << distCoeffs << std::endl;

	String filename2 = path.toStdString() + "/laser.xml";
	cv::FileStorage fs2(filename2, cv::FileStorage::READ);
	std::cout << "Reading laser" << std::endl;
	fs2["barB"] >> barB;
	fs2["normal"] >> normal;
	qDebug() << " barB";
	std::cout << barB << std::endl;
	qDebug() << " normal";
	std::cout << normal << std::endl;
	Vec3f a = Vec3f(normal.x, normal.y, normal.z);
	Vec3f b = normalize(a);
	cout << "My error normal: " << b[0] << ", " << b[1] << ", " << b[2] << endl << "            -0.889008104801178, - 0.0883127897977829, -0.44929441809654236" << endl;
	String filename3 = path.toStdString() + "/motor.xml";
	cv::FileStorage fs3(filename3, cv::FileStorage::READ);
	std::cout << "Reading motor" << std::endl;
	fs3["translation"] >> translation;
	qDebug() << " translation";
	std::cout << translation << std::endl;

	ui.pbScan->setEnabled(true);
}
//void LaserScanner::on_pbTest_clicked()
//{
//	camera->softwareTrigger();
//	qDebug() << "Trigger!";
//}
void LaserScanner::Scan()
{
	vector<Point2f> peaks;
	Rect roi;
	findPeakBR4Estimate(gray, peaks, roi);
	
	for (int i = 0; i < peaks.size(); i++)
	{
		gray.at<uchar>(peaks[i]) = 255;
	}
	imwrite(to_string(scanCount)+".jpg", gray);
	
	if (peaks.size() == 0)
	{
		qDebug() << "No laser detected!";
		return;
	}
	else
		cout << "Laser detected: " << peaks.size() << endl;
	undistortPoints(peaks, peaks, cameraMatrix, distCoeffs, noArray(), cameraMatrix);
	
	vector<Point3f> oneLaserLine;
	for (int i = 0; i < peaks.size(); i++)
	{
		Mat peaksFilmCoor(3, 1, CV_64F);
		peaksFilmCoor.at<double>(0, 0) = peaks[i].x;
		peaksFilmCoor.at<double>(1, 0) = peaks[i].y;
		peaksFilmCoor.at<double>(2, 0) = 1;

		Mat temp = cameraMatrix.inv() * peaksFilmCoor;
		Point3f peaksCameraCoor = Point3f(temp.at<double>(0, 0), temp.at<double>(1, 0), temp.at<double>(2, 0));
		double scaleF = barB / (peaksCameraCoor.x * normal.x + peaksCameraCoor.y * normal.y + peaksCameraCoor.z * normal.z);
		double linenum = scanCount;// scanResult.size();
		
		Point3f linetemp;
		if (ui.rbCW->isChecked())
			linetemp = Point3f(scaleF * peaksCameraCoor.x + translation.x * linenum, scaleF * peaksCameraCoor.y + translation.y * linenum, scaleF * peaksCameraCoor.z + translation.z * linenum);
		else
			linetemp = Point3f(scaleF * peaksCameraCoor.x - translation.x * linenum, scaleF * peaksCameraCoor.y - translation.y * linenum, scaleF * peaksCameraCoor.z - translation.z * linenum);
		oneLaserLine.push_back(linetemp);
	}
	scanResult.push_back(oneLaserLine);
}
void LaserScanner::Revolution() {
	if (ui.rbCW->isChecked())
	{
		elde = ui.distance->toPlainText();
		value = elde.toInt();
		cout << value << endl;
		if (value < 300)
		{
			QString srtemp = QString::number(value);
			srtemp.append('\n');
			String temp = srtemp.toStdString();
			serial->write(temp.c_str(), temp.size());
			serial->waitForBytesWritten();
		}
	}
	else
	{
		elde = ui.distance->toPlainText();
		value = -elde.toInt();
		if (value > -300)
		{
			QString srtemp = QString::number(value);
			srtemp.append('\n');
			String temp = srtemp.toStdString();
			serial->write(temp.c_str(), temp.size());
			serial->waitForBytesWritten();
		}
	}
}
void LaserScanner::startScan()
{
	cout << "Start scanning" << endl;
	camera->blockSignals(true);
	camera->camera_->ExposureTime.SetValue(833);
	camera->camera_->TriggerSource.SetValue(Basler_UsbCameraParams::TriggerSource_Line1);
	camera->blockSignals(false);

	scanTrigger = 1;
	scanCount = 0;
	if (ui.rbCW->isChecked())
	{
		elde = ui.distance->toPlainText();
		value = elde.toInt();
		if (value < 300)
		{
			QString srtemp = QString::number(value);
			srtemp.append('\n');
			String temp = srtemp.toStdString();
			serial->write(temp.c_str(), temp.size());
			serial->waitForBytesWritten();
		}
	}
	else
	{
		elde = ui.distance->toPlainText();
		value = -elde.toInt();
		if (value > -300)
		{
			QString srtemp = QString::number(value);
			srtemp.append('\n');
			String temp = srtemp.toStdString();
			serial->write(temp.c_str(), temp.size());
			serial->waitForBytesWritten();
		}
	}

}
bool LaserScanner::findPeakBR4Estimate(Mat& img, vector<Point2f>& peaks, Rect roi) {
	Mat gray_img;
	if (img.channels() == 1) {
		gray_img = img.clone();
	}
	else {
		cv::cvtColor(img, gray_img, cv::COLOR_RGB2GRAY);
	}
	int currentPosLeft = 0, currentPosRight = 0;
	for (int i = 0; i < gray_img.rows; i++) {
		const uchar* Mi = gray_img.ptr<uchar>(i);
		int max_right = 0;
		int pos_right = 0;

		int max_left = 0;
		int pos_left = 0;
		if (currentPosLeft == 0 && currentPosRight == 0)
		{
			for (int j = 0; j < gray_img.cols; j++) {
				if (Mi[j] >= max_right) {
					max_right = Mi[j];
					pos_right = j;
				}
				if (Mi[j] > max_left) {
					max_left = Mi[j];
					pos_left = j;
				}
				currentPosLeft = pos_left;
				currentPosRight = pos_right;
			}
		}
		else
		{
			int start = ((currentPosLeft + currentPosRight) / 2) - 3;
			start = start > 0 ? start : 0;
			int end = start + 5;
			end = (end < gray_img.cols) ? end : gray_img.cols;
			for (int j = start; j < end; j++) {
				if (Mi[j] >= max_right) {
					max_right = Mi[j];
					pos_right = j;
				}
				if (Mi[j] > max_left) {
					max_left = Mi[j];
					pos_left = j;
				}
				currentPosLeft = pos_left;
				currentPosRight = pos_right;
			}
		}
		int x_1 = (int)(pos_left + pos_right) / 2;
		int x_2 = x_1 + 1;

		if ((x_1 - 2) < 0 && (x_2 + 2) > gray_img.cols) continue;

		float g4_1 = Mi[x_1 - 2] + Mi[x_1 - 1] - Mi[x_1 + 1] - Mi[x_1 + 2];
		float g4_2 = Mi[x_2 - 2] + Mi[x_2 - 1] - Mi[x_2 + 1] - Mi[x_2 + 2];

		float value = std::ceil((g4_1 - g4_2) * 100.0) / 100.0;
		float delta_x = 0;
		if (value == 0) delta_x = 0;
		else {
			delta_x = (float)g4_1 / (g4_1 - g4_2);
		}
		if (Mi[(int)delta_x + x_1] < MINLASER)
		{
			currentPosLeft = 0;
			currentPosRight = 0;
			continue;
		}
		peaks.push_back(Point2f(delta_x + (float)(x_1 + roi.x), (float)(i + roi.y)));
	}
	return peaks.size() > 3 ? true : false;
}
void LaserScanner::Checking(void)
{
	if (scanTrigger == 1)
	{
		if (scanCount == value || scanCount == -value)
		{
			scanCount = 0;
			scanTrigger = 0;
			camera->camera_->AcquisitionStatusSelector.SetValue(Basler_UsbCameraParams::AcquisitionStatusSelector_FrameTriggerWait);
			camera->camera_->TriggerSource.SetValue(Basler_UsbCameraParams::TriggerSource_Software);
			camera->changeExposureTime(13333);
			double max = 0;
			for (int i = 0; i < scanResult.size(); i++)
				if (max < scanResult[i].size()) max = scanResult[i].size();
			pointCloud.clear();
			pointCloud.init_points(scanResult.size(), max);
			pointCloud.init_normals(scanResult.size(), max);
			double x = 0, y = 0, z = 0;
			for (register int i = 0; i < scanResult.size(); i++)
			{
				for (register int j = 0; j < max; j++)
				{
					cv::Vec3f& cloud_point = pointCloud.points.at<cv::Vec3f>(i, j);
					if (j >= scanResult[i].size())
					{
						cloud_point[0] = (double)x;
						cloud_point[1] = (double)y;
						cloud_point[2] = (double)z;
						continue;
					}
					if (!LaserScanner::INVALID(pointCloud.points.at<cv::Vec3f>(i, j)[0]))
					{
						continue;
					}
					x = (double)scanResult[i][j].x;
					cloud_point[0] = x;

					y = (double)scanResult[i][j].y;
					cloud_point[1] = y;
					z = (double)scanResult[i][j].z;
					cloud_point[2] = z;
				}
			}
			scanResult.clear();
			String filename = path.toStdString() + "/Scanned Result.ply";
			pointCloud.compute_normals(pointCloud);
			pointCloud.write_ply(filename, pointCloud, 0);
			pointCloud.clear();
		}
	}
	else
	{
		camera->softwareTrigger();
	}
}