#pragma once

#include <pylon/PylonIncludes.h>
#ifdef PYLON_WIN_BUILD
# include <pylon/PylonGUI.h>
#endif
#include <iostream>

#include <pylon/gige/BaslerGigEInstantCamera.h>
#include <pylon/usb/BaslerUsbInstantCamera.h>

#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"

#include "qobject.h"

using namespace Pylon;
using namespace cv;

enum TRIGGERSOURCE { SOFTWARE = 1, HARDWARE = 2 };

class BaslerCamera : public QObject
{
	Q_OBJECT
public:
	BaslerCamera(TRIGGERSOURCE trigger, const Pylon::String_t serial = 0);
	~BaslerCamera();

	bool initCam();
	bool startGrab();
	bool stopGrab();
	bool softwareTrigger();
	void destroyCam();
	void changeExposureTime(int expo);
	/*template<typename ObjectEvent>
	void imageEventHandle(ObjectEvent* rObject);*/

private:
	int expo=13333;

protected:
	bool showView_ = true;

public:
	TRIGGERSOURCE _trigger;
	Pylon::String_t  _serial;
	bool isConnected_ = false;
	Pylon::CBaslerUsbInstantCamera* camera_ = NULL;
	Pylon::IPylonDevice* pDevice_ = NULL;
public:
	Mat img;
signals:
	void NewImageCame(Mat img);
};

class BaslerImageEventHandler : public Pylon::CBaslerUsbImageEventHandler, public QObject {
public:

	BaslerImageEventHandler(BaslerCamera* cameraGrabber);

	virtual void OnImageGrabbed(Pylon::CBaslerUsbInstantCamera& camera, const Pylon::CBaslerUsbGrabResultPtr& ptrGrabResult);


protected:
	BaslerCamera* cameraGrabber_;

};