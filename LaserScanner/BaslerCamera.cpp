#include "BaslerCamera.h"

BaslerCamera::BaslerCamera(TRIGGERSOURCE trigger, const Pylon::String_t serial) : _trigger(trigger), _serial(serial)
{

}

BaslerCamera::~BaslerCamera()
{
	destroyCam();
}
bool BaslerCamera::initCam()
{
	Pylon::PylonInitialize();
	try {
		try {
			Pylon::CDeviceInfo info;
			if (_serial == "") {
				DeviceInfoList_t devices;
				CTlFactory& tlFactory = CTlFactory::GetInstance();
				int t = tlFactory.EnumerateDevices(devices);
				if (devices.size() > 0)
				{
					info.SetSerialNumber(devices[0].GetSerialNumber());
					info.SetDeviceClass(Pylon::CBaslerUsbInstantCamera::DeviceClass());
				}
				else
				{
					std::cout << "No camera found -_- " << std::endl;
					return false;
				}
			}
			else {
				// set serial number
				info.SetSerialNumber(_serial);
			}

			pDevice_ = Pylon::CTlFactory::GetInstance().CreateFirstDevice(info);
			camera_ = new Pylon::CBaslerUsbInstantCamera(pDevice_);

		}
		catch (const Pylon::GenericException& e) {
			std::cerr << "[ERROR] an exception occured : " << e.GetDescription() << std::endl;
			return false;
		}

		std::cout << "Using Device: " << camera_->GetDeviceInfo().GetModelName() << std::endl;
		std::cout << "Serial Number: " << camera_->GetDeviceInfo().GetSerialNumber() << std::endl;

		camera_->RegisterImageEventHandler(new BaslerImageEventHandler(this), Pylon::RegistrationMode_Append, Pylon::Cleanup_Delete);

		/// Open the camera , so that you could set camera parameter
		camera_->Open();
		camera_->MaxNumBuffer = 5;
		/// set hardware trigger parameter
		/// here we used a Arduino microcontroller to generate hardware trigger signal
		camera_->AcquisitionMode.SetValue(Basler_UsbCameraParams::AcquisitionMode_Continuous);
		camera_->TriggerSelector.SetValue(Basler_UsbCameraParams::TriggerSelector_FrameStart);
		camera_->TriggerMode.SetValue(Basler_UsbCameraParams::TriggerMode_On);
		camera_->AcquisitionFrameRateEnable.SetValue(false);

		if (_trigger == TRIGGERSOURCE::SOFTWARE) {
			camera_->AcquisitionStatusSelector.SetValue(Basler_UsbCameraParams::AcquisitionStatusSelector_FrameTriggerWait);
			camera_->TriggerSource.SetValue(Basler_UsbCameraParams::TriggerSource_Software);
		}
		else {
			camera_->TriggerSource.SetValue(Basler_UsbCameraParams::TriggerSource_Line1);
			std::cout << "Line 1" << std::endl;
		}
		camera_->ExposureMode.SetValue(Basler_UsbCameraParams::ExposureMode_Timed);
		camera_->ExposureTime.SetValue(expo);// 1666.67); //250000.0
		isConnected_ = true;
	}
	catch (const  GenericException& e) {
		std::cerr << "[ERROR] an exception occured open:" << e.GetDescription() << std::endl;
		isConnected_ = 0;
	}
	return isConnected_;
}
bool BaslerCamera::startGrab()
{
	std::cerr << "************[INFO] Start Grabbing Images************" << std::endl;
	//camera_->StartGrabbing(Pylon::GrabStrategy_OneByOne, Pylon::GrabLoop_ProvidedByInstantCamera);
	try {
		camera_->AcquisitionStart.Execute();
		camera_->StartGrabbing(Pylon::GrabStrategy_OneByOne, Pylon::GrabLoop_ProvidedByInstantCamera);
	}
	catch (const Pylon::GenericException& e) {
		std::cerr << "[ERROR] an exception occured :" << e.GetDescription() << std::endl;
	}
	return false;
}
bool BaslerCamera::stopGrab()
{
	std::cerr << "************[INFO] Grabbing images end***********" << std::endl;
	try {
		camera_->AcquisitionStop.Execute();

	}
	catch (const Pylon::GenericException& e) {
		std::cerr << "[ERROR] an exception occured : " << e.GetDescription() << std::endl;
	}
	return false;
}
bool BaslerCamera::softwareTrigger() {
	// Generate a software trigger signal
	if (!isConnected_) return false;
	if (!camera_->IsGrabbing()) return false;
	bool isWaitFrame = camera_->WaitForFrameTriggerReady(100, Pylon::TimeoutHandling_ThrowException);
	if (isWaitFrame == true) {
		camera_->TriggerSoftware.Execute();
		//std::cout << "Trigger created" << std::endl;
	}
	return isWaitFrame;
}
void BaslerCamera::destroyCam()
{
	std::cerr << "[INFO] Basler Camera Destroyed" << std::endl;
	// close the camera
	try {
		if (isConnected_) {
			camera_->Close();
		}

	}
	catch (const Pylon::GenericException& e) {
		std::cerr << "[ERROR] an exception occured : " << e.GetDescription() << std::endl;
	}
	Pylon::PylonTerminate();
}
void BaslerCamera::changeExposureTime(int e)
{
	expo = e;
	camera_->ExposureTime.SetValue(expo);
}

/////////////////////////////////Event Handler////////////////////////////////

BaslerImageEventHandler::BaslerImageEventHandler(BaslerCamera* cameraGrabber) {
	cameraGrabber_ = cameraGrabber;
}
///
/// \brief BaslerImageEventHandler::OnImageGrabbed : function called when the image grabbed event happened
/// \param camera
/// \param ptrGrabResult : a pointer used to save grabbed image buffer
///
void BaslerImageEventHandler::OnImageGrabbed(Pylon::CBaslerUsbInstantCamera& camera,
	const Pylon::CBaslerUsbGrabResultPtr& ptrGrabResult)
{
	int imgHeight = ptrGrabResult->GetHeight();
	int imgWidth = ptrGrabResult->GetWidth();

	////std::cout << "[INFO] Image Grabbed Result Report : " << std::endl; 
	////std::cout << "image height = " << imgHeight << " , image width = " << imgWidth << std::endl;

	// display image in an opencv window
	cv::Size imgSize(imgWidth, imgHeight);
	cv::Mat frame(imgSize, CV_8UC1, (uchar*)ptrGrabResult->GetBuffer());

	/*cv::imshow("frame", frame);
	cv::waitKey(1);*/
	emit cameraGrabber_->NewImageCame(frame);

	// save image to file
	// generate timeStamp
	///
	/// \brief timeStamp : timestamp stands for the exactly system time when you call the function gettimeofday()
	///tv_sec : the system time when you call the function and get the value (how many seconds)
	///tv_usec : the microseconds
}
