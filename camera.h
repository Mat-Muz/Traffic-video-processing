#include <opencv2/opencv.hpp>
#include <string.h>

using namespace cv;
using namespace std;

class Camera
{
public:
	Camera();
	
	bool open(std::string name);
	void play(double speed);
	bool close();
	
private:
	std::string m_fileName;
	VideoCapture m_cap;
	int m_fps;
	double speed;
		
	Mat m_frame;	
};
