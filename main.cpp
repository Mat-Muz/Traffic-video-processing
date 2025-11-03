#include "camera.h"

int main(int argc, char **argv)
{
	Camera myCam;
		
	double speed = 1.0; //Vitesse de base 
	if(argc == 2){
		speed = std::stod(argv[1]);
	}

	myCam.open("cctv.avi");
	
	myCam.play(speed);
	
	myCam.close();
	
	return 1;
	
}
