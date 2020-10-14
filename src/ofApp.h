#pragma once

#include "ofMain.h"
#include "ofxGui.h"
#include "ofxOpenCv.h"
//#define USE_FACE
#define USE_AUDIO
#include "NtKinect.h"
#include "delaunator.hpp"

class ofApp : public ofBaseApp{

	NtKinect kinect;
	cv::Mat matBGR;
	ofLight light;
	ofEasyCam cam;
	ofVboMesh pointCloud;
	ofShader shader;
	vector<double> coords;
	vector<glm::vec3> normals;
	const ofVec3f lightDir = ofVec3f(0, 500, 0);
	
	public:
		void setup() override;
		void update() override;
		void draw() override;
		void exit() override;

		void keyPressed(int key);
		void keyReleased(int key);
		void mouseMoved(int x, int y );
		void mouseDragged(int x, int y, int button);
		void mousePressed(int x, int y, int button);
		void mouseReleased(int x, int y, int button);
		void mouseEntered(int x, int y);
		void mouseExited(int x, int y);
		void windowResized(int w, int h);
		void dragEvent(ofDragInfo dragInfo);
		void gotMessage(ofMessage msg);
		
};
