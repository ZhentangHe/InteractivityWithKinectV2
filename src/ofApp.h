#pragma once

#include "ofMain.h"
#include "ofxGui.h"
#include "ofxOpenCv.h"
#include "ofxTriangle.h"
//#define USE_FACE
#define USE_AUDIO
#include "NtKinect.h"

class ofApp : public ofBaseApp{

	const size_t width = ofGetWidth();
	const size_t height = ofGetHeight();

	NtKinect kinect;

	int numRows = 1;
	int numColumns = 1;

	ofTexture texRGB;
	ofTexture texDepth;
	ofTexture texIR;
	ofTexture texSkeleton;

	cv::Mat matGray;
	cv::Mat matRGB;
	cv::Mat matBGR;
	cv::Mat matBodyIdx;

	ofImage frame;

	ofLight light;
	ofEasyCam cam;
	ofVboMesh pointCloud;
	ofShader shader;
	//ofxTriangle triangle;

	ofxIntSlider connectionDist, edgeThresh1, edgeThresh2;
	ofxPanel panel, gui;

	void updateDepth();
	void updateRGB();
	void updateBodyIdx();
	void updatePointCloud();

	void stylize();
	void cvtTo8Bit(cv::Mat& img);
	void reducePixels(cv::Mat& img);
	void toOf(const cv::Mat& src, ofImage& img);
	void copyRect(cv::Mat& src, cv::Mat& dst, int sx, int sy, int w, int h, int dx, int dy);

	void drawTextureAtRowAndColumn(const std::string& title,
		const ofTexture& tex,
		int row,
		int column);
	
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
