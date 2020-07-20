#pragma once

#include "ofMain.h"
#include "ofxGui.h"
#include "ofxOpenCv.h"

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

	cv::Mat greyImg;
	cv::Mat colorImg;

	ofImage frame;

	ofxPanel panel;

	inline void depthToOf(const cv::Mat& src, ofImage& img);
	inline void colorToOf(const cv::Mat& src, ofImage& img);
	inline void bodyIndexToOf(const cv::Mat& src, ofImage& img);
	inline void convertTo8Bit(cv::Mat& img);

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
