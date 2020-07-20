#include "ofApp.h"

// convert full 16-bits depth range to 8 bits
// use setDepth(false) for this to work
inline void ofApp::depthToOf(const cv::Mat& src, ofImage& img) {

    src.convertTo(greyImg, CV_8UC1, 1.0 / 256.0);
    ofPixels pix;
    pix.setFromExternalPixels(greyImg.data, greyImg.cols, greyImg.rows, greyImg.channels());
    img.setFromPixels(pix);
    img.update();
}

inline void ofApp::colorToOf(const cv::Mat& src, ofImage& img) {

    ofPixels pix;

    //convert
    cv::cvtColor(src, colorImg, CV_BGR2RGB, 3);

    //reduce color space
    //colorReduce(colorImg);
    //colorImg = quantizeImage(colorImg, 3);
    convertTo8Bit(colorImg);

    //reduce pixels
    cv::Mat temp;
    cv::resize(colorImg, temp, cv::Size(width / 2, height / 2), CV_INTER_LINEAR);
    cv::resize(temp, colorImg, cv::Size(width, height), CV_INTER_NN);

    pix.setFromExternalPixels(colorImg.data, colorImg.cols, colorImg.rows, colorImg.channels());
    img.setFromPixels(pix);
    img.update();
}

inline void ofApp::bodyIndexToOf(const cv::Mat& src, ofImage& img) {
    cv::cvtColor(src, colorImg, CV_BGR2RGB, 3);

    convertTo8Bit(colorImg);

    cv::Mat temp;
    cv::resize(colorImg, temp, cv::Size(width / 2, height / 2), CV_INTER_LINEAR);
    cv::resize(temp, colorImg, cv::Size(width, height), CV_INTER_NN);

    ofPixels pix;
    pix.setFromExternalPixels(colorImg.data, colorImg.cols, colorImg.rows, colorImg.channels());
    img.setFromPixels(pix);
    img.update();
}

//convert img into 8 bit truecolor RRRGGGBB.
inline void ofApp::convertTo8Bit(cv::Mat& img) {
    for (int i = 0; i < img.rows; i++) {
        for (int j = 0; j < img.cols; j++) {
            img.at<cv::Vec3b>(i, j)[0] = (img.at<cv::Vec3b>(i, j)[0] >> 6) << 6;
            img.at<cv::Vec3b>(i, j)[1] = (img.at<cv::Vec3b>(i, j)[1] >> 6) << 6;
            img.at<cv::Vec3b>(i, j)[2] = (img.at<cv::Vec3b>(i, j)[2] >> 6) << 6;
        }
    }
}

void ofApp::drawTextureAtRowAndColumn(const std::string& title, const ofTexture& tex, int row, int column) {
    float displayWidth = width / numColumns;
    float displayHeight = height / numRows;

    ofRectangle targetRectangle(
        row * displayWidth,
        column * displayHeight,
        displayWidth,
        displayHeight);

    ofNoFill();
    ofSetColor(ofColor::gray);
    ofDrawRectangle(targetRectangle);

    ofFill();
    ofSetColor(255);

    if (tex.isAllocated()) {
        ofRectangle textureRectangle(0, 0, tex.getWidth(), tex.getHeight());

        // Scale the texture rectangle to its new location and size.
        textureRectangle.scaleTo(targetRectangle);
        tex.draw(textureRectangle);
    }
    else {
        ofDrawBitmapStringHighlight("...",
            targetRectangle.getCenter().x,
            targetRectangle.getCenter().y);
    }

    ofDrawBitmapStringHighlight(title,
        targetRectangle.getPosition() + glm::vec3(14, 20, 0));
}

//--------------------------------------------------------------
void ofApp::setup() {

}

//--------------------------------------------------------------
void ofApp::update() {
    kinect.setRGB();
    colorToOf(kinect.rgbImage, frame);

    //kinect.setDepth(false);
    //depthToOf(kinect.depthImage, frame);

    //kinect.setRGB();
    //kinect.setBodyIndex();
    //bodyIndexToOf(kinect.bodyIndexImage, frame);
}

//--------------------------------------------------------------
void ofApp::draw() {
    drawTextureAtRowAndColumn("Kinect", frame.getTexture(), 0, 0);
}

//--------------------------------------------------------------
void ofApp::exit() {

}

//--------------------------------------------------------------
void ofApp::keyPressed(int key) {

}

//--------------------------------------------------------------
void ofApp::keyReleased(int key) {

}

//--------------------------------------------------------------
void ofApp::mouseMoved(int x, int y ) {

}

//--------------------------------------------------------------
void ofApp::mouseDragged(int x, int y, int button) {

}

//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button) {

}

//--------------------------------------------------------------
void ofApp::mouseReleased(int x, int y, int button) {

}

//--------------------------------------------------------------
void ofApp::mouseEntered(int x, int y) {

}

//--------------------------------------------------------------
void ofApp::mouseExited(int x, int y) {

}

//--------------------------------------------------------------
void ofApp::windowResized(int w, int h) {

}

//--------------------------------------------------------------
void ofApp::gotMessage(ofMessage msg) {

}

//--------------------------------------------------------------
void ofApp::dragEvent(ofDragInfo dragInfo) { 

}
