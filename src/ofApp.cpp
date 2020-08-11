#include "ofApp.h"

#pragma region Update
// convert full 16-bits depth range to 8 bits
// use setDepth(false) for this to work
void ofApp::updateDepth() {

    kinect.setDepth(false);//mapped to the luminance of the black and white image 
                           //of 0 (black) to 65535 (white) with the distance of 0 mm to 4500 mm.
    
    kinect.depthImage.convertTo(matGray, CV_8UC1, 255. / 4500.);
    //kinect.depthImage.convertTo(matGray, CV_8UC1, 1. / 257.);
    toOf(matGray, frame);
}

//This function converts a BGRA mat into an ofImage.
//Use NtKinect::setRGB() for this to work.
void ofApp::updateRGB() {

    kinect.setRGB();
    cv::cvtColor(kinect.rgbImage, matRGB, CV_BGRA2RGB, 3);
    cvtTo8Bit(matRGB);
    //colorReduce(matRGB);
    //matRGB = quantizeImage(matRGB, 3);
    reducePixels(matRGB);
    stylize();
    toOf(matRGB, frame);

}

auto getUniqueColors(cv::Mat& img) {

    cv::Mat im;
    cv::resize(img, im, cv::Size(8, 8));
    // partition wants a vector, so we need a copy ;(
    cv::Vec3b* p = im.ptr<cv::Vec3b>();
    vector<cv::Vec3b> pix(p, p + im.total());

    // now cluster:
    //vector<int> labels;
    //int unique = cv::partition(pix, labels,
    //    [](auto a, auto b) { return a == b; });
    //    [](auto a, auto b) { return cv::norm(a, b) < 32; });

    sort(pix.begin(), pix.end(), [](auto a, auto b) {
        if (a[0] == b[0])
            if (a[1] == b[1])
                return a[2] < b[2];
            else return a[1] < b[1];
        else return a[0] < b[0]; });

    pix.erase(unique(pix.begin(), pix.end()), pix.end());

    return pix;

    // debug output
    //cv::Mat lm = cv::Mat(labels).reshape(1, im.rows);
    //cout << lm << endl;
    //cout << unique << " unique colors" << endl;
}

//This function converts a BGRA mat into an ofImage.
//Use NtKinect::setRGB() and NtKinect::setBodyIndex() for this to work.
void ofApp::updateBodyIdx() {

    cv::Mat matBg, matFg;

    kinect.setRGB();
    cv::cvtColor(kinect.rgbImage, matFg, cv::COLOR_BGRA2BGR);
    matBg = cv::Mat::zeros(kinect.rgbImage.rows, kinect.rgbImage.cols, CV_8UC3);
    kinect.setDepth();
    kinect.setBodyIndex();
    for (size_t y = 0; y < kinect.bodyIndexImage.rows; y++) {
        for (size_t x = 0; x < kinect.bodyIndexImage.cols; x++) {
            UINT16 d = kinect.depthImage.at<UINT16>(y, x);
            uchar bi = kinect.bodyIndexImage.at<uchar>(y, x);
            if (bi == 255) continue;
            ColorSpacePoint cp;
            DepthSpacePoint dp;
            dp.X = x;
            dp.Y = y;
            kinect.coordinateMapper->MapDepthPointToColorSpace(dp, d, &cp);
            int cx = (int)cp.X, cy = (int)cp.Y;
            copyRect(matFg, matBg, cx - 2, cy - 2, 4, 4, cx - 2, cy - 2);
        }
    }

    cv::cvtColor(matBg, matBodyIdx, CV_BGRA2RGB, 3);
    cvtTo8Bit(matBodyIdx);
    reducePixels(matBodyIdx);

    //int erosion_size = 1;
    //auto element = cv::getStructuringElement(cv::MORPH_CROSS,
    //    cv::Size(2 * erosion_size + 1, 2 * erosion_size + 1),
    //    cv::Point(erosion_size, erosion_size));
    //cv::erode(matBodyIdx, matBodyIdx, element, cv::Point(-1, -1), 1, 0, cv::Scalar(0, 0, 0));

    //getUniqueColors(matBodyIdx);
    //stylize();
    toOf(matBodyIdx, frame);

}

void ofApp::rewriteCode() {

    cv::Mat resized;
    
    pointCloud.clear();
    kinect.setRGB();
    cv::resize(kinect.rgbImage, resized, cv::Size(512, 424));
    kinect.setDepth();
    kinect.setBodyIndex();

    for (size_t y = 0; y < kinect.bodyIndexImage.rows; y++) {
        for (size_t x = 0; x < kinect.bodyIndexImage.cols; x++) {
            UINT16 d = kinect.depthImage.at<UINT16>(y, x);
            uchar bi = kinect.bodyIndexImage.at<uchar>(y, x);
            if (bi == 255) continue;
            DepthSpacePoint dp;
            CameraSpacePoint cp;
            dp.X = x;
            dp.Y = y;
            kinect.coordinateMapper->MapDepthPointToCameraSpace(dp, d, &cp);
            //cout << y <<" "<< cp.X << ", " << cp.Y << ", " << cp.Z << endl;
            pointCloud.addVertex(ofVec3f(cp.X * 1000, cp.Y * 1000, cp.Z * 1000));
            pointCloud.addColor(ofFloatColor(resized.at<cv::Vec3f>(y, x)[0] / 255., resized.at<cv::Vec3f>(y, x)[1] / 255., resized.at<cv::Vec3f>(y, x)[2] / 255.));

            //ColorSpacePoint cp;
            //DepthSpacePoint dp;
            //dp.X = x;
            //dp.Y = y;
            //kinect.coordinateMapper->MapDepthPointToColorSpace(dp, d, &cp);
            //int cx = (int)cp.X, cy = (int)cp.Y;
            //copyRect(matFg, matBg, cx - 2, cy - 2, 4, 4, cx - 2, cy - 2);
        }
    }
    //cv::cvtColor(matBg, matBodyIdx, CV_BGRA2RGB, 3);
    //cvtTo8Bit(matBodyIdx);
    //reducePixels(matBodyIdx);

    //int erosion_size = 1;
    //auto element = cv::getStructuringElement(cv::MORPH_CROSS,
    //    cv::Size(2 * erosion_size + 1, 2 * erosion_size + 1),
    //    cv::Point(erosion_size, erosion_size));
    //cv::erode(matBodyIdx, matBodyIdx, element, cv::Point(-1, -1), 1, 0, cv::Scalar(0, 0, 0));

    //getUniqueColors(matBodyIdx);
    //stylize();
    //toOf(matBodyIdx, frame);

}

void ofApp::stylize() {

    //1. simple canny

    cv::Mat gray, dst, detected_edges, addweight;
    cv::cvtColor(matRGB, gray, cv::COLOR_RGB2GRAY);
    blur(gray, gray, cv::Size(3, 3));
    Canny(gray, detected_edges, edgeThresh1, edgeThresh2, 3);
    
    dst = cv::Scalar::all(0);
    
    matRGB.copyTo(dst, detected_edges); // copy part of src image according the canny output, canny is used as mask
    cvtColor(detected_edges, detected_edges, CV_GRAY2BGR); // convert canny image to bgr
    addWeighted(matRGB, 0.5, detected_edges, 0.5, 0.0, addweight); // blend src image with canny image
    matRGB += detected_edges; // add src image with canny image
    
    addweight.copyTo(matRGB);

    //2. taking too much time

    //cv::Mat segmented, gray, edges, edgesRgb;
    //cv::pyrMeanShiftFiltering(matRGB, segmented, 15, 40);
    //cv::cvtColor(segmented, gray, cv::COLOR_RGB2GRAY);
    //cv::Canny(gray, edges, 150, 150);
    //cv::cvtColor(edges, edgesRgb, cv::COLOR_GRAY2RGB);
    //matRGB -= edgesRgb;

    //3. similar to Bilateral Filtering but faster (still slow for 1080p 60fps)

    //cv::edgePreservingFilter(matRGB, matRGB, cv::RECURS_FILTER);

    //4. stylization (very slow)

    //cv::stylization(matRGB, matRGB);

    //5. pencil sketch (little bit latency)

    //cv::pencilSketch(matRGB, matGray, matRGB);

}

#pragma endregion

#pragma region Utils
//This function converts an image into 8 bit truecolor(RRRGGGBB).
void ofApp::cvtTo8Bit(cv::Mat& img) {
    //TODO: set palette
    for (int i = 0; i < img.rows; i++) {
        for (int j = 0; j < img.cols; j++) {            
            img.at<cv::Vec3b>(i, j)[0] = (img.at<cv::Vec3b>(i, j)[0] >> 6) << 6;
            img.at<cv::Vec3b>(i, j)[1] = (img.at<cv::Vec3b>(i, j)[1] >> 6) << 6;
            img.at<cv::Vec3b>(i, j)[2] = (img.at<cv::Vec3b>(i, j)[2] >> 5) << 5;
        }
    }
}

//This function scale an image down and up.
void ofApp::reducePixels(cv::Mat& img) {

    cv::resize(img, img, cv::Size(width / 2, height / 2), CV_INTER_LINEAR);
    cv::resize(img, img, cv::Size(width, height), CV_INTER_NN);
}

//This function converts a RGB-Mat to corresponding ofImage.
void ofApp::toOf(const cv::Mat& src, ofImage& img) {

    ofPixels pix;
    pix.setFromExternalPixels(src.data, src.cols, src.rows, src.channels());
    img.setFromPixels(pix);
    img.update();
}

void ofApp::copyRect(cv::Mat& src, cv::Mat& dst, int sx, int sy, int w, int h, int dx, int dy) {
    if (sx + w < 0 || sx >= src.cols || sy + h < 0 || sy >= src.rows) return;
    if (sx < 0) { w += sx; dx -= sx; sx = 0; }
    if (sx + w > src.cols) w = src.cols - sx;
    if (sy < 0) { h += sy; dy -= sy; sy = 0; }
    if (sy + h > src.rows) h = src.rows - sy;

    if (dx + w < 0 || dx >= dst.cols || dy + h < 0 || dy >= dst.rows) return;
    if (dx < 0) { w += dx; sx -= dx; dx = 0; }
    if (dx + w > dst.cols) w = dst.cols - dx;
    if (dy < 0) { h += dy; sy -= dy; dy = 0; }
    if (dy + h > dst.rows) h = dst.rows - dy;

    cv::Mat roiSrc(src, cv::Rect(sx, sy, w, h));
    cv::Mat roiDst(dst, cv::Rect(dx, dy, w, h));
    roiSrc.copyTo(roiDst);
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
#pragma endregion

#pragma region ofApp
//--------------------------------------------------------------
void ofApp::setup() {

    ofSetFrameRate(60);
    ofBackground(ofColor::black);
    ofSetBackgroundAuto(true);
    ofSetVerticalSync(true);
    ofEnableDepthTest();

    gui.setup();
    gui.add(edgeThresh1.setup("edgeThresh1", 60, 1, 500));
    gui.add(edgeThresh2.setup("edgeThresh2", 100, 1, 500));

}

//--------------------------------------------------------------
void ofApp::update() {
    //updateDepth();

    //updateRGB();

    //updateBodyIdx();

    rewriteCode();
}

//--------------------------------------------------------------
void ofApp::draw() {
    light.enable();
    cam.begin();
    ofDrawCylinder(10,20);
    if (pointCloud.getNumVertices() > 0)
        pointCloud.drawVertices();
    cam.end();
    light.disable();


    //drawTextureAtRowAndColumn("Kinect", frame.getTexture(), 0, 0);
    //gui.draw();
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
void ofApp::mouseMoved(int x, int y) {

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
#pragma endregion
