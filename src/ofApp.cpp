#include "ofApp.h"

cv::Mat createLUT(int numColors) {
    // When numColors=1 the LUT will only have 1 color which is black.
    if (numColors < 0) {
        numColors = 1;
    }
    if (numColors > 256) {
        numColors = 256;
    }

    cv::Mat lookupTable = cv::Mat::zeros(cv::Size(1, 256), CV_8UC1);

    int startIdx = 0;
    for (size_t x = 0; x < 256; x += 256.0 / numColors) {
        lookupTable.at<uchar>(0, x) = x;

        for (size_t y = startIdx; y < x; y++) {
            if (lookupTable.at<uchar>(0, y) == 0) {
                lookupTable.at<uchar>(0, y) = lookupTable.at<uchar>(0, x);
            }
        }
        startIdx = x;
    }
    return lookupTable;
}

void reduceColors(const cv::Mat& src, cv::Mat& dst, int numRed, int numGreen, int numBlue) {
    cv::Mat redLUT = createLUT(numRed);
    cv::Mat greenLUT = createLUT(numGreen);
    cv::Mat blueLUT = createLUT(numBlue);
    vector<cv::Mat> channels(3);
    split(src, channels);
    LUT(channels[0], blueLUT, channels[0]);
    LUT(channels[1], greenLUT, channels[1]);
    LUT(channels[2], redLUT, channels[2]);
    merge(channels, dst);
}

void cartoon(const cv::Mat& src, cv::Mat& dst, int numRed, int numGreen, int numBlue) {
    cv::Mat imgReduced, img;
    cvtColor(src, img, cv::COLOR_BGRA2BGR);
    reduceColors(img, imgReduced, numRed, numGreen, numBlue);
    cvtColor(img, dst, cv::COLOR_BGR2GRAY);
    medianBlur(dst, dst, 15);
    adaptiveThreshold(dst, dst, 255, cv::ADAPTIVE_THRESH_MEAN_C, cv::THRESH_BINARY, 19, 5);
    cvtColor(dst, dst, cv::COLOR_GRAY2BGR);
    bitwise_and(imgReduced, dst, dst);
}

//deprecated
//void cartoonify(const cv::Mat& imgBGRA, cv::Mat& imgCartoon) {
//
//    int num_down = 2, //number of downsampling steps
//        num_bilateral = 1; //number of bilateral filtering steps
//    cv::Mat imgColor, imgGray, imgBlur, imgEdge;
//    
//    //downsample image using Gaussian pyramid
//    cv::cvtColor(imgBGRA, imgColor, cv::COLOR_BGRA2BGR);
//    for (size_t i = 0; i < num_down; i++) {
//        cv::pyrDown(imgColor, imgColor);
//        //repeatedly apply small bilateral filter instead of
//        //applying one large filter
//        for (size_t j = 0; j < num_bilateral; j++) {
//            cv::edgePreservingFilter(imgColor, imgColor);
//            //cv::Mat tmp = imgColor.clone();
//            //cv::bilateralFilter(tmp, imgColor, 9, 9, 7);
//            //upsample image to original size
//            for (size_t k = 0; k < num_down; k++) {
//                cv::pyrUp(imgColor, imgColor);
//            }
//        }
//    }
//
//    //convert to grayscaleand apply median blur
//    cv::cvtColor(imgBGRA, imgGray, cv::COLOR_BGRA2GRAY);
//    cv::medianBlur(imgGray, imgBlur, 7);
//
//    //detectand enhance edges
//    cv::adaptiveThreshold(imgBlur, imgEdge, 255, cv::ADAPTIVE_THRESH_MEAN_C, cv::THRESH_BINARY, 9, 2);
//
//    //convert back to color, bit - AND with color image
//    //cv::cvtColor(imgEdge, imgEdge, cv::COLOR_GRAY2BGR);
//    //cv::bitwise_and(imgColor, imgColor, imgCartoon, imgEdge);
//    //imgCartoon = imgColor.clone();
//    //imgCartoon.setTo(cv::Scalar(0, 0, 0), ~imgEdge);
//
//    imgCartoon = imgColor.clone();
//
//}

//--------------------------------------------------------------
void ofApp::setup() {

    ofSetFrameRate(30);
    ofBackground(ofColor::black);
    ofSetBackgroundAuto(true);
    ofSetVerticalSync(true);
    ofEnableDepthTest();

    light.setPosition(lightDir);
    shader.load("toonshader");

}

//--------------------------------------------------------------
void ofApp::update() {
    //cv::Mat resized;
    pointCloud.clear();
    pointCloud.setMode(OF_PRIMITIVE_TRIANGLES);
    pointCloud.enableIndices();
    kinect.setRGB();
    cartoon(kinect.rgbImage, matBGR, 96, 96, 64);
    //cv::cvtColor(kinect.rgbImage, matBGR, cv::COLOR_BGRA2BGR);
    //int dilation_size = 7;
    //cv::Mat element = cv::getStructuringElement(cv::MORPH_DILATE,
    //    cv::Size(2 * dilation_size + 1, 2 * dilation_size + 1),
    //    cv::Point(dilation_size, dilation_size));
    //cv::dilate(matBGR, matBGR, element);
    kinect.setDepth();
    kinect.setBodyIndex();
    //cv::resize(kinect.rgbImage, resized, cv::Size(kinect.bodyIndexImage.cols, kinect.bodyIndexImage.rows));
    coords.resize(0);
    vector<vector<size_t>> indices(kinect.bodyIndexImage.rows, vector<size_t>(kinect.bodyIndexImage.cols, 0));
    const int stride = 4;
    int idxCounter = 0;
    for (size_t y = 0; y < kinect.bodyIndexImage.rows; y += stride) {
        for (size_t x = 0; x < kinect.bodyIndexImage.cols; x += stride) {
            UINT16 depth = kinect.depthImage.at<UINT16>(y, x);
            if (kinect.bodyIndexImage.at<uchar>(y, x) == 255)
                continue;
            coords.push_back(y);
            coords.push_back(x);
            indices[y][x] = idxCounter++;
            DepthSpacePoint dep;
            CameraSpacePoint cap;
            ColorSpacePoint cop;
            dep.X = x;
            dep.Y = y;
            kinect.coordinateMapper->MapDepthPointToCameraSpace(dep, depth, &cap);
            kinect.coordinateMapper->MapDepthPointToColorSpace(dep, depth, &cop);
            if (cap.X != -std::numeric_limits<float>::infinity()) {
                int cx = CLAMP((int)cop.X, 0, matBGR.cols - 1);
                int cy = CLAMP((int)cop.Y, 0, matBGR.rows - 1);
                pointCloud.addColor(ofFloatColor(
                    matBGR.at<cv::Vec3b>(cy, cx)[2] / 255.,
                    matBGR.at<cv::Vec3b>(cy, cx)[1] / 255.,
                    matBGR.at<cv::Vec3b>(cy, cx)[0] / 255.));
                pointCloud.addVertex(ofVec3f(cap.X * 1000, cap.Y * 1000, cap.Z * -1000));
            }
        }
    }

    shader.setUniform3f("lightDir", lightDir);
    //OF_PRIMITIVE_TRIANGLES
    const float threshold = 100;
    normals.resize(pointCloud.getNumVertices());
    if (pointCloud.getNumVertices() > 0) {
        delaunator::Delaunator d(coords);
        for (size_t i = 0; i < d.triangles.size(); i += 3) {
            size_t tx0 = d.coords[2 * d.triangles[i]],
                ty0 = d.coords[2 * d.triangles[i] + 1],
                tx1 = d.coords[2 * d.triangles[i + 1]],
                ty1 = d.coords[2 * d.triangles[i + 1] + 1],
                tx2 = d.coords[2 * d.triangles[i + 2]],
                ty2 = d.coords[2 * d.triangles[i + 2] + 1];
            if (ofDistSquared(tx0, ty0, tx1, ty1) < threshold
                && ofDistSquared(tx1, ty1, tx2, ty2) < threshold
                && ofDistSquared(tx2, ty2, tx0, ty0) < threshold) {
                //cout << indices[tx0][ty0] << ", " << indices[tx1][ty1] << ", " << indices[tx2][ty2] << endl;
                pointCloud.addIndex(indices[tx0][ty0]);
                pointCloud.addIndex(indices[tx1][ty1]);
                pointCloud.addIndex(indices[tx2][ty2]);


                ofVec3f v0 = pointCloud.getVertex(indices[tx0][ty0]),
                    v1 = pointCloud.getVertex(indices[tx1][ty1]),
                    v2 = pointCloud.getVertex(indices[tx2][ty2]);

                ofVec3f U = v1 - v0,
                    V = v2 - v0;

                ofVec3f normal((U.y * V.z) - (U.z * V.y), (U.z * V.x) - (U.x * V.z), (U.x * V.y) - (U.y * V.x));

                normal.normalize();

                normals[indices[tx0][ty0]] = normal;
                normals[indices[tx1][ty1]] = normal;
                normals[indices[tx2][ty2]] = normal;
            }
        }
        pointCloud.addNormals(normals);
    }

    //OF_PRIMITIVE_LINES
    //const int connectionDist = 22;
    //int numVerts = pointCloud.getNumVertices();
    //for (size_t a = 0; a < numVerts; ++a) {
    //    ofVec3f verta = pointCloud.getVertex(a);
    //    for (size_t b = a + 1; b < numVerts; ++b) {
    //        ofVec3f vertb = pointCloud.getVertex(b);
    //        float dist = verta.distance(vertb);
    //        if (dist <= connectionDist) {
    //            pointCloud.addIndex(a);
    //            pointCloud.addIndex(b);
    //        }
    //    }
    //}
}

//--------------------------------------------------------------
void ofApp::draw() {

    light.enable();
    cam.begin();
    //shader.begin();
    if (pointCloud.getNumVertices() > 0) {
        pointCloud.draw();
        //pointCloud.drawWireframe();
    }
    //shader.end();
    cam.end();
    light.disable();

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
