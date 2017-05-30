
// Daniel Shiffman
// blobtracking example
// encontrado en 
// https://github.com/CodingTrain/Rainbow-Code/blob/master/Tutorials/Processing/11_video/sketch_11_9_BlobTrackingIDs/sketch_11_9_BlobTrackingIDs.pde
// y depth tresholding example
// encontrado en
// https://github.com/shiffman/OpenKinect-for-Processing/blob/master/OpenKinect-Processing/examples/Kinect_v1/DepthThreshold/DepthThreshold.pde


import org.openkinect.freenect.*;
import org.openkinect.processing.*;

Kinect kinect;

// Depth image
PImage depthImg;

// Which pixels do we care about?
int minDepth =  0;
int maxDepth = 600;

// What is the kinect's angle
float angle;
int blobCounter = 0;

color trackColor; 
float threshold = 5;
float distThreshold = 50;

float midX = 320;
float midY = 240;
float midD = 0;
float angleY = 0;
float angleZ = 0;
ArrayList<Blob> blobs = new ArrayList<Blob>();
float maxangleY = -10000000;
float minangleY = 10000000;
int counter = 0;


void setup() {
  size(1280, 480, P3D);

  kinect = new Kinect(this);
  kinect.enableMirror(true);
  kinect.initDepth();
  angle = kinect.getTilt();
  frameRate(20);

  // Blank image
  depthImg = new PImage(kinect.width, kinect.height);
  trackColor = color(0, 255, 0);
}

void draw() {
  // Draw the raw image
  //image(kinect.getDepthImage(), 0, 0);
  background(0);
  // Threshold the depth image
  int[] rawDepth = kinect.getRawDepth();
  for (int i=0; i < rawDepth.length; i++) {
    if (rawDepth[i] >= minDepth && rawDepth[i] <= maxDepth) {
      depthImg.pixels[i] = color(0,255,0);
    } else {
      depthImg.pixels[i] = color(0);
    }
  }

  // Draw the thresholded image
  depthImg.updatePixels();
  image(depthImg, 0, 0);

  fill(0);
  text("TILT: " + angle, 10, 20);
  text("THRESHOLD: [" + minDepth + ", " + maxDepth + "]", 10, 36);
  
  
  ArrayList<Blob> currentBlobs = new ArrayList<Blob>();

  // Begin loop to walk through every pixel
  for (int x = 0; x < depthImg.width; x++ ) {
    for (int y = 0; y < depthImg.height; y++ ) {
      int loc = x + y * depthImg.width;
      // What is current color
      color currentColor = depthImg.pixels[loc];
      float r1 = red(currentColor);
      float g1 = green(currentColor);
      float b1 = blue(currentColor);
      float r2 = red(trackColor);
      float g2 = green(trackColor);
      float b2 = blue(trackColor);

      float d = distSq(r1, g1, b1, r2, g2, b2); 

      if (d < threshold*threshold) {

        boolean found = false;
        for (Blob b : currentBlobs) {
          if (b.isNear(x, y)) {
            b.add(x, y, rawDepth[loc]); //<>//
            found = true;
            break;
          }
        }

        if (!found) {
          Blob b = new Blob(x, y);
          currentBlobs.add(b);
        }
      }
    }
  }

  for (int i = currentBlobs.size()-1; i >= 0; i--) {
    if (currentBlobs.get(i).size() < 5000) {
      currentBlobs.remove(i);
    }
  }

  // There are no blobs!
  if (blobs.isEmpty() && currentBlobs.size() > 0) {
    println("Adding blobs!");
    for (Blob b : currentBlobs) {
      b.id = blobCounter;
      blobs.add(b);
      blobCounter++;
    }
  } else if (blobs.size() <= currentBlobs.size()) {
    // Match whatever blobs you can match
    for (Blob b : blobs) {
      float recordD = 1000;
      Blob matched = null;
      for (Blob cb : currentBlobs) {
        PVector centerB = b.getCenter();
        PVector centerCB = cb.getCenter();         
        float d = PVector.dist(centerB, centerCB);
        if (d < recordD && !cb.taken) {
          recordD = d; 
          matched = cb;
        }
      }
      matched.taken = true;
      b.become(matched);
    }

    // Whatever is leftover make new blobs
    for (Blob b : currentBlobs) {
      if (!b.taken) {
        b.id = blobCounter;
        blobs.add(b);
        blobCounter++;
      }
    }
  } else if (blobs.size() > currentBlobs.size()) {
    for (Blob b : blobs) {
      b.taken = false;
    }


    // Match whatever blobs you can match
    for (Blob cb : currentBlobs) {
      float recordD = 1000;
      Blob matched = null;
      for (Blob b : blobs) {
        PVector centerB = b.getCenter();
        PVector centerCB = cb.getCenter();         
        float d = PVector.dist(centerB, centerCB);
        if (d < recordD && !b.taken) {
          recordD = d; 
          matched = b;
        }
      }
      if (matched != null) {
        matched.taken = true;
        matched.become(cb);
      }
    }

    for (int i = blobs.size() - 1; i >= 0; i--) {
      Blob b = blobs.get(i);
      if (!b.taken) {
        blobs.remove(i);
      }
    }
  }
  int c = 0;
  if (!blobs.isEmpty()){
    midX = 0;
    midY = 0;
    midD = -50;
  }
  for (Blob b : blobs) {
    b.show();
    midX += b.getCenter().x;
    midY += b.getCenter().y;
    midD -= b.getDepth();
    c++;
  } 
  if (!blobs.isEmpty()){
    midX = midX/c;
    midY = midY/c;
    midD = midD/c+1;
    midD = map(midD, -450, -600, -100, -1000);
  }
  if (blobs.size() == 2){
    
    float D1 = blobs.get(0).getDepth();
    float D2 = blobs.get(1).getDepth();
    float x1 = blobs.get(0).getCenter().x;
    float x2 = blobs.get(1).getCenter().x;
    float y1 = blobs.get(0).getCenter().y;
    float y2 = blobs.get(1).getCenter().y;
    midY = ((y1 + y2)/2);
    midD = ((D1 + D2)/2);
    midX = ((x1 + x2)/2);
    angleZ = asin((y2-midY)/sqrt(distSq(midX,midY,x2,y2)));
    angleZ = map(angleZ, -1.5678263, 1.5263819, -3.14, 3.14);
    angleY = asin((midD-D2)/distSq(midX,midD,x2,D2));
    angleY = map(angleY, -0.0048440713, 0.020782402, -3.1, 3.5);
    midD = -125;
  }else{
    angleY = 0;
    angleZ = 0;
  }
  pushMatrix();
  translate(640,0);
  translate(midX,midY,midD);
  //println(angleY);
  //println(angleZ);
  rotateY(angleY);
  rotateZ(angleZ);
  if (angleY != 0)maxangleY = max(angleY,maxangleY);
  if (angleY != 0)minangleY = min(angleY,minangleY);
  //fill(255,255,0);
  noFill();
  stroke(150);
  box(150);
  popMatrix();
  //println(maxangleY);
  //println(minangleY);
  textAlign(RIGHT);
  fill(255);
  text(currentBlobs.size(), width-10, 40);
  //text(blobs.size(), width-10, 80);
  textSize(24);
  //text("color threshold: " + threshold, width-10, 50);  
  //text("distance threshold: " + distThreshold, width-10, 25);
}

// Adjust the angle and the depth threshold min and max
void keyPressed() {
  if (key == CODED) {
    if (keyCode == UP) {
      angle++;
    } else if (keyCode == DOWN) {
      angle--;
    }
    angle = constrain(angle, 0, 30);
    kinect.setTilt(angle);
  } else if (key == 'a') {
    minDepth = constrain(minDepth+10, 0, maxDepth);
  } else if (key == 's') {
    minDepth = constrain(minDepth-10, 0, maxDepth);
  } else if (key == 'z') {
    maxDepth = constrain(maxDepth+10, minDepth, 2047);
  } else if (key =='x') {
    maxDepth = constrain(maxDepth-10, minDepth, 2047);
  }
}


float distSq(float x1, float y1, float x2, float y2) {
  float d = (x2-x1)*(x2-x1) + (y2-y1)*(y2-y1);
  return d;
}


float distSq(float x1, float y1, float z1, float x2, float y2, float z2) {
  float d = (x2-x1)*(x2-x1) + (y2-y1)*(y2-y1) +(z2-z1)*(z2-z1);
  return d;
}