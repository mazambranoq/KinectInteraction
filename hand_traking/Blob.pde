// Adaptado de 
// Daniel Shiffman
// https://github.com/CodingTrain/Rainbow-Code/blob/master/Tutorials/Processing/11_video/sketch_11_9_BlobTrackingIDs/sketch_11_9_BlobTrackingIDs.pde

class Blob {
  float minx;
  float miny;
  float maxx;
  float maxy;
  float depth;
  long count;
  int id = 0;
  
  boolean taken = false;

  Blob(float x, float y) {
    minx = x;
    miny = y;
    maxx = x;
    maxy = y;
    depth = 0;
    count = 0;
  }
    
  void show() {
    stroke(0);
    fill(255, 100);
    strokeWeight(2);
    rectMode(CORNERS);
    rect(minx, miny, maxx, maxy);
    
    textAlign(CENTER);
    textSize(30);
    fill(0);
    text(this.size(), minx + (maxx-minx)*0.5, maxy - 10);
  }

  void add(float x, float y, float depth) {
    minx = min(minx, x);
    miny = min(miny, y);
    maxx = max(maxx, x);
    maxy = max(maxy, y);
    this.depth = this.depth + depth;
    count++;
  }
  
  float getDepth(){
    return depth/count;
  }
  
  void become(Blob other) {
    minx = other.minx;
    maxx = other.maxx;
    miny = other.miny;
    maxy = other.maxy;
    depth = other.depth;
    count = other.count;
  }

  float size() {
    return (maxx-minx)*(maxy-miny);
  }
  
  PVector getCenter() {
    float x = (maxx - minx)* 0.5 + minx;
    float y = (maxy - miny)* 0.5 + miny;    
    return new PVector(x,y); 
  }

  boolean isNear(float x, float y) {

    float cx = max(min(x, maxx), minx);
    float cy = max(min(y, maxy), miny);
    float d = distSq(cx, cy, x, y);

    if (d < distThreshold*distThreshold) {
      return true;
    } else {
      return false;
    }
  }
}