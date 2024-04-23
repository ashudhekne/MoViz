import processing.serial.*;
PGraphics background;
PGraphics foreground;
PGraphics foreground_imu;

int screen_width = 1920;
int screen_height=1080;

float prev_dist = 0;
float div_factor = 0.8;
int sub_factor = 500;
float acc_div_factor = 1000;

int minDist = 10;
int maxDist = 5000;
long frameCounter=0;

int target1 = 800;
int target2 = 200;

Serial myPort0;  // The serial port
Serial myPort1;

class acc_data {
   float x;
   float y;
   float z;
   boolean isValid;
   acc_data() {
     isValid=false;
   }
   
   acc_data subtract(acc_data b) {
     acc_data c = new acc_data();
     c.x = this.x - b.x;
     c.y = this.y - b.y;
     c.z = this.z - b.z;
     c.isValid = true;
     return c;
   }
   
   acc_data division(float divf) {
     acc_data c = new acc_data();
     c.x = this.x/divf;
     c.y = this.y/divf;
     c.z = this.z/divf;
     c.isValid = true;
     return c;
   }
};


void setup() {
  fullScreen();
  // List all the available serial ports
  background = createGraphics(screen_width,screen_height);
  foreground = createGraphics(screen_width,screen_height);
  foreground_imu = createGraphics(screen_width,screen_height);
  background.beginDraw();
  background.background(#000000);
  background.endDraw();
  printArray(Serial.list());
  myPort0 = new Serial(this, Serial.list()[0], 9600);
  myPort1 = new Serial(this, Serial.list()[1], 9600);
  frameRate(120);
  
}

void draw() {
  frameCounter++;
  String port0Str="";
  String port1Str="";
  while (myPort0.available() > 0) {
    int inByte = myPort0.read();
    port0Str = port0Str + (char)inByte;
    
  }
  //println("myPort0: " + port0Str);
  
  
  
  while (myPort1.available() > 0) {
    int inByte = myPort1.read();
    port1Str = port1Str + (char)inByte;
    //println("myPort1: " + inByte);
  }
  //println("myPort1: " + port1Str);
  
  //Both port0Str and port1Str will contain multiple lines. Parse them.
  float avgStr0 = findAvgUWBDist(port0Str);
  float avgStr1 = findAvgUWBDist(port1Str);
  
  float avgUWB = (avgStr0 + avgStr1)/2;
  if (avgUWB > 0)
  {
    float orig_dist = avgUWB;
    float plot_dist = (avgUWB-sub_factor)/div_factor;
    foreground.colorMode(RGB, 255,255,255,100);
    foreground.beginDraw();
    //foreground.clear();
    foreground.fill(orig_dist/8,orig_dist/8,255,10);
    foreground.stroke(orig_dist/8,orig_dist/8,255,10);
    foreground.strokeWeight(0);
    if (plot_dist > 0) {
      foreground.circle(screen_width/2, screen_height/2, plot_dist);
      prev_dist = plot_dist;
    } else {
      foreground.circle(screen_width/2, screen_height/2, prev_dist);
    }
    
    //Specify the targets
    foreground.fill(255,255,255,10);
    foreground.textSize(20);
    foreground.text("Target 1", screen_width/2, screen_height/2+target1/2+40);
    foreground.text("Target 2", screen_width/2, screen_height/2+target2/2+40);
    foreground.strokeWeight(2);
    foreground.noFill();
    foreground.stroke(255,255,255,10);
    foreground.circle(screen_width/2, screen_height/2, target2);
    foreground.noFill();
    foreground.stroke(255,255,255,10);
    foreground.circle(screen_width/2, screen_height/2, target1);
    
    
    //println("=========== " + plot_dist);
    foreground.endDraw();
    if (frameCounter%4==0) {
      framefader(foreground,15);
    }
  }
  
  acc_data myacc0 = findAvgAccVal(port0Str);
  acc_data myacc1 = findAvgAccVal(port1Str);
  acc_data diff_acc = myacc0.subtract(myacc1);
  
  foreground_imu.colorMode(RGB, 255,255,255,100);
  foreground_imu.beginDraw();
  acc_data small_acc = diff_acc.division(acc_div_factor);
  foreground_imu.fill(255-small_acc.x,255-small_acc.y,small_acc.z,10);
  foreground_imu.stroke(255,255,255,100);
  foreground_imu.strokeWeight(2);
  foreground_imu.circle(screen_width/4, screen_height/4, screen_height/8);
  
  //Target 1
  foreground_imu.textSize(20);
  foreground_imu.stroke(255,255,255,100);
  foreground_imu.fill(255,255,255,10);
  foreground_imu.text("Target 1", screen_width/4-screen_height/8-screen_height/8, screen_height/4-screen_height/8);
  foreground_imu.fill(221,221,4,10);
  foreground_imu.stroke(255,255,255,100);
  foreground_imu.strokeWeight(2);
  foreground_imu.circle(screen_width/4-screen_height/8, screen_height/4-screen_height/8, screen_height/16);
  
  //Target2
  foreground_imu.textSize(20);
  foreground_imu.stroke(255,255,255,100);
  foreground_imu.fill(255,255,255,10);
  foreground_imu.text("Target 2", screen_width/4-screen_height/8-screen_height/8, screen_height/4+screen_height/8);
  foreground_imu.fill(222,148,1,10);
  foreground_imu.stroke(255,255,255,100);
  foreground_imu.strokeWeight(2);
  foreground_imu.circle(screen_width/4-screen_height/8, screen_height/4+screen_height/8, screen_height/16);
  
  
  foreground_imu.endDraw();
  
  image(background,0,0);
  image(foreground,0,0);
  image(foreground_imu,0,0);
}

void framefader(PGraphics gr, int amt) {
  /*GraphicsContext ctx = ((Canvas) surface.getNative()).getGraphicsContext2D();
  GaussianBlur blur = new GaussianBlur();
  blur.setRadius(10);
  ctx.setEffect(blur);*/
  
  gr.beginDraw();
  gr.loadPixels();
  
  for (int i=0;i<gr.pixels.length;i++) {
    int alpha = (gr.pixels[i]>>24)& 0xFF;
    alpha = max(0, alpha-amt);
    gr.pixels[i]=alpha<<24 | ((gr.pixels[i])&0x00FFFFFF);
  }
  
  gr.updatePixels();
  gr.endDraw();
  
  
}


float findAvgUWBDist(String in) {
  double sum_dist = 0.0;
  int sum_counter = 0;
  String splitByNL[] = in.split("\n");
  for (int i=0;i<splitByNL.length;i++) {
    String uwbsplit[]={"0","0","0","0","0","0","0"};
    if (splitByNL[i].startsWith("1")) {
      uwbsplit = splitByNL[i].split(",");
      try{
        int reportedUWBDist = parseInt(uwbsplit[4]);
        if (reportedUWBDist>minDist && reportedUWBDist<maxDist) {
          sum_dist = sum_dist+reportedUWBDist;
          sum_counter++;
        }
      } catch (Exception ex) {
        ex.printStackTrace();
      }
    }
  }
  if (sum_counter>0) {
    return (float)(sum_dist/(double)sum_counter);
  } else {
    return 0.0;
  }
}


acc_data findAvgAccVal(String in) {
  try {
  
  double sum_acc_x = 0.0;
  double sum_acc_y = 0.0;
  double sum_acc_z = 0.0;
  int sum_counter = 0;
  String splitByNL[] = in.split("\n");
  for (int i=0;i<splitByNL.length;i++) {
    String uwbsplit[]={"0","0","0","0","0","0","0"};
    if (splitByNL[i].startsWith("i")) {
      
      uwbsplit = splitByNL[i].split(",");
      try{
        long reportedAccX = parseInt(uwbsplit[1]);
        long reportedAccY =  parseInt(uwbsplit[2]);
        long reportedAccZ =  parseInt(uwbsplit[3]);
        sum_acc_x += reportedAccX;
        sum_acc_y += reportedAccY;
        sum_acc_z += reportedAccZ;
        sum_counter++;
        
      } catch (Exception ex) {
        println(splitByNL[i]);
        ex.printStackTrace();
      }
    }
  }
  if (sum_counter>0) {
    acc_data mydata = new acc_data();
    mydata.x = (float)sum_acc_x/sum_counter;
    mydata.y = (float)sum_acc_y/sum_counter;
    mydata.z = (float)sum_acc_z/sum_counter;
    mydata.isValid = true;
    return (mydata);
  } else {
    acc_data mydata = new acc_data();
    mydata.x = 0.0;
    mydata.y = 0.0;
    mydata.z = 0.0;
    mydata.isValid = false;
    return (mydata);
  }
  }
  catch(Exception ex1) {
    ex1.printStackTrace();
    println(in);
    acc_data mydata = new acc_data();
    mydata.x = 0.0;
    mydata.y = 0.0;
    mydata.z = 0.0;
    mydata.isValid = false;
    return (mydata);
  }
}
