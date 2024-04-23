

PGraphics background;
PGraphics points;

int screen_width = 1920;
int screen_height=1080;
int side_width = 300;
//int skipLine = 0;

int all_values = 0;
int count = 0;

//VideoExport videoExport;
final String sketchname = getClass().getName();



int [][] define_centers = {{screen_width/2, screen_height/2},//torso
{screen_width/2 - screen_width/4, (screen_height/2 - screen_height/4) + screen_height/16},//left_wrist
{screen_width/2 + screen_width/4, (screen_height/2 - screen_height/4) - screen_height/16},//right_wrist
{screen_width/2 - screen_width/8, (screen_height/2 + screen_height/4) + screen_height/16},//left_ankle
{screen_width/2 + screen_width/8, (screen_height/2 + screen_height/4) - screen_height/16},//right_ankle
{screen_width/2, screen_height/2 - (screen_height/4 + screen_height/8)}};//head


int size_scaler = 300;
int color_scale = 3;
int circle_size;
int [] circle_color={0,0,0};
int max_color = 255;

int TORSO_DIST_INDEX = 0; //Note this starts from 0.
int HEAD_DIST_INDEX = 5;
int LEFT_WRIST_DIST_INDEX = 1;
int RIGHT_WRIST_DIST_INDEX = 2;
int LEFT_ANKLE_DIST_INDEX = 3;
int RIGHT_ANKLE_DIST_INDEX = 4;

int head_left_wrist;
int head_right_wrist;
int torso_left_wrist;
int torso_right_wrist;


int INIT_POS = 2;
int max_count = 0;
float jump_factor = 1;

boolean static_picture = true;

int SIMULTANEOUS_BUBBLES = 10;

int IMU_FACTOR = 1;
int max_count_acc = 0;
int max_count_dist = 0;
int count_imu = 0;


MyMoves newClass;
DistMatrixElement initMatrix;
int []color_val=new int[36];
float [][]acc_vals = new float[6][3];

void setup() {
  //size(1920,1080);
  fullScreen();
  //size(7680, 4320);
  
  background = createGraphics(screen_width,screen_height);
  points = createGraphics(screen_width,screen_height);
  frameRate(200);
  
  newClass = new MyMoves();
  newClass.setupPath(sketchPath(""));
  newClass.readAllData();
  //all_values = newClass.accDataStream.arr.size();
  //print("----------");
  //print(all_values);
  background.beginDraw();
  background.background(#000000);
  background.endDraw();
  
  
  int stream_count = newClass.allStreams.size();
  for (int istream = 0;istream<stream_count;istream++) {
    DataStream ds = newClass.allStreams.get(istream);
    max_count = ds.arr.size();
    if (ds.streamType == MyMoves.StreamType.ACC) {
      max_count_acc = ds.arr.size();
      ds.dataStats = new Stats();
      ds.dataStats.setDataStream(ds);
      ds.dataStats.setModePercentage(20,Stats.SpeedModes.FAST);
      ds.dataStats.setModePercentage(50,Stats.SpeedModes.MODERATE);
      ds.dataStats.setModePercentage(100,Stats.SpeedModes.ACCURATE);
      ds.dataStats.performStats(Stats.SpeedModes.FAST);
      println("Acc Median: " + ((AccElement)(ds.dataStats.getMedian(Stats.SpeedModes.FAST))).acc_x + ", Mean: " + ((AccElement)(ds.dataStats.getMean(Stats.SpeedModes.FAST))).acc_x + ", Var: " + ((AccElement)(ds.dataStats.getVariance(Stats.SpeedModes.FAST))).acc_x);
    }
    if (ds.streamType == MyMoves.StreamType.DIST) {
      max_count_dist = ds.arr.size();
    }
    jump_factor = (float)screen_width / (float)max_count;
    println("Max count: " + max_count + ", Jump Factor: " + jump_factor);

    if (ds.streamType == MyMoves.StreamType.DIST) {
      //The following demostrates how stats of distance can be obtained. 
      //This can be used, for example to determine limits and scaling.
      //Can also be used for sorting amongst the distance pairs based on obtained data.
      
      initMatrix = (DistMatrixElement)ds.arr.get(INIT_POS);
      ds.dataStats = new Stats();
      ds.dataStats.setDataStream(ds);
      ds.dataStats.setModePercentage(20,Stats.SpeedModes.FAST);
      ds.dataStats.setModePercentage(50,Stats.SpeedModes.MODERATE);
      ds.dataStats.setModePercentage(100,Stats.SpeedModes.ACCURATE);
      ds.dataStats.performStats(Stats.SpeedModes.FAST);
      for (int m=0;m<6;m++) {
        for (int n=0;n<6;n++) {
          println("Median: " + ((DistMatrixElement)ds.dataStats.getMedian(Stats.SpeedModes.FAST)).matrix.get(m).get(n).dist + ", Mean: " + ((DistMatrixElement)ds.dataStats.getMean(Stats.SpeedModes.FAST)).matrix.get(m).get(n).dist + ", Var:" + ((DistMatrixElement)ds.dataStats.getVariance(Stats.SpeedModes.FAST)).matrix.get(m).get(n).dist);
        }
      }
    }
  }
  if (max_count_dist>0)
  {
    IMU_FACTOR = max_count_acc/max_count_dist;
    println("IMU_FACTOR: " + IMU_FACTOR);
  }
  //println(sketchPath(""));
}



void draw() {
  count++;
  if (count<max_count || static_picture==false) {
  //for (int isimul=0;isimul<SIMULTANEOUS_BUBBLES;isimul++) {
    //count = (int)random(max_count);
    count_imu = IMU_FACTOR*count;
    //MyMoves newClass = new MyMoves();
    try {
    //MyMoves.printsomething("Hello " + random(100));
    } catch (Exception ex) {
      
    }
    points.colorMode(RGB, 255,255,255,100);
    points.beginDraw();
    
    points.noStroke();
    
    //print("--");
    //println(count);
    int stream_count = newClass.allStreams.size();
    //println("Stream Count: " + stream_count);
    for (int istream = 0;istream<stream_count;istream++) {
      DataStream ds = newClass.allStreams.get(istream);
      all_values = ds.arr.size();
      if (ds.streamType == MyMoves.StreamType.ACC) {
        AccElement ele = (AccElement)ds.arr.get(count_imu%all_values);
        //println("Size: " + abs((int)(size_scaler * (sqrt(ele.acc_x*ele.acc_x + ele.acc_y*ele.acc_y + ele.acc_z*ele.acc_z)-9.8))));
        //circle_size = abs((int)(size_scaler * (sqrt(ele.acc_x*ele.acc_x + ele.acc_y*ele.acc_y + ele.acc_z*ele.acc_z)-9.8)));
        if (ds.streamName.contains("head")) {
          acc_vals[HEAD_DIST_INDEX][0] = ele.acc_x;
          acc_vals[HEAD_DIST_INDEX][1] = ele.acc_y;
          acc_vals[HEAD_DIST_INDEX][2] = ele.acc_z;
        }
        if (ds.streamName.contains("left_wrist")) {
          acc_vals[LEFT_WRIST_DIST_INDEX][0] = ele.acc_x;
          acc_vals[LEFT_WRIST_DIST_INDEX][1] = ele.acc_y;
          acc_vals[LEFT_WRIST_DIST_INDEX][2] = ele.acc_z;
        }
        if (ds.streamName.contains("right_wrist")) {
          acc_vals[RIGHT_WRIST_DIST_INDEX][0] = ele.acc_x;
          acc_vals[RIGHT_WRIST_DIST_INDEX][1] = ele.acc_y;
          acc_vals[RIGHT_WRIST_DIST_INDEX][2] = ele.acc_z;
        }
        if (ds.streamName.contains("left_ankle")) {
          acc_vals[LEFT_ANKLE_DIST_INDEX][0] = ele.acc_x;
          acc_vals[LEFT_ANKLE_DIST_INDEX][1] = ele.acc_y;
          acc_vals[LEFT_ANKLE_DIST_INDEX][2] = ele.acc_z;
        }
        if (ds.streamName.contains("right_ankle")) {
          acc_vals[RIGHT_ANKLE_DIST_INDEX][0] = ele.acc_x;
          acc_vals[RIGHT_ANKLE_DIST_INDEX][1] = ele.acc_y;
          acc_vals[RIGHT_ANKLE_DIST_INDEX][2] = ele.acc_z;
        }
        if (ds.streamName.contains("torso")) {
          acc_vals[TORSO_DIST_INDEX][0] = ele.acc_x;
          acc_vals[TORSO_DIST_INDEX][1] = ele.acc_y;
          acc_vals[TORSO_DIST_INDEX][2] = ele.acc_z;
        }
          
      }
      /*
      //Color of the circle depends on the distance difference
      if (ds.streamType == MyMoves.StreamType.GYRO) {
        GyroElement ele = (GyroElement)ds.arr.get(count%all_values);
        circle_color[0] = min((int)(color_scaler*ele.gyro_x),max_color);
        circle_color[1] = min((int)(color_scaler*ele.gyro_y),max_color);
        circle_color[2] = min((int)(color_scaler*ele.gyro_z),max_color);
      }*/
      if (ds.streamType == MyMoves.StreamType.DIST) {
        DistMatrixElement matrixEle = (DistMatrixElement)ds.arr.get(count%all_values);
        int index_val = 0;
        for (int i=0;i<6;i++) {
          for(int j=i+1;j<6;j++) {
            color_val[index_val] = abs((matrixEle.matrix.get(i).get(j).dist - initMatrix.matrix.get(i).get(j).dist))/6;
            if (color_val[index_val]>100 && random(10)>9) {
              points.stroke(color(168,151,94));
              points.strokeWeight(1);
            } else if (color_val[index_val]<20) {
              points.stroke(color(50,50,50));
              points.strokeWeight(1);
            }
            else {
              points.noStroke();
            }
            
            //Add a tinge of color based on the acceleration
            float col_x = color_scale* (acc_vals[i][0]+acc_vals[j][0]);
            float col_y = color_scale* (acc_vals[i][1]+acc_vals[j][1]);
            float col_z = color_scale* (acc_vals[i][2]+acc_vals[j][2]);
            
            
            points.fill(color_val[index_val]+col_x, color_val[index_val]+col_y, color_val[index_val]+col_z,color_val[index_val]/10);
            points.circle((count*jump_factor)%screen_width,(screen_height/16)*(index_val+1)+random(30)-15,color_val[index_val]/2);
            
            index_val++;
          }
        }
      }
      
      
      /*
      //Where should this circle be drawn depends on the name of the stream, and distance
      //println(ds.streamName);
      int []where_plot = {screen_width/2, screen_height/2};
      if (ds.streamName.contains("head")) {
        where_plot[0] = define_centers[5][0];
        where_plot[1] = define_centers[5][1];
        //println(ds.streamName + "detected head");
        points.circle((where_plot[0]+count)%screen_width,where_plot[1],circle_size);
        points.fill(min(50,circle_color[0]), min(50,circle_color[1]), min(50,circle_color[2]),50);
      }
      if (ds.streamName.contains("left_wrist")) {
        where_plot[0] = define_centers[1][0];
        where_plot[1] = define_centers[1][1];
        //points.strokeWeight(10);
        //points.stroke(20);
        points.circle((where_plot[0]+count)%screen_width, where_plot[1], torso_left_wrist/10);
        points.fill(min(50,circle_color[0]), min(50,circle_color[1]), min(50,circle_color[2]),50);
        points.circle((where_plot[0]+count)%screen_width, where_plot[1]-(screen_height/16), head_left_wrist/10);
        points.fill(min(50,circle_color[0]), min(50,circle_color[1]), min(50,circle_color[2]),50);
        
      }
      if (ds.streamName.contains("right_wrist")) {
        where_plot[0] = define_centers[2][0];
        where_plot[1] = define_centers[2][1];
        //points.strokeWeight(10);
        //points.stroke(20);
        points.circle((where_plot[0]+count)%screen_width, where_plot[1], torso_right_wrist/10);
        points.fill(min(50,circle_color[0]), min(50,circle_color[1]), min(50,circle_color[2]),50);
        points.circle((where_plot[0]+count)%screen_width, where_plot[1]-(screen_height/16), head_right_wrist/10);
        points.fill(min(50,circle_color[0]), min(50,circle_color[1]), min(50,circle_color[2]),50);
      }
      if (ds.streamName.contains("left_ankle")) {
        where_plot[0] = define_centers[3][0];
        where_plot[1] = define_centers[3][1];
        //println(ds.streamName + "detected left_ankle");
        points.circle((where_plot[0]+count)%screen_width,where_plot[1],circle_size);
        points.fill(min(50,circle_color[0]), min(50,circle_color[1]), min(50,circle_color[2]),5);
      }
      if (ds.streamName.contains("right_ankle")) {
        where_plot[0] = define_centers[4][0];
        where_plot[1] = define_centers[4][1];
        //println(ds.streamName + "detected right_ankle");
        points.circle((where_plot[0]+count)%screen_width,where_plot[1],circle_size);
        points.fill(min(50,circle_color[0]), min(50,circle_color[1]), min(50,circle_color[2]),5);
      }
      if (ds.streamName.contains("torso")) {
        where_plot[0]=define_centers[0][0];
        where_plot[1]=define_centers[0][1];
        points.circle((where_plot[0]+count)%screen_width,where_plot[1],circle_size);
        points.fill(min(50,circle_color[0]), min(50,circle_color[1]), min(50,circle_color[2]),5);
      }
      */
      
      /*print("--Drawing Circle--\nAt: ");
      print(where_plot[0] + ", " + where_plot[1]);
      println("size: " + circle_size);
  */
      
      
      
    }
      
    
    /*
    
    AccElement ele = (AccElement)newClass.accDataStream.arr.get(count%all_values);
    points.circle(count%screen_width, (screen_height/6)+(10*ele.acc_x), 20);
    float r1 = random(255);
    points.fill(abs(10*ele.acc_x), r1, r1, 5);
    
    points.circle(count%screen_width, (screen_height/3)+(10*ele.acc_y), 20);
    float r2 = random(255);
    points.fill(r2, abs(10*ele.acc_y), r2, 5);
    
    points.circle(count%screen_width, (2*screen_height/3)+(10*ele.acc_z), 20);
    float r3 = random(255);
    points.fill(r3, r3, abs(10*ele.acc_z), 5);
    
    println((screen_height/2)+100*ele.acc_x);
    */
    
    points.endDraw();
    //if (count%5==0) {
       //framefader(points,1);
    //}
    image(background,0,0);    
    image(points,0,0);
  }
  
  
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
  
  
  //Attempted to blur, but too much work. So given up.
  /*
  long [][] pixelBuff = new long[points.pixelWidth][points.pixelHeight];
  int v = 1;
  int kernelsize = 3;
  int k_off = (kernelsize/2)-1;
  int [][] BlurMatrix ={{v, v, v}, {v, v, v}, {v, v, v}};
  for (int i=k_off;i<points.pixelWidth-k_off;i++) {
    for (int j=k_off;j<points.pixelHeight-k_off;j++) {
      //pixelBuff[i][j] = points.pixel[(j*points.pixelHeight)+i];
      for (int iMat=0;iMat<kernelsize;iMat++) {
        for(int jMat=0;jMat<kernelsize;jMat++) {
          sum += points.pixel[((j-(jMat+k_off)*points.pixelWidth)+i-(iMat+k_off)]*
  
  */
  gr.updatePixels();
  gr.endDraw();
  // do some drawing, anything drawn from here on will be blured
  // to disable the effect, do:
  //ctx.setEffect(null);
  
  
        }
      
