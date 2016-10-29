// This program looks for .csv files (requires NO spaces between data, only comma) within its directory
// and graphs the data. Graph function self scales both axis of the graph. The next .csv
// is selected upon mouse click, allowing you to scroll through the graphs of each .csv
// More info on rest of project located here: 
// https://hackaday.io/project/15425-rocket-real-time-transponder-and-gui
// 

import java.io.*;
int index = 0;
int[] altitude;
String inputDataFile = "";
String[] filenames;

// Setup filter to only look at files with .csv extensions
FilenameFilter csvFilter = new FilenameFilter() {
  public boolean accept(File dir, String name) {
    return name.toLowerCase().endsWith(".csv");
  }
};

void setup()   {
  String path = sketchPath();// +"/RocketNXP/data";
  filenames = listFileNames(path);
  size(1400, 1000);
}

void draw()  {
  Table altitudeData = loadTable(filenames[index]);
  int rowCount = altitudeData.getRowCount();
  altitude = new int[rowCount];
  // measure legth of table and put altitude column into array
  for (int i = 0; i < altitude.length; i++)  {
    altitude[i] = int(altitudeData.getInt(i, 4));    
  }
  
  int minimum = min(altitude);
  int maximum = max(altitude);
  long leftMargin = 70;  // margin adjustment for our graph
  long rightMargin =40;
  long bottomMargin = 100;
  long topMargin = 60;
  int hertz = 100; // sample measurement time
  int secondsSampled = (altitude.length)/hertz;  // number of samples/hertz 
  int timeAxisAdjustment = 10; 
  int heightAxisAdjustment = 100;
  int j = 0; // keep track of number of altitude lines

  background(0); // black
  noFill();
  stroke(75);
  line(leftMargin,(height -bottomMargin), leftMargin, topMargin); // line on left of x axis
  line(width - rightMargin,(height -bottomMargin), width - rightMargin, topMargin); //line of right of x axis
  
  // this draws the vertical graph lines (time markers)
  if(secondsSampled >= 1 && secondsSampled <= 30) { // not too many height marking lines, not too few
    timeAxisAdjustment = 1;                        // max 30 markers, (these are time) so 1 sec, 5 sec, 10 sec 60 sec intervals
  }
  else if(secondsSampled < 150) {
    timeAxisAdjustment = 5;
  }
  else if(secondsSampled < 300) {
    timeAxisAdjustment = 10;
  }  
  else if(secondsSampled < 1800) {
    timeAxisAdjustment = 60;
  }
  
  j = (secondsSampled/timeAxisAdjustment);
  for (int i = 0; i < j; i++) {  // time markers. 
    float y = map(i*(altitude.length/j), 0, altitude.length, leftMargin, width-rightMargin); 
    line(y, topMargin, y, height - bottomMargin); 
    text(i, y, height - 3*(bottomMargin/4));
  }

  // This draws height marker lines every heightAxisAdjustment feet of altitude
  if(maximum-minimum >= 1 && maximum-minimum <= 300) { // not too many time marking lines, not too few
    heightAxisAdjustment = 10;                         // keep it under 30 markers wide
  }  
  else if(maximum-minimum < 1500) {
    heightAxisAdjustment = 50;
  }
  else if(maximum-minimum < 3000) {
    heightAxisAdjustment = 100;
  }
  else if(maximum-minimum < 15000) {
    heightAxisAdjustment = 500;
  }
  else if(maximum-minimum < 50000) {
    heightAxisAdjustment = 1000;
  }

  stroke(75); // grey lines here
  for (int i = 0; i < ((maximum-minimum)/(heightAxisAdjustment)+1); i++) {  // sample markers. 
  //for (int i = 0; i < 30; i++) {  // sample markers. 
    float y = map(i, 0, ((maximum-minimum)/heightAxisAdjustment), height-bottomMargin, topMargin); 
    //float x = map(i, 0, ((maximum)/heightAxisAdjustment)-1, topMargin, height-bottomMargin); 
    text(i*heightAxisAdjustment,leftMargin - leftMargin/3,y); // draw text on axis for altitude values
    line(leftMargin, y, width-rightMargin, y); 
    println(maximum, minimum, heightAxisAdjustment);
    
  } 
    noFill();
    stroke(255, 0, 0);
    beginShape();
    for (int i = 0; i < altitude.length; i++)  {
      float x = map(i, 0, altitude.length-1, leftMargin, width-rightMargin);
      float y = map(altitude[i], minimum, maximum, height-bottomMargin, topMargin);
      vertex(x, y); 
    }

    endShape();
    // Print static axis labels
    textSize(22);
    text("Time in Seconds x "+ timeAxisAdjustment, width/2.5, height -(bottomMargin/2.5));
    pushMatrix(); //rotate our text
      translate(leftMargin/2,height - height/2.5);
      rotate(-HALF_PI);
      text("Altitude in Feet",0,0);
    popMatrix(); // unrotate
    textSize(18);
    text("File:     " + filenames[index], width/2.5, topMargin/2);  
    textSize(12);
}

  // This function returns all the files in a directory as an array of Strings
  // Its used to find all the .csv files in the diretory
String[] listFileNames(String dir) {
  File file = new File(dir);
  if (file.isDirectory()) {
    String names[] = file.list(csvFilter);
    return names;
  } else {
    // If it's not a directory
    return null;
  }
}  

// On mouse click, go to next csv file and graph the data for that csv file
void mousePressed() {
  if (index < (filenames.length)-1)  { 
      index = index+1;
  }
  else {
  index = 0;
  }
  redraw();
}
  