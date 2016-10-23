// This program looks at a .csv file (comma delimited with no spaces between data)
// and graphs the data for a model rocket. Size of the graph is adjustable, only have not taken into account
// resizing of text vs height and width of window. In this example, it is pulling 26,000 altitude samples
// that were actually taken while in a car driving up a hill. In this case, values at the bottom of the graph 
// are in 10 second intervals

int[] altitude;

void setup()   {
  size(1200, 900);
  //fullScreen();
  Table altitudeData = loadTable("1canada2.csv");
  int rowCount = altitudeData.getRowCount();
  altitude = new int[rowCount];
  for (int i = 0; i < altitude.length; i++)  {
    altitude[i] = int(altitudeData.getInt(i, 4));    
  }
}

void draw()  {
  int minimum = min(altitude);
  int maximum = max(altitude);
  long leftMargin = 70;  // margin adjustment for our graph
  long rightMargin =40;
  long bottomMargin = 100;
  long topMargin = 60;
  int hertz = 100; // sample measurement time
  int secondsSampled = (altitude.length)/hertz;  // number of samples/hertz 
  int aestheticsAdjustment = 10; // not too many time marking lines, not too few
  int j = 0; // keep track of number of altitude lines

  background(0); // black
  noFill();
  stroke(75);
  line(leftMargin,(height -bottomMargin), leftMargin, topMargin); // line on left of x axis
  line(width - rightMargin,(height -bottomMargin), width - rightMargin, topMargin); //line of right of x axis
  // this draws the vertical graph lines (time markers)
  j = (secondsSampled/aestheticsAdjustment);
  for (int i = 0; i < j; i++) {  // time markers. 
    float y = map(i*(altitude.length/j), 0, altitude.length, leftMargin, width-rightMargin); 
    line(y, topMargin, y, height - bottomMargin); 
    text(i, y, height - 3*(bottomMargin/4));
  }

  // This draws height marker lines every 100 feet of altitude
  stroke(75); // grey lines here
  j = (maximum/100)-1;
  for (int i = 0; i < maximum/100; i++) {  // sample markers. 
    float x = map(i, 0, (maximum/100)-1, topMargin, height-bottomMargin); 
    line(leftMargin, x, width-rightMargin, x); 
  } 
    noFill();
    stroke(255, 0, 0);
    beginShape();
    for (int i = 0; i < altitude.length; i++)  {
      float x = map(i, 0, altitude.length-1, leftMargin, width-rightMargin);
      float y = map(altitude[i], minimum, maximum, height-bottomMargin, topMargin);
      vertex(x, y); 
      float z = map(maximum, minimum, maximum, height-bottomMargin, topMargin); 
      text(maximum-minimum,leftMargin - leftMargin/3,z); // Draw apogee height at top of y axis at middle of leftMargin
    }
    // This draws text altitude markers for each of the altitude marking lines
    for (int i = 0; i < j; i++) {
      float z = map(i, 0, j, height-bottomMargin, topMargin);
      text(i*100,leftMargin - leftMargin/3, z);
    }
    endShape();
    // Print static axis labels
    textSize(22);
    text("Time in Seconds x "+ aestheticsAdjustment, width/2.5, height -(bottomMargin/2.5));
    pushMatrix(); //rotate our text
      translate(leftMargin/2,height - height/2.5);
      rotate(-HALF_PI);
      text("Altitude in Feet",0,0);
    popMatrix();
    textSize(12);
}
  
  