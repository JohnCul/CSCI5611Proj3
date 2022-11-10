int score = 0;
Vec2[] roots = new Vec2[2];
float[] lengths = new float[8];
float[] angles = new float[8];
float[] angleClamps = new float[8];
float[] angleMaxes = new float[8];
float[] angleMins = new float[8];
Vec2 post1Pos = new Vec2(50, 40);
Vec2 post2Pos = new Vec2(590, 40);

ArrayList<SoccerBall> balls = new ArrayList<SoccerBall>();
int time = 0;

// define lengths, maxes, mins, angular velo, angles
void setup(){
  size(640,480);
  surface.setTitle("Project3: Save the Soccer Balls with Inverse Kinematics");
  // root 1
  roots[0] = new Vec2(275, 100);
  // upper arm 1
  lengths[0] = 50.0;
  angles[0] = PI / 2;
  angleClamps[0] = 0.04;
  angleMaxes[0] = 7 * PI/8;
  angleMins[0] = 2 * PI/8;
  
  //lower arm 1
  lengths[1] = 30.0;
  angles[1] = 0.3;
  angleClamps[1] = 0.04;
  angleMaxes[1] = 0;
  angleMins[1] = -5 * PI / 8;
  
  // wrist on arm 1
  lengths[2] = 15;
  angles[2] = 0.3; //Wrist joint
  angleClamps[2] = .1;
  angleMaxes[2] = 0;
  angleMins[2] = -PI / 6;
  
  //hand arm 1
  lengths[3] = 15.0;
  angles[3] = 0.3; //Wrist joint
  angleClamps[3] = .5;
  angleMaxes[3] = PI/4;
  angleMins[3] = -PI / 4;
  
  //arm 2 and root 2
  roots[1] = new Vec2(325, 100);
  lengths[4] = 50.0;
  angles[4] = PI / 2;
  angleClamps[4] = 0.04;
  angleMaxes[4] = 6 * PI/8;
  angleMins[4] = 1 * PI/8;
  
  //lower arm 2
  lengths[5] = 30.0;
  angles[5] = PI / 2;
  angleClamps[5] = 0.04;
  angleMaxes[5] = 5 * PI / 8;
  angleMins[5] = 0;
  
  // wrist on arm 2
  lengths[6] = 15;
  angles[6] = PI / 12; 
  angleClamps[6] = .1;
  angleMaxes[6] = PI / 6;
  angleMins[6] = 0;
  
  //hand arm 2
  lengths[7] = 15.0;
  angles[7] = PI / 16;
  angleClamps[7] = .5;
  angleMaxes[7] = PI / 4;
  angleMins[7] = -PI/4;
  
}




Vec2 start_l1,start_l2, start_l3,endPoint1;
Vec2 start_l4,start_l5, start_l6,endPoint2;

void solve(){
  Vec2 goal = new Vec2(mouseX, mouseY);
  
  Vec2 startToGoal, startToEndEffector;
  float dotProd, angleDiff;
  
  for(int i = 3; i >= 0; i--){
    if(i == 3){
      startToGoal = goal.minus(start_l3);
      startToEndEffector = endPoint1.minus(start_l3);
    }else if(i == 2){
      startToGoal = goal.minus(start_l2);
      startToEndEffector = endPoint1.minus(start_l2);
    }else if(i == 1){
      startToGoal = goal.minus(start_l1);
      startToEndEffector = endPoint1.minus(start_l1);
    }else{
      startToGoal = goal.minus(roots[0]);
      startToEndEffector = endPoint1.minus(roots[0]);
    }
    dotProd = dot(startToGoal.normalized(),startToEndEffector.normalized());
    dotProd = clamp(dotProd,-1,1);
    angleDiff = clamp(acos(dotProd), -angleClamps[i], angleClamps[i]);
    if (cross(startToGoal,startToEndEffector) < 0)
      angles[i] += angleDiff;
    else
      angles[i] -= angleDiff;
  
    if(angles[i]> angleMaxes[i]){
      angles[i] = angleMaxes[i];
    }else if(angles[i] < angleMins[i]){
      angles[i] = angleMins[i];
    }
    //angles[i] = angles[i] % PI;
    /*TODO: Wrist joint limits here*/
    fk(); //Update link positions with fk (e.g. end effector changed)
  }
  goal = new Vec2(mouseX, mouseY);
  startToGoal = new Vec2(0,0);
  startToEndEffector = new Vec2(0,0);
  for(int i = 7; i >= 4; i--){
    if(i == 7){
      startToGoal = goal.minus(start_l6);
      startToEndEffector = endPoint2.minus(start_l6);
    }else if(i == 6){
      startToGoal = goal.minus(start_l5);
      startToEndEffector = endPoint2.minus(start_l5);
    }else if(i == 5){
      startToGoal = goal.minus(start_l4);
      startToEndEffector = endPoint2.minus(start_l4);
    }else{
      startToGoal = goal.minus(roots[1]);
      startToEndEffector = endPoint2.minus(roots[1]);
    }
    dotProd = dot(startToGoal.normalized(),startToEndEffector.normalized());
    dotProd = clamp(dotProd,-1,1);
    angleDiff = clamp(acos(dotProd), -angleClamps[i], angleClamps[i]);
    if (cross(startToGoal,startToEndEffector) < 0)
      angles[i] += angleDiff;
    else
      angles[i] -= angleDiff;
  
    if((angles[i] % PI )> angleMaxes[i]){
      angles[i] = angleMaxes[i];
    }else if((angles[i] % PI ) < angleMins[i]){
      angles[i] = angleMins[i];
    }
    /*TODO: Wrist joint limits here*/
    fk(); //Update link positions with fk (e.g. end effector changed)
  }
  
 
  //println("Angle 0:",angles[4],"Angle 1:",angles[5],"Angle 2:",angles[6],"Angle 3:",angles[7]);
}

void fk(){
  //arm 1
  start_l1 = new Vec2(cos(angles[0])*lengths[0],sin(angles[0])*lengths[0]).plus(roots[0]);
  start_l2 = new Vec2(cos(angles[0]+angles[1])*lengths[1],sin(angles[0]+angles[1])*lengths[1]).plus(start_l1);
  start_l3 = new Vec2(cos(angles[0]+angles[1]+angles[2])*lengths[2],sin(angles[0]+angles[1]+angles[2])*lengths[2]).plus(start_l2);
  endPoint1 = new Vec2(cos(angles[0]+angles[1]+angles[2] + angles[3])*lengths[3],sin(angles[0]+angles[1]+angles[2] + angles[3])*lengths[3]).plus(start_l3);
  
  // arm 2
  start_l4 = new Vec2(cos(angles[4])*lengths[4],sin(angles[4])*lengths[4]).plus(roots[1]);
  start_l5 = new Vec2(cos(angles[4]+angles[5])*lengths[5],sin(angles[4]+angles[5])*lengths[5]).plus(start_l4);
  start_l6 = new Vec2(cos(angles[4]+angles[5]+angles[6])*lengths[6],sin(angles[4]+angles[5]+angles[6])*lengths[6]).plus(start_l5);
  endPoint2 = new Vec2(cos(angles[4]+angles[5]+angles[6] + angles[7])*lengths[7],sin(angles[4]+angles[5]+angles[6] + angles[7])*lengths[7]).plus(start_l6);
  
}

float armW = 20;
void draw(){
  updateRoots();
  fk();
  solve();
  time += 1;
  
  if(time % 90 == 0 && !paused){
    balls.add(new SoccerBall(random(10,630)));
  }
  
  background(0,250,0);
  
  //field lines
  fill(255,255,255);
  pushMatrix();
  noStroke();
  translate(100,40);
  rotate(PI / 2);
  rect(0, -armW/2, 200, armW / 2);
  popMatrix();
  pushMatrix();
  noStroke();
  translate(540,40);
  rotate(PI / 2);
  rect(0, -armW/2, 200, armW / 2);
  popMatrix();
  pushMatrix();
  noStroke();
  translate(100,240);
  rect(0, -armW/2, 440, armW / 2);
  popMatrix();
  pushMatrix();
  noStroke();
  translate(0,40);
  rect(0, -armW/2, 680, armW / 2);
  popMatrix();
  pushMatrix();
  noStroke();
  circle(320,420,50);
  popMatrix();
 
  
  stroke(1);
  //torso
  fill(200,0,0);
  pushMatrix();
  translate(roots[0].x - 7,roots[0].y - 10);
  rect(0, -armW/2, 60, 3*armW/2);
  popMatrix();
  
  //arm 1
  fill(200,0,0);
  pushMatrix();
  translate(roots[0].x,roots[0].y);
  rotate(angles[0]);
  rect(0, -armW/2, lengths[0], armW);
  popMatrix();
  
  pushMatrix();
  translate(start_l1.x,start_l1.y);
  rotate(angles[0]+angles[1]);
  rect(0, -armW/2, lengths[1], armW);
  popMatrix();
  fill(210,180,140);
  pushMatrix();
  translate(start_l2.x,start_l2.y);
  rotate(angles[0]+angles[1]+angles[2]);
  rect(0, -armW/2, lengths[2], armW);
  popMatrix();
  fill(255,20,147);
  pushMatrix();
  translate(start_l3.x,start_l3.y);
  rotate(angles[0]+angles[1]+angles[2] + angles[3]);
  rect(0, -armW/2, lengths[3], armW);
  popMatrix();
  
  //arm 2
  
  fill(200,0,0);
  pushMatrix();
  translate(roots[1].x,roots[1].y);
  rotate(angles[4]);
  rect(0, -armW/2, lengths[4], armW);
  popMatrix();
  
  pushMatrix();
  translate(start_l4.x,start_l4.y);
  rotate(angles[4]+angles[5]);
  rect(0, -armW/2, lengths[5], armW);
  popMatrix();
  fill(210,180,140);
  pushMatrix();
  translate(start_l5.x,start_l5.y);
  rotate(angles[4]+angles[5]+angles[6]);
  rect(0, -armW/2, lengths[6], armW);
  popMatrix();
  fill(255,20,147);
  pushMatrix();
  translate(start_l6.x,start_l6.y);
  rotate(angles[4]+angles[5]+angles[6] + angles[7]);
  rect(0, -armW/2, lengths[7], armW);
  popMatrix();
  
    //head
  fill(0,0,0);
  pushMatrix();
  //translate(roots[0].x,roots[0].y);
  //rotate(0);
  circle(roots[0].x + 23,roots[0].y, 35);
  popMatrix();
  
  //draw the soccer balls
  for(int i = 0; i < balls.size(); i++){
    if(!paused){
      balls.get(i).update();
    }
    if(balls.get(i).position.y < 40 - balls.get(i).radius / 2 && balls.get(i).position.x > 70 && balls.get(i).position.x < 570){
      // ball hits goal: pause the game and end it
      paused = true;
    }
    //check if the ball is near enough to any of the end effectors
    if((balls.get(i).position.distanceTo(endPoint1) < balls.get(i).radius/2 || balls.get(i).position.distanceTo(endPoint2) < balls.get(i).radius/2 ) && !paused && !balls.get(i).saved){
      score++;
      //balls.get(i).position.y += balls.get(i).position.distanceTo(endPoint1) / 2;
      balls.get(i).velocity.y *= -1;
      balls.get(i).saved = true;
      fill(255,255,255);
      pushMatrix();
      //image(img, balls.get(i).position.x, balls.get(i).position.y, balls.get(i).radius, balls.get(i).radius);
      circle(balls.get(i).position.x, balls.get(i).position.y, balls.get(i).radius);
      popMatrix();
      pushMatrix();
      fill(0,0,0);
      translate(balls.get(i).position.x, balls.get(i).position.y);
      rotate(frameCount / balls.get(i).rotation);
      polygon(0, 0, 8, 5);  // Pentagon
      popMatrix();
      fill(255,255,255);
    }else if(balls.get(i).position.y > 500 || balls.get(i).position.y < 0){
      balls.remove(i);
    }else if(balls.get(i).position.distanceTo(post1Pos) < balls.get(i).radius/2 && !paused){
      
      float oldMagnitude = balls.get(i).velocity.length();
      Vec2 between = balls.get(i).position.minus(post1Pos);
      between.normalize();
      balls.get(i).velocity.y = between.y * oldMagnitude;
      balls.get(i).velocity.x = between.x * oldMagnitude;
      balls.get(i).position = balls.get(i).position.plus(between);
      fill(255,255,255);
      pushMatrix();
      //image(img, balls.get(i).position.x, balls.get(i).position.y, balls.get(i).radius, balls.get(i).radius);
      circle(balls.get(i).position.x, balls.get(i).position.y, balls.get(i).radius);
      popMatrix();
      pushMatrix();
      fill(0,0,0);
      translate(balls.get(i).position.x, balls.get(i).position.y);
      rotate(frameCount / balls.get(i).rotation);
      polygon(0, 0, 8, 5);  // Pentagon
      popMatrix();
      fill(255,255,255);
    }else if(balls.get(i).position.distanceTo(post2Pos) < balls.get(i).radius/2 && !paused){
      float oldMagnitude = balls.get(i).velocity.length();
      Vec2 between = balls.get(i).position.minus(post2Pos);
      between.normalize();
      balls.get(i).velocity.y = between.y * oldMagnitude;
      balls.get(i).velocity.x = between.x * oldMagnitude;
      balls.get(i).position = balls.get(i).position.plus(between);
      fill(255,255,255);
      pushMatrix();
      //image(img, balls.get(i).position.x, balls.get(i).position.y, balls.get(i).radius, balls.get(i).radius);
      circle(balls.get(i).position.x, balls.get(i).position.y, balls.get(i).radius);
      popMatrix();
      pushMatrix();
      fill(0,0,0);
      translate(balls.get(i).position.x, balls.get(i).position.y);
      rotate(frameCount / balls.get(i).rotation);
      polygon(0, 0, 8, 5);  // Pentagon
      popMatrix();
      fill(255,255,255);
    }else{
      fill(255,255,255);
      pushMatrix();
      //image(img, balls.get(i).position.x, balls.get(i).position.y, balls.get(i).radius, balls.get(i).radius);
      circle(balls.get(i).position.x, balls.get(i).position.y, balls.get(i).radius);
      popMatrix();
      pushMatrix();
      fill(0,0,0);
      translate(balls.get(i).position.x, balls.get(i).position.y);
      rotate(frameCount / balls.get(i).rotation);
      polygon(0, 0, 8, 5);  // Pentagon
      popMatrix();
      fill(255,255,255);
    }
  }
  strokeWeight(1);
    //goal
  fill(255,255,255);
  pushMatrix();
  noStroke();
  translate(50,0);
  rotate(PI/2);
  rect(0, -armW/2, 40, armW / 2);
  popMatrix();
  pushMatrix();
  noStroke();
  translate(590,0);
  rotate(PI/2);
  rect(0, -armW/2, 40, armW / 2);
  popMatrix();
  for(int i = 1; i < 10; i++){
    pushMatrix();
    noStroke();
    translate(50,4 * i);
    //rotate(PI/2);
    rect(0, -armW/8, 540, 2);
    popMatrix();
  }
  for(int i = 1; i < 40; i++){
    pushMatrix();
    noStroke();
    translate(50 + 13.4 * i,0);
    rotate(PI/2);
    rect(0, -armW/8, 40, 2);
    popMatrix();
  }
  pushMatrix();
  noStroke();
  translate(50,40);
  rect(0, -armW/2, 640, armW / 2);
  popMatrix();
  
  textSize(30);
  fill(0, 0, 0);
  text("score: " + score, 500, 440); 
  
  if(paused){
    pushMatrix();
    fill(0,0,0,150);
    rect(0,0,640,480);
    popMatrix();
    textSize(100);
    fill(255, 0, 0);
    text("GAME OVER", 70, 200); 
    text("" + score, 310, 300); 
    textSize(30);
    text("Press r to Play Again", 200, 370);
  }
  
}

boolean leftPressed, rightPressed, upPressed, downPressed, shiftPressed, paused;
void keyPressed(){
  if (keyCode == LEFT || key == 'a') leftPressed = true;
  if (keyCode == RIGHT || key=='d') rightPressed = true;
  if (keyCode == UP || key=='w') upPressed = true; 
  if (keyCode == DOWN || key=='s') downPressed = true;
  if (keyCode == SHIFT) shiftPressed = true;
  if (key == 'r'){
    //restart the game
    balls = new ArrayList<SoccerBall>();
    roots[0] = new Vec2(275, 100);
    roots[1] = new Vec2(325, 100);
    score = 0;
    paused = false;
    //pos = new Vec2[maxParticles];
    //vel = new Vec2[maxParticles];
    //numParticles = 0;
    //if(!paused){  
    //  paused = !paused;
    //}
  }
}

void keyReleased(){
  if (keyCode == LEFT || key == 'a') leftPressed = false;
  if (keyCode == RIGHT || key=='d') rightPressed = false;
  if (keyCode == UP || key=='w') upPressed = false; 
  if (keyCode == DOWN || key=='s') downPressed = false;
  if (keyCode == SHIFT) shiftPressed = false;
}


void updateRoots(){
  float movement = 2;
  if(shiftPressed){
    movement = 4;
  }
  if(leftPressed && roots[0].x - movement > 0 && !paused){
    roots[0].x -= movement;
    roots[1].x -= movement;
  }
  if(rightPressed && roots[0].x + movement < 550 && !paused){
    roots[0].x += movement;
    roots[1].x += movement;
  }
  if(downPressed && roots[0].y + movement < 440 && !paused){
    roots[0].y += movement;
    roots[1].y += movement;
  }
  if(upPressed && roots[0].y - movement > 0 && !paused){
    roots[0].y -= movement;
    roots[1].y -= movement;    
  }
  
}

void polygon(float x, float y, float radius, int npoints) {
  float angle = TWO_PI / npoints;
  beginShape();
  for (float a = 0; a < TWO_PI; a += angle) {
    float sx = x + cos(a) * radius;
    float sy = y + sin(a) * radius;
    vertex(sx, sy);
  }
  endShape(CLOSE);
}
