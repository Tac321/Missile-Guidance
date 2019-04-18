// Shawn Daniel //<>//


// Purpose: This is the main objec file used in painting the entire missile/target simulation.

class Pendulum {
  private PVector translation, rotation, Grad,Gradr, Gradl,Gradn, Om;  
  private PVector arm, arm2, arm3, arm4, thr, GradFrc;
  private PVector GradFrc2, GradFrc3, GradFrc4;
  private PVector finFrc1, finFrc2, finFrc3, finFrc4;
  private PVector[] baseJoint; //  Yes.
  private float baseRadius;  // Yes
  private float cg_displ, fuse_displ;
  private float fuse_mass_radius;
  private float finSpan=3;
  private PVector erquatErr;
  private PVector finwing, finwing4;
  private float largo;
  private PVector targ;
  private PVector euquat, eurwquat, eumrquat; 
  private PVector erquat;
  private PVector b0, brw0, bmr0, uu, uuu;
  private PVector b0m1;
  private PVector desOm, desOmm1, desOmDot ;
  // Seeker frame code

  private float ckey1= 0;
  private float ckey2= 0;
  private final float finAngles[] ={  //
   90, -90, -90, -90};
  private final float controllerf[] ={
    0,0,0,0}; 

  // Dynamics parameters.
  private final float SCALE_BASE_RADIUS = 50.0; 
  private final float SCALE_CG_Z_DISPL = 16; // 
  private final float SCALE_FUSE_DISPL = 25; // 
  
  public Pendulum(float s) {   
    // s=1.  The relative scaleeters

    Om = new PVector(); 
    finwing = new PVector();
    finwing4 = new PVector();

    euquat = new PVector(1, 0, 0); 
    eurwquat = new PVector(0, 1, 0); 
    eumrquat = new PVector(0, 0, 1);
    erquatErr = new PVector(1, 0, 0);
    b0 = new PVector();
    brw0 = new PVector(0, 1, 0);
    bmr0 = new PVector(0, 0, 1);
    uu = new PVector (0, 0, 0);
    uuu= new PVector(0, 0, 0);
    erquat = new PVector(1, 0, 0);
    b0m1 = new PVector(1,0,0);
    
    translation = new PVector(); 
    rotation = new PVector(); 
    Grad = new PVector();  // bon
    Gradr = new PVector(); 
    Gradl = new PVector(); 
    Gradn = new PVector();
    arm = new PVector();
    arm2 = new PVector();
    arm3 = new PVector();
    arm4 = new PVector();
    thr = new PVector();
    GradFrc= new PVector();
    GradFrc2= new PVector();
    GradFrc3= new PVector();
    GradFrc4= new PVector();
    finFrc1= new PVector();
    finFrc2= new PVector();
    finFrc3= new PVector();
    finFrc4= new PVector();
    targ=  new PVector();
    desOm = new PVector(0,0,0);
    desOmm1 = new PVector(0,0,0);
    desOmDot = new PVector(0,0,0); 
    baseJoint = new PVector[1];  
    baseRadius = s*SCALE_BASE_RADIUS;
    cg_displ    = s*SCALE_CG_Z_DISPL ;
    fuse_displ  = s*SCALE_FUSE_DISPL ;
    fuse_mass_radius=3.5;
    largo=64; 
    baseJoint[0] = new PVector(0, 0, 0); 
    calcQ(); 
  }  // End of the platform object.  

// Variables called in from the main file for use in painting the main missile sim.
  public void applyTranslationAndRotation(PVector t, PVector r, PVector Gradinertial, PVector q, PVector zz, PVector aa, PVector rr, PVector ll, PVector nn, PVector mm, PVector mm1,PVector mm2,PVector mm3,PVector F1,PVector F2,PVector F3,PVector F4,PVector ttt, PVector gradF, PVector gradF2, PVector gradF3, PVector gradF4, PVector targz) {   
    rotation.set(r);
    translation.set(t); 
    Grad.set(Gradinertial);
    Gradr.set(rr);
    Gradl.set(ll);
    Gradn.set(nn);
    arm.set(mm);
    arm2.set(mm1);
    arm3.set(mm2);
    arm4.set(mm3);
    thr.set(ttt);
    GradFrc.set(gradF);
    GradFrc2.set(gradF2);
    GradFrc3.set(gradF3);
    GradFrc4.set(gradF4);
    targ.set(targz);
    
    if(ckey1>1.0){ 
        b0m1.x= b0.x;
        b0m1.y= b0.y;
        b0m1.z= b0.z;
    }
    b0.set(q);
    
    finFrc1.set(F1);
    finFrc2.set(F2);
    finFrc3.set(F3);
    finFrc4.set(F4);
    finwing.set(zz);  
    finwing4.set(aa); 

    float b0Norm= sqrt (b0.x*b0.x + b0.y*b0.y + b0.z*b0.z);
    b0.x /= (float)b0Norm;
    b0.y /= (float)b0Norm;
    b0.z /= (float)b0Norm;
    
    // Update old desOm. For Feed Forward controller.
   if(ckey2>1.0){
      desOmm1.y = desOm.y;
      desOmm1.z = desOm.z;
    }

    uu.x= b0.y; 
    uu.y= -b0.x;
    uu.z= 0;
    float uuNorm= sqrt(uu.x*uu.x + uu.y*uu.y + uu.z*uu.z);
    uu.x /=uuNorm;
    uu.y /=uuNorm;
    uu.z /=uuNorm;
    uuu.x= euquat.y;
    uuu.y= -euquat.x;
    uuu.z= 0;
    float uuuNorm= sqrt(uuu.x*uuu.x + uuu.y*uuu.y + uuu.z*uuu.z);
    uuu.x /=uuuNorm;
    uuu.y /=uuuNorm;
    uuu.z /=uuuNorm;
    bmr0.x = uu.x*uu.x*b0.x + (uu.x*uu.y-uu.z)*b0.y + (uu.x*uu.z+uu.y)*b0.z;
    bmr0.y = (uu.x*uu.y+uu.z)*b0.x + uu.y*uu.y*b0.y + (uu.y*uu.z-uu.x)*b0.z;
    bmr0.z = (uu.x*uu.z-uu.y)*b0.x + (uu.y*uu.z+uu.x)*b0.y + uu.z*uu.z*b0.z;
    brw0.x = -1*(-b0.z*bmr0.y + b0.y*bmr0.z);
    brw0.y =  -1*(b0.z*bmr0.x - b0.x*bmr0.z);
    brw0.z = -1*(-b0.y*bmr0.x + b0.x*bmr0.y);
    
    float temp1x = uuu.x*uuu.x*euquat.x + (uuu.x*uuu.y-uuu.z)*euquat.y + (uuu.x*uuu.z+uuu.y)*euquat.z;
    float temp1y = (uuu.x*uuu.y+uuu.z)*euquat.x + uuu.y*uuu.y*euquat.y + (uuu.y*uuu.z-uuu.x)*euquat.z;
    float temp1z = (uuu.x*uuu.z-uuu.y)*euquat.x + (uuu.y*uuu.z+uuu.x)*euquat.y + uuu.z*uuu.z*euquat.z;

    eumrquat.x = (euquat.x*euquat.x*(1-cos(-rotation.x))+cos(-rotation.x))*temp1x + (euquat.x*euquat.y*(1-cos(-rotation.x))-euquat.z*sin(-rotation.x))*temp1y + (euquat.x*euquat.z*(1-cos(-rotation.x))+euquat.y*sin(-rotation.x))*temp1z ;   
    eumrquat.y = (euquat.x*euquat.y*(1-cos(-rotation.x))+euquat.z*sin(-rotation.x))*temp1x + (euquat.y*euquat.y*(1-cos(-rotation.x))+cos(-rotation.x))*temp1y + (euquat.y*euquat.z*(1-cos(-rotation.x))-euquat.x*sin(-rotation.x))*temp1z ;   
    eumrquat.z = (euquat.x*euquat.z*(1-cos(-rotation.x))-euquat.y*sin(-rotation.x))*temp1x + (euquat.y*euquat.z*(1-cos(-rotation.x))+euquat.x*sin(-rotation.x))*temp1y + (euquat.z*euquat.z*(1-cos(-rotation.x))+cos(-rotation.x))*temp1z ;

    float eumrNorm = sqrt(eumrquat.x*eumrquat.x+eumrquat.y*eumrquat.y+eumrquat.z*eumrquat.z);
    float eurwNorm = sqrt(eurwquat.x*eurwquat.x+eurwquat.y*eurwquat.y+eurwquat.z*eurwquat.z);

    eumrquat.x /= eumrNorm;
    eumrquat.y /= eumrNorm;
    eumrquat.z /= eumrNorm;

    eurwquat.x /= eurwNorm;
    eurwquat.y /= eurwNorm;
    eurwquat.z /= eurwNorm;

    eurwquat.x = (-euquat.y*eumrquat.z + euquat.z*eumrquat.y);
    eurwquat.y = ( euquat.x*eumrquat.z - euquat.z*eumrquat.x);
    eurwquat.z = (-euquat.x*eumrquat.y + euquat.y*eumrquat.x);

    // Renormalize.
    eurwNorm = sqrt(eurwquat.x*eurwquat.x+eurwquat.y*eurwquat.y+eurwquat.z*eurwquat.z);
    eurwquat.x /= eurwNorm;
    eurwquat.y /= eurwNorm;
    eurwquat.z /= eurwNorm; 
    
    calcQ(); 
  }


  private void calcQ() {     
    for (int i=0; i<1; i++) { 
      controllerf[0] = -finwing.z;
      controllerf[1] = finwing4.x; 
      controllerf[2] = finwing.x;
      controllerf[3] = -finwing.y;
    }
    quatRotOpsVecProdF(); 
  }
  private void quatRotOpsVecProdF() {  

      euquat.x = cos(rotation.z)*cos(rotation.y);
      euquat.y = -sin(rotation.z)*cos(rotation.y);
      euquat.z = sin(rotation.y); 
      erquat.x= euquat.x*b0.x  + euquat.y*b0.y  + euquat.z*b0.z;
      erquat.y= euquat.x*brw0.x+euquat.y*brw0.y+euquat.z*brw0.z;
      erquat.z= euquat.x*bmr0.x+euquat.y*bmr0.y+euquat.z*bmr0.z;
      
      if(ckey1<1.0){  //
          b0m1.x= b0.x; 
          b0m1.y= b0.y;
          b0m1.z= b0.z;
      }
      ckey1 = 2.0; 

      float b0bx= (euquat.x*b0.x + -eurwquat.x*b0.y + eumrquat.x*b0.z); 
      float b0by= (euquat.y*b0.x + -eurwquat.y*b0.y + eumrquat.y*b0.z);
      float b0bz= (euquat.z*b0.x + -eurwquat.z*b0.y + eumrquat.z*b0.z);
      
      float b0m1bx= (euquat.x*b0m1.x + -eurwquat.x*b0m1.y + eumrquat.x*b0m1.z); 
      float b0m1by= (euquat.y*b0m1.x + -eurwquat.y*b0m1.y + eumrquat.y*b0m1.z);
      float b0m1bz= (euquat.z*b0m1.x + -eurwquat.z*b0m1.y + eumrquat.z*b0m1.z);

      float thetat2 = atan2(b0bz,sqrt(b0bx*b0bx + b0by*b0by)) ;
      float psit2 =   -atan2(b0by,b0bx)       ;
      float thetat3 = atan2(b0m1bz,sqrt(b0m1bx*b0m1bx + b0m1by*b0m1by)) ;
      float psit3 =   -atan2(b0m1by,b0m1bx)   ;

      desOm.y = (thetat2-thetat3)/.05; 
      desOm.z = (psit2-psit3)/.05;
      // Neccesary initialization.
      if(ckey2<1.0){
        desOmm1.y = desOm.y;
        desOmm1.z = desOm.z;
      }
      ckey2=2.0; 
      // Extract 
      desOmDot.y = (desOm.y-desOmm1.y)/.05; 
      desOmDot.z = (desOm.z-desOmm1.z)/.05;
  }

  public PVector getEuQuat() { 
    return euquat; 
  } // */

  public PVector getTargQuat() {  
    return b0;
  } // */

  public PVector getDesOmega(){
    return desOm;
  } // */
  
  public PVector getDesOmegaDot(){
    return desOmDot;
  } // */
  
  public PVector getErQuatError() {
    float pk0 = 0;  // Make zero for pure quaternion.
    float pk1= 1;  
    float pk2= 0;
    float pk3= 0;

    float q0 = 0; 
    float q1= -erquat.x;  
    float q2= -erquat.y;
    float q3= -erquat.z;

    float r0 = pk0*q0+-pk1*q1+-pk2*q2+-pk3*q3;  
    float r1 = pk1*q0+pk0*q1+-pk3*q2+pk2*q3;
    float r2 = pk2*q0+pk3*q1+pk0*q2+-pk1*q3;
    float r3 = pk3*q0+-pk2*q1+pk1*q2+pk0*q3;

    erquatErr.x= r1;  
    erquatErr.y= r2; 
    erquatErr.z= r3;

    return erquatErr; 
  } 
  public PVector getEurwquat(){
    return eurwquat; 
  } 
  public PVector getEumrquat(){
    return eumrquat; 
  } 

  // DRAW THE SYSTEM.
  //  Draw the entire missile/target simulation
  public void draw() {  

    pushMatrix();
      translate(0, 0, -7.5);
      stroke(200);
      strokeWeight(0.5);
      fill(50);
      ellipse(0, 0, 15*baseRadius, 15*baseRadius); 
      stroke(200);
      strokeWeight(0.5);
      translate(0, 0, 1);
      fill(25);
      ellipse(0, 0, 10*baseRadius, 10*baseRadius); 
      stroke(200);
      strokeWeight(0.5);
      translate(0, 0, 1);
      fill(10);
      ellipse(0, 0, 5*baseRadius, 5*baseRadius); 
      stroke(255,255,102);
      line(targ.x, targ.y, targ.z, translation.x, translation.y, translation.z);
      translate(targ.x, targ.y, targ.z);
      stroke(255,0,0);
      if(finwing4.z>0.0){
          sphere(5);    // Make the target laser spot visible.
      }
    popMatrix();


    // DRAW CURRENT FRAME OF A/C
    //-------------------------------//
    // Draw missile body coordinate frame.
    pushMatrix();
      strokeWeight(2);
      stroke(255);
      translate(-60, 0, 80);
      line(0, 0, 0, 30*eurwquat.x, 30*eurwquat.y, 30*eurwquat.z);
    popMatrix(); // */
    pushMatrix();
      strokeWeight(2);
      stroke(0, 255, 255);
      translate(-60, 0, 80);
      line(0, 0, 0, 30*eumrquat.x, 30*eumrquat.y, 30*eumrquat.z);
    popMatrix(); // */
    pushMatrix();
      strokeWeight(2);
      stroke(0, 255, 0);
      translate(-60, 0, 80);
      line(0, 0, 0, 50*euquat.x, 50*euquat.y, 50*euquat.z);
    popMatrix();
    //-------------------------------//

    // DRAW TARGET QUATERNION
    //-------------------------------//
    float b0Norm= sqrt (b0.x*b0.x + b0.y*b0.y + b0.z*b0.z);
    float brw0Norm= sqrt (brw0.x*brw0.x + brw0.y*brw0.y + brw0.z*brw0.z);
    float bmr0Norm= sqrt (bmr0.x*bmr0.x + bmr0.y*bmr0.y + bmr0.z*bmr0.z);
    b0.x /= (float)b0Norm;
    b0.y /= (float)b0Norm;
    b0.z /= (float)b0Norm; 
    brw0.x /= (float)brw0Norm;
    brw0.y /= (float)brw0Norm;
    brw0.z /= (float)brw0Norm; 
    bmr0.x /= (float)bmr0Norm;
    bmr0.y /= (float)bmr0Norm;
    bmr0.z /= (float)bmr0Norm; 
    pushMatrix();   
      strokeWeight(4);
      translate(50, 0, 100); 
      stroke(182, 86, 93);
      line(0, 0, 0, 50*b0.x, 50*b0.y, 50*b0.z);
      stroke(255);
      line(0, 0, 0, 30*brw0.x, 30*brw0.y, 30*brw0.z);
      stroke(10, 200, 200);
      line(0, 0, 0, 30*bmr0.x, 30*bmr0.y, 30*bmr0.z);
    popMatrix(); 
    //-------------------------------//
    

    // DRAW ACTUATOR FIN GRADIENTS, REACTION FORCE, AND (C.G. to FIN CENTER OF PRESSURE MOMENT ARMS)
    //-------------------------------//
    pushMatrix();
      stroke(250);
      strokeWeight(4);
      stroke(204,0,0);
      translate(-64/2,0,0);  
      translate(-64/2,0,0); 
      translate(0,0,100);
      line(0,0,0,arm.x, arm.y, arm.z); 
      stroke(250);
      pushMatrix();
        translate(arm.x, arm.y, arm.z);
        stroke(250);
        line(0, 0, 0, 5*Gradl.x, 5*Gradl.y, 5*Gradl.z); 
        stroke(0,204,204);
        line(0,0,0,-finFrc1.x*7,-finFrc1.y*7,-finFrc1.z*7);
        stroke(240,30,30);
        line(0,0,0,GradFrc.x,GradFrc.y,GradFrc.z); 
      popMatrix();
      pushMatrix();
        translate(arm2.x, arm2.y, arm2.z);
        stroke(250);
        line(0, 0, 0, 5*Gradr.x, 5*Gradr.y, 5*Gradr.z); 
        stroke(0,204,204);
        line(0,0,0,-finFrc2.x*7,-finFrc2.y*7,-finFrc2.z*7);
        stroke(223,98,15);
        line(0,0,0,GradFrc2.x,GradFrc2.y,GradFrc2.z); 
      popMatrix();
      pushMatrix();
        translate(arm3.x, arm3.y, arm3.z);
        stroke(250);
        line(0, 0, 0, 5*Gradl.x, 5*Gradl.y, 5*Gradl.z);
        stroke(0,204,204);
        line(0,0,0,-finFrc3.x*7,-finFrc3.y*7,-finFrc3.z*7);
        stroke(204,204,20);
        line(0,0,0,GradFrc3.x,GradFrc3.y,GradFrc3.z);  
      popMatrix();
      pushMatrix();
        translate(arm4.x, arm4.y, arm4.z);
        stroke(250);
        line(0, 0, 0, 5*Gradr.x, 5*Gradr.y, 5*Gradr.z); 
        stroke(0,204,204);
        line(0,0,0,-finFrc4.x*7,-finFrc4.y*7,-finFrc4.z*7);
        stroke(17,249,31);
        line(0,0,0,GradFrc4.x,GradFrc4.y,GradFrc4.z);  
      popMatrix();
      pushMatrix();
        stroke(250);
        line(0, 0, 0, 15*Gradn.x, 15*Gradn.y, 15*Gradn.z); 
        stroke(0,204,0);
        line(0,0,0,thr.x*7,thr.y*7,thr.z*7);
      popMatrix();
      stroke(223,98,15);
      line(0,0,0,arm2.x, arm2.y, arm2.z); 
      stroke(204,204,0);
      line(0,0,0,arm3.x, arm3.y, arm3.z); 
      stroke(7,249,31);
      line(0,0,0,arm4.x, arm4.y, arm4.z); 
      stroke(250);
      stroke(250);
    popMatrix();
    //-------------------------------//

    //  TRANSLATION FOR MISSILE MOVEMENT.
    strokeWeight(.5);
    pushMatrix();
    translate(translation.x, translation.y, translation.z); 

    //  ROTATION FOR MISSILE MOVEMENT.
    rotateZ(-rotation.z);
    rotateY(-rotation.y);   
    rotateX(-rotation.x); 
    translate(0, 0, cg_displ);  // Move the c.g. up so missile at altitude=0  sit's on the ground.
    rotateZ(PI/2.0);  // to ensure the Helicopter is initially in inertial frame and makes sense.  

      pushMatrix();
        pushMatrix();
           
        // DRAW THE A/C FINS:
        //-------------------------------//
        pushMatrix();
          translate(0, 0, -3*fuse_mass_radius-finSpan*2);
          for (int i=0; i<4; i++) {
            rotateY(finAngles[i]*PI/180);
              pushMatrix();
              noStroke();
              fill(0, 190, 30);   //fill(76, 135, 80) " Blue fins"
              translate(0, 0, -3.5);  
              translate(0, 49, 0);  
              rotateY(PI/2); 
              rect(0, 0, 3, 12.5);
              translate(0, -5, 0); 
              triangle(0, 0, 0,5, 3, 5);
              translate(0, 61.5+5-49, 0);
              fill(200,255,255);  
              pushMatrix();
                translate(0, -61.5-5+12.48, 0); fill(0, 190, 30); 
                rect(0, 0, 2.5, 3+1/3);
              popMatrix();
              rotateX(controllerf[i]); // Deflect each of 4 fins
              fill(200,255,255);  
              rect(0, 0, 3, 2.5);
              popMatrix();
          }
        popMatrix();
    
        fill(10);
        translate(0, 0  , -fuse_displ/3);
        stroke(100);
        translate(0, 0, -fuse_displ/3);
        sphere(fuse_mass_radius*1.0);
        fill(50);
        drawCylinder(fuse_mass_radius*1, fuse_mass_radius*1, largo, 64);
        translate(0, 0, 2*fuse_displ/3);
        popMatrix(); // */
      popMatrix();  // End of the major push matrix

    popMatrix();
  } // End of the draw function.


  // CYLINDER DRAW FUNCTION.
  //    Draw Cyliner fuselage of missile simulation.
  void drawCylinder(float topRadius, float bottomRadius, float tall, int sides) {
    float angle = 0;
    float angleIncrement = TWO_PI / sides;
    beginShape(QUAD_STRIP);
    for (int i = 0; i < sides + 1; ++i) {
      vertex(topRadius*cos(angle), 0, topRadius*sin(angle));
      vertex(bottomRadius*cos(angle), tall, bottomRadius*sin(angle));
      angle += angleIncrement;
    }
    endShape();

    if (topRadius != 0) {
      angle = 0;
      beginShape(TRIANGLE_FAN);
      // Center point
      vertex(0, 0, 0);
      for (int i = 0; i < sides + 1; i++) {
        vertex(topRadius * cos(angle), 0, topRadius * sin(angle));
        angle += angleIncrement;
      }
      endShape();
    }

    if (bottomRadius != 0) {
      angle = 0;
      beginShape(TRIANGLE_FAN);

      // Center point
      vertex(0, tall, 0);
      for (int i = 0; i < sides + 1; i++) {
        vertex(bottomRadius * cos(angle), tall, bottomRadius * sin(angle));
        angle += angleIncrement;
      }
      endShape();
    }
  }  // END OF CYLINDER DRAW function.
}// End of the pendulum class.
