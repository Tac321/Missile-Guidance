
// ACGrad  Object
// Purpose: Gradient vectors of each missile plate are calculated

class ACGrad {
  private PVector rotation, Grad,GradLado, Gradr, Gradl,Gradn;  
  private PVector Gradd1,Gradd2,Gradd3,Gradd4;
  private PVector finplc, finplcLast;
  private float rotX, rotY, rotZ;
  private float ver;
  private float finplc1=0, finplc2=0, finplc3=0;  
  private float finplc4=0;
  private float xcomp, ycomp, zcomp;
  private float kxx,kyy,kzz;
  
  public ACGrad(float s) {  

    rotation = new PVector(); 
    Grad = new PVector(0, 0, 0);  // bon
    GradLado = new PVector(0, 0, 0); 
    Gradr = new PVector(0, 0, 0); 
    Gradl = new PVector(0, 0, 0); 
    Gradn = new PVector(0, 0, 0);
    Gradd1 = new PVector(0, 0, 0);
    Gradd2 = new PVector(0, 0, 0);
    Gradd3 = new PVector(0, 0, 0);
    Gradd4 = new PVector(0, 0, 0);
    finplc  = new PVector();
    finplcLast = new PVector();
    calcQ();  // initially sets Ikine parameters.
  }  // End of the constructor object

  public void applyTranslationAndRotation(PVector rrr, PVector fff, PVector fff4) {   
    rotation.set(rrr);
    finplc.set(fff); 
    finplcLast.set(fff4);
    calcQ(); 
  } // end of apply function

private void calcQ() {     
    quatRotOpsVecProdF(); 
  }

  private void quatRotOpsVecProdF() {  
    rotX= rotation.x;
    rotY= rotation.y;
    rotZ= rotation.z;
    Grad.x= sin(rotZ)*sin(rotX)-cos(rotZ)*sin(rotY)*cos(rotX); 
    Grad.y= cos(rotZ)*sin(rotX)+sin(rotZ)*sin(rotY)*cos(rotX); 
    Grad.z= cos(rotY)*cos(rotX); // */
  
    GradLado.x= (sin(rotZ)*cos(rotX)+cos(rotZ)*sin(rotY)*sin(rotX)); 
    GradLado.y= (cos(rotZ)*cos(rotX)-sin(rotZ)*sin(rotY)*sin(rotX)); 
    GradLado.z= (-cos(rotY)*sin(rotX))                             ; // */
  
    Gradr.x= -1*(sin(rotZ)*cos(rotX)+cos(rotZ)*sin(rotY)*sin(rotX)) + 0*(sin(rotZ)*sin(rotX)-cos(rotZ)*sin(rotY)*cos(rotX));
    Gradr.y= -1*(cos(rotZ)*cos(rotX)-sin(rotZ)*sin(rotY)*sin(rotX)) + 0*(cos(rotZ)*sin(rotX)+sin(rotZ)*sin(rotY)*cos(rotX));
    Gradr.z= -1*(-cos(rotY)*sin(rotX)) + 0*(cos(rotY)*cos(rotX));
  
    Gradl.x= 0*(sin(rotZ)*cos(rotX)+cos(rotZ)*sin(rotY)*sin(rotX)) + -1*(sin(rotZ)*sin(rotX)-cos(rotZ)*sin(rotY)*cos(rotX));
    Gradl.y= 0*(cos(rotZ)*cos(rotX)-sin(rotZ)*sin(rotY)*sin(rotX)) + -1*(cos(rotZ)*sin(rotX)+sin(rotZ)*sin(rotY)*cos(rotX));
    Gradl.z= 0*(-cos(rotY)*sin(rotX)) + -1*(cos(rotY)*cos(rotX));
  
    Gradn.x= cos(rotZ)*cos(rotY);
    Gradn.y= -sin(rotZ)*cos(rotY);
    Gradn.z= sin(rotY);
    
    finplc1= finplc.x;
    finplc2= finplc.y;
    finplc3= finplc.z;
    finplc4= finplcLast.x;
    
    ver= 1-cos(finplc1);
    xcomp= 0;
    ycomp=  0;
    zcomp=  -1;
    kxx= 0;          
    kyy= 1;
    kzz= 0;
    xcomp= (kxx*kxx*ver+cos(finplc1))    *xcomp + (kxx*kyy*ver-kzz*sin(finplc1))*ycomp + (kxx*kzz*ver+kyy*sin(finplc1))*zcomp; 
    ycomp= (kxx*kyy*ver+kzz*sin(finplc1))*xcomp + (kyy*kyy*ver+cos(finplc1))    *ycomp + (kyy*kzz*ver-kxx*sin(finplc1))*zcomp;
    zcomp= (kxx*kzz*ver-kyy*sin(finplc1))*xcomp + (kyy*kzz*ver+kxx*sin(finplc1))*ycomp + (kzz*kzz*ver+cos(finplc1))    *zcomp;
    Gradd1.x= cos(rotZ)*cos(rotY) *xcomp+ ycomp*(sin(rotZ)*cos(rotX)+cos(rotZ)*sin(rotY)*sin(rotX)) + zcomp*(sin(rotZ)*sin(rotX)-cos(rotZ)*sin(rotY)*cos(rotX)); // 
    Gradd1.y= -sin(rotZ)*cos(rotY)*xcomp+ ycomp*(cos(rotZ)*cos(rotX)-sin(rotZ)*sin(rotY)*sin(rotX)) + zcomp*(cos(rotZ)*sin(rotX)+sin(rotZ)*sin(rotY)*cos(rotX));
    Gradd1.z= sin(rotY)*           xcomp+ ycomp*(-cos(rotY)*sin(rotX))                              + zcomp*(cos(rotY)*cos(rotX));
  
    ver= 1-cos(-finplc3);
    xcomp= 0;
    ycomp=  0; 
    zcomp=  -1;
    kxx= 0;
    kyy= -1;
    kzz= 0;
    xcomp= (kxx*kxx*ver+cos(-finplc3))    *xcomp + (kxx*kyy*ver-kzz*sin(-finplc3))*ycomp + (kxx*kzz*ver+kyy*sin(-finplc3))*zcomp; 
    ycomp= (kxx*kyy*ver+kzz*sin(-finplc3))*xcomp + (kyy*kyy*ver+cos(-finplc3))    *ycomp + (kyy*kzz*ver-kxx*sin(-finplc3))*zcomp;
    zcomp= (kxx*kzz*ver-kyy*sin(-finplc3))*xcomp + (kyy*kzz*ver+kxx*sin(-finplc3))*ycomp + (kzz*kzz*ver+cos(-finplc3))    *zcomp;
    Gradd3.x= cos(rotZ)*cos(rotY) *xcomp+ ycomp*(sin(rotZ)*cos(rotX)+cos(rotZ)*sin(rotY)*sin(rotX)) + zcomp*(sin(rotZ)*sin(rotX)-cos(rotZ)*sin(rotY)*cos(rotX));
    Gradd3.y= -sin(rotZ)*cos(rotY)*xcomp+ ycomp*(cos(rotZ)*cos(rotX)-sin(rotZ)*sin(rotY)*sin(rotX)) + zcomp*(cos(rotZ)*sin(rotX)+sin(rotZ)*sin(rotY)*cos(rotX));
    Gradd3.z= sin(rotY)*           xcomp+ ycomp*(-cos(rotY)*sin(rotX))                              + zcomp*(cos(rotY)*cos(rotX));
  
    ver= 1-cos(-finplc2);
    xcomp= 0;
    ycomp=  -1;
    zcomp= 0;
    kxx= 0;
    kyy= 0;
    kzz= 1;
    xcomp= (kxx*kxx*ver+cos(-finplc2))    *xcomp + (kxx*kyy*ver-kzz*sin(-finplc2))*ycomp + (kxx*kzz*ver+kyy*sin(-finplc2))*zcomp; //
    ycomp= (kxx*kyy*ver+kzz*sin(-finplc2))*xcomp + (kyy*kyy*ver+cos(-finplc2))    *ycomp + (kyy*kzz*ver-kxx*sin(-finplc2))*zcomp;
    zcomp= (kxx*kzz*ver-kyy*sin(-finplc2))*xcomp + (kyy*kzz*ver+kxx*sin(-finplc2))*ycomp + (kzz*kzz*ver+cos(-finplc2))    *zcomp;
    Gradd2.x= cos(rotZ)*cos(rotY) *xcomp+ ycomp*(sin(rotZ)*cos(rotX)+cos(rotZ)*sin(rotY)*sin(rotX)) + zcomp*(sin(rotZ)*sin(rotX)-cos(rotZ)*sin(rotY)*cos(rotX));
    Gradd2.y= -sin(rotZ)*cos(rotY)*xcomp+ ycomp*(cos(rotZ)*cos(rotX)-sin(rotZ)*sin(rotY)*sin(rotX)) + zcomp*(cos(rotZ)*sin(rotX)+sin(rotZ)*sin(rotY)*cos(rotX));
    Gradd2.z= sin(rotY)*           xcomp+ ycomp*(-cos(rotY)*sin(rotX))                              + zcomp*(cos(rotY)*cos(rotX));
  
    ver= 1-cos(finplc4);
    xcomp= 0;
    ycomp=  -1; 
    zcomp=  0;
    kxx= 0;
    kyy= 0;
    kzz= -1;
    xcomp= (kxx*kxx*ver+cos(finplc4))    *xcomp + (kxx*kyy*ver-kzz*sin(finplc4))*ycomp + (kxx*kzz*ver+kyy*sin(finplc4))*zcomp; //
    ycomp= (kxx*kyy*ver+kzz*sin(finplc4))*xcomp + (kyy*kyy*ver+cos(finplc4))    *ycomp + (kyy*kzz*ver-kxx*sin(finplc4))*zcomp;
    zcomp= (kxx*kzz*ver-kyy*sin(finplc4))*xcomp + (kyy*kzz*ver+kxx*sin(finplc4))*ycomp + (kzz*kzz*ver+cos(finplc4))    *zcomp;
    Gradd4.x= cos(rotZ)*cos(rotY) *xcomp+ ycomp*(sin(rotZ)*cos(rotX)+cos(rotZ)*sin(rotY)*sin(rotX)) + zcomp*(sin(rotZ)*sin(rotX)-cos(rotZ)*sin(rotY)*cos(rotX));
    Gradd4.y= -sin(rotZ)*cos(rotY)*xcomp+ ycomp*(cos(rotZ)*cos(rotX)-sin(rotZ)*sin(rotY)*sin(rotX)) + zcomp*(cos(rotZ)*sin(rotX)+sin(rotZ)*sin(rotY)*cos(rotX));
    Gradd4.z= sin(rotY)*           xcomp+ ycomp*(-cos(rotY)*sin(rotX))                              + zcomp*(cos(rotY)*cos(rotX));
  // */
  }
  
  public PVector getGrad() {   
    return Grad;  
  } // */
  public PVector getGradLado() {
    return GradLado; 
  } // */
  public PVector getGradr() {   
    return Gradr;
  } // */
  public PVector getGradl() { 
    return Gradl; 
  } // */
  public PVector getGradn() { 
    return Gradn; //  ea
  } // */
  public PVector getGradd1() {
    return Gradd1; //  dynamic fins
  } // */
  public PVector getGradd2() {
    return Gradd2; //  e
  } // */
  public PVector getGradd3() {
    return Gradd3; //  e
  } // */
  public PVector getGradd4() {
    return Gradd4; //  e
  } // */
}