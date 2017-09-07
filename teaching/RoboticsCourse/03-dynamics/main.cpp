#include <Kin/kin.h>

void holdSteady(){
  mlr::KinematicWorld K("pegArm.ors");
  double noise = 0;
  bool gravity = true;
  uint n=K.getJointStateDimension();

  double tau=.01;
  uint T=200;
  
  arr q,qdot;
  arr M,F,u(n);
  double Kp = 200;
  double Kd = .5*sqrt(4.*Kp);
  
  K.getJointState(q, qdot);
  cout <<"initial posture (hit ENTER in the OpenGL window to continue!!)" <<endl;
  K.watch();
  
  ofstream dat("z.q");

  for(uint t=0;t<=T;t++){
    K.equationOfMotion(M,F);

    //no controller torques
    arr qddot = -Kp*q - Kd*qdot;
    u = M*qddot+F; //use M, F, and some desired qddot to compute u

    //dynamic simulation (simple Euler integration of the system dynamics, look into the code)
    K.stepDynamics(u, tau, noise, gravity);
    K.watch(false);
    K.getJointState(q, qdot);

    //qset = 1/Kp * (qref-q);
    //qdotset = 1/Kd * (qdotref-qdot);
    //K.setJointState(qset, qdotset);

    //some impuls
    if(t==100){
      qdot+=10.;
      K.setJointState(q, qdot);
    }

    cout  <<" t=" <<tau*t  <<"sec E=" <<K.getEnergy()  <<"  q = " <<q <<endl;
    dat <<q <<endl;
  }

  gnuplot("plot 'z.q' us 0:1, '' us 0:2, '' us 0:3");
  K.watch(true);
}


int main(int argc,char **argv){

  holdSteady();
  
  return 0;
}
