#include <stdlib.h>
#include <Core/array.h>
#include <Core/util.h>
#include <Plot/plot.h>
#include <Algo/MLcourse.h>

double f(const arr& x){
  CHECK(x.N==1, "");
  return -.3*cos(7.*x(0)) + sumOfSqr(x);
}

int main(int argc,char **argv){
  rnd.clockSeed(); //random seed rng

  //display the function: evaluate it over a grid
  arr X = grid(1, -2., 2., 100);
  arr f_y(X.d0);
  for(uint i=0;i<X.d0;i++) f_y(i) = f(X[i]);

  plotClear();
  plotFunction(X, f_y);
  plot(false);
  mlr::wait();

  //iterate taking samplings and updating a GP
  uint T=20;
  arr data_X, data_y;
  arr gp_sigma;

  for(uint t=0;t<T;t++){
    //a random decision
    uint N = 20;
    uint maxi;
    arr maxx = grid(1, 0, 0, 100);
    arr xs [N];
    for(uint i=0;i<N;i++){
      xs[i] = -2. + 4.*rand(1);
      if(gp_sigma(xs[i]) > gp_sigma(maxx)){
	maxx = xs[i];
 	maxi = i;
      }
    }
    arr x = xs[maxi]; 

    //query the function
    double y = f(x);

    //augment the data
    data_X.append(~x);
    data_y.append(y);

    //compute a GP
    double fmean = sum(data_y)/data_y.N;
    KernelRidgeRegression GP(data_X, data_y, defaultKernelFunction, -1., fmean);

    //Plotting: evaluate the GP over the grid X:
    arr gp_y = GP.evaluate(X, gp_sigma);

    plotClear();
    plotFunctionPrecision(X, gp_y, gp_y+gp_sigma, gp_y-gp_sigma);
    plotFunction(X, f_y);
    plotPoints(data_X, data_y);
    plot(false);
    mlr::wait();
  }

  return 0;
}
