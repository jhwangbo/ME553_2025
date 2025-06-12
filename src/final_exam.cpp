//
// Created by Jemin Hwangbo on 2025/05/16.
//

#include "raisim/RaisimServer.hpp"
#include "final_exam_STUDENTID.hpp"

#define _MAKE_STR(x) __MAKE_STR(x)
#define __MAKE_STR(x) #x

int main(int argc, char* argv[]) {
  auto binaryPath = raisim::Path::setFromArgv(argv[0]);

  // create raisim world
  raisim::World world; // physics world
  raisim::RaisimServer server(&world);

  // kinova
  auto cartPole = world.addArticulatedSystem(std::string(_MAKE_STR(RESOURCE_DIR)) + "/cartpole.urdf");

  // kinova configuration
  Eigen::VectorXd gc(cartPole->getGeneralizedCoordinateDim()), gv(cartPole->getDOF());
  gc << 0.1, 0.2, 0.3; /// Jemin: I'll randomize the gc, gv when grading
  gv << 0.1, 0.2, 0.3;
  cartPole->setState(gc, gv);
  server.focusOn(cartPole);
  server.launchServer();

  /// if you are using an old version of Raisim, you need this line
  world.integrate1();

  std::cout<<"nonlinearities should be \n"<< cartPole->getNonlinearities({0,0,-9.81}).e().transpose()<<std::endl;

  if((getNonlinearities(gc, gv) - cartPole->getNonlinearities({0,0,-9.81}).e()).norm() < 1e-8)
    std::cout<<"passed "<<std::endl;
  else
    std::cout<<"failed "<<std::endl;

  for (int i=0; i<2000000; i++)
    std::this_thread::sleep_for(std::chrono::microseconds(1000));
  }

  server.killServer();

  return 0;
}
