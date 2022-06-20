
#include <ct/optcon/optcon.h>
#include <Eigen/Dense>
#include "exampleDir.h"

using namespace ct::core;
using namespace ct::optcon;

static ct::core::Time dt = 0.1;
size_t nPoints = 100;
size_t horizon = 10;

static const size_t STATE_DIM = 2;
static const size_t CONTROL_DIM = 1;

static Eigen::MatrixXd matA = Eigen::MatrixXd::Identity(STATE_DIM, STATE_DIM);
static Eigen::MatrixXd matB = Eigen::MatrixXd::Identity(STATE_DIM, CONTROL_DIM);
static Eigen::MatrixXd vecF = Eigen::MatrixXd::Identity(STATE_DIM, 1);



template <typename SCALAR>
class DiscreteLTVSystem : public DiscreteControlledSystem<STATE_DIM, CONTROL_DIM, SCALAR>
{
public:
    typedef DiscreteControlledSystem<STATE_DIM, CONTROL_DIM, SCALAR> Base;


    typedef typename Base::state_vector_t state_vector_t;
    typedef typename Base::control_vector_t control_vector_t;
    typedef typename Base::time_t time_t;
    
    DiscreteLTVSystem() {
      matA(0,0) = 0.;
      matA(0,1) = 1.;
      matA(1,0) = -1.;
      matA(1,1) = -1.;
      matB(0,0) = 0;
      matB(1,0) = 1.;
      // discret-time mass-dumper-spring like system
    }
    virtual ~DiscreteLTVSystem() {}
    DiscreteLTVSystem* clone() const override { return new DiscreteLTVSystem(*this); }
    virtual void propagateControlledDynamics(const state_vector_t& state,
        const time_t n,
        const control_vector_t& control,
        state_vector_t& stateNext) override
    {
        //stateNext = vecF + matA * state + matB * control;
        stateNext = state + (matA * state + matB * control)*dt;
    }

private:
    SCALAR rate_;
};


int main(int argc, char** argv)
{    
    
    std::shared_ptr< DiscreteControlledSystem<STATE_DIM, CONTROL_DIM, double> > oscillatorDynamics(
	new DiscreteLTVSystem<double>);

    std::shared_ptr< DiscreteSystemLinearizer<STATE_DIM, CONTROL_DIM> > adLinearizer(
	new DiscreteSystemLinearizer<STATE_DIM, CONTROL_DIM>(oscillatorDynamics));


    StateVectorArray<STATE_DIM> x_a(nPoints, StateVector<STATE_DIM>::Zero());

    for(size_t i=0; i<nPoints; i++){
       x_a[i] << cos(i*dt), -sin(i*dt);
    }

    ControlVectorArray<CONTROL_DIM> u_a(nPoints, ControlVector<CONTROL_DIM>::Zero());

    ct::core::TimeArray t_a(dt, nPoints);
    ct::core::StateTrajectory<STATE_DIM> x_traj(t_a, x_a, ct::core::ZOH);
    ct::core::ControlTrajectory<CONTROL_DIM> u_traj(t_a, u_a, ct::core::ZOH);

    //for(size_t i=0; i<nPoints; i++){
    //   std::cout << i << ", "<< x_traj[i](0) << ", " << x_traj[i](1) << std::endl;
    //}


    ct::optcon::TermQuadTracking<STATE_DIM, CONTROL_DIM>::state_matrix_t Q(Eigen::Matrix<double, STATE_DIM, STATE_DIM>::Zero());
    ct::optcon::TermQuadTracking<STATE_DIM, CONTROL_DIM>::control_matrix_t R(Eigen::Matrix<double, CONTROL_DIM, CONTROL_DIM>::Zero());
    Q(0,0) = 10.;
    Q(1,1) = 10.;
    R(0,0) = 1.;

    std::shared_ptr<ct::optcon::TermQuadTracking<STATE_DIM, CONTROL_DIM>> intermediateCost(
        new ct::optcon::TermQuadTracking<STATE_DIM, CONTROL_DIM>(Q, R, ct::core::LIN, ct::core::ZOH, false));
    intermediateCost->setStateAndControlReference(x_traj, u_traj);
    
    std::shared_ptr<CostFunctionQuadratic<STATE_DIM, CONTROL_DIM>> costFunction(
        new CostFunctionAnalytical<STATE_DIM, CONTROL_DIM>());
    costFunction->addIntermediateTerm(intermediateCost);

    StateVector<STATE_DIM> x0;
    x0 << 1., 0.;
    

    ct::optcon::DiscreteOptConProblem<STATE_DIM, CONTROL_DIM> optConProblem(
        horizon, x0, oscillatorDynamics, costFunction, adLinearizer);


    ct::optcon::NLOptConSettings ilqr_settings;
    ilqr_settings.dt = dt;  // the control discretization in [sec]
    ilqr_settings.integrator = ct::core::IntegrationType::EULERCT;
    ilqr_settings.discretization = NLOptConSettings::APPROXIMATION::FORWARD_EULER;
    ilqr_settings.max_iterations = 10;
    ilqr_settings.nlocp_algorithm = NLOptConSettings::NLOCP_ALGORITHM::ILQR;
    ilqr_settings.lqocp_solver = NLOptConSettings::LQOCP_SOLVER::GNRICCATI_SOLVER;  // the LQ-problems are solved using a custom Gauss-Newton Riccati solver
    ilqr_settings.printSummary = false;

    FeedbackArray<STATE_DIM, CONTROL_DIM> u0_fb(horizon, FeedbackMatrix<STATE_DIM, CONTROL_DIM>::Zero());
    ct::optcon::NLOptConSolver<STATE_DIM, CONTROL_DIM>::Policy_t initController(x_a, u_a, u0_fb, ilqr_settings.dt);

    NLOptConSolver<STATE_DIM, CONTROL_DIM, STATE_DIM/2, STATE_DIM/2, double, false> iLQR(optConProblem, ilqr_settings);

    iLQR.setInitialGuess(initController);

    iLQR.solve();
    StateFeedbackController<STATE_DIM, CONTROL_DIM> initialSolution = iLQR.getSolution();

    auto start_time = std::chrono::high_resolution_clock::now();

    std::cout << "Starting to run MPC" << std::endl;


    for (size_t i = 0; i < nPoints; i++)
    {
        auto current_time = std::chrono::high_resolution_clock::now();
        if (i>0){
          x_traj.eraseFront(1, dt);
          u_traj.eraseFront(1, dt);
    
          if (x_traj.size()<horizon){
            x_traj.push_back(x_traj.back(), x_traj.finalTime(), false);
            u_traj.push_back(u_traj.back(), u_traj.finalTime(), false);
          }

          intermediateCost->setStateAndControlReference(x_traj, u_traj);
          std::shared_ptr<CostFunctionQuadratic<STATE_DIM, CONTROL_DIM>> costFunction(
            new CostFunctionAnalytical<STATE_DIM, CONTROL_DIM>());
          costFunction->addIntermediateTerm(intermediateCost);
          iLQR.changeCostFunction(costFunction);
        }


        iLQR.setInitialGuess(initController); // at MPC<OPTCON_SOLVER>::prepareIteration
        iLQR.prepareMPCIteration();  // at MPC<OPTCON_SOLVER>::prepareIteration
        iLQR.changeInitialState(x0); // at MPC<OPTCON_SOLVER>::finishIteration


        StateFeedbackController<STATE_DIM, CONTROL_DIM> newPolicy;


        current_time = std::chrono::high_resolution_clock::now();
        //t = 1e-6 * std::chrono::duration_cast<std::chrono::microseconds>(current_time - start_time).count();
        iLQR.finishMPCIteration(); // at MPC<OPTCON_SOLVER>::finishIteration


        newPolicy = iLQR.getSolution();
        std::cout << x0(0) << ", " << x0(1) << ", " << newPolicy.uff()[0] << std::endl;

        initController.update(newPolicy.x_ref(), newPolicy.uff(), newPolicy.K(), newPolicy.time());
        x0 = x0 + (matA * x0 + matB * newPolicy.uff()[0])*dt;
    }

}

