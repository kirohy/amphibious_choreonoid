#include <cnoid/MathUtil>
#include <cnoid/SimpleController>

class TurretController : public cnoid::SimpleController {
  private:
    cnoid::BodyPtr body_;
    cnoid::LinkPtr joints_[2];
    double q_ref_[2];
    double q_prev_[2];
    double dt_;

  public:
    virtual bool initialize(cnoid::SimpleControllerIO *io) override {
        body_ = io->body();
        joints_[0] = body_->link("TURRET_Y");
        joints_[1] = body_->link("TURRET_P");

        for (int i = 0; i < 2; i++) {
            joints_[i]->setActuationMode(cnoid::Link::JOINT_TORQUE);
            io->enableIO(joints_[i]);
            q_ref_[i] = q_prev_[i] = joints_[i]->q();
        }

        io->enableInput(io->body()->rootLink(), LinkPosition);
        dt_ = io->timeStep();

        return true;
    }

    virtual bool control() override {
        static const double P = 200.0;
        static const double D = 50.0;

        auto pos = body_->rootLink()->T();
        auto pos_z = pos.translation().z();

        if (pos_z > 0) {
            q_ref_[1] = cnoid::radian(-15.0);
        } else {
            q_ref_[1] = 0.0;
        }

        for (int i = 0; i < 2; i++) {
            double q = joints_[i]->q();
            double dq = (q - q_prev_[i]) / dt_;
            double dq_ref = 0.0;
            joints_[i]->u() = P * (q_ref_[i] - q) + D * (dq_ref - dq);
            q_prev_[i] = q;
        }

        return true;
    }
};

CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(TurretController)
