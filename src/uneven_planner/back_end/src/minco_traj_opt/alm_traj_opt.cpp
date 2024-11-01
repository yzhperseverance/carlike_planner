#include "back_end/minco_traj_opt/alm_traj_opt.h"

namespace carlike_planner
{
    ALMTrajOpt::ALMTrajOpt(ros::NodeHandle &nh)
    {
        nh.getParam("alm_traj_opt/rho_T", rho_T);
        nh.getParam("alm_traj_opt/rho_ter", rho_ter);
        nh.getParam("alm_traj_opt/max_vel", max_vel);
        nh.getParam("alm_traj_opt/max_acc_lon", max_acc_lon);
        nh.getParam("alm_traj_opt/max_acc_lat", max_acc_lat);
        nh.getParam("alm_traj_opt/max_kap", max_kap);
        nh.getParam("alm_traj_opt/min_cxi", min_cxi);
        nh.getParam("alm_traj_opt/max_sig", max_sig);
        nh.getParam("alm_traj_opt/dist0", dist0);
        nh.getParam("alm_traj_opt/use_scaling", use_scaling);
        nh.getParam("alm_traj_opt/rho", rho);
        nh.getParam("alm_traj_opt/beta", beta);
        nh.getParam("alm_traj_opt/gamma", gamma);
        nh.getParam("alm_traj_opt/epsilon_con", epsilon_con);
        nh.getParam("alm_traj_opt/max_iter", max_iter);
        nh.getParam("alm_traj_opt/g_epsilon", g_epsilon);
        nh.getParam("alm_traj_opt/min_step", min_step);
        nh.getParam("alm_traj_opt/inner_max_iter", inner_max_iter);
        nh.getParam("alm_traj_opt/delta", delta);
        nh.getParam("alm_traj_opt/mem_size", mem_size);
        nh.getParam("alm_traj_opt/past", past);
        nh.getParam("alm_traj_opt/int_K", int_K);
        nh.getParam("alm_traj_opt/in_test", in_test);
        nh.getParam("alm_traj_opt/in_debug", in_debug);

        if (in_debug)
        {
            debug_pub = nh.advertise<visualization_msgs::Marker>("/alm/debug_path", 1);
        }
            debug_pub = nh.advertise<visualization_msgs::Marker>("/alm/debug_path", 1);

        return;
    }


    static double innerCallback(void* ptrObj, const Eigen::VectorXd& x, Eigen::VectorXd& grad);
    static int earlyExit(void* ptrObj, const Eigen::VectorXd& x, const Eigen::VectorXd& grad, 
                         const double fx, const double step, int k, int ls);

    // TODO:又臭又长的代码，找时间给他重构了
    int ALMTrajOpt::optimizeSE2Traj(const Eigen::MatrixXd &initStateXY, \
                                    const Eigen::MatrixXd &endStateXY , \
                                    const Eigen::MatrixXd &innerPtsXY , \
                                    const Eigen::VectorXd &initYaw    , \
                                    const Eigen::VectorXd &endYaw     , \
                                    const Eigen::VectorXd &innerPtsYaw, \
                                    const double & totalTime            )
    {
        int ret_code = 0;

        piece_xy = innerPtsXY.cols() + 1;
        piece_yaw = innerPtsYaw.size() + 1;
        minco_se2.reset(piece_xy, piece_yaw);
        init_xy = initStateXY;
        end_xy = endStateXY;
        init_yaw = initYaw.transpose();
        end_yaw = endYaw.transpose();

        int variable_num = 2*(piece_xy-1) + (piece_yaw-1) + 1;
        // non-holonomic
        equal_num = piece_xy * (int_K + 1); // int_K是中间点的个数，不算每段的起始点，所以要+1
        // longitude velocity, longitude acceleration, latitude acceleration, curvature, attitude, surface variation
        non_equal_num = piece_xy * (int_K + 1) * 5; // 有5个约束
        hx.setZero(equal_num);
        lambda.setZero(equal_num);
        gx.setZero(non_equal_num);
        mu.setZero(non_equal_num);
        scale_fx = 1.0;
        scale_cx.resize(equal_num+non_equal_num);
        scale_cx.setConstant(1.0);

        // init solution
        Eigen::VectorXd x;
        x.resize(variable_num);

        dim_T = 1;
        double& tau = x(0);
        Eigen::Map<Eigen::MatrixXd> Pxy(x.data()+dim_T, 2, piece_xy-1);
        Eigen::Map<Eigen::MatrixXd> Pyaw(x.data()+dim_T+2*(piece_xy-1), 1, piece_yaw-1);

        tau = logC2(totalTime);
        Pxy = innerPtsXY;
        Pyaw = innerPtsYaw.transpose();

        // lbfgs params
        lbfgs::lbfgs_parameter_t lbfgs_params;
        lbfgs_params.mem_size = mem_size;
        lbfgs_params.past = past;
        lbfgs_params.g_epsilon = g_epsilon;
        lbfgs_params.min_step = min_step;
        lbfgs_params.delta = delta;
        lbfgs_params.max_iterations = inner_max_iter;
        double inner_cost;

        // begin PHR Augmented Lagrangian Method
        ros::Time start_time = ros::Time::now();
        int iter = 0;
        if (use_scaling)
            initScaling(x);
        // ALM将有约束优化问题转化为无约束优化问题, L-BFGS是一种用于无约束优化的迭代方法
        while (true)
        {
            int result = lbfgs::lbfgs_optimize(x, inner_cost, &innerCallback, nullptr, 
                                               &earlyExit, this, lbfgs_params);

            if (result == lbfgs::LBFGS_CONVERGENCE ||
                result == lbfgs::LBFGS_CANCELED ||
                result == lbfgs::LBFGS_STOP || 
                result == lbfgs::LBFGSERR_MAXIMUMITERATION)
            {
                ROS_INFO_STREAM("[Inner] optimization success! cost: " << inner_cost );
            }
            else if (result == lbfgs::LBFGSERR_MAXIMUMLINESEARCH)
            {
                ROS_WARN("[Inner] The line-search routine reaches the maximum number of evaluations.");
            }
            else
            {
                ret_code = 1;
                ROS_ERROR("[Inner] Solver error. Return = %d, %s.", result, lbfgs::lbfgs_strerror(result));
                break;
            }

            updateDualVars();

            if(judgeConvergence())
            {
                ROS_WARN_STREAM("[ALM] Convergence! iters: "<<iter);
                break;
            }

            if(++iter > max_iter)
            {
                ret_code = 2;
                ROS_WARN("[ALM] Reach max iteration");
                break;
            }

        }
        ROS_INFO_STREAM("[ALM] Time consuming: "<<(ros::Time::now()-start_time).toSec() * 1000.0 << " ms");
        ROS_INFO_STREAM("[ALM] Jerk cost: "<<minco_se2.getTrajJerkCost() << " traj time:" << minco_se2.getTraj().getTotalDuration());

        return ret_code;
    }

    static double innerCallback(void* ptrObj, const Eigen::VectorXd& x, Eigen::VectorXd& grad)
    {
        ALMTrajOpt& obj = *(ALMTrajOpt*)ptrObj;
        // dim_T应该是时间的维度，他除了1也不可能有别的值啊？
        // get x
        const double& tau = x(0);
        double& grad_tau = grad(0);
        //通过 obj.dim_T 偏移指针，以便从 x 的某个特定位置开始映射。
        //这意味着映射将从 x 中的第 dim_T 个元素开始。

        // 映射x的数据到Pxy里,指向内存地址
        Eigen::Map<const Eigen::MatrixXd> Pxy(x.data()+obj.dim_T, 2, obj.piece_xy-1);
        Eigen::Map<const Eigen::MatrixXd> Pyaw(x.data()+obj.dim_T+2*(obj.piece_xy-1), 1, obj.piece_yaw-1);
        Eigen::Map<Eigen::MatrixXd> gradPxy(grad.data() + obj.dim_T, 2, obj.piece_xy-1);
        Eigen::Map<Eigen::MatrixXd> gradPyaw(grad.data()+obj.dim_T+2*(obj.piece_xy-1), 1, obj.piece_yaw-1);

        // x的第一个元素是T
        // get T from τ, generate MINCO trajectory
        // T = e^τ
        // 采用均匀时间,xy和yaw的时间好像是一样的？
        Eigen::VectorXd Txy, Tyaw;
        Txy.resize(obj.piece_xy);
        Tyaw.resize(obj.piece_yaw);
        obj.calTfromTau(tau, Txy);
        obj.calTfromTau(tau, Tyaw);
        obj.minco_se2.generate(obj.init_xy, obj.end_xy, Pxy, Txy, \
                               obj.init_yaw, obj.end_yaw, Pyaw, Tyaw);
        
        // get jerk cost with grad (C,T)
        double jerk_cost = 0.0;
        Eigen::MatrixXd gdCxy_jerk;
        Eigen::VectorXd gdTxy_jerk;
        Eigen::MatrixXd gdCyaw_jerk;
        Eigen::VectorXd gdTyaw_jerk;
        obj.minco_se2.calJerkGradCT(gdCxy_jerk, gdTxy_jerk, gdCyaw_jerk, gdTyaw_jerk);
        jerk_cost = obj.minco_se2.getTrajJerkCost() * obj.scale_fx;
        if (obj.use_scaling)
            jerk_cost *= scale_trick_jerk;

        // get constrain cost with grad (C,T)
        double constrain_cost = 0.0;
        Eigen::MatrixXd gdCxy_constrain;
        Eigen::VectorXd gdTxy_constrain;
        Eigen::MatrixXd gdCyaw_constrain;
        Eigen::VectorXd gdTyaw_constrain;
        obj.calConstrainCostGrad(constrain_cost, gdCxy_constrain, gdTxy_constrain, \
                                 gdCyaw_constrain, gdTyaw_constrain);

        // K =  jerk_cost +  constrain_cost
        if (obj.use_scaling)
        {
            gdCxy_jerk *= scale_trick_jerk;
            gdTxy_jerk *= scale_trick_jerk;
            gdCyaw_jerk *= scale_trick_jerk;
            gdTyaw_jerk *= scale_trick_jerk;
        }
        // get grad (q, T) from (C, T)
        Eigen::MatrixXd gdCxy = gdCxy_jerk * obj.scale_fx + gdCxy_constrain;
        Eigen::VectorXd gdTxy = gdTxy_jerk * obj.scale_fx + gdTxy_constrain;
        Eigen::MatrixXd gdCyaw = gdCyaw_jerk * obj.scale_fx + gdCyaw_constrain;
        Eigen::VectorXd gdTyaw = gdTyaw_jerk * obj.scale_fx + gdTyaw_constrain;
        Eigen::MatrixXd gradPxy_temp;
        Eigen::MatrixXd gradPyaw_temp;
        // 把∂K/∂c和∂K/∂T转化成∂W/∂q,∂W/∂T
        obj.minco_se2.calGradCTtoQT(gdCxy, gdTxy, gradPxy_temp, gdCyaw, gdTyaw, gradPyaw_temp);
        gradPxy = gradPxy_temp;
        gradPyaw = gradPyaw_temp;

        // get tau cost with grad
        double tau_cost = obj.rho_T * obj.expC2(tau) * obj.scale_fx;
        double grad_Tsum = obj.rho_T * obj.scale_fx + \
                           gdTxy.sum() / obj.piece_xy + \
                           gdTyaw.sum() / obj.piece_yaw;
        grad_tau = grad_Tsum * obj.getTtoTauGrad(tau);

        return jerk_cost + constrain_cost + tau_cost;
    }

    void ALMTrajOpt::initScaling(Eigen::VectorXd x0)
    {       
        // get x
        const double& tau = x0(0);
        Eigen::Map<const Eigen::MatrixXd> Pxy(x0.data()+dim_T, 2, piece_xy-1);
        Eigen::Map<const Eigen::MatrixXd> Pyaw(x0.data()+dim_T+2*(piece_xy-1), 1, piece_yaw-1);
        
        // get T from τ, generate MINCO trajectory
        Eigen::VectorXd Txy, Tyaw;
        Txy.resize(piece_xy);
        Tyaw.resize(piece_yaw);
        calTfromTau(tau, Txy);
        calTfromTau(tau, Tyaw);
        minco_se2.generate(init_xy, end_xy, Pxy, Txy, \
                           init_yaw, end_yaw, Pyaw, Tyaw);
        
        // get jerk grad (C,T)
        Eigen::MatrixXd gdCxy_fx;
        Eigen::VectorXd gdTxy_fx;
        Eigen::MatrixXd gdCyaw_fx;
        Eigen::VectorXd gdTyaw_fx;
        minco_se2.calJerkGradCT(gdCxy_fx, gdTxy_fx, gdCyaw_fx, gdTyaw_fx);
        
        std::vector<Eigen::MatrixXd> gdCxy;
        std::vector<Eigen::VectorXd> gdTxy;
        std::vector<Eigen::MatrixXd> gdCyaw;
        std::vector<Eigen::VectorXd> gdTyaw;

        for (int i=0; i<equal_num+non_equal_num; i++)
        {
            gdCxy.push_back(Eigen::MatrixXd::Zero(6*piece_xy, 2));
            gdTxy.push_back(Eigen::VectorXd::Zero(piece_xy));
            gdCyaw.push_back(Eigen::MatrixXd::Zero(6*piece_yaw, 1));
            gdTyaw.push_back(Eigen::VectorXd::Zero(piece_yaw));
        }

        Eigen::Vector2d pos, vel, acc, jer;
        Eigen::Vector2d xb, yb;
        Eigen::Vector3d se2_pos;
        vector<double> terrain_values;
        vector<Eigen::Vector3d> terrain_grads;
        double          yaw, dyaw, d2yaw;
        double          cyaw, syaw, v_norm;
        double          lon_acc, lat_acc, curv_snorm, vx, wz, ax, ay;
        double          dist;
        Eigen::Vector2d grad_p = Eigen::Vector2d::Zero();
        Eigen::Vector2d grad_v = Eigen::Vector2d::Zero();
        Eigen::Vector2d grad_a = Eigen::Vector2d::Zero();
        Eigen::Vector2d grad_dist = Eigen::Vector2d::Zero();

        double          grad_yaw = 0.0;
        double          grad_dyaw = 0.0;
        double          grad_d2yaw = 0.0;
        double          grad_vx2 = 0.0;
        double          grad_wz = 0.0;
        double          grad_ax = 0.0;
        double          grad_ay = 0.0;
        Eigen::Matrix<double, 6, 1> beta0_xy, beta1_xy, beta2_xy, beta3_xy;
        Eigen::Matrix<double, 6, 1> beta0_yaw, beta1_yaw, beta2_yaw, beta3_yaw;
        double s1;
        double s1_yaw;
        double step, alpha, omega;

        int constrain_idx = 0;
        int yaw_idx = 0;
        double base_time = 0.0;
        for (int i=0; i<piece_xy; i++)
        {
            const Eigen::Matrix<double, 6, 2> &c_xy = minco_se2.pos_minco.getCoeffs().block<6, 2>(i * 6, 0);
            step = minco_se2.pos_minco.T1(i) / int_K;
            s1 = 0.0;

            for (int j=0; j<=int_K; j++)
            {
                alpha = 1.0 / int_K * j;

                // set zero
                grad_p.setZero();
                grad_v.setZero();
                grad_a.setZero();
                grad_yaw = 0.0;
                grad_dyaw = 0.0;
                grad_d2yaw = 0.0;
                grad_vx2 = 0.0;
                grad_wz = 0.0;
                grad_ax = 0.0;
                grad_ay = 0.0;

                // analyse xy
                computeBeta(s1, beta0_xy, beta1_xy, beta2_xy, beta3_xy);
                pos = c_xy.transpose() * beta0_xy;
                vel = c_xy.transpose() * beta1_xy;
                acc = c_xy.transpose() * beta2_xy;
                jer = c_xy.transpose() * beta3_xy;

                // analyse yaw
                double now_time = s1 + base_time;
                yaw_idx = int((now_time) / minco_se2.yaw_minco.T1(i));
                if (yaw_idx >= piece_yaw)
                    yaw_idx = piece_yaw - 1;
                const Eigen::Matrix<double, 6, 1> &c_yaw = minco_se2.yaw_minco.getCoeffs().block<6, 1>(yaw_idx * 6, 0);
                s1_yaw = now_time - yaw_idx * minco_se2.yaw_minco.T1(i);
                computeBeta(s1_yaw, beta0_yaw, beta1_yaw, beta2_yaw, beta3_yaw);
                yaw = c_yaw.transpose() * beta0_yaw;
                dyaw = c_yaw.transpose() * beta1_yaw;
                d2yaw = c_yaw.transpose() * beta2_yaw;

                // analyse complex variable
                syaw = sin(yaw);
                cyaw = cos(yaw);
                v_norm = vel.norm();
                xb = Eigen::Vector2d(cyaw, syaw);
                yb = Eigen::Vector2d(-syaw, cyaw);
                lon_acc = acc.dot(xb);
                lat_acc = acc.dot(yb);


                vx = v_norm;
                wz = dyaw;
                ax = lon_acc;
                ay = lat_acc;
                curv_snorm = wz * wz / (vx*vx + delta_sigl);
                sdf_map->evaluateEDTWithGrad(pos, -1.0, dist, grad_dist);

                // non-holonomic
                grad_v = Eigen::Vector2d(syaw, -cyaw);
                grad_yaw = vel.dot(xb);
                gdCxy[constrain_idx].block<6, 2>(i * 6, 0) += beta1_xy * grad_v.transpose();
                gdTxy[constrain_idx](i) += grad_v.dot(acc) * alpha;
                gdCyaw[constrain_idx].block<6, 1>(yaw_idx * 6, 0) += beta0_yaw * grad_yaw;
                gdTyaw[constrain_idx](yaw_idx) += -(grad_yaw * dyaw) * yaw_idx;
                gdTxy[constrain_idx](i) += (grad_yaw * dyaw) * (alpha+i);
                constrain_idx++;

                // longitude velocity
                grad_vx2 = 1.0;
                grad_v = grad_vx2 * 2.0 * vel;
                gdCxy[constrain_idx].block<6, 2>(i * 6, 0) += beta1_xy * grad_v.transpose();
                gdTxy[constrain_idx](i) += grad_v.dot(acc) * alpha;
                constrain_idx++;

                // longitude acceleration
                grad_ax = 2.0 * ax;
                grad_a = grad_ax * xb;
                grad_yaw = grad_ax * lat_acc;
                gdCxy[constrain_idx].block<6, 2>(i * 6, 0) += beta2_xy * grad_a.transpose();
                gdTxy[constrain_idx](i) += grad_a.dot(jer) * alpha;
                gdCyaw[constrain_idx].block<6, 1>(yaw_idx * 6, 0) += beta0_yaw * grad_yaw;
                gdTyaw[constrain_idx](yaw_idx) += -(grad_yaw * dyaw) * yaw_idx;
                gdTxy[constrain_idx](i) += (grad_yaw * dyaw) * (alpha+i);
                constrain_idx++;

                // latitude acceleration
                grad_ay = 2.0 * ay;
                grad_a = grad_ay * yb;
                grad_yaw = -grad_ay * lon_acc;
                gdCxy[constrain_idx].block<6, 2>(i * 6, 0) += beta2_xy * grad_a.transpose();
                gdTxy[constrain_idx](i) += grad_a.dot(jer) * alpha;
                gdCyaw[constrain_idx].block<6, 1>(yaw_idx * 6, 0) += beta0_yaw * grad_yaw;
                gdTyaw[constrain_idx](yaw_idx) += -(grad_yaw * dyaw) * yaw_idx;
                gdTxy[constrain_idx](i) += (grad_yaw * dyaw) * (alpha+i);
                constrain_idx++;

                // obstacle distance
                grad_p = -2.0 * dist * grad_dist;
                gdCxy[constrain_idx].block<6, 2>(i * 6, 0) += beta0_xy * grad_p.transpose();
                gdTxy[constrain_idx](i) += grad_p.dot(vel) * alpha;
                constrain_idx++;

                // curvature
                double denominator = 1.0 / (vx*vx + delta_sigl);
                grad_wz = denominator * 2.0 * wz;
                grad_vx2 = -curv_snorm * denominator;
                grad_dyaw = grad_wz;
                grad_v = grad_vx2 * 2.0 * vel;

                gdCxy[constrain_idx].block<6, 2>(i * 6, 0) += beta1_xy * grad_v.transpose();
                gdTxy[constrain_idx](i) += grad_v.dot(acc) * alpha;
                gdCyaw[constrain_idx].block<6, 1>(yaw_idx * 6, 0) += beta1_yaw * grad_dyaw;
                gdTyaw[constrain_idx](yaw_idx) += -grad_dyaw * d2yaw * yaw_idx;
                gdTxy[constrain_idx](i) += grad_dyaw * d2yaw * (alpha+i);
                constrain_idx++;

                s1 += step;
            }
            base_time += minco_se2.pos_minco.T1(i);
        }
        
        Eigen::MatrixXd gdPxy_fx;
        Eigen::MatrixXd gdPyaw_fx;
        std::vector<Eigen::MatrixXd> gdPxy;
        std::vector<Eigen::MatrixXd> gdPyaw;
        std::vector<double>          gdTau;
        minco_se2.calGradCTtoQT(gdCxy_fx, gdTxy_fx, gdPxy_fx, gdCyaw_fx, gdTyaw_fx, gdPyaw_fx);
        double grad_Tsum_fx = rho_T + \
                              gdTxy_fx.sum() / piece_xy + \
                              gdTyaw_fx.sum() / piece_yaw;
        double gdTau_fx = grad_Tsum_fx * getTtoTauGrad(tau);
        for (int i=0; i<gdCxy.size(); i++)
        {
            Eigen::MatrixXd gdPxy_temp;
            Eigen::MatrixXd gdPyaw_temp;
            minco_se2.calGradCTtoQT(gdCxy[i], gdTxy[i], gdPxy_temp, gdCyaw[i], gdTyaw[i], gdPyaw_temp);
            double grad_Tsum = gdTxy[i].sum() / piece_xy + \
                               gdTyaw[i].sum() / piece_yaw;
            gdTau.push_back(grad_Tsum * getTtoTauGrad(tau));
            gdPxy.push_back(gdPxy_temp);
            gdPyaw.push_back(gdPyaw_temp);
        }

        gdPxy_fx.resize((piece_xy-1)*2, 1);
        gdPyaw_fx.resize(piece_yaw-1, 1);
        scale_fx = 1.0 / max(1.0, max(max(gdPxy_fx.lpNorm<Eigen::Infinity>(), \
                                      gdPyaw_fx.lpNorm<Eigen::Infinity>()), fabs(gdTau_fx)));
        for (int i=0; i<equal_num+non_equal_num; i++)
        {
            gdPxy[i].resize((piece_xy-1)*2, 1);
            gdPyaw[i].resize(piece_yaw-1, 1);
            scale_cx(i) = 1.0 / max(1.0, max(max(gdPxy[i].lpNorm<Eigen::Infinity>(), \
                                      gdPyaw[i].lpNorm<Eigen::Infinity>()), fabs(gdTau[i])));
        }
    }

    void ALMTrajOpt::calConstrainCostGrad(double& cost, Eigen::MatrixXd& gdCxy, Eigen::VectorXd &gdTxy, \
                                          Eigen::MatrixXd& gdCyaw, Eigen::VectorXd &gdTyaw)
    {
        cost = 0.0;
        gdCxy.setZero(6*piece_xy, 2);
        gdTxy.setZero(piece_xy);
        gdCyaw.setZero(6*piece_yaw, 1);
        gdTyaw.setZero(piece_yaw);

        Eigen::Vector2d pos, vel, acc, jer;
        Eigen::Vector2d xb, yb;

        double          yaw, dyaw, d2yaw;
        double          cyaw, syaw, v_norm;
        double          lon_acc, lat_acc, curv_snorm, vx, wz, ax, ay;
        double          dist;
        Eigen::Vector2d grad_p = Eigen::Vector2d::Zero();
        Eigen::Vector2d grad_v = Eigen::Vector2d::Zero();
        Eigen::Vector2d grad_a = Eigen::Vector2d::Zero();
        Eigen::Vector3d grad_se2 = Eigen::Vector3d::Zero();
        Eigen::Vector2d grad_dist = Eigen::Vector2d::Zero();

        double          grad_yaw = 0.0;
        double          grad_dyaw = 0.0;
        double          grad_d2yaw = 0.0;
        double          aug_grad = 0.0;
        double          grad_vx2 = 0.0;
        double          grad_wz = 0.0;
        double          grad_ax = 0.0;
        double          grad_ay = 0.0;

        Eigen::Matrix<double, 6, 1> beta0_xy, beta1_xy, beta2_xy, beta3_xy;
        Eigen::Matrix<double, 6, 1> beta0_yaw, beta1_yaw, beta2_yaw, beta3_yaw;
        double s1;
        double s1_yaw;
        double step, alpha, omega;

        int equal_idx = 0;
        int non_equal_idx = 0;
        int constrain_idx = 0;
        int yaw_idx = 0;
        double base_time = 0.0;
        for (int i=0; i<piece_xy; i++)
        {
            const Eigen::Matrix<double, 6, 2> &c_xy = minco_se2.pos_minco.getCoeffs().block<6, 2>(i * 6, 0);
            step = minco_se2.pos_minco.T1(i) / int_K; // int_K是每段点的个数
            s1 = 0.0;

            for (int j=0; j<=int_K; j++)
            {
                alpha = 1.0 / int_K * j;

                // set zero
                grad_p.setZero();
                grad_v.setZero();
                grad_a.setZero();
                grad_yaw = 0.0;
                grad_dyaw = 0.0;
                grad_d2yaw = 0.0;
                grad_vx2 = 0.0;
                grad_wz = 0.0;
                grad_ax = 0.0;
                grad_ay = 0.0;
                grad_se2.setZero();

                // analyse xy
                computeBeta(s1, beta0_xy, beta1_xy, beta2_xy, beta3_xy);
                pos = c_xy.transpose() * beta0_xy;
                vel = c_xy.transpose() * beta1_xy;
                acc = c_xy.transpose() * beta2_xy;
                jer = c_xy.transpose() * beta3_xy;

                // analyse yaw
                double now_time = s1 + base_time; // base_time是每段开始的时间戳
                yaw_idx = int((now_time) / minco_se2.yaw_minco.T1(i)); // 计算yaw对应的是哪段
                if (yaw_idx >= piece_yaw)
                    yaw_idx = piece_yaw - 1;
                // 对应段多项式的系数
                const Eigen::Matrix<double, 6, 1> &c_yaw = minco_se2.yaw_minco.getCoeffs().block<6, 1>(yaw_idx * 6, 0);
                s1_yaw = now_time - yaw_idx * minco_se2.yaw_minco.T1(i);
                computeBeta(s1_yaw, beta0_yaw, beta1_yaw, beta2_yaw, beta3_yaw);

                // 当前时刻的角度，角速度和角加速度
                yaw = c_yaw.transpose() * beta0_yaw;
                dyaw = c_yaw.transpose() * beta1_yaw;
                d2yaw = c_yaw.transpose() * beta2_yaw;

                // analyse complex variable
                syaw = sin(yaw);
                cyaw = cos(yaw);
                v_norm = vel.norm();
                xb = Eigen::Vector2d(cyaw, syaw);
                yb = Eigen::Vector2d(-syaw, cyaw);
                lon_acc = acc.dot(xb);
                lat_acc = acc.dot(yb);


                vx = v_norm;
                wz = dyaw;
                ax = lon_acc;
                ay = lat_acc;
                curv_snorm = wz * wz / (vx*vx + delta_sigl); // wz^2 / (vx^2 + δ)
                sdf_map->evaluateEDTWithGrad(pos, -1.0, dist, grad_dist);
                //std::cout << "dist=" << dist << ",grad_dist=" << grad_dist(0) << "," << grad_dist(1) << std::endl;
                // hx是等式约束，gx是不等式约束
                // non-holonomic
                // 这里保证了xy和yaw的方向不会冲突，因为只有速度方向和yaw的方向一致这项才会趋于0
                double nonh_lambda = lambda[equal_idx];
                Eigen::Vector2d non_holonomic_yaw(syaw, -cyaw);
                hx[equal_idx] = vel.dot(non_holonomic_yaw) * scale_cx(constrain_idx);
                cost += getAugmentedCost(hx[equal_idx], nonh_lambda);
                double nonh_grad = getAugmentedGrad(hx[equal_idx], nonh_lambda) * scale_cx(constrain_idx);
                grad_v += nonh_grad * non_holonomic_yaw;
                grad_yaw += nonh_grad * vel.dot(xb);
                equal_idx++;
                constrain_idx++;


                // longitude velocity
                double v_mu = mu[non_equal_idx];
                gx[non_equal_idx] = (vx*vx - max_vel*max_vel) * scale_cx(constrain_idx);
                if (rho * gx[non_equal_idx] + v_mu > 0)
                {
                    cost += getAugmentedCost(gx[non_equal_idx], v_mu);
                    aug_grad = getAugmentedGrad(gx[non_equal_idx], v_mu) * scale_cx(constrain_idx);
                    grad_vx2 += aug_grad;
                }
                else
                {
                    cost += -0.5 * v_mu * v_mu / rho;
                }
                non_equal_idx++;
                constrain_idx++;

                // longitude acceleration
                double lona_mu = mu[non_equal_idx];
                gx[non_equal_idx] = (ax*ax - max_acc_lon*max_acc_lon) * scale_cx(constrain_idx);
                if (rho * gx[non_equal_idx] + lona_mu > 0) // 判断是否超过约束的条件吧,gx要求<=0
                {
                    cost += getAugmentedCost(gx[non_equal_idx], lona_mu);
                    aug_grad = getAugmentedGrad(gx[non_equal_idx], lona_mu) * scale_cx(constrain_idx);
                    grad_ax += aug_grad * 2.0 * ax;
                }
                else
                {
                    cost += -0.5 * lona_mu * lona_mu / rho;
                }
                non_equal_idx++;
                constrain_idx++;

                // latitude acceleration
                double lata_mu = mu[non_equal_idx];
                gx[non_equal_idx] = (ay*ay - max_acc_lat*max_acc_lat) * scale_cx(constrain_idx);
                if (rho * gx[non_equal_idx] + lata_mu > 0)
                {
                    cost += getAugmentedCost(gx[non_equal_idx], lata_mu);
                    aug_grad = getAugmentedGrad(gx[non_equal_idx], lata_mu) * scale_cx(constrain_idx);
                    grad_ay += aug_grad * 2.0 * ay;
                }
                else
                {
                    cost += -0.5 * lata_mu * lata_mu / rho;
                }
                non_equal_idx++;
                constrain_idx++;

                // obstacle distance
                double dist_mu = mu[non_equal_idx];
                gx[non_equal_idx] = (dist0*dist0 - dist*dist) * scale_cx(constrain_idx);

                if (rho * gx[non_equal_idx] + dist_mu > 0 && dist < dist0)
                {
                    cost += getAugmentedCost(gx[non_equal_idx], dist_mu);
                    aug_grad = getAugmentedGrad(gx[non_equal_idx], dist_mu) * scale_cx(constrain_idx);
                    grad_p += -aug_grad * 2.0 * dist * grad_dist;
                }
                else
                {
                    cost += -0.5 * dist_mu * dist_mu / rho;
                }
                non_equal_idx++;
                constrain_idx++;

                // curvature 曲率 这就是车的转角约束啊！！！
                double curv_mu = mu[non_equal_idx];
                if (use_scaling)
                    gx[non_equal_idx] = (curv_snorm - max_kap*max_kap) * scale_cx(constrain_idx);
                else
                    gx[non_equal_idx] = (curv_snorm - max_kap*max_kap) * cur_scale;
                if (rho * gx[non_equal_idx] + curv_mu > 0)
                {
                    double denominator = 1.0 / (vx*vx + delta_sigl);
                    cost += getAugmentedCost(gx[non_equal_idx], curv_mu);
                    if (use_scaling)
                        aug_grad = getAugmentedGrad(gx[non_equal_idx], curv_mu) * scale_cx(constrain_idx);
                    else
                        aug_grad = getAugmentedGrad(gx[non_equal_idx], curv_mu) * cur_scale;
                    // g对wz的导数
                    grad_wz += aug_grad * denominator * 2.0 * wz;
                    grad_vx2 -= aug_grad * curv_snorm * denominator;
                }
                else
                {
                    cost += -0.5 * curv_mu * curv_mu / rho;
                }
                non_equal_idx++;
                constrain_idx++;


                // process with vx, wz, ax
                grad_v += grad_vx2 * 2.0 * vel; //  把vx2的导数传给v
                //std::cout << "grad_v=" << grad_v(0) << "," << grad_v(1) << std::endl;
                grad_dyaw += grad_wz;

                grad_a += grad_ax * xb;
                grad_yaw += grad_ax * lat_acc;

                grad_a += grad_ay * yb;
                grad_yaw -= grad_ay * lon_acc;

                //K(c,T) = jerk_cost + constrain_cost,这里是在算∂K/∂constrain_cost
                // add all grad into C,T
                // note that xy = Cxy*β(j/K*T_xy), yaw = Cyaw*β(i*T_xy+j/K*T_xy-yaw_idx*T_yaw)
                // ∂p/∂Cxy, ∂v/∂Cxy, ∂a/∂Cxy
                gdCxy.block<6, 2>(i * 6, 0) += (beta0_xy * grad_p.transpose() + \
                                                beta1_xy * grad_v.transpose() + \
                                                beta2_xy * grad_a.transpose());
                // ∂p/∂Txy, ∂v/∂Txy, ∂a/∂Txy  P对T的导数就是v，以此类推
                gdTxy(i) += (grad_p.dot(vel) + \
                             grad_v.dot(acc) + \
                             grad_a.dot(jer) ) * alpha;
                // ∂yaw/∂Cyaw, ∂dyaw/∂Cyaw, ∂d2yaw/∂Cyaw
                gdCyaw.block<6, 1>(yaw_idx * 6, 0) += (beta0_yaw * grad_yaw + \
                                                       beta1_yaw * grad_dyaw + \
                                                       beta2_yaw * grad_d2yaw);
                // ∂yaw/∂Tyaw, ∂dyaw/∂Tyaw, ∂d2yaw/∂Tyaw
                gdTyaw(yaw_idx) += -(grad_yaw * dyaw +
                                     grad_dyaw * d2yaw) * yaw_idx;
                // ∂yaw/∂Txy, ∂dyaw/∂Txy, ∂d2yaw/∂Txy
                gdTxy(i) += (grad_yaw * dyaw +
                             grad_dyaw * d2yaw) * (alpha+i);
                
                s1 += step;
            }
            base_time += minco_se2.pos_minco.T1(i);
        }
    }

    void ALMTrajOpt::computeBeta(double s, Eigen::Matrix<double, 6, 1> &beta0, Eigen::Matrix<double, 6, 1> &beta1,
                     Eigen::Matrix<double, 6, 1> &beta2, Eigen::Matrix<double, 6, 1> &beta3){
        double s2 = s * s, s3 = s2 * s, s4 = s2 * s2, s5 = s4 * s;
        beta0 << 1.0, s, s2, s3, s4, s5;
        beta1 << 0.0, 1.0, 2.0 * s, 3.0 * s2, 4.0 * s3, 5.0 * s4;
        beta2 << 0.0, 0.0, 2.0, 6.0 * s, 12.0 * s2, 20.0 * s3;
        beta3 << 0.0, 0.0, 0.0, 6.0, 24.0 * s, 60.0 * s2;
    }
    static int earlyExit(void* ptrObj, const Eigen::VectorXd& x, const Eigen::VectorXd& grad, 
                         const double fx, const double step, int k, int ls)
    {
        ALMTrajOpt& obj = *(ALMTrajOpt*)ptrObj;
        if (obj.in_debug)
        {
            const double& tau = x(0);
            Eigen::Map<const Eigen::MatrixXd> Pxy(x.data()+obj.dim_T, 2, obj.piece_xy-1);
            Eigen::Map<const Eigen::MatrixXd> Pyaw(x.data()+obj.dim_T+2*(obj.piece_xy-1), 1, obj.piece_yaw-1);
            
            // get T from τ, generate MINCO trajectory
            Eigen::VectorXd Txy, Tyaw;
            Txy.resize(obj.piece_xy);
            Tyaw.resize(obj.piece_yaw);
            obj.calTfromTau(tau, Txy);
            obj.calTfromTau(tau, Tyaw);
            obj.minco_se2.generate(obj.init_xy, obj.end_xy, Pxy, Txy, \
                                obj.init_yaw, obj.end_yaw, Pyaw, Tyaw);
            auto traj = obj.minco_se2.getTraj();
            obj.pubDebugTraj(traj);

            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
        return k > 1e3;
    }

    void ALMTrajOpt::pubDebugTraj(const SE2Trajectory& traj)
    {
        int id = 0;
        double scale = 0.03;
        visualization_msgs::Marker sphere, line_strip;
        sphere.header.frame_id = line_strip.header.frame_id = "world";
        sphere.header.stamp = line_strip.header.stamp = ros::Time::now();
        sphere.type = visualization_msgs::Marker::SPHERE_LIST;
        line_strip.type = visualization_msgs::Marker::LINE_STRIP;
        sphere.action = line_strip.action = visualization_msgs::Marker::ADD;
        sphere.id = id;
        line_strip.id = id + 1000;
        id++;

        sphere.pose.orientation.w = line_strip.pose.orientation.w = 1.0;
        sphere.color.r = line_strip.color.r = 1;
        sphere.color.g = line_strip.color.g = 0;
        sphere.color.b = line_strip.color.b = 1;
        sphere.color.a = line_strip.color.a = 1;
        sphere.scale.x = scale;
        sphere.scale.y = scale;
        sphere.scale.z = scale;
        line_strip.scale.x = scale / 2;
        geometry_msgs::Point pt;

        double dur = traj.pos_traj.getTotalDuration();
        for (double i = 0; i < dur - 1e-4; i+=0.02)
        {
            Eigen::Vector2d dur_p = traj.pos_traj.getValue(i);
            pt.x = dur_p(0);
            pt.y = dur_p(1);
            pt.z = 0.0;
            line_strip.points.push_back(pt);
        }
        Eigen::VectorXd ts = traj.pos_traj.getDurations();
        double time = 0.0;
        for (int i=0; i<ts.size(); i++)
        {
            time += ts(i);
            Eigen::Vector2d dur_p = traj.pos_traj.getValue(time);
            pt.x = dur_p(0);
            pt.y = dur_p(1);
            pt.z = 0.0;
            sphere.points.push_back(pt); 
        }
        debug_pub.publish(sphere);
        debug_pub.publish(line_strip);
    }

}