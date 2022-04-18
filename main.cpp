#include <iostream>
#include <vector>
#include <string>
#include <algorithm>
#include "../AirLib/include/vehicles/multirotor/api/MultirotorRpcLibClient.hpp"
#include "../AirLib/deps/rpclib/include/rpc/client.h"
#include "../AirLib/deps/rpclib/include/rpc/msgpack.hpp"
#include "../AirLib/include/api/RpcLibAdapatorsBase.hpp"


struct trajectory{
    Eigen::Vector3f pos_d;
    Eigen::Vector3f dpos_d;
    Eigen::Vector3f ddpos_d;
    Eigen::Vector3f dddpos_d;
};
Eigen::Matrix<float,4,1> pose_control(const Kinematics::State &state, const Eigen::Vector3f &pos_d, const msr::airlib::Environment::State &env);
Eigen::Vector4f forces_to_pwm(const Eigen::Vector4f &f_u,const msr::airlib::Environment::State &env);
Eigen::Vector4f trajectory_tracking(const Kinematics::State &state, const struct trajectory& traj, const msr::airlib::Environment::State &env);
void get_gate_poses(rpc::client &rpc_client, std::vector<std::vector<Eigen::Vector3f>> &A, Eigen::Vector3f &ipos);
void sim_start_race(rpc::client &rpc_client, int tier);
void sim_reset_race(rpc::client &rpc_client);
struct trajectory get_quintic_hermite_trajectory(float t,int i, std::vector<std::vector<Eigen::Vector3f>> &A);
double calc_quintic_segment_length(double l,double t1, double t2,std::vector<Eigen::Vector3f> W1,std::vector<Eigen::Vector3f> W2);
double calc_quintic_path_parameter(double t,double l, std::vector<Eigen::Vector3f> W1,std::vector<Eigen::Vector3f> W2);


int main()
{
    using namespace std;
    msr::airlib::MultirotorRpcLibClient client;
    rpc::client rpc_client("127.0.0.1",41451);
    rpc_client.call("simLoadLevel","Qualifier_Tier_1");
    msr::airlib::Environment::State env = client.simGetGroundTruthEnvironment();
    cout << env.air_density << "," << env.gravity << "," << env.air_pressure << "," << env.temperature << '\n';
    cout << "Press Enter to enable API control "; cin.get();
    client.enableApiControl(true);
    std::vector<std::vector<Eigen::Vector3f>> gates;
    Eigen::Vector3f ipos = client.getMultirotorState().kinematics_estimated.pose.position;
    get_gate_poses(rpc_client,gates,ipos);
    double total_length = 0;
    std::vector<int> gate_lengths;
    for(int i = 0; i < gates.size()-1;++i){
        double len = 0.22;
        double s = calc_quintic_segment_length(len/100,0,1,gates[i],gates[i+1]);
        total_length += s;
        gate_lengths.push_back((s/len+1));
    }
    cout << "Total length: " << total_length << '\n';
    cout << "Press Enter to arm the drone "; cin.get();
    client.armDisarm(true);

    client.setPositionControllerGains({0.0,0.0,0.0},{0.0,0.0,0.0},{0.0,0.0,0.0});
    client.setVelocityControllerGains({0.0,0.0,0.0},{0.0,0.0,0.0},{0.0,0.0,0.0});
    client.setAngleLevelControllerGains({0.0,0.0,0.0},{0.0,0.0,0.0},{0.0,0.0,0.0});
    client.setAngleRateControllerGains({0.0,0.0,0.0},{0.0,0.0,0.0},{0.0,0.0,0.0});
    Eigen::Vector3f pos_d;
    bool race_started = false;
    sim_start_race(rpc_client,1);
    pos_d = client.getMultirotorState().kinematics_estimated.pose.position+Eigen::Vector3f{0,0,-1};
    for(;;){
        if(race_started){
            cout << "number of gates: " << gates.size() << '\n';
            for(int i = 0;i < gates.size()-1;++i){
                double t = 0;
                double len = 0.22;
                int N = gate_lengths[i];

                cout << "current gate: " << i << '\n';
                cout << gates[i][0] << " : " << gates[i+1][1] << '\n';
                std::cout << "Segment " << i << " length: " << N << '\n';
                Eigen::Vector4f f_u(9.81,0,0,0);
                for(int n = 0; n<N;++n){
                    struct trajectory traj = get_quintic_hermite_trajectory(t,i,gates);
                    traj.dpos_d *= 1.0/N;
                    traj.ddpos_d *= 1.0/N;
                    traj.dddpos_d *= 1.0/N;
                    f_u = trajectory_tracking(client.getMultirotorState().kinematics_estimated, traj, env);
                    auto pwms = forces_to_pwm(f_u,env);
                    client.moveByMotorPWMsAsync(pwms[0],pwms[1],pwms[2],pwms[3],1/1000.0)->waitOnLastTask();
                    t = calc_quintic_path_parameter(t,len,gates[i],gates[i+1]);
                    if(t > 0.99)
                        break;
                }
            }
            break;
        } else {
            while(!race_started){
                auto f_u = pose_control(client.getMultirotorState().kinematics_estimated, pos_d, env);
                auto pwms = forces_to_pwm(f_u,env);
                client.moveByMotorPWMsAsync(pwms[0],pwms[1],pwms[2],pwms[3],1.0/100.0)->waitOnLastTask();
                if((pos_d-client.getMultirotorState().kinematics_estimated.pose.position).norm() < 0.5){
                    race_started = true;
                }
            }
        }
    }

    return 0;
}

Eigen::Matrix<float,4,1> pose_control(const Kinematics::State &state, const Eigen::Vector3f &pos_d, const msr::airlib::Environment::State &env)
{
    Eigen::Vector3f pos = state.pose.position;
    Eigen::Quaternion<float,2> q = state.pose.orientation;
    Eigen::Vector3f v = state.twist.linear;
    Eigen::Vector3f w = state.twist.angular;
    float m = 1, kd = 10, kw = 0.1, kq = 0.0, alpha = 0.3, g = env.gravity[2];
    Eigen::Matrix<float,3,3> kp;
    kp << 1,0,0,0,1,0,0,0,30;
    Eigen::Vector3f e3(0,0,1);

    Eigen::Matrix<float,3,1> err_p = pos_d - pos;
    Eigen::Matrix<float,3,1> err_v = -v;
    Eigen::Matrix<float,3,1> f = m*(g*e3-kp*err_p-kd*err_v);

    Eigen::Matrix<float,4,1> f_u(0,0,0,0);
    Eigen::Matrix<float,3,1> tilde_f(0,0,0);

    f_u[0] = f.norm();
    if(f_u[0] > alpha){
        tilde_f = alpha/f_u[0]*f;
    }

    Eigen::Vector3f f_b = q.conjugate()._transformVector(tilde_f);

    f_u.block<3,1>(1,0) = -f_b.cross(e3)-kw*w-kq*q.vec();

    return f_u;
}

Eigen::Vector4f trajectory_tracking(const Kinematics::State &state, const struct trajectory& traj, const msr::airlib::Environment::State &env)
{
    Eigen::Vector3f pos = state.pose.position;
    Eigen::Quaternion<float,2> q = state.pose.orientation;
    Eigen::Vector3f v = state.twist.linear;
    Eigen::Vector3f w = state.twist.angular;
    float m = 1, alpha = 1.5, kw = 0.2, g = env.gravity[2];
    Eigen::Matrix<float,3,3> kp;
    kp << 6,0,0,0,6,0,0,0,20;
    Eigen::Matrix<float,3,3> kd;
    kd <<  4,0,0,0, 4,0,0,0,10;
    Eigen::Vector3f e3(0,0,1);
    Eigen::Matrix<float,3,3> J;

    J << 0.006721187500000001, 0, 0,  0, 0.0080406875, 0,  0, 0, 0.014278874999999998;

    Eigen::Vector3f err_p = traj.pos_d - pos;
    Eigen::Vector3f err_v = traj.dpos_d - v;
    Eigen::Vector3f f = m*(g*e3-kp*err_p-kd*err_v-traj.ddpos_d);

    Eigen::Matrix<float,4,1> f_u(0,0,0,0);
    Eigen::Matrix<float,3,1> tilde_f(0,0,0);

    f_u[0] = f.norm();
    if(f_u[0] > alpha){
        tilde_f = alpha/f_u[0]*f;
    }

    Eigen::Vector3f f_b = q.conjugate()._transformVector(tilde_f);

    f_u.block<3,1>(1,0) = -f_b.cross(e3) - kw*w + w.cross(J*w);

    return f_u;
}
Eigen::Vector4f forces_to_pwm(const Eigen::Vector4f &f_u,const msr::airlib::Environment::State &env)
{
    float propeller_diameter = 0.2286, standard_air_density = 1.225, d = 0.2275; //d is arm_length
    float c_T = 0.109919*pow(propeller_diameter, 4)*standard_air_density;
    float c_Q = 0.040164*pow(propeller_diameter, 5)*standard_air_density/(2*M_PI);

    Eigen::Matrix<float,4,4> M;
    M <<    c_T,    c_T,   c_T,    c_T,
         -d*c_T,  d*c_T, d*c_T, -d*c_T,
          d*c_T, -d*c_T, d*c_T, -d*c_T,
            c_Q,    c_Q,  -c_Q,   -c_Q;

    Eigen::Vector4f thrust =  c_T*M.inverse()*f_u;

    float max_thrust = 4.179446268;
    float air_density_ratio = env.air_density/standard_air_density;
    Eigen::Vector4f pwm;
    float max_pwm = -1000;

    for(int i = 0;i<thrust.size();++i){
        //pwm[i] = std::max<float>(0.0,std::min<float>(1.0,thrust[i]/(air_density_ratio*max_thrust)));
        pwm[i] = std::max<float>(0,thrust[i]/(air_density_ratio*max_thrust));
        max_pwm = std::max<float>(max_pwm,pwm[i]);
    }
    if(max_pwm > 1)
        for(int i = 0;i<pwm.size();++i)
            pwm[i] /= max_pwm;

    return pwm;
}
void get_gate_poses(rpc::client &rpc_client, std::vector<std::vector<Eigen::Vector3f>> &A,Eigen::Vector3f &ipos)
{
    std::vector<std::string> gate_list = rpc_client.call("simListSceneObjects","Gate.*").as<std::vector<string>>();
    std::sort(gate_list.begin(),gate_list.end());
    double tangent_scale = 1.75;
    double curvature_scale = 200;
    for(auto s : gate_list){
        std::cout << s << '\n';
        msr::airlib::Pose gatepose;
        do{
            gatepose = rpc_client.call("simGetObjectPose",s,false).as<msr::airlib_rpclib::RpcLibAdapatorsBase::Pose>().to();
        }while(gatepose.position.hasNaN() || gatepose.orientation.vec().hasNaN());
        Eigen::Quaternionf tangent = gatepose.orientation*Eigen::Quaternionf{0,0,1,0}*gatepose.orientation.conjugate();
        A.push_back({gatepose.position,tangent.vec()});
    }
    for(int i = 0; i < A.size()-1;++i){
        auto dist = (A[i+1][0]-A[i][0]).norm()/tangent_scale;
        A[i][1] *= dist;
        if(i == 11)
            A[i][1]*=tangent_scale;
    }
    A.push_back({A[A.size()-1][0]+A[A.size()-1][1]*5/A[A.size()-1][1].norm(),10*A[A.size()-1][1]});
    A.insert(A.begin(),{ipos+Eigen::Vector3f{0,0,-1},A[0][1]});
    //Setup curvature vectors
    A[0].push_back(curvature_scale/(A[0][0]-A[1][0]).norm()*A[0][1]/A[0][1].norm());
    A[A.size()-1].push_back(curvature_scale/(A[A.size()-1][0]-A[A.size()-2][0]).norm()*A[A.size()-1][1]/A[A.size()-1][1].norm());
    for(int i = 1; i < A.size()-1; ++i){
        auto t1 = (A[i-1][0]-A[i][0])/(A[i-1][0]-A[i][0]).norm();
        auto t2 = (A[i+1][0]-A[i][0])/(A[i+1][0]-A[i][0]).norm();
        auto dist = (A[i+1][0]-A[i-1][0]).norm();
        A[i].push_back(curvature_scale/dist*(t1+t2)/2);
    }
}

void sim_start_race(rpc::client &rpc_client,int tier)
{
    rpc_client.call("simStartRace",tier);
}
void sim_reset_race(rpc::client &rpc_client)
{
    rpc_client.call("simResetRace");
}

double calc_quintic_segment_length(double l,double t1, double t2,std::vector<Eigen::Vector3f> W1,std::vector<Eigen::Vector3f> W2)
{
    /*
     *  H1 = 1-10*t^3+15*t^4-6*t^5;
        H2 = 10*t^3-15*t^4+6*t^5;
        H3 = (t-6*t^3+8*t^4-3*t^5);
        H4 = (-4*t^3+7*t^4-3*t^5);
        H5 = (0.5*t^2-3/2*t^3+3/2*t^4-0.5*t^5);
        H6 = (0.5*t^3-t^4+0.5*t^5);
        */
    auto H1 = [](double t){return 1-10*t*t*t+15*t*t*t*t-6*t*t*t*t*t;};
    auto H2 = [](double t){return 10*t*t*t-15*t*t*t*t+6*t*t*t*t*t;};
    auto H3 = [](double t){return t-6*t*t*t+8*t*t*t*t-3*t*t*t*t*t;};
    auto H4 = [](double t){return -4*t*t*t+7*t*t*t*t-3*t*t*t*t*t;};
    auto H5 = [](double t){return 0.5*t*t-3.0/2.0*t*t*t+3.0/2.0*t*t*t*t-0.5*t*t*t*t*t;};
    auto H6 = [](double t){return 0.5*t*t*t-t*t*t*t+0.5*t*t*t*t*t;};
    double S = 0;
    for(double t = t1; t <= t2-l; t+= l){
        auto p1 = H1(t)*W1[0]+H2(t)*W2[0]+H3(t)*W1[1]+H4(t)*W2[1]+H5(t)*W1[2]+H6(t)*W2[2];
        auto p2 = H1(t+l)*W1[0]+H2(t+l)*W2[0]+H3(t+l)*W1[1]+H4(t+l)*W2[1]+H5(t+l)*W1[2]+H6(t+l)*W2[2];
        S += (p2-p1).norm();
    }
    return S;
}
double calc_quintic_path_parameter(double t,double l, std::vector<Eigen::Vector3f> W1,std::vector<Eigen::Vector3f> W2)
{
    double t1 = t;
    double t2 = t;
    do{
        t2 += l/100;
    }while(calc_quintic_segment_length(0.001,t1,t2,W1,W2)-l < 0);
    if(t2 > 1)
        return 1;
    else
        return t2;
}
struct trajectory get_quintic_hermite_trajectory(float t,int i, std::vector<std::vector<Eigen::Vector3f>> &A)
{
    struct trajectory trac;
    trac.pos_d = {0,0,0};
    trac.dpos_d = {0,0,0};
    trac.ddpos_d = {0,0,0};
    trac.dddpos_d = {0,0,0};

    float H1 = 1-10*t*t*t+15*t*t*t*t-6*t*t*t*t*t;
    float H2 = 10*t*t*t-15*t*t*t*t+6*t*t*t*t*t;
    float H3 = t-6*t*t*t+8*t*t*t*t-3*t*t*t*t*t;
    float H4 = -4*t*t*t+7*t*t*t*t-3*t*t*t*t*t;
    float H5 = 0.5*t*t-3.0/2.0*t*t*t+3.0/2.0*t*t*t*t-0.5*t*t*t*t*t;
    float H6 = 0.5*t*t*t-t*t*t*t+0.5*t*t*t*t*t;

    float DH1 = -30*t*t + 60*t*t*t - 30*t*t*t*t;
    float DH2 = 30*t*t - 60*t*t*t + 30*t*t*t*t;
    float DH3 = 1 - 18*t*t + 32*t*t*t - 15*t*t*t*t;
    float DH4 = -12*t*t + 28*t*t*t - 15*t*t*t*t;
    float DH5 = t - 9.0/2.0*t*t + 12.0/2.0*t*t*t - 2.5*t*t*t*t;
    float DH6 = 1.5*t*t - 4*t*t*t + 2.5*t*t*t*t;

    float DDH1 = -60*t + 180*t*t - 120*t*t*t;
    float DDH2 = 60*t - 180*t*t + 120*t*t*t;
    float DDH3 = -36*t + 96*t*t - 60*t*t*t;
    float DDH4 = -24*t + 84*t*t - 60*t*t*t;
    float DDH5 = 1 - 18.0/2.0*t + 36.0/2.0*t*t - 10*t*t*t;
    float DDH6 = 3*t - 12*t*t + 10*t*t*t;

    float DDDH1 = -60 + 360*t - 360*t*t;
    float DDDH2 = 60 - 360*t + 360*t*t;
    float DDDH3 = -36 + 192*t - 180*t*t;
    float DDDH4 = -24 + 168*t - 180*t*t;
    float DDDH5 = -18.0/2.0 + 36.0*t - 30*t*t;
    float DDDH6 = 3 - 24*t + 30*t*t;

    trac.pos_d = H1*A[i][0] + H2*A[i+1][0] + H3*A[i][1] + H4*A[i+1][1] + H5*A[i][2]+H6*A[i+1][2];
    trac.dpos_d = DH1*A[i][0] + DH2*A[i+1][0] + DH3*A[i][1] + DH4*A[i+1][1] + DH5*A[i][2]+DH6*A[i+1][2];
    trac.ddpos_d = DDH1*A[i][0] + DDH2*A[i+1][0] + DDH3*A[i][1] + DDH4*A[i+1][1] + DDH5*A[i][2]+DDH6*A[i+1][2];
    trac.dddpos_d = DDDH1*A[i][0] + DDDH2*A[i+1][0] + DDDH3*A[i][1] + DDDH4*A[i+1][1] + DDDH5*A[i][2]+DDDH6*A[i+1][2];
    return trac;
}
