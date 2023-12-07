#include <hw_tool.h>

using namespace std;
using namespace Eigen;

void Homeworktool::initGridMap(double _resolution, Vector3d global_xyz_l, Vector3d global_xyz_u, int max_x_id, int max_y_id, int max_z_id)
{   
    gl_xl = global_xyz_l(0);
    gl_yl = global_xyz_l(1);
    gl_zl = global_xyz_l(2);

    gl_xu = global_xyz_u(0);
    gl_yu = global_xyz_u(1);
    gl_zu = global_xyz_u(2);
    
    GLX_SIZE = max_x_id;
    GLY_SIZE = max_y_id;
    GLZ_SIZE = max_z_id;
    GLYZ_SIZE  = GLY_SIZE * GLZ_SIZE;
    GLXYZ_SIZE = GLX_SIZE * GLYZ_SIZE;

    resolution = _resolution;
    inv_resolution = 1.0 / _resolution;    

    data = new uint8_t[GLXYZ_SIZE];
    memset(data, 0, GLXYZ_SIZE * sizeof(uint8_t));
}

void Homeworktool::setObs(const double coord_x, const double coord_y, const double coord_z)
{   
    if( coord_x < gl_xl  || coord_y < gl_yl  || coord_z <  gl_zl || 
        coord_x >= gl_xu || coord_y >= gl_yu || coord_z >= gl_zu )
        return;

    int idx_x = static_cast<int>( (coord_x - gl_xl) * inv_resolution);
    int idx_y = static_cast<int>( (coord_y - gl_yl) * inv_resolution);
    int idx_z = static_cast<int>( (coord_z - gl_zl) * inv_resolution);      
    
    data[idx_x * GLYZ_SIZE + idx_y * GLZ_SIZE + idx_z] = 1;
}

bool Homeworktool::isObsFree(const double coord_x, const double coord_y, const double coord_z)
{
    Vector3d pt;
    Vector3i idx;
    
    pt(0) = coord_x;
    pt(1) = coord_y;
    pt(2) = coord_z;
    idx = coord2gridIndex(pt);

    int idx_x = idx(0);
    int idx_y = idx(1);
    int idx_z = idx(2);

    return (idx_x >= 0 && idx_x < GLX_SIZE && idx_y >= 0 && idx_y < GLY_SIZE && idx_z >= 0 && idx_z < GLZ_SIZE && 
           (data[idx_x * GLYZ_SIZE + idx_y * GLZ_SIZE + idx_z] < 1));
}

Vector3d Homeworktool::gridIndex2coord(const Vector3i & index) 
{
    Vector3d pt;

    pt(0) = ((double)index(0) + 0.5) * resolution + gl_xl;
    pt(1) = ((double)index(1) + 0.5) * resolution + gl_yl;
    pt(2) = ((double)index(2) + 0.5) * resolution + gl_zl;

    return pt;
}

Vector3i Homeworktool::coord2gridIndex(const Vector3d & pt) 
{
    Vector3i idx;
    idx <<  min( max( int( (pt(0) - gl_xl) * inv_resolution), 0), GLX_SIZE - 1),
            min( max( int( (pt(1) - gl_yl) * inv_resolution), 0), GLY_SIZE - 1),
            min( max( int( (pt(2) - gl_zl) * inv_resolution), 0), GLZ_SIZE - 1);                  
  
    return idx;
}

Eigen::Vector3d Homeworktool::coordRounding(const Eigen::Vector3d & coord)
{
    return gridIndex2coord(coord2gridIndex(coord));
}

double Homeworktool::OptimalBVP(Eigen::Vector3d _start_position,Eigen::Vector3d _start_velocity,Eigen::Vector3d _target_position)
{
    double optimal_cost = 100000; // this just to initial the optimal_cost, you can delete it 
    /*
    STEP 2: go to the hw_tool.cpp and finish the function Homeworktool::OptimalBVP
    the solving process has been given in the document
    because the final point of trajectory is the start point of OBVP, so we input the pos,vel to the OBVP
    after finish Homeworktool::OptimalBVP, the Trajctory_Cost will record the optimal cost of this trajectory
    */

    double dx = _target_position.x() - _start_position.x();
    double dy = _target_position.y() - _start_position.y();
    double dz = _target_position.z() - _start_position.z();

    // assume that vf = 0
    double delta_vx = 0 - _start_velocity.x();
    double delta_vy = 0 - _start_velocity.y();
    double delta_vz = 0 - _start_velocity.z();

    double vx0 = _start_velocity.x();
    double vy0 = _start_velocity.y();
    double vz0 = _start_velocity.z();

    // to get the optimal T, we should solve this equation:
    // T^4 - (4B + 12D + 12F) * T^2 + (24C + 48E) * T - 36A = 0
    // for more details in mathematical derivation, see document 'obvp_derivation.pdf' in 'ros/' directory
    double A = dx * dx + dy * dy + dz * dz;
    double B = delta_vx * delta_vx + delta_vy * delta_vy + delta_vz * delta_vz;
    double C = dx * delta_vx + dy * delta_vy + dz * delta_vz;
    double D = vx0 * vx0 + vy0 * vy0 + vz0 * vz0;
    double E = vx0 * dx + vy0 * dy + vz0 * dz;
    double F = vx0 * delta_vx + vy0 * delta_vy + vz0 * delta_vz;

    // define the adjoint matrix parameters
    double c0 = - 36 * A;
    double c1 = 24 * C + 48 * E;
    double c2 = - 4 * B - 12 * D - 12 * F;
    double c3 = 0;

    // construct adjoint matrix of the polynomial equation
    Eigen::Matrix<double, 4, 4> adjoint_matrix;
    adjoint_matrix << 0, 0, 0, -c0,
                      1, 0, 0, -c1,
                      0, 1, 0, -c2,
                      0, 0, 1, -c3;
    Eigen::Matrix<complex<double>, Eigen::Dynamic, Eigen::Dynamic> eigen_values;
    eigen_values = adjoint_matrix.eigenvalues();
    double T = 100000; // set initial T as a large number
    double T_opt = T;
    double T_cost = optimal_cost;
    for (int i = 0; i < eigen_values.size(); i++)
    {
        if (eigen_values(i).real() > 0 && eigen_values(i).imag() < 1e-16) // T_optimal shouldn't be negative or complex number
        {
            T = eigen_values(i).real();
            T_cost = T + 12 * A / pow(T, 3) - (12 * C + 24 * E) / pow(T, 2) + (4 * B + 12 * D + 12 * F) / T;
            if (T_cost < optimal_cost)
            {
                T_opt = T;
                optimal_cost = T_cost;
            }
        }
    }
    ROS_INFO("the optimal_cost is: ");
    std::cout << optimal_cost << endl;

    return optimal_cost;
}
