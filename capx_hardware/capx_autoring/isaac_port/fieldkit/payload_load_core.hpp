#pragma once
// Pure request validation. No SDK connection and no simulated mass defaults.
#include <Eigen/Dense>
#include <Eigen/Eigenvalues>
#include <array>
#include <cmath>
#include <istream>
#include <stdexcept>
#include <string>

namespace m80_payload {
struct Parameters {
  double mass{};
  std::array<double,3> center{};  // flange F -> load COM, metres
  std::array<double,9> inertia{}; // about COM, F axes, column major kg m^2
};
struct Request { Parameters tool, before, after; std::array<double,16> F_T_EE{}; };

inline void validate(const Parameters& p) {
  if(!std::isfinite(p.mass)||p.mass<0||p.mass>5.)throw std::runtime_error("Payload mass must be explicit finite 0..5kg");
  for(double x:p.center)if(!std::isfinite(x)||std::abs(x)>1.)throw std::runtime_error("Explicit flange-frame COM in metres required");
  for(double x:p.inertia)if(!std::isfinite(x))throw std::runtime_error("Explicit finite COM inertia required");
  Eigen::Map<const Eigen::Matrix<double,3,3,Eigen::ColMajor>> I(p.inertia.data());
  if((I-I.transpose()).norm()>1e-10)throw std::runtime_error("COM inertia must be symmetric");
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(I);
  if(solver.info()!=Eigen::Success)throw std::runtime_error("Inertia decomposition failed");
  const auto eigen=solver.eigenvalues();
  if(eigen[0]<-1e-12||eigen[2]>eigen[0]+eigen[1]+1e-10)throw std::runtime_error("Physical PSD inertia and principal-moment triangle inequality required");
  if(p.mass==0.) {
    for(double x:p.center)if(x!=0.)throw std::runtime_error("Zero payload requires zero COM");
    for(double x:p.inertia)if(x!=0.)throw std::runtime_error("Zero payload requires zero inertia");
  } else if(eigen[0]<=0.)throw std::runtime_error("Positive mass requires positive COM inertia; missing inertia is not zero");
}

inline Request parse(std::istream& stream) {
  int version=0;if(!(stream>>version)||version!=1)throw std::runtime_error("Explicit payload request schema 1 required");
  Request request;
  for(auto* p:{&request.tool,&request.before,&request.after}) {
    if(!(stream>>p->mass))throw std::runtime_error("Missing measured payload mass");
    for(auto& x:p->center)if(!(stream>>x))throw std::runtime_error("Missing measured COM");
    for(auto& x:p->inertia)if(!(stream>>x))throw std::runtime_error("Missing measured inertia");
    validate(*p);
  }
  for(auto& x:request.F_T_EE)if(!(stream>>x)||!std::isfinite(x))throw std::runtime_error("Measured flange-to-TCP frame required");
  Eigen::Map<const Eigen::Matrix<double,4,4,Eigen::ColMajor>> frame(request.F_T_EE.data());
  const Eigen::Matrix3d rotation=frame.topLeftCorner<3,3>();
  if((rotation.transpose()*rotation-Eigen::Matrix3d::Identity()).norm()>1e-8||std::abs(rotation.determinant()-1.)>1e-8||
     (frame.row(3)-Eigen::RowVector4d(0.,0.,0.,1.)).norm()>1e-8)
    throw std::runtime_error("Measured proper flange-to-TCP transform required");
  std::string trailing;if(stream>>trailing)throw std::runtime_error("Trailing payload request fields");
  if(request.tool.mass<=0.)throw std::runtime_error("Measured configured Hand/tool mass required separately from payload");
  if(request.tool.mass+request.before.mass>3.||request.tool.mass+request.after.mass>3.)
    throw std::runtime_error("FR3 3kg total tool-plus-payload ceiling exceeded");
  return request;
}

inline bool same(const Parameters& a,const Parameters& b) {
  if(!std::isfinite(a.mass)||std::abs(a.mass-b.mass)>1e-6)return false;
  for(std::size_t j=0;j<3;++j)if(!std::isfinite(a.center[j])||std::abs(a.center[j]-b.center[j])>1e-7)return false;
  for(std::size_t j=0;j<9;++j)if(!std::isfinite(a.inertia[j])||std::abs(a.inertia[j]-b.inertia[j])>1e-9)return false;
  return true;
}
} // namespace m80_payload
