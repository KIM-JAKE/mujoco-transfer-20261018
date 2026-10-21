// Same USD joint frames and Rodrigues formula; numeric transforms only.
#include <Eigen/Dense>
#include <cmath>
using M4=Eigen::Matrix4d;
using M3=Eigen::Matrix3d;
using V3=Eigen::Vector3d;
using R4=Eigen::Matrix<double,4,4,Eigen::RowMajor>;
struct Model { M4 base,tip,a[7],b[7]; V3 axes[7]; };
extern "C" void* fk_create(const double* base,const double* tip,const double* frames,const double* axes) {
  auto* m=new Model; m->base=Eigen::Map<const R4>(base); m->tip=Eigen::Map<const R4>(tip);
  for(int j=0;j<7;++j) {m->a[j]=Eigen::Map<const R4>(frames+32*j);m->b[j]=Eigen::Map<const R4>(frames+32*j+16);m->axes[j]=Eigen::Map<const V3>(axes+3*j);}
  return m;
}
extern "C" void fk_destroy(void* p) { delete static_cast<Model*>(p); }
static M4 chain(const Model& m,const double* q,double* bodies) {
  M4 T=m.base;
  for(int j=0;j<7;++j) {
    V3 v=m.axes[j]*q[j]; double t=v.norm(); M4 r=M4::Identity();
    if(t>=1e-12) {V3 n=v/t;M3 K;K<<0.,-n.z(),n.y(),n.z(),0.,-n.x(),-n.y(),n.x(),0.;r.block<3,3>(0,0)=M3::Identity()+std::sin(t)*K+(1.-std::cos(t))*(K*K);}
    T=(T*m.a[j]*r*m.b[j]).eval();
    if(bodies) {Eigen::Map<R4> dst(bodies+16*j);dst=T;}
  }
  return T;
}
extern "C" void fk_tcp(void* p,const double* q,double* out) {
  const auto& m=*static_cast<Model*>(p);Eigen::Map<R4> dst(out);dst=chain(m,q,nullptr)*m.tip;
}
extern "C" void fk_bodies(void* p,const double* q,double opening,double* out) {
  const auto& m=*static_cast<Model*>(p);M4 tcp=chain(m,q,out)*m.tip;M4 offset=M4::Identity();offset(2,3)=-.1034;M4 H=tcp*offset;
  Eigen::Map<R4> hand(out+16*7);hand=H;
  for(int i=0;i<2;++i) {double sign=i==0?1.:-1.;M4 finger=M4::Identity();finger(1,3)=sign*opening/2.;finger(2,3)=.0584;if(i==1){finger(0,0)=-1.;finger(1,1)=-1.;}Eigen::Map<R4> dst(out+16*(8+i));dst=H*finger;}
}
