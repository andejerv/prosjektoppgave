
#ifndef DYNAMIC_POSITIONING_CPP_MODELS_HPP
#define DYNAMIC_POSITIONING_CPP_MODELS_HPP

#include <Eigen/Dense>
#include <chrono>

class ThreeDOFModel
{
   public:
    ThreeDOFModel() = default;
    ThreeDOFModel(const Eigen::Vector3d& initNorthEastHead, const Eigen::Vector3d& initSurgeSwayYawRate);
    void step(const Eigen::Vector3d& tau, const std::chrono::duration<double>& stepSize);
    Eigen::Vector3d getNorthEastHeading();
    Eigen::Vector3d getSurgeSwayYawRate();

    void setM(Eigen::Matrix3d M);
    void setD(Eigen::Matrix3d D);

   private:
    Eigen::Vector3d m_NorthEastHeading;
    Eigen::Vector3d m_SurgeSwayYawRate;

    Eigen::Matrix3d m_Mmatrix;
    Eigen::Matrix3d m_Minverse;
    Eigen::Matrix3d m_Dmatrix;
    Eigen::Matrix3d m_DLinmatrix;
    Eigen::Matrix3d m_DQuadmatrix;
    Eigen::Matrix3d m_DCubmatrix;
    Eigen::Matrix3d m_CMatrix;
};

#endif  // DYNAMIC_POSITIONING_CPP_MODELS_HPP
