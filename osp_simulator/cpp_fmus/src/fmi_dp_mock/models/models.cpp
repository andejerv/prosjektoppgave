#include "models.hpp"

#include <zeabuz/common/utilities/math.hpp>
#include <zeabuz/common/utilities/string.hpp>

using zeabuz::common::utilities::math::heading_to_rotation_matrix;
using zeabuz::common::utilities::math::inf2pipi;

ThreeDOFModel::ThreeDOFModel(const Eigen::Vector3d& initNorthEastHead, const Eigen::Vector3d& initSurgeSwayYawRate)
{
    m_NorthEastHeading = initNorthEastHead;
    m_SurgeSwayYawRate = initSurgeSwayYawRate;

    // Simplified ma2 model
    m_Mmatrix << 6891.9, 0.0, 0.0, 0.0, 7059.1, 0.0, 0.0, 0.0, 3863.4;
    m_Minverse = m_Mmatrix.inverse();

    m_DLinmatrix << 238.6, 0.0, 0.0, 0.0, 65.9, 0.0, 0.0, 0.0, 880.1;

    m_CMatrix = Eigen::Matrix3d::Zero(3, 3);
    m_Dmatrix = m_DLinmatrix;
}

void ThreeDOFModel::step(const Eigen::Vector3d& tau, const std::chrono::duration<double>& stepSize)
{
    Eigen::Vector3d nu_dot = m_Minverse * (tau - m_CMatrix * m_SurgeSwayYawRate - m_Dmatrix * m_SurgeSwayYawRate);
    m_SurgeSwayYawRate += nu_dot * stepSize.count();

    Eigen::Vector3d eta_dot = heading_to_rotation_matrix(m_NorthEastHeading(2)) * m_SurgeSwayYawRate;
    m_NorthEastHeading = m_NorthEastHeading + eta_dot * stepSize.count();
    m_NorthEastHeading(2) = inf2pipi(m_NorthEastHeading(2));
}
Eigen::Vector3d ThreeDOFModel::getNorthEastHeading()
{
    return m_NorthEastHeading;
}
Eigen::Vector3d ThreeDOFModel::getSurgeSwayYawRate()
{
    return m_SurgeSwayYawRate;
}

void ThreeDOFModel::setM(const Eigen::Matrix3d M)
{
    m_Mmatrix = M;
}
void ThreeDOFModel::setD(const Eigen::Matrix3d D)
{
    m_Dmatrix = D;
}
