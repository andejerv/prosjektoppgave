#include <gtest/gtest.h>

#include "fmi_dp_mock/models/models.hpp"

using namespace std::chrono_literals;
class DynamicModelsUnitTests : public ::testing::Test
{
   public:
    DynamicModelsUnitTests() = default;
    void SetUp() override
    {
    }
};

TEST_F(DynamicModelsUnitTests, T01_SetInitialState)
{
    Eigen::Vector3d eta0{1.0, 2.0, 0.5};
    Eigen::Vector3d nu0{0.0, 0.0, 0.0};
    auto model = std::make_unique<ThreeDOFModel>(eta0, nu0);

    Eigen::Vector3d eta1 = model->getNorthEastHeading();
    Eigen::Vector3d nu1 = model->getSurgeSwayYawRate();

    EXPECT_EQ(eta0, eta1);
    EXPECT_EQ(nu0, nu1);
}

TEST_F(DynamicModelsUnitTests, T02_StepModel_ZeroForcesZeroInitVel_StayInPlace)
{
    Eigen::Vector3d eta0{1.0, 2.0, 0.5};
    Eigen::Vector3d nu0{0.0, 0.0, 0.0};
    auto model = std::make_unique<ThreeDOFModel>(eta0, nu0);

    Eigen::Vector3d eta1 = model->getNorthEastHeading();
    Eigen::Vector3d nu1 = model->getSurgeSwayYawRate();

    EXPECT_EQ(eta0, eta1);
    EXPECT_EQ(nu0, nu1);

    Eigen::Vector3d tau{0, 0, 0};

    int nSteps = 50;
    for (int i = 0; i < nSteps; i++) {
        model->step(tau, 0.05s);
        Eigen::Vector3d eta = model->getNorthEastHeading();
        EXPECT_EQ(eta0, eta);
    }
}

TEST_F(DynamicModelsUnitTests, T03_StepModel_ZeroForcesNonZeroVel)
{
    Eigen::Vector3d eta0{1.0, 2.0, 0.5};
    Eigen::Vector3d nu0{0.0, 0.0, 0.1};
    auto model = std::make_unique<ThreeDOFModel>(eta0, nu0);

    Eigen::Vector3d eta1 = model->getNorthEastHeading();
    Eigen::Vector3d nu1 = model->getSurgeSwayYawRate();

    EXPECT_EQ(eta0, eta1);
    EXPECT_EQ(nu0, nu1);

    Eigen::Vector3d tau{0, 0, 0};

    int nSteps = 100;
    for (int i = 0; i < nSteps; i++) {
        model->step(tau, 0.05s);
        // Eigen::Vector3d eta = model->getNorthEastHeading();
    }
}
