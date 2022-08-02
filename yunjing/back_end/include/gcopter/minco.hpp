/*
    MIT License

    Copyright (c) 2021 Zhepei Wang (wangzhepei@live.com)

    Permission is hereby granted, free of charge, to any person obtaining a copy
    of this software and associated documentation files (the "Software"), to deal
    in the Software without restriction, including without limitation the rights
    to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
    copies of the Software, and to permit persons to whom the Software is
    furnished to do so, subject to the following conditions:

    The above copyright notice and this permission notice shall be included in all
    copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
    SOFTWARE.
*/

#ifndef MINCO_HPP
#define MINCO_HPP

#include "gcopter/trajectory.hpp"

#include <Eigen/Eigen>

#include <cmath>
#include <vector>

namespace minco
{

    // The banded system class is used for solving
    // banded linear system Ax=b efficiently.
    // A is an N*N band matrix with lower band width lowerBw
    // and upper band width upperBw.
    // Banded LU factorization has O(N) time complexity.
    class BandedSystem
    {
    public:
        // The size of A, as well as the lower/upper
        // banded width p/q are needed
        inline void create(const int &n, const int &p, const int &q)
        {
            // In case of re-creating before destroying
            // ROS_INFO("       destroy");
            destroy();
            N = n;
            lowerBw = p;
            upperBw = q;
            int actualSize = N * (lowerBw + upperBw + 1);
            ptrData = new double[actualSize];
            std::fill_n(ptrData, actualSize, 0.0);
            return;
        }

        inline void destroy()
        {
            if (ptrData != nullptr)
            {
                // ROS_INFO("        delete[] ptrData;");
                // std::cout<<sizeof(ptrData)/sizeof(double)<< " " <<*ptrData<<std::endl;
                delete[] ptrData;
                // ROS_INFO("        ptrData = nullptr");
                ptrData = nullptr;
            }
            return;
        }

    private:
        int N;
        int lowerBw;
        int upperBw;
        // Compulsory nullptr initialization here
        double *ptrData = nullptr;

    public:
        // Reset the matrix to zero
        inline void reset(void)
        {
            std::fill_n(ptrData, N * (lowerBw + upperBw + 1), 0.0);
            return;
        }

        // The band matrix is stored as suggested in "Matrix Computation"
        inline const double &operator()(const int &i, const int &j) const
        {
            return ptrData[(i - j + upperBw) * N + j];
        }

        inline double &operator()(const int &i, const int &j)
        {
            return ptrData[(i - j + upperBw) * N + j];
        }

        // This function conducts banded LU factorization in place
        // Note that NO PIVOT is applied on the matrix "A" for efficiency!!!
        inline void factorizeLU()
        {
            int iM, jM;
            double cVl;
            for (int k = 0; k <= N - 2; k++)
            {
                iM = std::min(k + lowerBw, N - 1);
                cVl = operator()(k, k);
                for (int i = k + 1; i <= iM; i++)
                {
                    if (operator()(i, k) != 0.0)
                    {
                        operator()(i, k) /= cVl;
                    }
                }
                jM = std::min(k + upperBw, N - 1);
                for (int j = k + 1; j <= jM; j++)
                {
                    cVl = operator()(k, j);
                    if (cVl != 0.0)
                    {
                        for (int i = k + 1; i <= iM; i++)
                        {
                            if (operator()(i, k) != 0.0)
                            {
                                operator()(i, j) -= operator()(i, k) * cVl;
                            }
                        }
                    }
                }
            }
            return;
        }

        // This function solves Ax=b, then stores x in b
        // The input b is required to be N*m, i.e.,
        // m vectors to be solved.
        template <typename EIGENMAT>
        inline void solve(EIGENMAT &b) const
        {
            int iM;
            for (int j = 0; j <= N - 1; j++)
            {
                iM = std::min(j + lowerBw, N - 1);
                for (int i = j + 1; i <= iM; i++)
                {
                    if (operator()(i, j) != 0.0)
                    {
                        b.row(i) -= operator()(i, j) * b.row(j);
                    }
                }
            }
            for (int j = N - 1; j >= 0; j--)
            {
                b.row(j) /= operator()(j, j);
                iM = std::max(0, j - upperBw);
                for (int i = iM; i <= j - 1; i++)
                {
                    if (operator()(i, j) != 0.0)
                    {
                        b.row(i) -= operator()(i, j) * b.row(j);
                    }
                }
            }
            return;
        }

        // This function solves ATx=b, then stores x in b
        // The input b is required to be N*m, i.e.,
        // m vectors to be solved.
        template <typename EIGENMAT>
        inline void solveAdj(EIGENMAT &b) const
        {
            int iM;
            for (int j = 0; j <= N - 1; j++)
            {
                b.row(j) /= operator()(j, j);
                iM = std::min(j + upperBw, N - 1);
                for (int i = j + 1; i <= iM; i++)
                {
                    if (operator()(j, i) != 0.0)
                    {
                        b.row(i) -= operator()(j, i) * b.row(j);
                    }
                }
            }
            for (int j = N - 1; j >= 0; j--)
            {
                iM = std::max(0, j - lowerBw);
                for (int i = iM; i <= j - 1; i++)
                {
                    if (operator()(j, i) != 0.0)
                    {
                        b.row(i) -= operator()(j, i) * b.row(j);
                    }
                }
            }
            return;
        }
    };


    // MINCO for s=3 and non-uniform time
    class MINCO_S3NU
    {
    public:
        MINCO_S3NU() = default;
        ~MINCO_S3NU() { A.destroy(); }
        MINCO_S3NU(int freedom){
            Freedom_ = freedom;
            headPVA.resize(Freedom_,3);
            tailPVA.resize(Freedom_,3);
        }

    private:
        int N;
        int Freedom_;
        Eigen::MatrixXd headPVA;
        Eigen::MatrixXd tailPVA;
        BandedSystem A;
        Eigen::MatrixXd b;
        double T1;
        double T2;
        double T3;
        double T4;
        double T5;

    public:
        inline void setConditions(const Eigen::MatrixXd &headState,//Freedom_*3
                                  const Eigen::MatrixXd &tailState,
                                  const int &pieceNum,
                                  const int &freedom)
        {
            Freedom_ = freedom;
            N = pieceNum;
            headPVA = headState;
            tailPVA = tailState;
            // ROS_INFO("A.create");
            A.create(6 * N, 6, 6);
            // ROS_INFO("b.resize");
            b.resize(6 * N, Freedom_);
            // ROS_INFO("PVAresize");
            // headPVA.resize(Freedom_,3);
            // tailPVA.resize(Freedom_,3);
            return;
        }

        inline void setHConditions(const Eigen::MatrixXd &headState){
            headPVA = headState;                              
        }

        inline void setTConditions(const Eigen::MatrixXd &tailState){
            tailPVA = tailState;                                
        }

        inline void setParameters(const Eigen::MatrixXd &inPs,
                                  const double &ts)
        {
            T1 = ts;
            T2 = T1*T1;
            T3 = T2*T1;
            T4 = T2*T2;
            T5 = T3*T2;
            A.reset();
            b.setZero();
            A(0, 0) = 1.0;
            A(1, 1) = 1.0;
            A(2, 2) = 2.0;
            b.row(0) = headPVA.col(0).transpose();
            b.row(1) = headPVA.col(1).transpose();
            b.row(2) = headPVA.col(2).transpose();
            for (int i = 0; i < N - 1; i++)
            {
                A(6 * i + 3, 6 * i + 3) = 6.0;
                A(6 * i + 3, 6 * i + 4) = 24.0 * T1;
                A(6 * i + 3, 6 * i + 5) = 60.0 * T2;
                A(6 * i + 3, 6 * i + 9) = -6.0;
                A(6 * i + 4, 6 * i + 4) = 24.0;
                A(6 * i + 4, 6 * i + 5) = 120.0 * T1;
                A(6 * i + 4, 6 * i + 10) = -24.0;
                A(6 * i + 5, 6 * i) = 1.0;
                A(6 * i + 5, 6 * i + 1) = T1;
                A(6 * i + 5, 6 * i + 2) = T2;
                A(6 * i + 5, 6 * i + 3) = T3;
                A(6 * i + 5, 6 * i + 4) = T4;
                A(6 * i + 5, 6 * i + 5) = T5;
                A(6 * i + 6, 6 * i) = 1.0;
                A(6 * i + 6, 6 * i + 1) = T1;
                A(6 * i + 6, 6 * i + 2) = T2;
                A(6 * i + 6, 6 * i + 3) = T3;
                A(6 * i + 6, 6 * i + 4) = T4;
                A(6 * i + 6, 6 * i + 5) = T5;
                A(6 * i + 6, 6 * i + 6) = -1.0;
                A(6 * i + 7, 6 * i + 1) = 1.0;
                A(6 * i + 7, 6 * i + 2) = 2 * T1;
                A(6 * i + 7, 6 * i + 3) = 3 * T2;
                A(6 * i + 7, 6 * i + 4) = 4 * T3;
                A(6 * i + 7, 6 * i + 5) = 5 * T4;
                A(6 * i + 7, 6 * i + 7) = -1.0;
                A(6 * i + 8, 6 * i + 2) = 2.0;
                A(6 * i + 8, 6 * i + 3) = 6 * T1;
                A(6 * i + 8, 6 * i + 4) = 12 * T2;
                A(6 * i + 8, 6 * i + 5) = 20 * T3;
                A(6 * i + 8, 6 * i + 8) = -2.0;

                b.row(6 * i + 5) = inPs.col(i).transpose();
            }

            A(6 * N - 3, 6 * N - 6) = 1.0;
            A(6 * N - 3, 6 * N - 5) = T1;
            A(6 * N - 3, 6 * N - 4) = T2;
            A(6 * N - 3, 6 * N - 3) = T3;
            A(6 * N - 3, 6 * N - 2) = T4;
            A(6 * N - 3, 6 * N - 1) = T5;
            A(6 * N - 2, 6 * N - 5) = 1.0;
            A(6 * N - 2, 6 * N - 4) = 2 * T1;
            A(6 * N - 2, 6 * N - 3) = 3 * T2;
            A(6 * N - 2, 6 * N - 2) = 4 * T3;
            A(6 * N - 2, 6 * N - 1) = 5 * T4;
            A(6 * N - 1, 6 * N - 4) = 2;
            A(6 * N - 1, 6 * N - 3) = 6 * T1;
            A(6 * N - 1, 6 * N - 2) = 12 * T2;
            A(6 * N - 1, 6 * N - 1) = 20 * T3;

            b.row(6 * N - 3) = tailPVA.col(0).transpose();
            b.row(6 * N - 2) = tailPVA.col(1).transpose();
            b.row(6 * N - 1) = tailPVA.col(2).transpose();

            A.factorizeLU();
            A.solve(b);

            return;
        }
        // 这里的实例化可能有问题
        inline void getTrajectory(Trajectory<5, 2> &traj) const
        {
            if(traj.getDim() != Freedom_){
                std::cout<<"\033[1m\033[33m"<<"traj.getDim() != Freedom_!!!!! remember to change the dim of traj!!!!!!!!!!!!!"<<"\033[0m"<<std::endl;
                return;
            }
            traj.clear();
            traj.reserve(N);
            for (int i = 0; i < N; i++)
            {
                traj.emplace_back(T1,
                                  b.block(6*i, 0 , 6, Freedom_)
                                      .transpose()
                                      .rowwise()
                                      .reverse());
            }
            return;
        }

        inline void getEnergy(double &energy) const
        {
            energy = 0.0;
            for (int i = 0; i < N; i++)
            {
                energy += 36.0 * b.row(6 * i + 3).squaredNorm() * T1 +
                          144.0 * b.row(6 * i + 4).dot(b.row(6 * i + 3)) * T2 +
                          192.0 * b.row(6 * i + 4).squaredNorm() * T3 +
                          240.0 * b.row(6 * i + 5).dot(b.row(6 * i + 3)) * T3 +
                          720.0 * b.row(6 * i + 5).dot(b.row(6 * i + 4)) * T4 +
                          720.0 * b.row(6 * i + 5).squaredNorm() * T5;
            }
            return;
        }

        inline const Eigen::MatrixXd &getCoeffs(void) const
        {
            return b;
        }

        inline const double getT1(void) const
        {
            return T1;
        }

        inline void getEnergyPartialGradByCoeffs(Eigen::MatrixXd &gdC) const
        {
            gdC.resize(6 * N, Freedom_);
            gdC.setZero();
            for (int i = 0; i < N; i++)
            {
                gdC.row(6 * i + 5) = 240.0 * b.row(6 * i + 3) * T3 +
                                     720.0 * b.row(6 * i + 4) * T4 +
                                     1440.0 * b.row(6 * i + 5) * T5;
                gdC.row(6 * i + 4) = 144.0 * b.row(6 * i + 3) * T2 +
                                     384.0 * b.row(6 * i + 4) * T3 +
                                     720.0 * b.row(6 * i + 5) * T4;
                gdC.row(6 * i + 3) = 72.0 * b.row(6 * i + 3) * T1 +
                                     144.0 * b.row(6 * i + 4) * T2 +
                                     240.0 * b.row(6 * i + 5) * T3;
                gdC.block(6*i, 0, 3, Freedom_).setZero();
            }
            return;
        }

        inline void getEnergyPartialGradByTimes(Eigen::VectorXd &gdT) const
        {
            gdT.resize(N);
            gdT.setZero();
            for (int i = 0; i < N; i++)
            {
                gdT(i) = 36.0 * b.row(6 * i + 3).squaredNorm() +
                         288.0 * b.row(6 * i + 4).dot(b.row(6 * i + 3)) * T1 +
                         576.0 * b.row(6 * i + 4).squaredNorm() * T2 +
                         720.0 * b.row(6 * i + 5).dot(b.row(6 * i + 3)) * T2 +
                         2880.0 * b.row(6 * i + 5).dot(b.row(6 * i + 4)) * T3 +
                         3600.0 * b.row(6 * i + 5).squaredNorm() * T4;
            }
            return;
        }

        inline void propogateGrad(const Eigen::MatrixXd &partialGradByCoeffs,
                                  const Eigen::VectorXd &partialGradByTimes,
                                  Eigen::MatrixXd &gradByPoints,
                                  Eigen::VectorXd &gradByTimes)

        {
            gradByPoints.resize(Freedom_, N - 1);
            gradByTimes.resize(N);
            gradByPoints.setZero();
            gradByTimes.setZero();
            Eigen::MatrixXd adjGrad = partialGradByCoeffs;
            A.solveAdj(adjGrad);

            for (int i = 0; i < N - 1; i++)
            {
                gradByPoints.col(i) = adjGrad.row(6 * i + 5).transpose();
            }

            Eigen::MatrixXd B1;
            B1.resize(6,Freedom_);
            Eigen::MatrixXd B2;
            B2.resize(3,Freedom_);
            for (int i = 0; i < N - 1; i++)
            {
                // negative velocity
                B1.row(2) = -(b.row(i * 6 + 1) +
                              2.0 * T1 * b.row(i * 6 + 2) +
                              3.0 * T2 * b.row(i * 6 + 3) +
                              4.0 * T3 * b.row(i * 6 + 4) +
                              5.0 * T4 * b.row(i * 6 + 5));
                B1.row(3) = B1.row(2);

                // negative acceleration
                B1.row(4) = -(2.0 * b.row(i * 6 + 2) +
                              6.0 * T1 * b.row(i * 6 + 3) +
                              12.0 * T2 * b.row(i * 6 + 4) +
                              20.0 * T3 * b.row(i * 6 + 5));

                // negative jerk
                B1.row(5) = -(6.0 * b.row(i * 6 + 3) +
                              24.0 * T1 * b.row(i * 6 + 4) +
                              60.0 * T2 * b.row(i * 6 + 5));

                // negative snap
                B1.row(0) = -(24.0 * b.row(i * 6 + 4) +
                              120.0 * T1 * b.row(i * 6 + 5));

                // negative crackle
                B1.row(1) = -120.0 * b.row(i * 6 + 5);

                gradByTimes(i) = B1.cwiseProduct(adjGrad.block(6 * i + 3, 0, 6, Freedom_)).sum();
            }

            // negative velocity
            B2.row(0) = -(b.row(6 * N - 5) +
                          2.0 * T1 * b.row(6 * N - 4) +
                          3.0 * T2 * b.row(6 * N - 3) +
                          4.0 * T3 * b.row(6 * N - 2) +
                          5.0 * T4 * b.row(6 * N - 1));

            // negative acceleration
            B2.row(1) = -(2.0 * b.row(6 * N - 4) +
                          6.0 * T1 * b.row(6 * N - 3) +
                          12.0 * T2 * b.row(6 * N - 2) +
                          20.0 * T3 * b.row(6 * N - 1));

            // negative jerk
            B2.row(2) = -(6.0 * b.row(6 * N - 3) +
                          24.0 * T1 * b.row(6 * N - 2) +
                          60.0 * T2 * b.row(6 * N - 1));

            gradByTimes(N - 1) = B2.cwiseProduct(adjGrad.block(6 * N - 3, 0, 3, Freedom_)).sum();

            gradByTimes += partialGradByTimes;
        }

        // 含有初末pva的梯度
        inline void propogateGrad(const Eigen::MatrixXd &partialGradByCoeffs,
                            const Eigen::VectorXd &partialGradByTimes,
                            Eigen::MatrixXd &gradByPoints,
                            Eigen::VectorXd &gradByTimes,
                            Eigen::MatrixXd &gradInit,
                            Eigen::MatrixXd &gradTail)

        {
            gradByPoints.resize(Freedom_, N - 1);
            gradByTimes.resize(N);
            gradByPoints.setZero();
            gradByTimes.setZero();
            gradInit.resize(Freedom_,3);
            gradTail.resize(Freedom_,3);
            Eigen::MatrixXd adjGrad = partialGradByCoeffs;
            A.solveAdj(adjGrad);

            gradInit = adjGrad.topRows(3).transpose();
            for (int i = 0; i < N - 1; i++)
            {
                gradByPoints.col(i) = adjGrad.row(6 * i + 5).transpose();
            }
            gradTail = adjGrad.bottomRows(3).transpose();

            Eigen::MatrixXd B1;
            B1.resize(6,Freedom_);
            Eigen::MatrixXd B2;
            B2.resize(3,Freedom_);
            for (int i = 0; i < N - 1; i++)
            {
                // negative velocity
                B1.row(2) = -(b.row(i * 6 + 1) +
                              2.0 * T1 * b.row(i * 6 + 2) +
                              3.0 * T2 * b.row(i * 6 + 3) +
                              4.0 * T3 * b.row(i * 6 + 4) +
                              5.0 * T4 * b.row(i * 6 + 5));
                B1.row(3) = B1.row(2);

                // negative acceleration
                B1.row(4) = -(2.0 * b.row(i * 6 + 2) +
                              6.0 * T1 * b.row(i * 6 + 3) +
                              12.0 * T2 * b.row(i * 6 + 4) +
                              20.0 * T3 * b.row(i * 6 + 5));

                // negative jerk
                B1.row(5) = -(6.0 * b.row(i * 6 + 3) +
                              24.0 * T1 * b.row(i * 6 + 4) +
                              60.0 * T2 * b.row(i * 6 + 5));

                // negative snap
                B1.row(0) = -(24.0 * b.row(i * 6 + 4) +
                              120.0 * T1 * b.row(i * 6 + 5));

                // negative crackle
                B1.row(1) = -120.0 * b.row(i * 6 + 5);

                gradByTimes(i) = B1.cwiseProduct(adjGrad.block(6 * i + 3, 0, 6, Freedom_)).sum();
            }

            // negative velocity
            B2.row(0) = -(b.row(6 * N - 5) +
                          2.0 * T1 * b.row(6 * N - 4) +
                          3.0 * T2 * b.row(6 * N - 3) +
                          4.0 * T3 * b.row(6 * N - 2) +
                          5.0 * T4 * b.row(6 * N - 1));

            // negative acceleration
            B2.row(1) = -(2.0 * b.row(6 * N - 4) +
                          6.0 * T1 * b.row(6 * N - 3) +
                          12.0 * T2 * b.row(6 * N - 2) +
                          20.0 * T3 * b.row(6 * N - 1));

            // negative jerk
            B2.row(2) = -(6.0 * b.row(6 * N - 3) +
                          24.0 * T1 * b.row(6 * N - 2) +
                          60.0 * T2 * b.row(6 * N - 1));

            gradByTimes(N - 1) = B2.cwiseProduct(adjGrad.block(6 * N - 3, 0, 3, Freedom_)).sum();

            gradByTimes += partialGradByTimes;
        }
    };

}

#endif
