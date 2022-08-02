#ifndef MINCO_HPP
#define MINCO_HPP

#include <Eigen/Eigen>

#include <cmath>
#include <vector>

class BandedSystem
{
public:
    // The size of A, as well as the lower/upper
    // banded width p/q are needed
    inline void create(const int &n, const int &p, const int &q)
    {
        // In case of re-creating before destroying
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
            delete[] ptrData;
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


class MINCO_S4
{
private:
  int N;
  int angle_num;
  Eigen::MatrixXd headPVAJ;
  Eigen::MatrixXd tailPVAJ;
  BandedSystem A;
  Eigen::MatrixXd b;
  double T1;
  double T2;
  double T3;
  double T4;
  double T5;
  double T6;
  double T7;

public:
  MINCO_S4(int angle_num_){
    angle_num = angle_num_;
    headPVAJ.resize(angle_num,4);
    tailPVAJ.resize(angle_num,4);
  }
  MINCO_S4() = default;
  ~MINCO_S4() { A.destroy(); };

  inline void setConditions(const Eigen::MatrixX4d &headState,
                            const Eigen::MatrixX4d &tailState,
                            const int &pieceNum,
                            const int &angleNum)
  {
    angle_num = angleNum;
    headPVAJ.resize(angle_num,4);
    tailPVAJ.resize(angle_num,4);
    
    N = pieceNum;
    headPVAJ = headState;
    tailPVAJ = tailState;
    A.create(8 * N, 8, 8);
    b.resize(8 * N, angleNum);
    return;
  }
  
  inline void setParameters(const Eigen::MatrixXd &inPs,
                            const double &ts)
  {
    T1 = ts;
    T2 = T1*T1;
    T3 = T2*T1;
    T4 = T2*T2;
    T5 = T3*T2;
    T6 = T3*T3;
    T7 = T4*T3;
    
    A.reset();
    b.setZero();
    
    A(0, 0) = 1.0;
    A(1, 1) = 1.0;
    A(2, 2) = 2.0;
    A(3, 3) = 6.0;
    b.row(0) = headPVAJ.col(0).transpose();
    b.row(1) = headPVAJ.col(1).transpose();
    b.row(2) = headPVAJ.col(2).transpose();
    b.row(3) = headPVAJ.col(3).transpose();


    /////这里应该可以扔掉循环
    for (int i = 0; i < N - 1; i++)
    {
        A(8 * i + 4, 8 * i + 4) = 24.0;
        A(8 * i + 4, 8 * i + 5) = 120.0 * T1;
        A(8 * i + 4, 8 * i + 6) = 360.0 * T2;
        A(8 * i + 4, 8 * i + 7) = 840.0 * T3;
        A(8 * i + 4, 8 * i + 12) = -24.0;
        A(8 * i + 5, 8 * i + 5) = 120.0;
        A(8 * i + 5, 8 * i + 6) = 720.0 * T1;
        A(8 * i + 5, 8 * i + 7) = 2520.0 * T2;
        A(8 * i + 5, 8 * i + 13) = -120.0;
        A(8 * i + 6, 8 * i + 6) = 720.0;
        A(8 * i + 6, 8 * i + 7) = 5040.0 * T1;
        A(8 * i + 6, 8 * i + 14) = -720.0;
        A(8 * i + 7, 8 * i) = 1.0;
        A(8 * i + 7, 8 * i + 1) = T1;
        A(8 * i + 7, 8 * i + 2) = T2;
        A(8 * i + 7, 8 * i + 3) = T3;
        A(8 * i + 7, 8 * i + 4) = T4;
        A(8 * i + 7, 8 * i + 5) = T5;
        A(8 * i + 7, 8 * i + 6) = T6;
        A(8 * i + 7, 8 * i + 7) = T7;
        A(8 * i + 8, 8 * i) = 1.0;
        A(8 * i + 8, 8 * i + 1) = T1;
        A(8 * i + 8, 8 * i + 2) = T2;
        A(8 * i + 8, 8 * i + 3) = T3;
        A(8 * i + 8, 8 * i + 4) = T4;
        A(8 * i + 8, 8 * i + 5) = T5;
        A(8 * i + 8, 8 * i + 6) = T6;
        A(8 * i + 8, 8 * i + 7) = T7;
        A(8 * i + 8, 8 * i + 8) = -1.0;
        A(8 * i + 9, 8 * i + 1) = 1.0;
        A(8 * i + 9, 8 * i + 2) = 2.0 * T1;
        A(8 * i + 9, 8 * i + 3) = 3.0 * T2;
        A(8 * i + 9, 8 * i + 4) = 4.0 * T3;
        A(8 * i + 9, 8 * i + 5) = 5.0 * T4;
        A(8 * i + 9, 8 * i + 6) = 6.0 * T5;
        A(8 * i + 9, 8 * i + 7) = 7.0 * T6;
        A(8 * i + 9, 8 * i + 9) = -1.0;
        A(8 * i + 10, 8 * i + 2) = 2.0;
        A(8 * i + 10, 8 * i + 3) = 6.0 * T1;
        A(8 * i + 10, 8 * i + 4) = 12.0 * T2;
        A(8 * i + 10, 8 * i + 5) = 20.0 * T3;
        A(8 * i + 10, 8 * i + 6) = 30.0 * T4;
        A(8 * i + 10, 8 * i + 7) = 42.0 * T5;
        A(8 * i + 10, 8 * i + 10) = -2.0;
        A(8 * i + 11, 8 * i + 3) = 6.0;
        A(8 * i + 11, 8 * i + 4) = 24.0 * T1;
        A(8 * i + 11, 8 * i + 5) = 60.0 * T2;
        A(8 * i + 11, 8 * i + 6) = 120.0 * T3;
        A(8 * i + 11, 8 * i + 7) = 210.0 * T4;
        A(8 * i + 11, 8 * i + 11) = -6.0;

        b.row(8 * i + 7) = inPs.col(i).transpose();
    }
    A(8 * N - 4, 8 * N - 8) = 1.0;
    A(8 * N - 4, 8 * N - 7) = T1;
    A(8 * N - 4, 8 * N - 6) = T2;
    A(8 * N - 4, 8 * N - 5) = T3;
    A(8 * N - 4, 8 * N - 4) = T4;
    A(8 * N - 4, 8 * N - 3) = T5;
    A(8 * N - 4, 8 * N - 2) = T6;
    A(8 * N - 4, 8 * N - 1) = T7;
    A(8 * N - 3, 8 * N - 7) = 1.0;
    A(8 * N - 3, 8 * N - 6) = 2.0 * T1;
    A(8 * N - 3, 8 * N - 5) = 3.0 * T2;
    A(8 * N - 3, 8 * N - 4) = 4.0 * T3;
    A(8 * N - 3, 8 * N - 3) = 5.0 * T4;
    A(8 * N - 3, 8 * N - 2) = 6.0 * T5;
    A(8 * N - 3, 8 * N - 1) = 7.0 * T6;
    A(8 * N - 2, 8 * N - 6) = 2.0;
    A(8 * N - 2, 8 * N - 5) = 6.0 * T1;
    A(8 * N - 2, 8 * N - 4) = 12.0 * T2;
    A(8 * N - 2, 8 * N - 3) = 20.0 * T3;
    A(8 * N - 2, 8 * N - 2) = 30.0 * T4;
    A(8 * N - 2, 8 * N - 1) = 42.0 * T5;
    A(8 * N - 1, 8 * N - 5) = 6.0;
    A(8 * N - 1, 8 * N - 4) = 24.0 * T1;
    A(8 * N - 1, 8 * N - 3) = 60.0 * T2;
    A(8 * N - 1, 8 * N - 2) = 120.0 * T3;
    A(8 * N - 1, 8 * N - 1) = 210.0 * T4;

    b.row(8 * N - 4) = tailPVAJ.col(0).transpose();
    b.row(8 * N - 3) = tailPVAJ.col(1).transpose();
    b.row(8 * N - 2) = tailPVAJ.col(2).transpose();
    b.row(8 * N - 1) = tailPVAJ.col(3).transpose();

    A.factorizeLU();
    A.solve(b);

    return;
  }
  
  inline void getEnergy(double &energy) const
  {
      energy = 0.0;
      for (int i = 0; i < N; i++)
      {
          energy += 576.0 * b.row(8 * i + 4).squaredNorm() * T1 +
                    2880.0 * b.row(8 * i + 4).dot(b.row(8 * i + 5)) * T2 +
                    4800.0 * b.row(8 * i + 5).squaredNorm() * T3 +
                    5760.0 * b.row(8 * i + 4).dot(b.row(8 * i + 6)) * T3 +
                    21600.0 * b.row(8 * i + 5).dot(b.row(8 * i + 6)) * T4 +
                    10080.0 * b.row(8 * i + 4).dot(b.row(8 * i + 7)) * T4 +
                    25920.0 * b.row(8 * i + 6).squaredNorm() * T5 +
                    40320.0 * b.row(8 * i + 5).dot(b.row(8 * i + 7)) * T5 +
                    100800.0 * b.row(8 * i + 6).dot(b.row(8 * i + 7)) * T6 +
                    100800.0 * b.row(8 * i + 7).squaredNorm() * T7;
      }
      return;
    }

    inline const Eigen::MatrixXd &getCoeffs(void) const
    {
        return b;
    }

    inline void getEnergyPartialGradByCoeffs(Eigen::MatrixXd &gdC) const
    {
        gdC.resize(8 * N, angle_num);
        gdC.setZero();
        for (int i = 0; i < N; i++)
        {
            gdC.row(8 * i + 7) = 10080.0 * b.row(8 * i + 4) * T4 +
                                  40320.0 * b.row(8 * i + 5) * T5 +
                                  100800.0 * b.row(8 * i + 6) * T6 +
                                  201600.0 * b.row(8 * i + 7) * T7;
            gdC.row(8 * i + 6) = 5760.0 * b.row(8 * i + 4) * T3 +
                                  21600.0 * b.row(8 * i + 5) * T4 +
                                  51840.0 * b.row(8 * i + 6) * T5 +
                                  100800.0 * b.row(8 * i + 7) * T6;
            gdC.row(8 * i + 5) = 2880.0 * b.row(8 * i + 4) * T2 +
                                  9600.0 * b.row(8 * i + 5) * T3 +
                                  21600.0 * b.row(8 * i + 6) * T4 +
                                  40320.0 * b.row(8 * i + 7) * T5;
            gdC.row(8 * i + 4) = 1152.0 * b.row(8 * i + 4) * T1 +
                                  2880.0 * b.row(8 * i + 5) * T2 +
                                  5760.0 * b.row(8 * i + 6) * T3 +
                                  10080.0 * b.row(8 * i + 7) * T4;
            // gdC.block<4, 3>(8 * i, 0).setZero();
            gdC.block(8*i, 0, 4, angle_num).setZero();
        }
        return;
    }

    inline void getEnergyPartialGradByTimes(Eigen::VectorXd &gdT) const
    {
        gdT.resize(N);
        gdT.setZero();
        for (int i = 0; i < N; i++)
        {
            gdT(i) = 576.0 * b.row(8 * i + 4).squaredNorm() +
                      5760.0 * b.row(8 * i + 4).dot(b.row(8 * i + 5)) * T1 +
                      14400.0 * b.row(8 * i + 5).squaredNorm() * T2 +
                      17280.0 * b.row(8 * i + 4).dot(b.row(8 * i + 6)) * T2 +
                      86400.0 * b.row(8 * i + 5).dot(b.row(8 * i + 6)) * T3 +
                      40320.0 * b.row(8 * i + 4).dot(b.row(8 * i + 7)) * T3 +
                      129600.0 * b.row(8 * i + 6).squaredNorm() * T4 +
                      201600.0 * b.row(8 * i + 5).dot(b.row(8 * i + 7)) * T4 +
                      604800.0 * b.row(8 * i + 6).dot(b.row(8 * i + 7)) * T5 +
                      705600.0 * b.row(8 * i + 7).squaredNorm() * T6;
        }
        return;
    }

    inline void propogateGrad(const Eigen::MatrixXd &partialGradByCoeffs,
                                  const Eigen::VectorXd &partialGradByTimes,
                                  Eigen::MatrixXd &gradByPoints,
                                  Eigen::VectorXd &gradByTimes)
    {
      gradByPoints.resize(angle_num, N - 1);
      gradByTimes.resize(N);
      Eigen::MatrixXd adjGrad = partialGradByCoeffs;
      A.solveAdj(adjGrad);
      for (int i = 0; i < N - 1; i++)
      {
          gradByPoints.col(i) = adjGrad.row(8 * i + 7).transpose();
      }

      Eigen::MatrixXd B1;
      B1.resize(8,angle_num);
      Eigen::MatrixXd B2;
      B2.resize(4,angle_num);
      for (int i = 0; i < N - 1; i++)
      {
          // negative velocity
          B1.row(3) = -(b.row(i * 8 + 1) +
                        2.0 * T1 * b.row(i * 8 + 2) +
                        3.0 * T2 * b.row(i * 8 + 3) +
                        4.0 * T3 * b.row(i * 8 + 4) +
                        5.0 * T4 * b.row(i * 8 + 5) +
                        6.0 * T5 * b.row(i * 8 + 6) +
                        7.0 * T6 * b.row(i * 8 + 7));
          B1.row(4) = B1.row(3);

          // negative acceleration
          B1.row(5) = -(2.0 * b.row(i * 8 + 2) +
                        6.0 * T1 * b.row(i * 8 + 3) +
                        12.0 * T2 * b.row(i * 8 + 4) +
                        20.0 * T3 * b.row(i * 8 + 5) +
                        30.0 * T4 * b.row(i * 8 + 6) +
                        42.0 * T5 * b.row(i * 8 + 7));

          // negative jerk
          B1.row(6) = -(6.0 * b.row(i * 8 + 3) +
                        24.0 * T1 * b.row(i * 8 + 4) +
                        60.0 * T2 * b.row(i * 8 + 5) +
                        120.0 * T3 * b.row(i * 8 + 6) +
                        210.0 * T4 * b.row(i * 8 + 7));

          // negative snap
          B1.row(7) = -(24.0 * b.row(i * 8 + 4) +
                        120.0 * T1 * b.row(i * 8 + 5) +
                        360.0 * T2 * b.row(i * 8 + 6) +
                        840.0 * T3 * b.row(i * 8 + 7));

          // negative crackle
          B1.row(0) = -(120.0 * b.row(i * 8 + 5) +
                        720.0 * T1 * b.row(i * 8 + 6) +
                        2520.0 * T2 * b.row(i * 8 + 7));

          // negative d_crackle
          B1.row(1) = -(720.0 * b.row(i * 8 + 6) +
                        5040.0 * T1 * b.row(i * 8 + 7));

          // negative dd_crackle
          B1.row(2) = -5040.0 * b.row(i * 8 + 7);

        //   gradByTimes(i) = B1.cwiseProduct(adjGrad.block<8, 3>(8 * i + 4, 0)).sum();
        gradByTimes(i) = B1.cwiseProduct(adjGrad.block(8 * i + 4,0 ,8 ,angle_num)).sum();
      }
      // negative velocity
      B2.row(0) = -(b.row(8 * N - 7) +
                    2.0 * T1 * b.row(8 * N - 6) +
                    3.0 * T2 * b.row(8 * N - 5) +
                    4.0 * T3 * b.row(8 * N - 4) +
                    5.0 * T4 * b.row(8 * N - 3) +
                    6.0 * T5 * b.row(8 * N - 2) +
                    7.0 * T6 * b.row(8 * N - 1));

      // negative acceleration
      B2.row(1) = -(2.0 * b.row(8 * N - 6) +
                    6.0 * T1 * b.row(8 * N - 5) +
                    12.0 * T2 * b.row(8 * N - 4) +
                    20.0 * T3 * b.row(8 * N - 3) +
                    30.0 * T4 * b.row(8 * N - 2) +
                    42.0 * T5 * b.row(8 * N - 1));

      // negative jerk
      B2.row(2) = -(6.0 * b.row(8 * N - 5) +
                    24.0 * T1 * b.row(8 * N - 4) +
                    60.0 * T2 * b.row(8 * N - 3) +
                    120.0 * T3 * b.row(8 * N - 2) +
                    210.0 * T4 * b.row(8 * N - 1));

      // negative snap
      B2.row(3) = -(24.0 * b.row(8 * N - 4) +
                    120.0 * T1 * b.row(8 * N - 3) +
                    360.0 * T2 * b.row(8 * N - 2) +
                    840.0 * T3 * b.row(8 * N - 1));
    //   gradByTimes(N - 1) = B2.cwiseProduct(adjGrad.block<4, 3>(8 * N - 4, 0)).sum();
      gradByTimes(N - 1) = B2.cwiseProduct(adjGrad.block(8 * N - 4, 0, 4 , angle_num)).sum();
      gradByTimes += partialGradByTimes;
    }

    inline void getTrajectory(Eigen::MatrixXd &allangle)
    {
        double stepT = 0.001;
        int num = ceil(N*T1/stepT);

        allangle.resize(num,angle_num);
        allangle.setZero();
        double maxT = N*T1;
        int count=0;
        for(double i=0;i<maxT;i+=stepT){
            int n = floor(i/T1);
            double t = i-n*T1;
            Eigen::VectorXd betat;
            betat.resize(8);
            betat(0) = 1, betat(1) = t, betat(2) = betat(1)*betat(1), betat(3) = betat(1)*betat(2);
            betat(4) = betat(2)*betat(2), betat(5) = betat(2)*betat(3), betat(6) = betat(3)*betat(3), betat(7) = betat(4)*betat(3);
            allangle.row(count) = betat.transpose()*b.block(8*n,0,8,angle_num);
            
            count += 1;
        }
        return;
    }

};





#endif