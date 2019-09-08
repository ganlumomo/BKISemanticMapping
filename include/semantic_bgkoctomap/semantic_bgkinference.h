#ifndef LA3DM_SEMANTIC_BGK_H
#define LA3DM_SEMANTIC_BGK_H

#include "semantic_bgkoctomap.h"

namespace la3dm {

	/*
     * @brief Bayesian Generalized Kernel Inference on Bernoulli distribution
     * @param dim dimension of data (2, 3, etc.)
     * @param T data type (float, double, etc.)
     * @ref Nonparametric Bayesian inference on multivariate exponential families
     */
    template<int dim, typename T>
    class SemanticBGKInference {
    public:
        /// Eigen matrix type for training and test data and kernel
        using MatrixXType = Eigen::Matrix<T, -1, dim, Eigen::RowMajor>;
        using MatrixKType = Eigen::Matrix<T, -1, -1, Eigen::RowMajor>;
        using MatrixDKType = Eigen::Matrix<T, -1, 1>;
        using MatrixYType = Eigen::Matrix<T, -1, 1>;

        SemanticBGKInference(T sf2, T ell, int nc) : sf2(sf2), ell(ell), nc(nc), trained(false) { }

        /*
         * @brief Fit BGK Model
         * @param x input vector (3N, row major)
         * @param y target vector (N)
         */
        void train(const std::vector<T> &x, const std::vector<T> &y) {
            assert(x.size() % dim == 0 && (int) (x.size() / dim) == y.size());
            MatrixXType _x = Eigen::Map<const MatrixXType>(x.data(), x.size() / dim, dim);
            MatrixYType _y = Eigen::Map<const MatrixYType>(y.data(), y.size(), 1);
            this->y_vec = y;
            train(_x, _y);
        }

        /*
         * @brief Fit BGK Model
         * @param x input matrix (NX3)
         * @param y target matrix (NX1)
         */
        void train(const MatrixXType &x, const MatrixYType &y) {
            this->x = MatrixXType(x);
            this->y = MatrixYType(y);
            trained = true;
        }

       
	void predict(const std::vector<T> &xs, std::vector<std::vector<T>> &ybars) {
	    assert(xs.size() % dim == 0);
	    MatrixXType _xs = Eigen::Map<const MatrixXType>(xs.data(), xs.size() / dim, dim);
	    assert(trained == true);
	    MatrixKType Ks;

	    //covCountingSensorModel(_xs, x, Ks);
	    covSparse(_xs, x, Ks);
	    
	    ybars.resize(_xs.rows());
	    for (int r = 0; r < _xs.rows(); ++r)
	      ybars[r].resize(nc);

        MatrixYType _y_vec = Eigen::Map<const MatrixYType>(y_vec.data(), y_vec.size(), 1);
        for (int k = 0; k < nc; ++k) {
          for (int i = 0; i < y_vec.size(); ++i) {
            if (y_vec[i] == k)
              _y_vec(i, 0) = 1;
            else
              _y_vec(i, 0) = 0;
          }
	      
	      MatrixYType _ybar;
	      _ybar = (Ks * _y_vec);
	      
	      for (int r = 0; r < _ybar.rows(); ++r)
	        ybars[r][k] = _ybar(r, 0);
	    }
	}


        /*
         * @brief Predict with BGK Model
         * @param xs input vector (3M, row major)
         * @param ybar positive class kernel density estimate (\bar{y})
         * @param kbar kernel density estimate (\bar{k})
         */
        void predict(const std::vector<T> &xs, std::vector<T> &abar, std::vector<T> &bbar) const {
            assert(xs.size() % dim == 0);
            MatrixXType _xs = Eigen::Map<const MatrixXType>(xs.data(), xs.size() / dim, dim);
            //std::cout << "xs: " << _xs << std::endl;

            MatrixYType _abar, _bbar;
            predict(_xs, _abar, _bbar);

            abar.resize(_abar.rows());
            bbar.resize(_bbar.rows());
            for (int r = 0; r < _abar.rows(); ++r) {
                abar[r] = _abar(r, 0);
                bbar[r] = _bbar(r, 0);
            }
        }

        /*
         * @brief Predict with nonparametric Bayesian generalized kernel inference
         * @param xs input vector (M x 3)
         * @param ybar positive class kernel density estimate (M x 1)
         * @param kbar kernel density estimate (M x 1)
         */
        void predict(const MatrixXType &xs, MatrixYType &abar, MatrixYType &bbar) const {
            assert(trained == true);
	        MatrixKType Ks;
          
          //covSparse(xs, x, Ks);
          covCountingSensorModel(xs, x, Ks);

          abar = (Ks * y).array();
          MatrixYType ones = MatrixYType::Ones(y.rows(), 1);
          bbar = (Ks * (ones-y)).array();
          //kbar = Ks.rowwise().sum().array();
        
        }

    private:
        /*
         * @brief Compute Euclid distances between two vectors.
         * @param x input vector
         * @param z input vecotr
         * @return d distance matrix
         */
        void dist(const MatrixXType &x, const MatrixXType &z, MatrixKType &d) const {
            d = MatrixKType::Zero(x.rows(), z.rows());
            for (int i = 0; i < x.rows(); ++i) {
                d.row(i) = (z.rowwise() - x.row(i)).rowwise().norm();
            }
        }

        /*
         * @brief Matern3 kernel.
         * @param x input vector
         * @param z input vector
         * @return Kxz covariance matrix
         */
        void covMaterniso3(const MatrixXType &x, const MatrixXType &z, MatrixKType &Kxz) const {
            dist(1.73205 / ell * x, 1.73205 / ell * z, Kxz);
            Kxz = ((1 + Kxz.array()) * exp(-Kxz.array())).matrix() * sf2;
        }

        /*
         * @brief Sparse kernel.
         * @param x input vector
         * @param z input vector
         * @return Kxz covariance matrix
         * @ref A sparse covariance function for exact gaussian process inference in large datasets.
         */
        void covSparse(const MatrixXType &x, const MatrixXType &z, MatrixKType &Kxz) const {
            dist(x / ell, z / ell, Kxz);
            Kxz = (((2.0f + (Kxz * 2.0f * 3.1415926f).array().cos()) * (1.0f - Kxz.array()) / 3.0f) +
                  (Kxz * 2.0f * 3.1415926f).array().sin() / (2.0f * 3.1415926f)).matrix() * sf2;

            // Clean up for values with distance outside length scale
            // Possible because Kxz <= 0 when dist >= ell
            for (int i = 0; i < Kxz.rows(); ++i)
            {
                for (int j = 0; j < Kxz.cols(); ++j)
                    if (Kxz(i,j) < 0.0)
                        Kxz(i,j) = 0.0f;
            }
        }

        void covCountingSensorModel(const MatrixXType &x, const MatrixXType &z, MatrixKType &Kxz) const {
          Kxz = MatrixKType::Ones(x.rows(), z.rows());
        }

        T sf2;    // signal variance
        T ell;    // length-scale
        int nc;   // number of classes

        MatrixXType x;   // temporary storage of training data
        MatrixYType y;   // temporary storage of training labels
        std::vector<T> y_vec;

        bool trained;    // true if bgkinference stored training data
    };

    typedef SemanticBGKInference<3, float> SemanticBGK3f;

}
#endif // LA3DM_SEMANTIC_BGK_H
