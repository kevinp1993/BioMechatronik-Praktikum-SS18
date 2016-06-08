#ifndef MATRIX_H_
#define MATRIX_H_

#include <algorithm> // std::fill, std::copy

namespace Matrix {

  typedef unsigned int uint;

  /**
    * Multiplication of two matrices Z=X*Y
    *
    * @param *X Pointer to first element of first matrix
    * @param Xrows Rows of X
    * @param Xcols Cols of X
    * @param *Y Pointer to first element of second matrix
    * @param Yrows Rows of Y
    * @param Ycols Cols of Y
    * @param *Z Pointer to first element of result matrix (needs to be init with 0)
    * @param Zrows Rows of Z
    * @param Zcols Cols of Z
    */
  template <typename T>
  void XdotY(T* X,uint Xrows,uint Xcols,T* Y,uint Yrows, uint Ycols,T* Z, uint Zrows,uint Zcols) {

    // Check if the multiplication will work
//       if (Zrows != Xrows || Zcols != Ycols || Xcols != Yrows) {
//   return 1;
//       }

    // Do the multiplication
    // Z_{i,k} = sum_{n} X_{i,n} * Y_{n,k}
    for (uint i = 0; i < Zrows; i++) {
      for (uint k = 0; k < Zcols; k++) {
  for (uint n = 0; n < Xcols; n++) {
    Z[i*Zcols+k] += X[i*Xcols+n] * Y[n*Ycols+k];
  }
      }
    }
//       return 0;
  };

  /**
    * Multiplication of two matrices Z=X*Y^{T},
    * while Y will be transposed during calculation
    *
    * @param *X Pointer to first element of first matrix
    * @param Xrows Rows of X
    * @param Xcols Cols of X
    * @param *Y Pointer to first element of second matrix
    * @param Yrows Rows of Y
    * @param Ycols Cols of Y
    * @param *Z Pointer to first element of result matrix (needs to be init with 0)
    * @param Zrows Rows of Z
    * @param Zcols Cols of Z
    */
  template <typename T>
  void XdotYtrans(T* X,uint Xrows,uint Xcols,T* Y,uint Yrows, uint Ycols,T* Z, uint Zrows,uint Zcols) {

    // Check if the multiplication will work
//       if (Zrows != Xrows || Zcols != Yrows || Xcols != Ycols) {
//   return 1;
//       }

    // Do the multiplication
    // Z_{i,k} = sum_{n} X_{i,n} * Y_{k,n}
    for (uint i = 0; i < Zrows; i++) {
      for (uint k = 0; k < Zcols; k++) {
  for (uint n = 0; n < Xcols; n++) {
    Z[i*Zcols+k] += X[i*Xcols+n] * Y[n+k*Ycols];
  }
      }
    }
//       return 0;
  };

/**
    * Multiplication of two matrices Z=X^{T}*Y,
    * while X will be transposed during calculation
    *
    * @param *X Pointer to first element of first of matrix
    * @param Xrows Rows of X
    * @param Xcols Cols of X
    * @param *Y Pointer to first element of second matrix
    * @param Yrows Rows of Y
    * @param Ycols Cols of Y
    * @param *Z Pointer to first element of result matrix (needs to be init with 0)
    * @param Zrows Rows of Z
    * @param Zcols Cols of Z
    */
  template <typename T>
  void XtransDotY(T* X, uint Xrows,uint Xcols,T* Y,uint Yrows, uint Ycols,T* Z, uint Zrows,uint Zcols) {

    // Check if the multiplication will work
//       if (Zrows != Xrows || Zcols != Yrows || Xcols != Ycols) {
//   return 1;
//       }

    // Do the multiplication
    // Z_{i,k} = sum_{n} X_{n,i} * Y_{k,n}
    for (uint i = 0; i < Zrows; i++) {
      for (uint k = 0; k < Zcols; k++) {
        for (uint n = 0; n < Xcols; n++) {
          Z[i*Zcols+k] += X[n*Xcols+i] * Y[n+k*Ycols];
        }
      }
    }
//       return 0;
  };

    /**
    * Addition of two matrices Z=X+Y,
    *
    * @param *X Pointer to first element of first matrix
    * @param Xrows Rows of X
    * @param Xcols Cols of X
    * @param *Y Pointer to first element of second matrix
    * @param Yrows Rows of Y
    * @param Ycols Cols of Y
    * @param *Z Pointer to first element of result matrix (needs to be init with 0)
    * @param Zrows Rows of Z
    * @param Zcols Cols of Z
    */
  template <typename T>
  void XplusY(T* X,uint Xrows,uint Xcols,T* Y,uint Yrows, uint Ycols,T* Z, uint Zrows,uint Zcols) {

    // Check if the multiplication will work
//       if (Zrows != Xrows || Xrows != Yrows || Zcols != Ycols || Ycols != Xcols {
//   return 1;
//       }

    // Do the summation
  for (uint idx = 0; idx < Xrows*Xcols; idx++) {
      Z[idx] += X[idx] + Y[idx];
    }
//       return 0;
  };

  /**
    * Initializing matrix with any element
    *
    * @param *X Pointer to first element of first matrix
    * @param Xrows Rows of X
    * @param Xcols Cols of X
    * @param value value of all elements
    */
  template <typename T>
  void init(T* X, uint Xrows, uint Xcols, T value) {
    std::fill (X,X+Xrows*Xcols,value);
  };

  /**
    * Copy a matrix Y=X
    *
    * @param *X Pointer to first element of source matrix
    * @param Xrows Rows of X
    * @param Xcols Cols of X
    * @param *Y Pointer to first element of result matrix
    * @param Yrows Rows of Y
    * @param Ycols Cols of Y
    *
    */
  template <typename T>
  void copy(T* X, uint Xrows, uint Xcols, T* Y, uint Yrows, uint Ycols) {
    // if (Xrows != Yrows || Xcols != Ycols ) {
//         return 1;
//       }
    std::copy (X,X+Xrows*Xcols,Y);
//       return 0;
  };

}

#endif /* MATRIX_ */
