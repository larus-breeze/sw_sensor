/*
 * Linear_Least_Square_Fit.h
 *
 *  Created on: Jan 3, 2021
 *      Author: schaefer
 */

#ifndef LINEAR_LEAST_SQUARE_FIT_H_
#define LINEAR_LEAST_SQUARE_FIT_H_

#include "my_assert.h"

template<typename type>
  class linear_least_square_result
  {
  public:
    type a;	//!< y offset
    type b;	//!< slope
    type var_a; //!< variance offset
    type var_b; //!< variance slope
    char id; 	//!< optional identifier
  };

//! @brief linear fit y = a + b * x
template<typename type>
  class linear_least_square_fit
  {
  public:
    linear_least_square_fit (void)
    {
      reset ();
    }
    void
    add_value (const type x, const type y)
    {
      sum_x += x;
      sum_xx += x * x;
      sum_y += y;
      sum_yy += y * y;
      sum_xy += x * y;
      ++n;
    }
    void
    reset (void)
    {
      sum_x = sum_xx = sum_y = sum_yy = sum_xy = n = 0.0;
    }
    void
    evaluate (type &a, type &b, type &variance_a, type &variance_b) const
    {
      type inv_n = 1.0 / n;

      type Qx = sum_xx - inv_n * sum_x * sum_x;
      type invQx = 1.0 / Qx; // performance tuning
      type Qy = sum_yy - inv_n * sum_y * sum_y;
      type Qxy = sum_xy - inv_n * sum_x * sum_y;

      ASSERT( n > 2.0);
      type Vyx = (Qy - Qxy * Qxy / Qx) / (n - 2.0);

      b = Qxy * invQx;
      a = sum_y * inv_n - b * sum_x * inv_n;

      variance_a = Vyx * (inv_n + (sum_xx * inv_n) * (sum_xx * inv_n) * invQx);
      variance_b = Vyx * invQx;
    }
    void
    evaluate (linear_least_square_result<type> &r) const
    {
      evaluate (r.a, r.b, r.var_a, r.var_b);
    }
    unsigned
    get_count (void) const
    {
      return (unsigned) n;
    }
  private:
    type sum_x;
    type sum_y;
    type sum_xx;
    type sum_yy;
    type sum_xy;
    type n;
  };

#endif /* LINEAR_LEAST_SQUARE_FIT_H_ */
