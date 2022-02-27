/*
 * Linear_Least_Square_Fit.h
 *
 *  Created on: Jan 3, 2021
 *      Author: schaefer
 */

#ifndef LINEAR_LEAST_SQUARE_FIT_H_
#define LINEAR_LEAST_SQUARE_FIT_H_

#include "my_assert.h"
#include "embedded_math.h"

template<typename type>
  class linear_least_square_result
  {
  public:
    type y_offset;
    type slope;
    type variance_offset;
    type variance_slope;
    uint32_t id; //!< channel identifier (for logging)
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
      sum_x = sum_xx = sum_y = sum_yy = sum_xy = n = ZERO;
    }
    void
    evaluate (type &a, type &b, type &variance_a, type &variance_b) const
    {
      type inv_n = ONE / n;

      type x_mean = sum_x * inv_n;
      type Qx = sum_xx - inv_n * sum_x * sum_x;
      type invQx = ONE / Qx;
      type Qy = sum_yy - inv_n * sum_y * sum_y;
      type Qxy = sum_xy - inv_n * sum_x * sum_y;

      ASSERT( n > TWO);
      type Vyx = (Qy - Qxy * Qxy / Qx) / (n - TWO);

      b = Qxy * invQx;
      a = sum_y * inv_n - b * x_mean;

      variance_a = Vyx * (inv_n + SQR( x_mean) * invQx);
      variance_b = Vyx * invQx;
    }
    void
    evaluate (linear_least_square_result<type> &r) const
    {
      evaluate (r.y_offset, r.slope, r.variance_offset, r.variance_slope);
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
