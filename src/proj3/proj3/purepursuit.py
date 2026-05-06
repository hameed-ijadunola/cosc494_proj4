from __future__ import division
import numpy as np

from proj3.base_controller import BaseController
from proj3.base_controller import compute_position_in_frame


class PurePursuitController(BaseController):
  def __init__(self, car_length=0.33, **kwargs):
    self.car_length = float(car_length)

    # Get the keyword args that we didn't consume with the above initialization
    super(PurePursuitController, self).__init__(**kwargs)


  def get_error(self, pose, reference_xytv):
    """Compute the Pure Pursuit error.

    Args:
        pose: current state of the vehicle [x, y, heading]
        reference_xytv: reference state and speed

    Returns:
        error: Pure Pursuit error
    """
    return compute_position_in_frame(reference_xytv[:3], pose)


  def get_control(self, pose, reference_xytv, error):
    """Compute the Pure Pursuit control law.

    Args:
        pose: current state of the vehicle [x, y, heading]
        reference_xytv: reference state and speed
        error: error vector from get_error

    Returns:
        control: np.array of velocity and steering angle
    """
    # BEGIN QUESTION 3.1

    v = reference_xytv[3]

    # Pure pursuit uses lateral error in vehicle frame
    e_x = error[0]
    e_y = error[1]

    # Lookahead distance
    Ld = np.sqrt(e_x**2 + e_y**2)

    if Ld < 1e-6:
      return np.array([0.0, 0.0])

    # curvature
    kappa = 2.0 * e_y / (Ld**2)

    # steering angle
    delta = np.arctan(self.car_length * kappa)

    return np.array([v, delta])

    # END QUESTION 3.1
