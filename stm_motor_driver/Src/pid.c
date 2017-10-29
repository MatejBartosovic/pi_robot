/*
 * pid.c
 *
 *  Created on: Oct 15, 2017
 *      Author: matejko
 */
#include "pid.h"
#include <stdbool.h>


void pid(volatile PidConfig *config){
  bool int_ok;      /* Whether or not the integrator should update */
  float new_i;       /* Proposed new integrator value */

	  /* Compute new integrator and the final control output. */
  config->e = config->goal - *config->y;
  new_i = config->integral + config->e;
  config->u = (config->p_gain *config->e + config->i_gain * new_i);

  /* Check for saturation.  In the event of saturation in any one direction,
     inhibit saving the integrator if doing so would deepen the saturation. */
  int_ok = true;
  /* Positive saturation? */
  if (config->u > config->max)
  {
    /* Clamp the output */
	  config->u = config->max;
    /* Error is the same sign? Inhibit integration. */
    if (config->e > 0)
    {
      int_ok = false;
    }
  }
  /* Repeat for negative sign */
  else if (config->u < config->min)
  {
    config->u = config->min;
    if (config->e < 0)
    {
      int_ok = false;
    }
  }
  /* Update the integrator if allowed. */
  if (int_ok)
  {
    config->integral = new_i;
  }

}

