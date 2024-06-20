/***************************************************************************//**
 *   @file   maxim_pwm.c
 *   @brief  Implementation of MAX32655 PWM driver.
 *   @author Dragos Bogdan (dragos.bogdan@analog.com)
********************************************************************************
 * Copyright 2023(c) Analog Devices, Inc.
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *  - Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  - Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 *  - Neither the name of Analog Devices, Inc. nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *  - The use of this software may or may not infringe the patent rights
 *    of one or more patent holders.  This license does not release you
 *    from the requirement that you obtain separate licenses from these
 *    patent holders to use this software.
 *  - Use of the software either in source or binary form, must be run
 *    on or directly connected to an Analog Devices Inc. component.
 *
 * THIS SOFTWARE IS PROVIDED BY ANALOG DEVICES "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, NON-INFRINGEMENT,
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL ANALOG DEVICES BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, INTELLECTUAL PROPERTY RIGHTS, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*******************************************************************************/

#include "tmr.h"
#include "no_os_alloc.h"
#include "no_os_error.h"
#include "no_os_util.h"
#include "no_os_units.h"
#include "maxim_pwm.h"

/**
 * @brief Convert the prescaler from a clock division value to
 * an enum's value for the init function.
 * @param div - clock division value (this should be a power of 2).
 * @param prescaler - enum value to be provided to the init function.
 * @return 0 in case of success, -EINVAL otherwise.
 */
static int _get_prescaler(uint32_t div, mxc_tmr_pres_t *prescaler)
{
	if (div > 4096)
		return -EINVAL;

	if (div < 2)
		*prescaler = TMR_PRES_1;
	else if (div < 4)
		*prescaler = TMR_PRES_2;
	else if (div < 8)
		*prescaler = TMR_PRES_4;
	else if (div < 16)
		*prescaler = TMR_PRES_8;
	else if (div < 32)
		*prescaler = TMR_PRES_16;
	else if (div < 64)
		*prescaler = TMR_PRES_32;
	else if (div < 128)
		*prescaler = TMR_PRES_64;
	else if (div < 256)
		*prescaler = TMR_PRES_128;
	else if (div < 512)
		*prescaler = TMR_PRES_256;
	else if (div < 1024)
		*prescaler = TMR_PRES_512;
	else if (div < 2048)
		*prescaler = TMR_PRES_1024;
	else if (div < 4096)
		*prescaler = TMR_PRES_2048;
	else
		*prescaler = TMR_PRES_4096;


	return 0;
}

/**
 * @brief Enable the PWM.
 * @param desc - PWM descriptor.
 * @return 0 in case of success, negative error code otherwise.
 */
int32_t max_pwm_enable(struct no_os_pwm_desc *desc)
{
	MXC_TMR_Start(MXC_TMR_GET_TMR(desc->id));

	return 0;
}

/**
 * @brief Disable the PWM.
 * @param desc - PWM descriptor.
 * @return 0 in case of success, negative error otherwise.
 */
int32_t max_pwm_disable(struct no_os_pwm_desc *desc)
{
	MXC_TMR_Shutdown(MXC_TMR_GET_TMR(desc->id));

	return 0;
}

/**
 * @brief Set the PWM period.
 * @param desc - Pointer where the configured instance is stored.
 * @param period_ns - PWM period.
 * @return 0 in case of success, negative error code otherwise.
 */
int32_t max_pwm_set_period(struct no_os_pwm_desc *desc,
			   uint32_t period_ns)
{
	mxc_tmr_pres_t prescaler;
	mxc_tmr_cfg_t tmr_cfg;
	uint32_t period_ticks;
	int32_t ret;

	tmr_cfg = *(mxc_tmr_cfg_t *)desc->extra;
	prescaler = tmr_cfg.pres;

	period_ticks = MXC_TMR_GetPeriod(MXC_TMR_GET_TMR(desc->id),
					 MXC_TMR_APB_CLK,
					 MAX_PWM_PRESCALER_VAL(prescaler),
					 NO_OS_DIV_ROUND_CLOSEST_ULL(NANO, period_ns));

	tmr_cfg.mode = TMR_MODE_PWM;
	tmr_cfg.bitMode = TMR_BIT_MODE_32;
	tmr_cfg.clock = MXC_TMR_APB_CLK;
	tmr_cfg.cmp_cnt = period_ticks;
	tmr_cfg.pol = desc->polarity;

	max_pwm_disable(desc);

	ret = MXC_TMR_Init(MXC_TMR_GET_TMR(desc->id), &tmr_cfg, 1);
	if (ret)
		return ret;

	desc->period_ns = period_ns;

	return 0;
}

/**
 * @brief Get the PWM period.
 * @param desc - Pointer where the configured instance is stored.
 * @param period_ns - PWM period.
 * @return 0 in case of success, negative error code otherwise.
 */
int32_t max_pwm_get_period(struct no_os_pwm_desc *desc,
			   uint32_t *period_ns)
{
	*period_ns = desc->period_ns;

	return 0;
}

/**
 * @brief Set the PWM duty cycle.
 * @param desc - Pointer where the configured instance is stored.
 * @param duty_cycle_ns - PWM duty cycle.
 * @return 0 in case of success, negative error code otherwise.
 */
int32_t max_pwm_set_duty_cycle(struct no_os_pwm_desc *desc,
			       uint32_t duty_cycle_ns)
{
	uint32_t duty_ticks, div;
	mxc_tmr_pres_t prescaler;
	mxc_tmr_cfg_t *cfg;
	int32_t ret;

	div = PeripheralClock / NO_OS_DIV_ROUND_CLOSEST_ULL(NANO, duty_cycle_ns);
	ret = _get_prescaler(div, &prescaler);
	if (ret)
		return ret;

	duty_ticks = MXC_TMR_GetPeriod(MXC_TMR_GET_TMR(desc->id),
				       MXC_TMR_APB_CLK,
				       MAX_PWM_PRESCALER_VAL(prescaler),
				       NO_OS_DIV_ROUND_CLOSEST_ULL(NANO, duty_cycle_ns));

	ret = MXC_TMR_SetPWM(MXC_TMR_GET_TMR(desc->id), duty_ticks);
	if (ret)
		return ret;

	cfg = desc->extra;
	cfg->pres = prescaler;
	desc->extra = cfg;
	desc->duty_cycle_ns = duty_cycle_ns;

	return 0;
}

/**
 * @brief Get the PWM duty cycle.
 * @param desc - Pointer where the configured instance is stored.
 * @param duty_cycle_ns - PWM duty cycle.
 * @return 0 in case of success, negative error code otherwise.
 */
int32_t max_pwm_get_duty_cycle(struct no_os_pwm_desc *desc,
			       uint32_t *duty_cycle_ns)
{
	mxc_tmr_pres_t prescaler;
	mxc_tmr_cfg_t *cfg;
	mxc_tmr_regs_t *tmr;
	uint32_t duty_ticks;

	cfg = desc->extra;
	prescaler = cfg->pres;

	tmr = MXC_TMR_GET_TMR(desc->id);

	/* Return actual measured value of the duty cycle. */
	*duty_cycle_ns = NO_OS_DIV_ROUND_CLOSEST_ULL(NANO * tmr->pwm *
			 MAX_PWM_PRESCALER_VAL(prescaler), PeripheralClock);

	return 0;
}

/**
 * @brief Configure the VDDIO level for the PWM pins
 * @param id - the identification number.
 * @param vssel - the VDDIO level.
 * @return 0 in case of success, -EINVAL otherwise.
 */
static int32_t _max_pwm_pins_vddio(uint32_t id, mxc_gpio_vssel_t vssel)
{
	mxc_gpio_cfg_t pwm_pins;

	switch (id) {
	case 0:
		pwm_pins = gpio_cfg_tmr0;
		break;
	case 1:
		pwm_pins = gpio_cfg_tmr1;
		break;
	case 2:
		pwm_pins = gpio_cfg_tmr2;
		break;
	case 3:
		pwm_pins = gpio_cfg_tmr3;
		break;
	default:
		return -EINVAL;
	}

	pwm_pins.vssel = vssel;

	return MXC_GPIO_Config(&pwm_pins);
}

/**
 * @brief Initialize the PWM.
 * @param desc - PWM descriptor.
 * @param param - The structure containing PWM init parameters.
 * @return 0 in case of success, negative error code otherwise.
 */
int32_t max_pwm_init(struct no_os_pwm_desc **desc,
		     const struct no_os_pwm_init_param *param)
{
	struct max_pwm_init_param *extra_init;
	struct no_os_pwm_desc *descriptor;
	mxc_tmr_pres_t prescaler;
	mxc_tmr_cfg_t *cfg;
	uint32_t div;
	int32_t ret;

	descriptor = (struct no_os_pwm_desc *)no_os_calloc(1, sizeof(*descriptor));
	if (!descriptor)
		return -ENOMEM;

	cfg = (mxc_tmr_cfg_t *)no_os_calloc(1, sizeof(*cfg));
	if (!cfg) {
		return -ENOMEM;
		goto error;
	}

	descriptor->id = param->id;
	descriptor->polarity = param->polarity;

	if (param->extra)
		extra_init = param->extra;

	div = PeripheralClock / NO_OS_DIV_ROUND_CLOSEST_ULL(NANO, param->duty_cycle_ns);
	ret = _get_prescaler(div, &prescaler);
	if (ret)
		goto free_cfg;

	cfg->pres = prescaler;
	descriptor->extra = cfg;

	ret = max_pwm_set_period(descriptor, param->period_ns);
	if (ret)
		goto free_cfg;

	ret = _max_pwm_pins_vddio(param->id, extra_init->vssel);
	if (ret)
		goto free_cfg;

	ret = max_pwm_set_duty_cycle(descriptor, param->duty_cycle_ns);
	if (ret)
		goto free_cfg;

	max_pwm_enable(descriptor);

	*desc = descriptor;

	return 0;

free_cfg:
	no_os_free(cfg);
error:
	no_os_free(descriptor);

	return ret;
}

/**
 * @brief Free the resources allocated by max_pwm_init().
 * @param desc - PWM descriptor
 * @return 0 in case of success, negative error code otherwise.
 */
int32_t max_pwm_remove(struct no_os_pwm_desc *desc)
{
	if (!desc)
		return -EINVAL;

	no_os_free(desc->extra);
	no_os_free(desc);

	return 0;
}

/**
* @brief MAX32655 specific PWM ops
*/
const struct no_os_pwm_platform_ops max_pwm_ops = {
	.pwm_ops_enable = &max_pwm_enable,
	.pwm_ops_disable = &max_pwm_disable,
	.pwm_ops_set_period = &max_pwm_set_period,
	.pwm_ops_get_period = &max_pwm_get_period,
	.pwm_ops_set_duty_cycle = &max_pwm_set_duty_cycle,
	.pwm_ops_get_duty_cycle = &max_pwm_get_duty_cycle,
	.pwm_ops_init = &max_pwm_init,
	.pwm_ops_remove = &max_pwm_remove
};
