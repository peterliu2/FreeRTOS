/**
 * \file
 *
 * \brief Power Management Controller (PMC) driver for SAM.
 *
 * Copyright (c) 2011 - 2013 Atmel Corporation. All rights reserved.
 *
 * \asf_license_start
 *
 * \page License
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. The name of Atmel may not be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * 4. This software may only be redistributed and used in connection with an
 *    Atmel microcontroller product.
 *
 * THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * EXPRESSLY AND SPECIFICALLY DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * \asf_license_stop
 *
 */

#include "pmc.h"

#if (SAM3N)
# define MAX_PERIPH_ID    31
#elif (SAM3XA)
# define MAX_PERIPH_ID    44
#elif (SAM3U)
# define MAX_PERIPH_ID    29
#elif (SAM3S || SAM4S)
# define MAX_PERIPH_ID    34
#elif (SAM4E)
# define MAX_PERIPH_ID    47
#elif (SAM4N)
# define MAX_PERIPH_ID    31
#endif

/// @cond 0
/**INDENT-OFF**/
#ifdef __cplusplus
extern "C" {
#endif
/**INDENT-ON**/
/// @endcond

/**
 * \defgroup sam_drivers_pmc_group Power Management Controller (PMC)
 *
 * \par Purpose
 *
 * The Power Management Controller (PMC) optimizes power consumption by
 * controlling all system and user peripheral clocks. The PMC enables/disables
 * the clock inputs to many of the peripherals and the Cortex-M Processor.
 *
 * @{
 */

/**
 * \brief Set the prescaler of the MCK.
 *
 * \param ul_pres Prescaler value.
 */
void pmc_mck_set_prescaler(uint32_t ul_pres)
{
    PMC->PMC_MCKR =
        (PMC->PMC_MCKR & (~PMC_MCKR_PRES_Msk)) | ul_pres;
    while (!(PMC->PMC_SR & PMC_SR_MCKRDY));
}

/**
 * \brief Set the source of the MCK.
 *
 * \param ul_source Source selection value.
 */
void pmc_mck_set_source(uint32_t ul_source)
{
    PMC->PMC_MCKR =
        (PMC->PMC_MCKR & (~PMC_MCKR_CSS_Msk)) | ul_source;
    while (!(PMC->PMC_SR & PMC_SR_MCKRDY));
}

/**
 * \brief Switch master clock source selection to slow clock.
 *
 * \param ul_pres Processor clock prescaler.
 *
 * \retval 0 Success.
 * \retval 1 Timeout error.
 */
uint32_t pmc_switch_mck_to_sclk(uint32_t ul_pres)
{
    uint32_t ul_timeout;

    PMC->PMC_MCKR = (PMC->PMC_MCKR & (~PMC_MCKR_CSS_Msk)) |
                    PMC_MCKR_CSS_SLOW_CLK;
    for (ul_timeout = PMC_TIMEOUT; !(PMC->PMC_SR & PMC_SR_MCKRDY);
            --ul_timeout) {
        if (ul_timeout == 0) {
            return 1;
        }
    }

    PMC->PMC_MCKR = (PMC->PMC_MCKR & (~PMC_MCKR_PRES_Msk)) | ul_pres;
    for (ul_timeout = PMC_TIMEOUT; !(PMC->PMC_SR & PMC_SR_MCKRDY);
            --ul_timeout) {
        if (ul_timeout == 0) {
            return 1;
        }
    }

    return 0;
}

/**
 * \brief Switch master clock source selection to main clock.
 *
 * \param ul_pres Processor clock prescaler.
 *
 * \retval 0 Success.
 * \retval 1 Timeout error.
 */
uint32_t pmc_switch_mck_to_mainck(uint32_t ul_pres)
{
    uint32_t ul_timeout;

    PMC->PMC_MCKR = (PMC->PMC_MCKR & (~PMC_MCKR_CSS_Msk)) |
                    PMC_MCKR_CSS_MAIN_CLK;
    for (ul_timeout = PMC_TIMEOUT; !(PMC->PMC_SR & PMC_SR_MCKRDY);
            --ul_timeout) {
        if (ul_timeout == 0) {
            return 1;
        }
    }

    PMC->PMC_MCKR = (PMC->PMC_MCKR & (~PMC_MCKR_PRES_Msk)) | ul_pres;
    for (ul_timeout = PMC_TIMEOUT; !(PMC->PMC_SR & PMC_SR_MCKRDY);
            --ul_timeout) {
        if (ul_timeout == 0) {
            return 1;
        }
    }

    return 0;
}

/**
 * \brief Switch master clock source selection to PLLA clock.
 *
 * \param ul_pres Processor clock prescaler.
 *
 * \retval 0 Success.
 * \retval 1 Timeout error.
 */
uint32_t pmc_switch_mck_to_pllack(uint32_t ul_pres)
{
    uint32_t ul_timeout;

    PMC->PMC_MCKR = (PMC->PMC_MCKR & (~PMC_MCKR_PRES_Msk)) | ul_pres;
    for (ul_timeout = PMC_TIMEOUT; !(PMC->PMC_SR & PMC_SR_MCKRDY);
            --ul_timeout) {
        if (ul_timeout == 0) {
            return 1;
        }
    }

    PMC->PMC_MCKR = (PMC->PMC_MCKR & (~PMC_MCKR_CSS_Msk)) |
                    PMC_MCKR_CSS_PLLA_CLK;

    for (ul_timeout = PMC_TIMEOUT; !(PMC->PMC_SR & PMC_SR_MCKRDY);
            --ul_timeout) {
        if (ul_timeout == 0) {
            return 1;
        }
    }

    return 0;
}

#if (SAM3S || SAM4S)
/**
 * \brief Switch master clock source selection to PLLB clock.
 *
 * \param ul_pres Processor clock prescaler.
 *
 * \retval 0 Success.
 * \retval 1 Timeout error.
 */
uint32_t pmc_switch_mck_to_pllbck(uint32_t ul_pres)
{
    uint32_t ul_timeout;

    PMC->PMC_MCKR = (PMC->PMC_MCKR & (~PMC_MCKR_PRES_Msk)) | ul_pres;
    for (ul_timeout = PMC_TIMEOUT; !(PMC->PMC_SR & PMC_SR_MCKRDY);
            --ul_timeout) {
        if (ul_timeout == 0) {
            return 1;
        }
    }

    PMC->PMC_MCKR = (PMC->PMC_MCKR & (~PMC_MCKR_CSS_Msk)) |
                    PMC_MCKR_CSS_PLLB_CLK;
    for (ul_timeout = PMC_TIMEOUT; !(PMC->PMC_SR & PMC_SR_MCKRDY);
            --ul_timeout) {
        if (ul_timeout == 0) {
            return 1;
        }
    }

    return 0;
}
#endif

#if (SAM3XA || SAM3U)
/**
 * \brief Switch master clock source selection to UPLL clock.
 *
 * \param ul_pres Processor clock prescaler.
 *
 * \retval 0 Success.
 * \retval 1 Timeout error.
 */
uint32_t pmc_switch_mck_to_upllck(uint32_t ul_pres)
{
    uint32_t ul_timeout;

    PMC->PMC_MCKR = (PMC->PMC_MCKR & (~PMC_MCKR_PRES_Msk)) | ul_pres;
    for (ul_timeout = PMC_TIMEOUT; !(PMC->PMC_SR & PMC_SR_MCKRDY);
            --ul_timeout) {
        if (ul_timeout == 0) {
            return 1;
        }
    }

    PMC->PMC_MCKR = (PMC->PMC_MCKR & (~PMC_MCKR_CSS_Msk)) |
                    PMC_MCKR_CSS_UPLL_CLK;
    for (ul_timeout = PMC_TIMEOUT; !(PMC->PMC_SR & PMC_SR_MCKRDY);
            --ul_timeout) {
        if (ul_timeout == 0) {
            return 1;
        }
    }

    return 0;
}
#endif

/**
 * \brief Switch slow clock source selection to external 32k (Xtal or Bypass).
 *
 * \note This function disables the PLLs.
 *
 * \note Switching SCLK back to 32krc is only possible by shutting down the
 *       VDDIO power supply.
 *
 * \param ul_bypass 0 for Xtal, 1 for bypass.
 */
void pmc_switch_sclk_to_32kxtal(uint32_t ul_bypass)
{
    /* Set Bypass mode if required */
    if (ul_bypass == 1) {
        SUPC->SUPC_MR |= SUPC_MR_KEY(SUPC_KEY_VALUE) |
                         SUPC_MR_OSCBYPASS;
    }

    SUPC->SUPC_CR = SUPC_CR_KEY(SUPC_KEY_VALUE) | SUPC_CR_XTALSEL;
}

/**
 * \brief Check if the external 32k Xtal is ready.
 *
 * \retval 1 External 32k Xtal is ready.
 * \retval 0 External 32k Xtal is not ready.
 */
uint32_t pmc_osc_is_ready_32kxtal(void)
{
    return ((SUPC->SUPC_SR & SUPC_SR_OSCSEL)
            && (PMC->PMC_SR & PMC_SR_OSCSELS));
}

/**
 * \brief Switch main clock source selection to internal fast RC.
 *
 * \param ul_moscrcf Fast RC oscillator(4/8/12Mhz).
 *
 * \retval 0 Success.
 * \retval 1 Timeout error.
 * \retval 2 Invalid frequency.
 */
void pmc_switch_mainck_to_fastrc(uint32_t ul_moscrcf)
{
    uint32_t ul_needXTEN = 0;

    /* Enable Fast RC oscillator but DO NOT switch to RC now */
    if (PMC->CKGR_MOR & CKGR_MOR_MOSCXTEN) {
        PMC->CKGR_MOR = (PMC->CKGR_MOR & ~CKGR_MOR_MOSCRCF_Msk) |
                        PMC_CKGR_MOR_KEY_VALUE | CKGR_MOR_MOSCRCEN |
                        ul_moscrcf;
    } else {
        ul_needXTEN = 1;
        PMC->CKGR_MOR = (PMC->CKGR_MOR & ~CKGR_MOR_MOSCRCF_Msk) |
                        PMC_CKGR_MOR_KEY_VALUE | CKGR_MOR_MOSCRCEN |
                        CKGR_MOR_MOSCXTEN | CKGR_MOR_MOSCXTST_Msk |
                        ul_moscrcf;
    }

    /* Wait the Fast RC to stabilize */
    while (!(PMC->PMC_SR & PMC_SR_MOSCRCS));

    /* Switch to Fast RC */
    PMC->CKGR_MOR = (PMC->CKGR_MOR & ~CKGR_MOR_MOSCSEL) |
                    PMC_CKGR_MOR_KEY_VALUE;

    /* Disable xtal oscillator */
    if (ul_needXTEN) {
        PMC->CKGR_MOR = (PMC->CKGR_MOR & ~CKGR_MOR_MOSCXTEN) |
                        PMC_CKGR_MOR_KEY_VALUE;
    }
}

/**
 * \brief Enable fast RC oscillator.
 *
 * \param ul_rc Fast RC oscillator(4/8/12Mhz).
 */
void pmc_osc_enable_fastrc(uint32_t ul_rc)
{
    /* Enable Fast RC oscillator but DO NOT switch to RC now.
     * Keep MOSCSEL to 1 */
    PMC->CKGR_MOR = PMC_CKGR_MOR_KEY_VALUE | CKGR_MOR_MOSCSEL |
                    CKGR_MOR_MOSCXTEN | CKGR_MOR_MOSCRCEN | ul_rc;
    /* Wait the Fast RC to stabilize */
    while (!(PMC->PMC_SR & PMC_SR_MOSCRCS));
}

/**
 * \brief Disable the internal fast RC.
 */
void pmc_osc_disable_fastrc(void)
{
    /* Disable Fast RC oscillator */
    PMC->CKGR_MOR = (PMC->CKGR_MOR & ~CKGR_MOR_MOSCRCEN &
                     ~CKGR_MOR_MOSCRCF_Msk)
                    | PMC_CKGR_MOR_KEY_VALUE;
}

/**
 * \brief Check if the main fastrc is ready.
 *
 * \retval 0 Xtal is not ready, otherwise ready.
 */
uint32_t pmc_osc_is_ready_fastrc(void)
{
    return (PMC->PMC_SR & PMC_SR_MOSCRCS);
}

/**
 * \brief Enable main XTAL oscillator.
 *
 * \param ul_xtal_startup_time Xtal start-up time, in number of slow clocks.
 */
void pmc_osc_enable_main_xtal(uint32_t ul_xtal_startup_time)
{
    uint32_t mor = PMC->CKGR_MOR;
    mor &= ~(CKGR_MOR_MOSCXTBY|CKGR_MOR_MOSCXTEN);
    mor |= PMC_CKGR_MOR_KEY_VALUE | CKGR_MOR_MOSCXTEN |
           CKGR_MOR_MOSCXTST(ul_xtal_startup_time);
    PMC->CKGR_MOR = mor;
    /* Wait the main Xtal to stabilize */
    while (!(PMC->PMC_SR & PMC_SR_MOSCXTS));
}

/**
 * \brief Bypass main XTAL.
 */
void pmc_osc_bypass_main_xtal(void)
{
    uint32_t mor = PMC->CKGR_MOR;
    mor &= ~(CKGR_MOR_MOSCXTBY|CKGR_MOR_MOSCXTEN);
    mor |= PMC_CKGR_MOR_KEY_VALUE | CKGR_MOR_MOSCXTBY;
    /* Enable Crystal oscillator but DO NOT switch now. Keep MOSCSEL to 0 */
    PMC->CKGR_MOR = mor;
    /* The MOSCXTS in PMC_SR is automatically set */
}

/**
 * \brief Disable the main Xtal.
 */
void pmc_osc_disable_main_xtal(void)
{
    uint32_t mor = PMC->CKGR_MOR;
    mor &= ~(CKGR_MOR_MOSCXTBY|CKGR_MOR_MOSCXTEN);
    PMC->CKGR_MOR = PMC_CKGR_MOR_KEY_VALUE | mor;
}

/**
 * \brief Check if the main crystal is bypassed.
 *
 * \retval 0 Xtal is bypassed, otherwise not.
 */
uint32_t pmc_osc_is_bypassed_main_xtal(void)
{
    return (PMC->CKGR_MOR & CKGR_MOR_MOSCXTBY);
}

/**
 * \brief Check if the main crystal is ready.
 *
 * \note If main crystal is bypassed, it's always ready.
 *
 * \retval 0 main crystal is not ready, otherwise ready.
 */
uint32_t pmc_osc_is_ready_main_xtal(void)
{
    return (PMC->PMC_SR & PMC_SR_MOSCXTS);
}

/**
 * \brief Switch main clock source selection to external Xtal/Bypass.
 *
 * \note The function may switch MCK to SCLK if MCK source is MAINCK to avoid
 *       any system crash.
 *
 * \note If used in Xtal mode, the Xtal is automatically enabled.
 *
 * \param ul_bypass 0 for Xtal, 1 for bypass.
 *
 * \retval 0 Success.
 * \retval 1 Timeout error.
 */
void pmc_switch_mainck_to_xtal(uint32_t ul_bypass,
                               uint32_t ul_xtal_startup_time)
{
    /* Enable Main Xtal oscillator */
    if (ul_bypass) {
        PMC->CKGR_MOR = (PMC->CKGR_MOR & ~CKGR_MOR_MOSCXTEN) |
                        PMC_CKGR_MOR_KEY_VALUE | CKGR_MOR_MOSCXTBY |
                        CKGR_MOR_MOSCSEL;
    } else {
        PMC->CKGR_MOR = (PMC->CKGR_MOR & ~CKGR_MOR_MOSCXTBY) |
                        PMC_CKGR_MOR_KEY_VALUE | CKGR_MOR_MOSCXTEN |
                        CKGR_MOR_MOSCXTST(ul_xtal_startup_time);
        /* Wait the Xtal to stabilize */
        while (!(PMC->PMC_SR & PMC_SR_MOSCXTS));

        PMC->CKGR_MOR |= PMC_CKGR_MOR_KEY_VALUE | CKGR_MOR_MOSCSEL;
    }
}

/**
 * \brief Disable the external Xtal.
 *
 * \param ul_bypass 0 for Xtal, 1 for bypass.
 */
void pmc_osc_disable_xtal(uint32_t ul_bypass)
{
    /* Disable xtal oscillator */
    if (ul_bypass) {
        PMC->CKGR_MOR = (PMC->CKGR_MOR & ~CKGR_MOR_MOSCXTBY) |
                        PMC_CKGR_MOR_KEY_VALUE;
    } else {
        PMC->CKGR_MOR = (PMC->CKGR_MOR & ~CKGR_MOR_MOSCXTEN) |
                        PMC_CKGR_MOR_KEY_VALUE;
    }
}

/**
 * \brief Check if the MAINCK is ready. Depending on MOSCEL, MAINCK can be one
 * of Xtal, bypass or internal RC.
 *
 * \retval 1 Xtal is ready.
 * \retval 0 Xtal is not ready.
 */
uint32_t pmc_osc_is_ready_mainck(void)
{
    return PMC->PMC_SR & PMC_SR_MOSCSELS;
}

/**
 * \brief Select Main Crystal or internal RC as main clock source.
 *
 * \note This function will not enable/disable RC or Main Crystal.
 *
 * \param ul_xtal_rc 0 internal RC is selected, otherwise Main Crystal.
 */
void pmc_mainck_osc_select(uint32_t ul_xtal_rc)
{
    uint32_t mor = PMC->CKGR_MOR;
    if (ul_xtal_rc) {
        mor |=  CKGR_MOR_MOSCSEL;
    } else {
        mor &= ~CKGR_MOR_MOSCSEL;
    }
    PMC->CKGR_MOR = PMC_CKGR_MOR_KEY_VALUE | mor;
}

/**
 * \brief Enable PLLA clock.
 *
 * \param mula PLLA multiplier.
 * \param pllacount PLLA counter.
 * \param diva Divider.
 */
void pmc_enable_pllack(uint32_t mula, uint32_t pllacount, uint32_t diva)
{
    /* first disable the PLL to unlock the lock */
    pmc_disable_pllack();

    PMC->CKGR_PLLAR = CKGR_PLLAR_ONE | CKGR_PLLAR_DIVA(diva) |
                      CKGR_PLLAR_PLLACOUNT(pllacount) | CKGR_PLLAR_MULA(mula);
    while ((PMC->PMC_SR & PMC_SR_LOCKA) == 0);
}

/**
 * \brief Disable PLLA clock.
 */
void pmc_disable_pllack(void)
{
    PMC->CKGR_PLLAR = CKGR_PLLAR_ONE | CKGR_PLLAR_MULA(0);
}

/**
 * \brief Is PLLA locked?
 *
 * \retval 0 Not locked.
 * \retval 1 Locked.
 */
uint32_t pmc_is_locked_pllack(void)
{
    return (PMC->PMC_SR & PMC_SR_LOCKA);
}

#if (SAM3S || SAM4S)
/**
 * \brief Enable PLLB clock.
 *
 * \param mulb PLLB multiplier.
 * \param pllbcount PLLB counter.
 * \param divb Divider.
 */
void pmc_enable_pllbck(uint32_t mulb, uint32_t pllbcount, uint32_t divb)
{
    /* first disable the PLL to unlock the lock */
    pmc_disable_pllbck();

    PMC->CKGR_PLLBR =
        CKGR_PLLBR_DIVB(divb) | CKGR_PLLBR_PLLBCOUNT(pllbcount)
        | CKGR_PLLBR_MULB(mulb);
    while ((PMC->PMC_SR & PMC_SR_LOCKB) == 0);
}

/**
 * \brief Disable PLLB clock.
 */
void pmc_disable_pllbck(void)
{
    PMC->CKGR_PLLBR = CKGR_PLLBR_MULB(0);
}

/**
 * \brief Is PLLB locked?
 *
 * \retval 0 Not locked.
 * \retval 1 Locked.
 */
uint32_t pmc_is_locked_pllbck(void)
{
    return (PMC->PMC_SR & PMC_SR_LOCKB);
}
#endif

#if (SAM3XA || SAM3U)
/**
 * \brief Enable UPLL clock.
 */
void pmc_enable_upll_clock(void)
{
    PMC->CKGR_UCKR = CKGR_UCKR_UPLLCOUNT(3) | CKGR_UCKR_UPLLEN;

    /* Wait UTMI PLL Lock Status */
    while (!(PMC->PMC_SR & PMC_SR_LOCKU));
}

/**
 * \brief Disable UPLL clock.
 */
void pmc_disable_upll_clock(void)
{
    PMC->CKGR_UCKR &= ~CKGR_UCKR_UPLLEN;
}

/**
 * \brief Is UPLL locked?
 *
 * \retval 0 Not locked.
 * \retval 1 Locked.
 */
uint32_t pmc_is_locked_upll(void)
{
    return (PMC->PMC_SR & PMC_SR_LOCKU);
}
#endif

/**
 * \brief Enable the specified peripheral clock.
 *
 * \note The ID must NOT be shifted (i.e., 1 << ID_xxx).
 *
 * \param ul_id Peripheral ID (ID_xxx).
 *
 * \retval 0 Success.
 * \retval 1 Invalid parameter.
 */
uint32_t pmc_enable_periph_clk(uint32_t ul_id)
{
    if (ul_id > MAX_PERIPH_ID) {
        return 1;
    }

    if (ul_id < 32) {
        if ((PMC->PMC_PCSR0 & (1u << ul_id)) != (1u << ul_id)) {
            PMC->PMC_PCER0 = 1 << ul_id;
        }
#if (SAM3S || SAM3XA || SAM4S || SAM4E)
    } else {
        ul_id -= 32;
        if ((PMC->PMC_PCSR1 & (1u << ul_id)) != (1u << ul_id)) {
            PMC->PMC_PCER1 = 1 << ul_id;
        }
#endif
    }

    return 0;
}

/**
 * \brief Disable the specified peripheral clock.
 *
 * \note The ID must NOT be shifted (i.e., 1 << ID_xxx).
 *
 * \param ul_id Peripheral ID (ID_xxx).
 *
 * \retval 0 Success.
 * \retval 1 Invalid parameter.
 */
uint32_t pmc_disable_periph_clk(uint32_t ul_id)
{
    if (ul_id > MAX_PERIPH_ID) {
        return 1;
    }

    if (ul_id < 32) {
        if ((PMC->PMC_PCSR0 & (1u << ul_id)) == (1u << ul_id)) {
            PMC->PMC_PCDR0 = 1 << ul_id;
        }
#if (SAM3S || SAM3XA || SAM4S || SAM4E)
    } else {
        ul_id -= 32;
        if ((PMC->PMC_PCSR1 & (1u << ul_id)) == (1u << ul_id)) {
            PMC->PMC_PCDR1 = 1 << ul_id;
        }
#endif
    }
    return 0;
}

/**
 * \brief Enable all peripheral clocks.
 */
void pmc_enable_all_periph_clk(void)
{
    PMC->PMC_PCER0 = PMC_MASK_STATUS0;
    while ((PMC->PMC_PCSR0 & PMC_MASK_STATUS0) != PMC_MASK_STATUS0);

#if (SAM3S || SAM3XA || SAM4S || SAM4E)
    PMC->PMC_PCER1 = PMC_MASK_STATUS1;
    while ((PMC->PMC_PCSR1 & PMC_MASK_STATUS1) != PMC_MASK_STATUS1);
#endif
}

/**
 * \brief Disable all peripheral clocks.
 */
void pmc_disable_all_periph_clk(void)
{
    PMC->PMC_PCDR0 = PMC_MASK_STATUS0;
    while ((PMC->PMC_PCSR0 & PMC_MASK_STATUS0) != 0);

#if (SAM3S || SAM3XA || SAM4S || SAM4E)
    PMC->PMC_PCDR1 = PMC_MASK_STATUS1;
    while ((PMC->PMC_PCSR1 & PMC_MASK_STATUS1) != 0);
#endif
}

/**
 * \brief Check if the specified peripheral clock is enabled.
 *
 * \note The ID must NOT be shifted (i.e., 1 << ID_xxx).
 *
 * \param ul_id Peripheral ID (ID_xxx).
 *
 * \retval 0 Peripheral clock is disabled or unknown.
 * \retval 1 Peripheral clock is enabled.
 */
uint32_t pmc_is_periph_clk_enabled(uint32_t ul_id)
{
    if (ul_id > MAX_PERIPH_ID) {
        return 0;
    }

#if (SAM3S || SAM3XA || SAM4S || SAM4E)
    if (ul_id < 32) {
#endif
        if ((PMC->PMC_PCSR0 & (1u << ul_id))) {
            return 1;
        } else {
            return 0;
        }
#if (SAM3S || SAM3XA || SAM4S || SAM4E)
    } else {
        ul_id -= 32;
        if ((PMC->PMC_PCSR1 & (1u << ul_id))) {
            return 1;
        } else {
            return 0;
        }
    }
#endif
}

/**
 * \brief Set the prescaler for the specified programmable clock.
 *
 * \param ul_id Peripheral ID.
 * \param ul_pres Prescaler value.
 */
void pmc_pck_set_prescaler(uint32_t ul_id, uint32_t ul_pres)
{
    PMC->PMC_PCK[ul_id] =
        (PMC->PMC_PCK[ul_id] & ~PMC_PCK_PRES_Msk) | ul_pres;
    while ((PMC->PMC_SCER & (PMC_SCER_PCK0 << ul_id))
            && !(PMC->PMC_SR & (PMC_SR_PCKRDY0 << ul_id)));
}

/**
 * \brief Set the source oscillator for the specified programmable clock.
 *
 * \param ul_id Peripheral ID.
 * \param ul_source Source selection value.
 */
void pmc_pck_set_source(uint32_t ul_id, uint32_t ul_source)
{
    PMC->PMC_PCK[ul_id] =
        (PMC->PMC_PCK[ul_id] & ~PMC_PCK_CSS_Msk) | ul_source;
    while ((PMC->PMC_SCER & (PMC_SCER_PCK0 << ul_id))
            && !(PMC->PMC_SR & (PMC_SR_PCKRDY0 << ul_id)));
}

/**
 * \brief Switch programmable clock source selection to slow clock.
 *
 * \param ul_id Id of the programmable clock.
 * \param ul_pres Programmable clock prescaler.
 *
 * \retval 0 Success.
 * \retval 1 Timeout error.
 */
uint32_t pmc_switch_pck_to_sclk(uint32_t ul_id, uint32_t ul_pres)
{
    uint32_t ul_timeout;

    PMC->PMC_PCK[ul_id] = PMC_PCK_CSS_SLOW_CLK | ul_pres;
    for (ul_timeout = PMC_TIMEOUT;
            !(PMC->PMC_SR & (PMC_SR_PCKRDY0 << ul_id)); --ul_timeout) {
        if (ul_timeout == 0) {
            return 1;
        }
    }

    return 0;
}

/**
 * \brief Switch programmable clock source selection to main clock.
 *
 * \param ul_id Id of the programmable clock.
 * \param ul_pres Programmable clock prescaler.
 *
 * \retval 0 Success.
 * \retval 1 Timeout error.
 */
uint32_t pmc_switch_pck_to_mainck(uint32_t ul_id, uint32_t ul_pres)
{
    uint32_t ul_timeout;

    PMC->PMC_PCK[ul_id] = PMC_PCK_CSS_MAIN_CLK | ul_pres;
    for (ul_timeout = PMC_TIMEOUT;
            !(PMC->PMC_SR & (PMC_SR_PCKRDY0 << ul_id)); --ul_timeout) {
        if (ul_timeout == 0) {
            return 1;
        }
    }

    return 0;
}

/**
 * \brief Switch programmable clock source selection to PLLA clock.
 *
 * \param ul_id Id of the programmable clock.
 * \param ul_pres Programmable clock prescaler.
 *
 * \retval 0 Success.
 * \retval 1 Timeout error.
 */
uint32_t pmc_switch_pck_to_pllack(uint32_t ul_id, uint32_t ul_pres)
{
    uint32_t ul_timeout;

    PMC->PMC_PCK[ul_id] = PMC_PCK_CSS_PLLA_CLK | ul_pres;
    for (ul_timeout = PMC_TIMEOUT;
            !(PMC->PMC_SR & (PMC_SR_PCKRDY0 << ul_id)); --ul_timeout) {
        if (ul_timeout == 0) {
            return 1;
        }
    }

    return 0;
}

#if (SAM3S || SAM4S)
/**
 * \brief Switch programmable clock source selection to PLLB clock.
 *
 * \param ul_id Id of the programmable clock.
 * \param ul_pres Programmable clock prescaler.
 *
 * \retval 0 Success.
 * \retval 1 Timeout error.
 */
uint32_t pmc_switch_pck_to_pllbck(uint32_t ul_id, uint32_t ul_pres)
{
    uint32_t ul_timeout;

    PMC->PMC_PCK[ul_id] = PMC_PCK_CSS_PLLB_CLK | ul_pres;
    for (ul_timeout = PMC_TIMEOUT;
            !(PMC->PMC_SR & (PMC_SR_PCKRDY0 << ul_id));
            --ul_timeout) {
        if (ul_timeout == 0) {
            return 1;
        }
    }

    return 0;
}
#endif

#if (SAM3XA || SAM3U)
/**
 * \brief Switch programmable clock source selection to UPLL clock.
 *
 * \param ul_id Id of the programmable clock.
 * \param ul_pres Programmable clock prescaler.
 *
 * \retval 0 Success.
 * \retval 1 Timeout error.
 */
uint32_t pmc_switch_pck_to_upllck(uint32_t ul_id, uint32_t ul_pres)
{
    uint32_t ul_timeout;

    PMC->PMC_PCK[ul_id] = PMC_PCK_CSS_UPLL_CLK | ul_pres;
    for (ul_timeout = PMC_TIMEOUT;
            !(PMC->PMC_SR & (PMC_SR_PCKRDY0 << ul_id));
            --ul_timeout) {
        if (ul_timeout == 0) {
            return 1;
        }
    }

    return 0;
}
#endif

/**
 * \brief Enable the specified programmable clock.
 *
 * \param ul_id Id of the programmable clock.
 */
void pmc_enable_pck(uint32_t ul_id)
{
    PMC->PMC_SCER = PMC_SCER_PCK0 << ul_id;
}

/**
 * \brief Disable the specified programmable clock.
 *
 * \param ul_id Id of the programmable clock.
 */
void pmc_disable_pck(uint32_t ul_id)
{
    PMC->PMC_SCDR = PMC_SCER_PCK0 << ul_id;
}

/**
 * \brief Enable all programmable clocks.
 */
void pmc_enable_all_pck(void)
{
    PMC->PMC_SCER = PMC_SCER_PCK0 | PMC_SCER_PCK1 | PMC_SCER_PCK2;
}

/**
 * \brief Disable all programmable clocks.
 */
void pmc_disable_all_pck(void)
{
    PMC->PMC_SCDR = PMC_SCDR_PCK0 | PMC_SCDR_PCK1 | PMC_SCDR_PCK2;
}

/**
 * \brief Check if the specified programmable clock is enabled.
 *
 * \param ul_id Id of the programmable clock.
 *
 * \retval 0 Programmable clock is disabled or unknown.
 * \retval 1 Programmable clock is enabled.
 */
uint32_t pmc_is_pck_enabled(uint32_t ul_id)
{
    if (ul_id > 2) {
        return 0;
    }

    return (PMC->PMC_SCSR & (PMC_SCSR_PCK0 << ul_id));
}

#if (SAM3S || SAM3XA || SAM4S || SAM4E)
/**
 * \brief Switch UDP (USB) clock source selection to PLLA clock.
 *
 * \param ul_usbdiv Clock divisor.
 */
void pmc_switch_udpck_to_pllack(uint32_t ul_usbdiv)
{
    PMC->PMC_USB = PMC_USB_USBDIV(ul_usbdiv);
}
#endif

#if (SAM3S || SAM4S)
/**
 * \brief Switch UDP (USB) clock source selection to PLLB clock.
 *
 * \param ul_usbdiv Clock divisor.
 */
void pmc_switch_udpck_to_pllbck(uint32_t ul_usbdiv)
{
    PMC->PMC_USB = PMC_USB_USBDIV(ul_usbdiv) | PMC_USB_USBS;
}
#endif

#if (SAM3XA)
/**
 * \brief Switch UDP (USB) clock source selection to UPLL clock.
 *
 * \param dw_usbdiv Clock divisor.
 */
void pmc_switch_udpck_to_upllck(uint32_t ul_usbdiv)
{
    PMC->PMC_USB = PMC_USB_USBS | PMC_USB_USBDIV(ul_usbdiv);
}
#endif

#if (SAM3S || SAM3XA || SAM4S || SAM4E)
/**
 * \brief Enable UDP (USB) clock.
 */
void pmc_enable_udpck(void)
{
# if (SAM3S || SAM4S || SAM4E)
    PMC->PMC_SCER = PMC_SCER_UDP;
# else
    PMC->PMC_SCER = PMC_SCER_UOTGCLK;
# endif
}

/**
 * \brief Disable UDP (USB) clock.
 */
void pmc_disable_udpck(void)
{
# if (SAM3S || SAM4S || SAM4E)
    PMC->PMC_SCDR = PMC_SCDR_UDP;
# else
    PMC->PMC_SCDR = PMC_SCDR_UOTGCLK;
# endif
}
#endif

/**
 * \brief Enable PMC interrupts.
 *
 * \param ul_sources Interrupt sources bit map.
 */
void pmc_enable_interrupt(uint32_t ul_sources)
{
    PMC->PMC_IER = ul_sources;
}

/**
 * \brief Disable PMC interrupts.
 *
 * \param ul_sources Interrupt sources bit map.
 */
void pmc_disable_interrupt(uint32_t ul_sources)
{
    PMC->PMC_IDR = ul_sources;
}

/**
 * \brief Get PMC interrupt mask.
 *
 * \return The interrupt mask value.
 */
uint32_t pmc_get_interrupt_mask(void)
{
    return PMC->PMC_IMR;
}

/**
 * \brief Get current status.
 *
 * \return The current PMC status.
 */
uint32_t pmc_get_status(void)
{
    return PMC->PMC_SR;
}

/**
 * \brief Set the wake-up inputs for fast startup mode registers
 *        (event generation).
 *
 * \param ul_inputs Wake up inputs to enable.
 */
void pmc_set_fast_startup_input(uint32_t ul_inputs)
{
    ul_inputs &= PMC_FAST_STARTUP_Msk;
    PMC->PMC_FSMR |= ul_inputs;
}

/**
 * \brief Clear the wake-up inputs for fast startup mode registers
 *        (remove event generation).
 *
 * \param ul_inputs Wake up inputs to disable.
 */
void pmc_clr_fast_startup_input(uint32_t ul_inputs)
{
    ul_inputs &= PMC_FAST_STARTUP_Msk;
    PMC->PMC_FSMR &= ~ul_inputs;
}

/**
 * \brief Enable Sleep Mode.
 * Enter condition: (WFE or WFI) + (SLEEPDEEP bit = 0) + (LPM bit = 0)
 *
 * \param uc_type 0 for wait for interrupt, 1 for wait for event.
 * \note For SAM4S and SAM4E series, since only WFI is effective, uc_type = 1
 * will be treated as uc_type = 0.
 */
void pmc_enable_sleepmode(uint8_t uc_type)
{
#if !defined(SAM4S) || !defined(SAM4E) || !defined(SAM4N)
    PMC->PMC_FSMR &= (uint32_t) ~ PMC_FSMR_LPM; // Enter Sleep mode
#endif
    SCB->SCR &= (uint32_t) ~ SCB_SCR_SLEEPDEEP_Msk; // Deep sleep

#if (SAM4S || SAM4E || SAM4N)
    UNUSED(uc_type);
    __WFI();
#else
    if (uc_type == 0) {
        __WFI();
    } else {
        __WFE();
    }
#endif
}

#if (SAM4S || SAM4E || SAM4N)
static uint32_t ul_flash_in_wait_mode = PMC_WAIT_MODE_FLASH_DEEP_POWERDOWN;
/**
 * \brief Set the embedded flash state in wait mode
 *
 * \param ul_flash_state PMC_WAIT_MODE_FLASH_STANDBY flash in standby mode,
 * PMC_WAIT_MODE_FLASH_DEEP_POWERDOWN flash in deep power down mode.
 */
void pmc_set_flash_in_wait_mode(uint32_t ul_flash_state)
{
    ul_flash_in_wait_mode = ul_flash_state;
}

/**
 * \brief Enable Wait Mode. Enter condition: (WAITMODE bit = 1) +
 * (SLEEPDEEP bit = 0) + FLPM
 */
void pmc_enable_waitmode(void)
{
    uint32_t i;

    /* Flash in Deep Power Down mode */
    i = PMC->PMC_FSMR;
    i &= ~PMC_FSMR_FLPM_Msk;
    i |= ul_flash_in_wait_mode;
    PMC->PMC_FSMR = i;

    /* Clear SLEEPDEEP bit */
    SCB->SCR &= (uint32_t) ~ SCB_SCR_SLEEPDEEP_Msk;

    /* Backup FWS setting and set Flash Wait State at 0 */
#if defined(ID_EFC)
    uint32_t fmr_backup;
    fmr_backup = EFC->EEFC_FMR;
    EFC->EEFC_FMR &= (uint32_t) ~ EEFC_FMR_FWS_Msk;
#endif
#if defined(ID_EFC0)
    uint32_t fmr0_backup;
    fmr0_backup = EFC0->EEFC_FMR;
    EFC0->EEFC_FMR &= (uint32_t) ~ EEFC_FMR_FWS_Msk;
#endif
#if defined(ID_EFC1)
    uint32_t fmr1_backup;
    fmr1_backup = EFC1->EEFC_FMR;
    EFC1->EEFC_FMR &= (uint32_t) ~ EEFC_FMR_FWS_Msk;
#endif

    /* Set the WAITMODE bit = 1 */
    PMC->CKGR_MOR |= CKGR_MOR_KEY(0x37u) | CKGR_MOR_WAITMODE;

    /* Waiting for Master Clock Ready MCKRDY = 1 */
    while (!(PMC->PMC_SR & PMC_SR_MCKRDY));

    /* Waiting for MOSCRCEN bit cleared is strongly recommended
     * to ensure that the core will not execute undesired instructions
     */
    for (i = 0; i < 500; i++) {
        __NOP();
    }
    while (!(PMC->CKGR_MOR & CKGR_MOR_MOSCRCEN));

    /* Restore EFC FMR setting */
#if defined(ID_EFC)
    EFC->EEFC_FMR = fmr_backup;
#endif
#if defined(ID_EFC0)
    EFC0->EEFC_FMR = fmr0_backup;
#endif
#if defined(ID_EFC1)
    EFC1->EEFC_FMR = fmr1_backup;
#endif
}
#else
/**
 * \brief Enable Wait Mode. Enter condition: WFE + (SLEEPDEEP bit = 0) +
 * (LPM bit = 1)
 */
void pmc_enable_waitmode(void)
{
    uint32_t i;

    PMC->PMC_FSMR |= PMC_FSMR_LPM; /* Enter Wait mode */
    SCB->SCR &= (uint32_t) ~ SCB_SCR_SLEEPDEEP_Msk; /* Deep sleep */
    __WFE();

    /* Waiting for MOSCRCEN bit cleared is strongly recommended
     * to ensure that the core will not execute undesired instructions
     */
    for (i = 0; i < 500; i++) {
        __NOP();
    }
    while (!(PMC->CKGR_MOR & CKGR_MOR_MOSCRCEN));
}
#endif

/**
 * \brief Enable Backup Mode. Enter condition: WFE/(VROFF bit = 1) +
 * (SLEEPDEEP bit = 1)
 */
void pmc_enable_backupmode(void)
{
    SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;
#if (SAM4S || SAM4E || SAM4N)
    SUPC->SUPC_CR = SUPC_CR_KEY(SUPC_KEY_VALUE) | SUPC_CR_VROFF_STOP_VREG;
#else
    __WFE();
#endif
}

/**
 * \brief Enable Clock Failure Detector.
 */
void pmc_enable_clock_failure_detector(void)
{
    uint32_t ul_reg = PMC->CKGR_MOR;

    PMC->CKGR_MOR = PMC_CKGR_MOR_KEY_VALUE | CKGR_MOR_CFDEN | ul_reg;
}

/**
 * \brief Disable Clock Failure Detector.
 */
void pmc_disable_clock_failure_detector(void)
{
    uint32_t ul_reg = PMC->CKGR_MOR & (~CKGR_MOR_CFDEN);

    PMC->CKGR_MOR = PMC_CKGR_MOR_KEY_VALUE | ul_reg;
}

#if SAM4N
/**
 * \brief Enable Slow Crystal Oscillator Frequency Monitoring.
 */
void pmc_enable_sclk_osc_freq_monitor(void)
{
    uint32_t ul_reg = PMC->CKGR_MOR;

    PMC->CKGR_MOR = PMC_CKGR_MOR_KEY_VALUE | CKGR_MOR_XT32KFME | ul_reg;
}

/**
 * \brief Disable Slow Crystal Oscillator Frequency Monitoring.
 */
void pmc_disable_sclk_osc_freq_monitor(void)
{
    uint32_t ul_reg = PMC->CKGR_MOR & (~CKGR_MOR_XT32KFME);

    PMC->CKGR_MOR = PMC_CKGR_MOR_KEY_VALUE | ul_reg;
}
#endif

/**
 * \brief Enable or disable write protect of PMC registers.
 *
 * \param ul_enable 1 to enable, 0 to disable.
 */
void pmc_set_writeprotect(uint32_t ul_enable)
{
    if (ul_enable) {
        PMC->PMC_WPMR = PMC_WPMR_WPKEY_VALUE | PMC_WPMR_WPEN;
    } else {
        PMC->PMC_WPMR = PMC_WPMR_WPKEY_VALUE;
    }
}

/**
 * \brief Return write protect status.
 *
 * \retval 0 Protection disabled.
 * \retval 1 Protection enabled.
 */
uint32_t pmc_get_writeprotect_status(void)
{
    return PMC->PMC_WPMR & PMC_WPMR_WPEN;
}

/// @cond 0
/**INDENT-OFF**/
#ifdef __cplusplus
}
#endif
/**INDENT-ON**/
/// @endcond
